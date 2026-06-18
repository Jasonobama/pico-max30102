#include "algorithm.h"
#include <math.h>
#include <string.h>

/* AC 信号（去直流后）采样窗口，按时间顺序排列。 */
static float    ir_ac_buf[SPO2_BUFFER_SIZE];
static float    red_ac_buf[SPO2_BUFFER_SIZE];
static uint16_t buf_index;

/* 连续 DC 跟踪器（一阶 IIR 低通滤波器）。
 * 这些值跨窗口持续更新，避免 DC 估计每 4 秒"重置"一次。           */
static float dc_ir;
static float dc_red;
static bool  dc_initialized;

/* 流式 RMS：增量累加 AC 平方和，避免 compute() 中每次重算 100 个样本 */
static float sum_sq_ir;
static float sum_sq_red;

#define DC_ALPHA  0.95f   /* 越接近 1.0 -> DC 跟踪越慢（越平滑） */

/* 心率搜索范围，以自相关滞后样本数表示。
 * 40-200 BPM，采样率 25 Hz -> 滞后 7..37 个采样点。               */
#define HR_MIN_LAG  (SPO2_SAMPLE_RATE_HZ * 60 / 200)
#define HR_MAX_LAG  (SPO2_SAMPLE_RATE_HZ * 60 / 40)

void spo2_algorithm_init(void) {
    buf_index = 0;
    dc_ir = 0.0f;
    dc_red = 0.0f;
    dc_initialized = false;
    sum_sq_ir  = 0.0f;
    sum_sq_red = 0.0f;
    memset(ir_ac_buf, 0, sizeof(ir_ac_buf));
    memset(red_ac_buf, 0, sizeof(red_ac_buf));
}

void spo2_algorithm_add_sample(uint32_t ir_sample, uint32_t red_sample) {
    float ir  = (float)ir_sample;
    float red = (float)red_sample;

    /* 首次采样：直接用原始值初始化 DC 跟踪器 */
    if (!dc_initialized) {
        dc_ir  = ir;
        dc_red = red;
        dc_initialized = true;
    } else {
        /* 后续采样：IIR 低通滤波逐步更新 DC 估计值 */
        dc_ir  = DC_ALPHA * dc_ir  + (1.0f - DC_ALPHA) * ir;
        dc_red = DC_ALPHA * dc_red + (1.0f - DC_ALPHA) * red;
    }

    /* 减去 DC 分量得到 AC 信号，存入环形窗口 */
    if (buf_index < SPO2_BUFFER_SIZE) {
        float ir_ac  = ir  - dc_ir;
        float red_ac = red - dc_red;
        ir_ac_buf[buf_index]  = ir_ac;
        red_ac_buf[buf_index] = red_ac;
        buf_index++;

        /* 流式累加 AC 平方和，compute() 中直接取用 */
        sum_sq_ir  += ir_ac  * ir_ac;
        sum_sq_red += red_ac * red_ac;
    }
}

bool spo2_algorithm_ready(void) {
    return buf_index >= SPO2_BUFFER_SIZE;
}

spo2_result_t spo2_algorithm_compute(void) {
    spo2_result_t result;
    memset(&result, 0, sizeof(result));

    result.dc_ir          = (uint32_t)dc_ir;
    result.dc_red         = (uint32_t)dc_red;
    result.finger_present = (dc_ir > SPO2_FINGER_THRESHOLD);

    /* 数据不足或未检测到手指：重置窗口并返回 */
    if (!spo2_algorithm_ready() || !result.finger_present) {
        buf_index  = 0;
        sum_sq_ir  = 0.0f;
        sum_sq_red = 0.0f;
        return result;
    }

    /* ---- SpO2：AC RMS 与 DC 电平的比率-比率法 ----
     * R = (RMS(red_ac)/DC_red) / (RMS(ir_ac)/DC_ir)
     * SpO2 (%) ≈ 110 - 25 × R
     * 使用流式累加的 sum_sq，无需遍历数组。                  */
    float rms_ir  = sqrtf(sum_sq_ir  / SPO2_BUFFER_SIZE);
    float rms_red = sqrtf(sum_sq_red / SPO2_BUFFER_SIZE);

    if (rms_ir > 1.0f && dc_ir > 1.0f && dc_red > 1.0f) {
        float ratio = (rms_red / dc_red) / (rms_ir / dc_ir);
        float spo2  = 110.0f - 25.0f * ratio;

        /* 钳位并检查数值合法性（防止 NaN/Inf 污染显示） */
        if (spo2 > 100.0f) spo2 = 100.0f;
        if (spo2 < 0.0f)   spo2 = 0.0f;
        if (isfinite(spo2)) {
            result.spo2       = spo2;
            result.spo2_valid = true;
        }
    }

    /* ---- 心率：IR AC 信号的自相关峰值检测 ---- */
    int   best_lag  = -1;
    float best_corr = 0.0f;

    for (int lag = HR_MIN_LAG; lag <= HR_MAX_LAG; lag++) {
        float corr = 0.0f;
        for (int i = 0; i < SPO2_BUFFER_SIZE - lag; i++) {
            corr += ir_ac_buf[i] * ir_ac_buf[i + lag];
        }
        if (corr > best_corr) {
            best_corr = corr;
            best_lag  = lag;
        }
    }

    if (best_lag > 0) {
        float hr = (60.0f * SPO2_SAMPLE_RATE_HZ) / (float)best_lag;
        /* 检查计算结果合法性，防止异常值进入显示 */
        if (isfinite(hr) && hr >= 30.0f && hr <= 250.0f) {
            result.heart_rate = hr;
            result.hr_valid   = true;
        }
    }

    /* 重置窗口和累加器，开始收集下一个窗口 */
    buf_index  = 0;
    sum_sq_ir  = 0.0f;
    sum_sq_red = 0.0f;
    return result;
}