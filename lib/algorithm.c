#include "algorithm.h"
#include <math.h>
#include <string.h>

/* AC (DC-removed) sample windows, in chronological order. */
static float    ir_ac_buf[SPO2_BUFFER_SIZE];
static float    red_ac_buf[SPO2_BUFFER_SIZE];
static uint16_t buf_index;

/* Continuous DC trackers (simple first-order IIR low-pass filter).
 * These persist across windows so the DC estimate doesn't "reset"
 * every 4 seconds.                                                    */
static float dc_ir;
static float dc_red;
static bool  dc_initialized;

#define DC_ALPHA  0.95f   /* closer to 1.0 -> slower DC tracking */

/* Heart-rate search range, expressed as autocorrelation lags.
 * 40-200 BPM at 25 Hz -> lags 7..37 samples.                          */
#define HR_MIN_LAG  (SPO2_SAMPLE_RATE_HZ * 60 / 200)
#define HR_MAX_LAG  (SPO2_SAMPLE_RATE_HZ * 60 / 40)

void spo2_algorithm_init(void) {
    buf_index = 0;
    dc_ir = 0.0f;
    dc_red = 0.0f;
    dc_initialized = false;
    memset(ir_ac_buf, 0, sizeof(ir_ac_buf));
    memset(red_ac_buf, 0, sizeof(red_ac_buf));
}

void spo2_algorithm_add_sample(uint32_t ir_sample, uint32_t red_sample) {
    float ir  = (float)ir_sample;
    float red = (float)red_sample;

    if (!dc_initialized) {
        dc_ir  = ir;
        dc_red = red;
        dc_initialized = true;
    } else {
        dc_ir  = DC_ALPHA * dc_ir  + (1.0f - DC_ALPHA) * ir;
        dc_red = DC_ALPHA * dc_red + (1.0f - DC_ALPHA) * red;
    }

    if (buf_index < SPO2_BUFFER_SIZE) {
        ir_ac_buf[buf_index]  = ir  - dc_ir;
        red_ac_buf[buf_index] = red - dc_red;
        buf_index++;
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

    if (!spo2_algorithm_ready() || !result.finger_present) {
        buf_index = 0; /* start a fresh window regardless */
        return result;
    }

    /* ---- SpO2: ratio-of-ratios of AC RMS to DC level ----
     * R = (RMS(red_ac)/DC_red) / (RMS(ir_ac)/DC_ir)
     * SpO2 (%) ~= 110 - 25 * R   (commonly used linear approximation;
     * see README.md for calibration notes - NOT a medical-grade
     * calibration).                                                   */
    float sum_sq_ir = 0.0f, sum_sq_red = 0.0f;
    for (int i = 0; i < SPO2_BUFFER_SIZE; i++) {
        sum_sq_ir  += ir_ac_buf[i]  * ir_ac_buf[i];
        sum_sq_red += red_ac_buf[i] * red_ac_buf[i];
    }

    float rms_ir  = sqrtf(sum_sq_ir  / SPO2_BUFFER_SIZE);
    float rms_red = sqrtf(sum_sq_red / SPO2_BUFFER_SIZE);

    if (rms_ir > 1.0f && dc_ir > 1.0f && dc_red > 1.0f) {
        float ratio = (rms_red / dc_red) / (rms_ir / dc_ir);
        float spo2  = 110.0f - 25.0f * ratio;

        if (spo2 > 100.0f) spo2 = 100.0f;
        if (spo2 < 0.0f)   spo2 = 0.0f;

        result.spo2       = spo2;
        result.spo2_valid = true;
    }

    /* ---- Heart rate: autocorrelation peak of the IR AC signal ---- */
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
        result.heart_rate = (60.0f * SPO2_SAMPLE_RATE_HZ) / (float)best_lag;
        result.hr_valid   = true;
    }

    buf_index = 0; /* start collecting the next window */
    return result;
}
