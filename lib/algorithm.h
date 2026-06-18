#ifndef SPO2_ALGORITHM_H
#define SPO2_ALGORITHM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 有效 FIFO 输出速率 = SPO2 采样率 / 采样平均次数。
 * 默认传感器配置（max30102_setup(sensor, NULL)）使用
 * SPO2_SR_100 + SAMPLEAVG_4 → 100 / 4 = 25 Hz。                     */
#define SPO2_SAMPLE_RATE_HZ     25

/* 分析窗口长度（25 Hz 下对应 4 秒）。                                 */
#define SPO2_BUFFER_SIZE        100

/* 认定为"手指存在"的最低 IR DC 电平（原始 18 位 FIFO 值，范围
 * 0..262143）。该值取决于 LED 电流、ADC 量程和环境光——
 * 请根据实际硬件调整（参见 README.md）。                             */
#define SPO2_FINGER_THRESHOLD   50000

typedef struct {
    float    spo2;             /* 估算的血氧饱和度（百分比）             */
    bool     spo2_valid;       /* SpO2 值是否有效                       */
    float    heart_rate;       /* 估算的心率（BPM）                     */
    bool     hr_valid;         /* 心率值是否有效                        */
    bool     finger_present;   /* 是否检测到手指                        */
    uint32_t dc_ir;            /* 最新 IR DC 电平（诊断用）              */
    uint32_t dc_red;           /* 最新 RED DC 电平（诊断用）            */
} spo2_result_t;

/* 重置内部状态（DC 跟踪器和采样窗口）。主循环启动前调用一次。          */
void spo2_algorithm_init(void);

/* 将一对 (IR, RED) 采样数据送入分析窗口。
 * 每从 MAX30102 FIFO 拉取一个样本调用一次。
 * 注意：在新版 MAX30102 驱动下，MAX30102.c 使用
 * spo2_algorithm_add_sample(red, ir) 顺序以适应修正后的字节序。      */
void spo2_algorithm_add_sample(uint32_t ir_sample, uint32_t red_sample);

/* 当收集满 SPO2_BUFFER_SIZE 个新样本后返回 true，表示可以计算结果。    */
bool spo2_algorithm_ready(void);

/* 基于当前窗口计算 SpO2 / 心率，然后重置窗口以便下次收集新样本。      */
spo2_result_t spo2_algorithm_compute(void);

#ifdef __cplusplus
}
#endif

#endif /* SPO2_ALGORITHM_H */
