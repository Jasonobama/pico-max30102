#ifndef SPO2_ALGORITHM_H
#define SPO2_ALGORITHM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Effective FIFO output rate = SPO2 sample rate / sample averaging.
 * Default sensor config (max30102_setup(sensor, NULL)) uses
 * SPO2_SR_100 with SAMPLEAVG_4 -> 100 / 4 = 25 Hz.                    */
#define SPO2_SAMPLE_RATE_HZ     25

/* Analysis window length (4 seconds at 25 Hz).                        */
#define SPO2_BUFFER_SIZE        100

/* Minimum IR DC level (raw 18-bit FIFO code, 0..262143) considered
 * "finger present". Depends on LED current, ADC range and ambient
 * light - tune this for your hardware (see README.md).               */
#define SPO2_FINGER_THRESHOLD   50000

typedef struct {
    float    spo2;             /* Estimated SpO2 in percent          */
    bool     spo2_valid;
    float    heart_rate;       /* Estimated heart rate in BPM        */
    bool     hr_valid;
    bool     finger_present;
    uint32_t dc_ir;             /* Latest IR DC level (diagnostics)   */
    uint32_t dc_red;            /* Latest RED DC level (diagnostics)  */
} spo2_result_t;

/* Reset internal state (DC trackers and sample window). Call once
 * before the main loop starts.                                       */
void spo2_algorithm_init(void);

/* Feed one (IR, RED) sample pair into the analysis window. Call once
 * per sample pulled from the MAX30102 FIFO.                          */
void spo2_algorithm_add_sample(uint32_t ir_sample, uint32_t red_sample);

/* Returns true once SPO2_BUFFER_SIZE fresh samples have been
 * collected and a result is ready to be computed.                    */
bool spo2_algorithm_ready(void);

/* Compute SpO2 / heart rate from the current window, then reset the
 * window so the next call collects a fresh set of samples.           */
spo2_result_t spo2_algorithm_compute(void);

#ifdef __cplusplus
}
#endif

#endif /* SPO2_ALGORITHM_H */
