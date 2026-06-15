#ifndef MAX30102_PICO_H
#define MAX30102_PICO_H

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX30102_I2C_ADDR        0x57
#define MAX30102_PART_ID         0x15

#define MAX30102_FIFO_DEPTH      32
#define MAX30102_STORAGE_SIZE    32

/* Register addresses */
#define REG_INTR_STATUS_1        0x00
#define REG_INTR_STATUS_2        0x01
#define REG_INTR_ENABLE_1        0x02
#define REG_INTR_ENABLE_2        0x03
#define REG_FIFO_WR_PTR          0x04
#define REG_OVF_COUNTER           0x05
#define REG_FIFO_RD_PTR          0x06
#define REG_FIFO_DATA            0x07
#define REG_FIFO_CONFIG          0x08
#define REG_MODE_CONFIG          0x09
#define REG_SPO2_CONFIG          0x0A
#define REG_LED1_PA              0x0C
#define REG_LED2_PA              0x0D
#define REG_PILOT_PA             0x10
#define REG_MULTI_LED_CTRL1      0x11
#define REG_MULTI_LED_CTRL2      0x12
#define REG_TEMP_INTR            0x1F
#define REG_TEMP_FRAC            0x20
#define REG_TEMP_CONFIG          0x21
#define REG_PROX_INT_THRESH      0x30
#define REG_REV_ID               0xFE
#define REG_PART_ID              0xFF

/* Sample average options for FIFO_CONFIG */
#define SAMPLEAVG_1              0x00
#define SAMPLEAVG_2              0x20
#define SAMPLEAVG_4              0x40
#define SAMPLEAVG_8              0x60
#define SAMPLEAVG_16             0x80
#define SAMPLEAVG_32             0xA0

/* Mode config */
#define MODE_HR                  0x02
#define MODE_SPO2                0x03
#define MODE_MULTI_LED           0x07
#define MODE_SHDN                0x80
#define MODE_RESET               0x40

/* SPO2 config: ADC range */
#define SPO2_ADC_RGE_2048        0x00
#define SPO2_ADC_RGE_4096        0x20
#define SPO2_ADC_RGE_8192        0x40
#define SPO2_ADC_RGE_16384       0x60

/* SPO2 config: sample rate */
#define SPO2_SR_50               0x00
#define SPO2_SR_100              0x04
#define SPO2_SR_200              0x08
#define SPO2_SR_400              0x0C
#define SPO2_SR_800              0x10
#define SPO2_SR_1000             0x14
#define SPO2_SR_1600             0x18
#define SPO2_SR_3200             0x1C

/* SPO2 config: pulse width */
#define LED_PW_69                0x00
#define LED_PW_118               0x01
#define LED_PW_215               0x02
#define LED_PW_411               0x03

typedef struct {
    i2c_inst_t *i2c;
    uint8_t     addr;
    struct {
        uint32_t red[MAX30102_STORAGE_SIZE];
        uint32_t ir[MAX30102_STORAGE_SIZE];
        uint8_t  head;
        uint8_t  tail;
    } fifo;
} max30102_t;

typedef struct {
    uint8_t  sample_avg;
    uint8_t  mode;
    uint8_t  spo2_adc_range;
    uint8_t  spo2_sample_rate;
    uint8_t  led_pulse_width;
    uint8_t  led1_current;
    uint8_t  led2_current;
    uint8_t  pilot_current;
    uint8_t  fifo_almost_full;
    bool     fifo_rollover;
} max30102_config_t;

/* Initialise sensor, verify device ID. Returns true on success. */
bool max30102_init(max30102_t *sensor, i2c_inst_t *i2c, uint8_t addr);

/* Apply configuration and start sampling.
   If config is NULL, defaults are used: avg=4, SPO2 mode, 100Hz, 411us, 6mA LEDs. */
void max30102_setup(max30102_t *sensor, const max30102_config_t *config);

/* Soft-reset the sensor. */
void max30102_reset(max30102_t *sensor);

/* Shutdown sensor (low-power). */
void max30102_shutdown(max30102_t *sensor);

/* Wake sensor from shutdown. */
void max30102_wake(max30102_t *sensor);

/* Read raw temperature. Returns Celsius. */
float max30102_read_temperature(max30102_t *sensor);

/* Read temperature as integer * 100 (e.g. 2550 = 25.50 C).
   Avoids floating point for embedded use. */
int16_t max30102_read_temperature_fixed(max30102_t *sensor);

/* Poll FIFO for new data. Returns number of new samples pulled. */
uint16_t max30102_check(max30102_t *sensor);

/* Number of unread samples in buffer. */
uint8_t max30102_available(max30102_t *sensor);

/* Advance read pointer; discards oldest sample. */
void max30102_next_sample(max30102_t *sensor);

/* Get latest Red / IR readings. */
uint32_t max30102_get_red(const max30102_t *sensor);
uint32_t max30102_get_ir(const max30102_t *sensor);

/* Low-level helpers (public for advanced use). */
bool max30102_read_reg(const max30102_t *sensor, uint8_t reg, uint8_t *value);
bool max30102_write_reg(const max30102_t *sensor, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif
