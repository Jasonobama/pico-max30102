#include "max30102.h"
#include <string.h>

#define BYTES_PER_SAMPLE 6
#define MAX_BURST         32

/* ------------------------------------------------------------------ */
/*  Internal helpers                                                   */
/* ------------------------------------------------------------------ */

static inline bool read_reg(const max30102_t *sensor, uint8_t reg, uint8_t *value) {
    int ret = i2c_write_blocking(sensor->i2c, sensor->addr, &reg, 1, true);
    if (ret != 1) return false;
    ret = i2c_read_blocking(sensor->i2c, sensor->addr, value, 1, false);
    return (ret == 1);
}

static inline bool write_reg(const max30102_t *sensor, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    int ret = i2c_write_blocking(sensor->i2c, sensor->addr, buf, 2, false);
    return (ret == 2);
}

bool max30102_read_reg(const max30102_t *sensor, uint8_t reg, uint8_t *value) {
    return read_reg(sensor, reg, value);
}

bool max30102_write_reg(const max30102_t *sensor, uint8_t reg, uint8_t value) {
    return write_reg(sensor, reg, value);
}

/* ------------------------------------------------------------------ */
/*  Public API                                                         */
/* ------------------------------------------------------------------ */

bool max30102_init(max30102_t *sensor, i2c_inst_t *i2c, uint8_t addr) {
    memset(sensor, 0, sizeof(*sensor));
    sensor->i2c  = i2c;
    sensor->addr = addr;

    uint8_t part_id;
    if (!read_reg(sensor, REG_PART_ID, &part_id)) return false;
    if (part_id != MAX30102_PART_ID) return false;
    return true;
}

void max30102_reset(max30102_t *sensor) {
    write_reg(sensor, REG_MODE_CONFIG, MODE_RESET);
    sleep_ms(500);
}

void max30102_shutdown(max30102_t *sensor) {
    uint8_t val;
    read_reg(sensor, REG_MODE_CONFIG, &val);
    write_reg(sensor, REG_MODE_CONFIG, val | MODE_SHDN);
}

void max30102_wake(max30102_t *sensor) {
    uint8_t val;
    read_reg(sensor, REG_MODE_CONFIG, &val);
    write_reg(sensor, REG_MODE_CONFIG, val & ~MODE_SHDN);
}

void max30102_setup(max30102_t *sensor, const max30102_config_t *config) {
    max30102_config_t cfg;

    if (config) {
        cfg = *config;
    } else {
        cfg.sample_avg       = SAMPLEAVG_4;
        cfg.mode             = MODE_SPO2;
        cfg.spo2_adc_range   = SPO2_ADC_RGE_4096;
        cfg.spo2_sample_rate = SPO2_SR_100;
        cfg.led_pulse_width  = LED_PW_411;
        cfg.led1_current     = 0x17;
        cfg.led2_current     = 0x17;
        cfg.pilot_current    = 0x1F;
        cfg.fifo_almost_full = 17;
        cfg.fifo_rollover    = false;
    }

    max30102_reset(sensor);

    write_reg(sensor, REG_FIFO_WR_PTR,  0x00);
    write_reg(sensor, REG_OVF_COUNTER,   0x00);
    write_reg(sensor, REG_FIFO_RD_PTR,  0x00);

    uint8_t fcfg = cfg.sample_avg | (cfg.fifo_rollover ? 0x10 : 0x00) |
                   (cfg.fifo_almost_full & 0x0F);
    write_reg(sensor, REG_FIFO_CONFIG, fcfg);
    write_reg(sensor, REG_MODE_CONFIG,  cfg.mode);

    uint8_t spcfg = cfg.spo2_adc_range | cfg.spo2_sample_rate | cfg.led_pulse_width;
    write_reg(sensor, REG_SPO2_CONFIG,  spcfg);
    write_reg(sensor, REG_LED1_PA,      cfg.led1_current);
    write_reg(sensor, REG_LED2_PA,      cfg.led2_current);
    write_reg(sensor, REG_PILOT_PA,     cfg.pilot_current);

    sensor->fifo.head = 0;
    sensor->fifo.tail = 0;
}

float max30102_read_temperature(max30102_t *sensor) {
    uint8_t tint, tfrac;
    read_reg(sensor, REG_TEMP_INTR, &tint);
    read_reg(sensor, REG_TEMP_FRAC, &tfrac);
    return (float)(int8_t)tint + (float)(tfrac & 0x0F) * 0.0625f;
}

int16_t max30102_read_temperature_fixed(max30102_t *sensor) {
    uint8_t tint, tfrac;
    read_reg(sensor, REG_TEMP_INTR, &tint);
    read_reg(sensor, REG_TEMP_FRAC, &tfrac);
    int16_t integer = (int8_t)tint * 100;
    int16_t fraction = (int16_t)(tfrac & 0x0F) * 625 / 100;
    return integer + fraction;
}

uint8_t max30102_available(max30102_t *sensor) {
    int8_t n = sensor->fifo.head - sensor->fifo.tail;
    if (n < 0) n += MAX30102_STORAGE_SIZE;
    return (uint8_t)n;
}

uint32_t max30102_get_red(const max30102_t *sensor) {
    return sensor->fifo.red[sensor->fifo.tail];
}

uint32_t max30102_get_ir(const max30102_t *sensor) {
    return sensor->fifo.ir[sensor->fifo.tail];
}

void max30102_next_sample(max30102_t *sensor) {
    if (max30102_available(sensor)) {
        sensor->fifo.tail++;
        sensor->fifo.tail %= MAX30102_STORAGE_SIZE;
    }
}

uint16_t max30102_check(max30102_t *sensor) {
    uint8_t rd_ptr, wr_ptr;
    if (!read_reg(sensor, REG_FIFO_RD_PTR, &rd_ptr)) return 0;
    if (!read_reg(sensor, REG_FIFO_WR_PTR, &wr_ptr)) return 0;

    if (rd_ptr == wr_ptr) return 0;

    int16_t samples = (int16_t)wr_ptr - (int16_t)rd_ptr;
    if (samples < 0) samples += MAX30102_FIFO_DEPTH;

    uint8_t bytes_to_read = (uint8_t)samples * BYTES_PER_SAMPLE;
    if (bytes_to_read > MAX_BURST) bytes_to_read = MAX_BURST;

    uint8_t buf[MAX_BURST];
    uint8_t reg = REG_FIFO_DATA;
    int ret = i2c_write_blocking(sensor->i2c, sensor->addr, &reg, 1, true);
    if (ret != 1) return 0;
    ret = i2c_read_blocking(sensor->i2c, sensor->addr, buf, bytes_to_read, false);
    if (ret != bytes_to_read) return 0;

    for (uint8_t i = 0; i < bytes_to_read; i += BYTES_PER_SAMPLE) {
        sensor->fifo.head++;
        sensor->fifo.head %= MAX30102_STORAGE_SIZE;
        sensor->fifo.ir[sensor->fifo.head]  =
            (((uint32_t)buf[i]     << 16) |
             ((uint32_t)buf[i + 1] << 8)  |
             ((uint32_t)buf[i + 2])) & 0x3FFFF;
        sensor->fifo.red[sensor->fifo.head] =
            (((uint32_t)buf[i + 3] << 16) |
             ((uint32_t)buf[i + 4] << 8)  |
             ((uint32_t)buf[i + 5])) & 0x3FFFF;
    }

    return (uint16_t)samples;
}
