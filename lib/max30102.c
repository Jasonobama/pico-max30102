/**
 * @file    max30102.c
 * @brief   MAX30102 脉搏血氧/心率传感器驱动实现
 *
 * 特性：
 * - FIFO 突发读取（单次 I2C 事务读取指针 + 数据，最多 5 样本/次）
 * - 温度测量触发 + 轮询等待完成
 * - 复位轮询（发复位命令后轮询寄存器直到完成，最快 1-2ms）
 * - 所有 I2C 写入均检查返回值，配置失败可检测
 * - 环形缓冲区使用 `FIFO_MASK` 位掩码（无分支）
 */

#include "max30102.h"
#include <string.h>

/* ---------- FIFO 通信参数 ---------- */
#define BYTES_PER_SAMPLE      6     /* 每样本 6 字节：RED[2] + IR[2] + 各 1 字节填充 */
#define MAX_SAMPLES_PER_BURST 5     /* 单次 I2C 突发最多读取 5 个样本 */
#define MAX_BURST (MAX_SAMPLES_PER_BURST * BYTES_PER_SAMPLE)  /* 30 字节 */

/* ==================================================================
 *  底层 I2C 寄存器读写
 * ================================================================== */

bool max30102_read_reg(const max30102_t *sensor, uint8_t reg, uint8_t *value) {
    /* 先写寄存器地址（restart），再读 1 字节 */
    if (i2c_write_blocking(sensor->i2c, sensor->addr, &reg, 1, true) != 1)
        return false;
    return i2c_read_blocking(sensor->i2c, sensor->addr, value, 1, false) == 1;
}

bool max30102_write_reg(const max30102_t *sensor, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_write_blocking(sensor->i2c, sensor->addr, buf, 2, false) == 2;
}

/* ==================================================================
 *  温度测量触发
 * ================================================================== */

/**
 * @brief  触发一次温度转换并轮询等待完成。
 *         向 REG_TEMP_CONFIG 写 0x01 启动转换，然后循环读该寄存器
 *         直到 bit0 清零（转换完成）。最长等待 40ms。
 * @return 转换成功返回 true，超时或 I2C 错误返回 false
 */
static bool trigger_temperature(max30102_t *sensor) {
    /* 写 0x01 触发温度转换 */
    if (!max30102_write_reg(sensor, REG_TEMP_CONFIG, 0x01))
        return false;

    uint8_t cfg;
    /* 轮询等待转换完成（bit0 清零），最多 40 次 × 1ms */
    for (int i = 0; i < 40; i++) {
        sleep_ms(1);
        if (!max30102_read_reg(sensor, REG_TEMP_CONFIG, &cfg))
            return false;
        if (!(cfg & 0x01))          /* 转换完成 */
            return true;
    }
    return false;                   /* 超时 */
}

/* ==================================================================
 *  初始化与配置
 * ================================================================== */

bool max30102_init(max30102_t *sensor, i2c_inst_t *i2c, uint8_t addr) {
    memset(sensor, 0, sizeof(*sensor));
    sensor->i2c  = i2c;
    sensor->addr = addr;

    uint8_t part;
    /* 验证器件 ID 是否为预期的 0x15 */
    return max30102_read_reg(sensor, REG_PART_ID, &part)
        && part == MAX30102_PART_ID;
}

void max30102_reset(max30102_t *sensor) {
    /* 写复位位（bit6 = 1） */
    max30102_write_reg(sensor, REG_MODE_CONFIG, MODE_RESET);

    uint8_t val;
    /* 轮询等待复位完成（bit6 自动清零），最长 10ms */
    for (int i = 0; i < 10; i++) {
        sleep_ms(1);
        if (max30102_read_reg(sensor, REG_MODE_CONFIG, &val)
            && !(val & MODE_RESET))
            return;                 /* 复位完成 */
    }
}

bool max30102_setup(max30102_t *sensor, const max30102_config_t *config) {
    /* 使用传入配置或默认值（SPO2 模式, 100Hz, 4x 平均, 6mA LED） */
    max30102_config_t cfg = config ? *config :
        (max30102_config_t) {
            SAMPLEAVG_4, MODE_SPO2,
            SPO2_ADC_RGE_4096, SPO2_SR_100, LED_PW_411,
            0x17, 0x17, 0x1F,   /* LED 电流 ≈ 6mA, 6mA, 8mA */
            17, false            /* FIFO 几乎满阈值, 不翻转 */
        };

    /* 软件复位 + 等待完成 */
    max30102_reset(sensor);

    /* 清零 FIFO 指针和溢出计数器 */
    if (!max30102_write_reg(sensor, REG_FIFO_WR_PTR, 0)) return false;
    if (!max30102_write_reg(sensor, REG_OVF_COUNTER,  0)) return false;
    if (!max30102_write_reg(sensor, REG_FIFO_RD_PTR,  0)) return false;

    /* 组装并写入配置寄存器 */
    uint8_t fcfg = cfg.sample_avg
                 | (cfg.fifo_rollover ? 0x10 : 0x00)
                 | (cfg.fifo_almost_full & 0x0F);

    uint8_t spcfg = cfg.spo2_adc_range
                  | cfg.spo2_sample_rate
                  | cfg.led_pulse_width;

    /* 每一步写入检查返回值，任一步失败则返回 false */
    if (!max30102_write_reg(sensor, REG_FIFO_CONFIG,  fcfg))   return false;
    if (!max30102_write_reg(sensor, REG_MODE_CONFIG,  cfg.mode)) return false;
    if (!max30102_write_reg(sensor, REG_SPO2_CONFIG,  spcfg))  return false;
    if (!max30102_write_reg(sensor, REG_LED1_PA, cfg.led1_current)) return false;
    if (!max30102_write_reg(sensor, REG_LED2_PA, cfg.led2_current)) return false;
    if (!max30102_write_reg(sensor, REG_PILOT_PA, cfg.pilot_current)) return false;

    /* 重置软件缓冲区指针 */
    sensor->fifo.head = 0;
    sensor->fifo.tail = 0;
    return true;
}

/* ==================================================================
 *  电源管理
 * ================================================================== */

void max30102_shutdown(max30102_t *sensor) {
    uint8_t v;
    if (max30102_read_reg(sensor, REG_MODE_CONFIG, &v))
        max30102_write_reg(sensor, REG_MODE_CONFIG, v | MODE_SHDN);
}

void max30102_wake(max30102_t *sensor) {
    uint8_t v;
    if (max30102_read_reg(sensor, REG_MODE_CONFIG, &v))
        max30102_write_reg(sensor, REG_MODE_CONFIG, v & ~MODE_SHDN);
}

/* ==================================================================
 *  温度传感器
 * ================================================================== */

float max30102_read_temperature(max30102_t *sensor) {
    /* 必须先触发一次新转换，否则可能读到旧/无效数据 */
    if (!trigger_temperature(sensor))
        return 0.0f;

    uint8_t tint, tfrac;
    max30102_read_reg(sensor, REG_TEMP_INTR, &tint);    /* 整数部分 */
    max30102_read_reg(sensor, REG_TEMP_FRAC, &tfrac);   /* 高 4 位为小数 */

    /* 温度 = 整数值 + 小数位 × 0.0625℃/步 */
    return (float)(int8_t)tint + (float)(tfrac & 0x0F) * 0.0625f;
}

int16_t max30102_read_temperature_fixed(max30102_t *sensor) {
    /* 转换为定点数 ×100，带四舍五入 */
    if (!trigger_temperature(sensor))
        return 0;

    uint8_t tint, tfrac;
    max30102_read_reg(sensor, REG_TEMP_INTR, &tint);
    max30102_read_reg(sensor, REG_TEMP_FRAC, &tfrac);

    /* 整数 ×100 + 小数 ×6.25 + 0.5 四舍五入 */
    return ((int8_t)tint * 100) + (((tfrac & 0x0F) * 625 + 50) / 100);
}

/* ==================================================================
 *  FIFO 数据管理
 * ================================================================== */

uint8_t max30102_available(const max30102_t *sensor) {
    /* 环形缓冲区可用样本数（head - tail 取模） */
    return (sensor->fifo.head - sensor->fifo.tail) & FIFO_MASK;
}

uint32_t max30102_get_red(const max30102_t *sensor) {
    return sensor->fifo.red[sensor->fifo.tail];
}

uint32_t max30102_get_ir(const max30102_t *sensor) {
    return sensor->fifo.ir[sensor->fifo.tail];
}

void max30102_next_sample(max30102_t *sensor) {
    /* 读取指针前移（带掩码取模） */
    if (max30102_available(sensor))
        sensor->fifo.tail = (sensor->fifo.tail + 1u) & FIFO_MASK;
}

/* ==================================================================
 *  硬件 FIFO 轮询与突发读取
 * ================================================================== */

uint16_t max30102_check(max30102_t *sensor) {
    uint8_t ptrs[3], reg = REG_FIFO_WR_PTR;

    /* 一次性读取 WR_PTR（0x04）、OVF_COUNTER（0x05）、RD_PTR（0x06） */
    if (i2c_write_blocking(sensor->i2c, sensor->addr, &reg, 1, true) != 1)
        return 0;
    if (i2c_read_blocking(sensor->i2c, sensor->addr, ptrs, 3, false) != 3)
        return 0;

    uint8_t wr_ptr = ptrs[0];       /* FIFO 写指针 */
    /* ptrs[1] = OVF_COUNTER（暂未使用） */
    uint8_t rd_ptr = ptrs[2];       /* FIFO 读指针 */

    if (wr_ptr == rd_ptr) return 0; /* FIFO 空 */

    /* 计算待读取样本数（环形 FIFO 差值） */
    int16_t samples = wr_ptr - rd_ptr;
    if (samples < 0)
        samples += MAX30102_FIFO_DEPTH;

    /* 单次突发最多读 5 个样本（30 字节），超出部分下次再读 */
    uint8_t bytes_to_read = (uint8_t)samples * BYTES_PER_SAMPLE;
    if (bytes_to_read > MAX_BURST)
        bytes_to_read = MAX_BURST;

    /* 突发读取 FIFO 数据 */
    uint8_t buf[MAX_BURST];
    reg = REG_FIFO_DATA;
    if (i2c_write_blocking(sensor->i2c, sensor->addr, &reg, 1, true) != 1)
        return 0;
    if (i2c_read_blocking(sensor->i2c, sensor->addr, buf, bytes_to_read, false) != bytes_to_read)
        return 0;

    /* 解析样本数据并存入环形缓冲区 */
    uint16_t decoded = 0;
    for (uint8_t i = 0; i < bytes_to_read; i += BYTES_PER_SAMPLE) {
        uint8_t head = sensor->fifo.head;

        /* MAX30102 SPO2 模式 FIFO 格式：RED[18bit] 在前，IR[18bit] 在后
         * 每通道 3 字节（MSB 在前），取低 18 位有效数据 */
        sensor->fifo.red[head] = ((((uint32_t)buf[i])     << 16) |
                                  (((uint32_t)buf[i + 1]) << 8)  |
                                   ((uint32_t)buf[i + 2])) & 0x3FFFF;

        sensor->fifo.ir[head]  = ((((uint32_t)buf[i + 3]) << 16) |
                                  (((uint32_t)buf[i + 4]) << 8)  |
                                   ((uint32_t)buf[i + 5])) & 0x3FFFF;

        sensor->fifo.head = (head + 1u) & FIFO_MASK;
        decoded++;
    }

    return decoded;                 /* 返回实际解码的样本数 */
}
