/**
 * @file    max30102.h
 * @brief   MAX30102 脉搏血氧/心率传感器驱动 (Pico / Pico 2)
 *
 * I2C 接口，支持 SPO2 / HR 双模式、FIFO 突发读取、温度检测。
 * 可通过 max30102_config_t 自定义采样参数。
 */

#ifndef MAX30102_PICO_H
#define MAX30102_PICO_H

#include "hardware/i2c.h"
#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- I2C 地址与设备识别 ---------- */
#define MAX30102_I2C_ADDR        0x57    /* 7-bit I2C 地址 */
#define MAX30102_PART_ID         0x15    /* 器件 ID，用于校验 */

/* ---------- FIFO 参数 ---------- */
#define MAX30102_FIFO_DEPTH      32      /* 硬件 FIFO 深度 */
#define MAX30102_STORAGE_SIZE    32      /* 本地软件缓冲区大小 */
#define FIFO_MASK                (MAX30102_STORAGE_SIZE - 1u)  /* 环形缓冲区掩码（32 = 2^5） */

/* ---------- 寄存器地址 ---------- */
#define REG_INTR_STATUS_1        0x00    /* 中断状态 1 */
#define REG_INTR_STATUS_2        0x01    /* 中断状态 2 */
#define REG_INTR_ENABLE_1        0x02    /* 中断使能 1 */
#define REG_INTR_ENABLE_2        0x03    /* 中断使能 2 */
#define REG_FIFO_WR_PTR          0x04    /* FIFO 写指针 */
#define REG_OVF_COUNTER          0x05    /* FIFO 溢出计数器 */
#define REG_FIFO_RD_PTR          0x06    /* FIFO 读指针 */
#define REG_FIFO_DATA            0x07    /* FIFO 数据寄存器（连续读取） */
#define REG_FIFO_CONFIG          0x08    /* FIFO 配置（平均、翻转、阈值） */
#define REG_MODE_CONFIG          0x09    /* 模式配置（关断、复位、SPO2/HR） */
#define REG_SPO2_CONFIG          0x0A    /* SPO2 配置（ADC 范围、采样率、脉宽） */
#define REG_LED1_PA              0x0C    /* LED1 电流（RED，0x00-0xFF） */
#define REG_LED2_PA              0x0D    /* LED2 电流（IR，0x00-0xFF） */
#define REG_PILOT_PA             0x10    /* Pilot LED 电流 */
#define REG_MULTI_LED_CTRL1      0x11    /* 多 LED 模式控制 1 */
#define REG_MULTI_LED_CTRL2      0x12    /* 多 LED 模式控制 2 */
#define REG_TEMP_INTR            0x1F    /* 温度整数部分 */
#define REG_TEMP_FRAC            0x20    /* 温度小数部分（高 4 位） */
#define REG_TEMP_CONFIG          0x21    /* 温度配置（写 0x01 触发测量） */
#define REG_PROX_INT_THRESH      0x30    /* 接近中断阈值 */
#define REG_REV_ID               0xFE    /* 版本 ID */
#define REG_PART_ID              0xFF    /* 器件 ID（应为 0x15） */

/* ---------- 采样平均 ---------- */
#define SAMPLEAVG_1              0x00    /* 不平均 */
#define SAMPLEAVG_2              0x20    /* 2 次平均 */
#define SAMPLEAVG_4              0x40    /* 4 次平均 */
#define SAMPLEAVG_8              0x60    /* 8 次平均 */
#define SAMPLEAVG_16             0x80    /* 16 次平均 */
#define SAMPLEAVG_32             0xA0    /* 32 次平均 */

/* ---------- 工作模式 ---------- */
#define MODE_HR                  0x02    /* 心率模式（仅 RED LED） */
#define MODE_SPO2                0x03    /* SPO2 模式（RED + IR LED） */
#define MODE_MULTI_LED           0x07    /* 多 LED 模式 */
#define MODE_SHDN                0x80    /* 关断模式（低功耗） */
#define MODE_RESET               0x40    /* 软件复位 */

/* ---------- SPO2 ADC 量程 ---------- */
#define SPO2_ADC_RGE_2048        0x00    /* 2048 nA */
#define SPO2_ADC_RGE_4096        0x20    /* 4096 nA */
#define SPO2_ADC_RGE_8192        0x40    /* 8192 nA */
#define SPO2_ADC_RGE_16384       0x60    /* 16384 nA */

/* ---------- SPO2 采样率 ---------- */
#define SPO2_SR_50               0x00    /* 50 Hz */
#define SPO2_SR_100              0x04    /* 100 Hz */
#define SPO2_SR_200              0x08    /* 200 Hz */
#define SPO2_SR_400              0x0C    /* 400 Hz */
#define SPO2_SR_800              0x10    /* 800 Hz */
#define SPO2_SR_1000             0x14    /* 1000 Hz */
#define SPO2_SR_1600             0x18    /* 1600 Hz */
#define SPO2_SR_3200             0x1C    /* 3200 Hz */

/* ---------- LED 脉宽 ---------- */
#define LED_PW_69                0x00    /* 69 us 脉宽（低功耗，低精度） */
#define LED_PW_118               0x01    /* 118 us */
#define LED_PW_215               0x02    /* 215 us */
#define LED_PW_411               0x03    /* 411 us（推荐，高精度） */

/* ---------- 数据结构 ---------- */

/** @brief MAX30102 传感器实例 */
typedef struct {
    i2c_inst_t *i2c;            /**< I2C 外设句柄 */
    uint8_t     addr;            /**< 7-bit I2C 地址 */
    struct {
        uint32_t red[MAX30102_STORAGE_SIZE];  /**< RED LED 采样环形缓冲区 */
        uint32_t ir[MAX30102_STORAGE_SIZE];   /**< IR  LED 采样环形缓冲区 */
        uint8_t  head;           /**< 写入索引 */
        uint8_t  tail;           /**< 读取索引 */
    } fifo;
} max30102_t;

/** @brief 传感器配置参数 */
typedef struct {
    uint8_t sample_avg;         /**< 采样平均次数 */
    uint8_t mode;               /**< 工作模式（SPO2 / HR） */
    uint8_t spo2_adc_range;     /**< ADC 量程 */
    uint8_t spo2_sample_rate;   /**< 采样率 */
    uint8_t led_pulse_width;    /**< LED 脉宽 */
    uint8_t led1_current;       /**< LED1（RED）电流 0x00-0xFF */
    uint8_t led2_current;       /**< LED2（IR） 电流 0x00-0xFF */
    uint8_t pilot_current;      /**< Pilot LED 电流 */
    uint8_t fifo_almost_full;   /**< FIFO 几乎满阈值（0-15） */
    bool    fifo_rollover;      /**< FIFO 满后是否覆盖旧数据 */
} max30102_config_t;

/* ---------- API 函数 ---------- */

/** @brief 初始化传感器，验证器件 ID。
 *  @param sensor  传感器实例指针
 *  @param i2c     I2C 外设（i2c0 / i2c1）
 *  @param addr    7-bit I2C 地址
 *  @return        成功返回 true */
bool max30102_init(max30102_t *sensor, i2c_inst_t *i2c, uint8_t addr);

/** @brief 配置传感器并启动采样。
 *  @param config  配置参数指针（传 NULL 使用默认值）
 *  @return        所有寄存器写入成功返回 true */
bool max30102_setup(max30102_t *sensor, const max30102_config_t *config);

/** @brief 软件复位传感器（轮询等待完成，最长 10ms） */
void max30102_reset(max30102_t *sensor);

/** @brief 关断传感器（低功耗模式） */
void max30102_shutdown(max30102_t *sensor);

/** @brief 唤醒传感器 */
void max30102_wake(max30102_t *sensor);

/** @brief 读取芯片温度（摄氏度）。
 *  触发一次新的温度转换，轮询等待完成后返回结果。
 *  @return 温度值（℃），失败返回 0.0 */
float max30102_read_temperature(max30102_t *sensor);

/** @brief 读取芯片温度（定点数，避免浮点运算）。
 *  @return 温度 × 100（例如 2550 = 25.50℃），失败返回 0 */
int16_t max30102_read_temperature_fixed(max30102_t *sensor);

/** @brief 从硬件 FIFO 拉取新数据到本地缓冲区。
 *  一次 I2C 突发读取最多 5 个样本（30 字节）。
 *  @return 实际解码的样本数 */
uint16_t max30102_check(max30102_t *sensor);

/** @brief 本地缓冲区中未读取的样本数 */
uint8_t max30102_available(const max30102_t *sensor);

/** @brief 丢弃最旧的样本（读取指针前移） */
void max30102_next_sample(max30102_t *sensor);

/** @brief 获取当前样本的 RED 通道值 */
uint32_t max30102_get_red(const max30102_t *sensor);

/** @brief 获取当前样本的 IR 通道值 */
uint32_t max30102_get_ir(const max30102_t *sensor);

/** @brief 底层寄存器读取（暴露用于调试） */
bool max30102_read_reg(const max30102_t *sensor, uint8_t reg, uint8_t *value);

/** @brief 底层寄存器写入（暴露用于调试） */
bool max30102_write_reg(const max30102_t *sensor, uint8_t reg, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* MAX30102_PICO_H */
