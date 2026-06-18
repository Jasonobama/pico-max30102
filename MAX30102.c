/* ------------------------------------------------------------------
 * SpO2 / 心率监测仪，适用于 Raspberry Pi Pico / Pico 2
 *
 * 双核版本：
 *   核心0 — 传感器采集 (I2C0) + OLED 显示 (I2C1)
 *   核心1 — SpO2 / 心率计算
 *
 * 如需回退到单核模式调试，取消下行注释：
 *   #define SINGLE_CORE
 *
 * 调试输出开关（发布版请注释掉）：
 *   #define DEBUG
 *
 * 硬件连接：
 *   MAX30102 传感器 -> I2C0  (GP4 = SDA, GP5 = SCL)
 *   SSD1306 OLED     -> I2C1 (GP6 = SDA, GP7 = SCL)
 * ------------------------------------------------------------------ */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

#include "max30102.h"
#include "ssd1306.h"
#include "algorithm.h"

#ifndef SINGLE_CORE
#include "pico/multicore.h"
#endif

#define MAX30102_I2C_PORT i2c0
#define MAX30102_SDA_PIN  4
#define MAX30102_SCL_PIN  5

/* ---------- 可调参数 ---------- */

/* OLED 刷新间隔（毫秒），默认 1000ms = 1 秒 */
#define OLED_REFRESH_MS     1000

/* 有手指时的轮询间隔（毫秒），50Hz */
#define POLL_ACTIVE_MS      20

/* 无手指时的轮询间隔（毫秒），10Hz 省电 */
#define POLL_IDLE_MS        100


/* 看门狗超时（毫秒），主循环停滞超此时间自动复位 */
#define WATCHDOG_TIMEOUT_MS 3000

/* ---------- 传感器默认配置 ---------- */

static const max30102_config_t g_sensor_cfg = {
    .sample_avg       = SAMPLEAVG_4,
    .mode             = MODE_SPO2,
    .spo2_adc_range   = SPO2_ADC_RGE_4096,
    .spo2_sample_rate = SPO2_SR_100,
    .led_pulse_width  = LED_PW_411,
    .led1_current     = 0x17,
    .led2_current     = 0x17,
    .pilot_current    = 0x1F,
    .fifo_almost_full = 0,
    .fifo_rollover    = false,
};

/* ---------- 调试宏 ---------- */

#ifdef DEBUG
#define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
#define DBG_PRINTF(...) ((void)0)
#endif

/* ================================================================
 *  核间共享数据结构
 * ================================================================ */

#ifndef SINGLE_CORE

#define SHARED_FIFO_SIZE  64
#define SHARED_FIFO_MASK  (SHARED_FIFO_SIZE - 1u)

/** @brief 单生产者单消费者 FIFO，由硬件 spinlock 保护。
 *
 *  核心0（采集器）：在 `head` 处写入，递增 `head`。
 *  核心1（计算器）：从 `tail` 处读取，递增 `tail`。
 *
 *  FIFO 满条件：(head + 1) %% SIZE == tail。
 *  FIFO 空条件： head == tail。                          */
typedef struct {
    uint32_t         red[SHARED_FIFO_SIZE];
    uint32_t         ir[SHARED_FIFO_SIZE];
    volatile uint8_t head;                /* 核心0 写入索引 */
    volatile uint8_t tail;                /* 核心1 读取索引 */
    spin_lock_t     *lock;
} shared_fifo_t;

/** @brief 计算结果缓冲区。
 *
 *  核心1 写入 `result` 并置 `updated = true`。
 *  核心0 轮询 `updated`，复制 `result`，然后清除标志。 */
typedef struct {
    spo2_result_t  result;
    volatile bool  updated;
    spin_lock_t   *lock;
} shared_result_t;

/* 文件作用域全局变量，双核均可访问 */
static shared_fifo_t   g_fifo;
static shared_result_t g_result;

#endif /* !SINGLE_CORE */

/* ----------------------------------------------------------------
 *  显示辅助函数
 * ---------------------------------------------------------------- */
static void draw_dashboard(const spo2_result_t *r) {
    char line[22];

    ssd1306_clear_page_range(0, 7);
    ssd1306_draw_string_centered(0, "SpO2 Monitor");

    if (!r->finger_present) {
        ssd1306_draw_string_centered(3, "Place finger");
        ssd1306_draw_string_centered(4, "on sensor");
        return;
    }

    if (r->spo2_valid) {
        snprintf(line, sizeof(line), "SpO2 : %3d %%", (int)(r->spo2 + 0.5f));
    } else {
        snprintf(line, sizeof(line), "SpO2 : --  %%");
    }
    ssd1306_draw_string(2, 0, line);

    if (r->hr_valid) {
        snprintf(line, sizeof(line), "Pulse: %3d bpm", (int)(r->heart_rate + 0.5f));
    } else {
        snprintf(line, sizeof(line), "Pulse: --  bpm");
    }
    ssd1306_draw_string(4, 0, line);

    snprintf(line, sizeof(line), "IR DC: %lu", (unsigned long)r->dc_ir);
    ssd1306_draw_string(6, 0, line);
}

/* ----------------------------------------------------------------
 *  初始化进度显示
 * ---------------------------------------------------------------- */
static void show_init_progress(int step) {
    ssd1306_clear_page_range(3, 5);
    switch (step) {
    case 1:
        ssd1306_draw_string_centered(3, "Initializing...");
        ssd1306_draw_string_centered(4, "Sensor detect");
        break;
    case 2:
        ssd1306_draw_string_centered(3, "Sensor found!");
        ssd1306_draw_string_centered(4, "Acquiring data...");
        break;
    default:
        break;
    }
}


#ifndef SINGLE_CORE
/* ================================================================
 *  核心1 入口 — SpO2 / 心率计算
 *
 *  永久循环：取出样本、喂入算法、发布结果。
 * ================================================================ */
static void core1_main(void) {
    spo2_algorithm_init();

    while (true) {
        bool     got_sample = false;
        uint32_t red = 0, ir = 0;

        /* 从共享 FIFO 取出一个样本（spinlock 保护） */
        {
            uint32_t state = spin_lock_blocking(g_fifo.lock);
            if (g_fifo.head != g_fifo.tail) {
                uint8_t idx = g_fifo.tail;
                red    = g_fifo.red[idx];
                ir     = g_fifo.ir[idx];
                g_fifo.tail = (idx + 1u) & SHARED_FIFO_MASK;
                got_sample  = true;
            }
            spin_unlock(g_fifo.lock, state);
        }

        if (got_sample) {
            /* 保持原始调用顺序以兼容校准参数
             * （RED 在前，IR 在后）。参见 README-zh.md。 */
            spo2_algorithm_add_sample(red, ir);
        }

        /* 当 100 样本窗口填满后，计算并发布结果 */
        if (spo2_algorithm_ready()) {
            spo2_result_t res = spo2_algorithm_compute();

            uint32_t state = spin_lock_blocking(g_result.lock);
            g_result.result  = res;
            g_result.updated = true;
            spin_unlock(g_result.lock, state);
        }

        /* 空闲时短暂休眠（1ms），提高响应速度 */
        if (!got_sample) {
            sleep_ms(1);
        }
    }
}
#endif /* !SINGLE_CORE */


/* ================================================================
 *  核心0 — main()：传感器采集 + OLED 显示
 * ================================================================ */
int main(void) {
    stdio_init_all();
    sleep_ms(2000);

    DBG_PRINTF("SpO2 / heart-rate monitor (MAX30102 + SSD1306)\n");
#ifdef SINGLE_CORE
    DBG_PRINTF("  Mode: single-core\n");
#else
    DBG_PRINTF("  Mode: dual-core\n");
#endif

    /* ---- I2C0 -> MAX30102 传感器 (400 kHz) ---- */
    i2c_init(MAX30102_I2C_PORT, 400 * 1000);
    gpio_set_function(MAX30102_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MAX30102_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MAX30102_SDA_PIN);
    gpio_pull_up(MAX30102_SCL_PIN);

    /* ---- I2C1 -> SSD1306 OLED ---- */
    ssd1306_init();
    show_init_progress(1);

    /* ---- MAX30102 传感器初始化 ---- */
    max30102_t sensor;
    if (!max30102_init(&sensor, MAX30102_I2C_PORT, MAX30102_I2C_ADDR)) {
        DBG_PRINTF("MAX30102 not found at 0x%02X\n", MAX30102_I2C_ADDR);
        ssd1306_clear_page_range(0, 7);
        ssd1306_draw_string_centered(3, "Sensor not found");
        ssd1306_draw_string_centered(4, "Check wiring");
        while (true) tight_loop_contents();
    }
    DBG_PRINTF("MAX30102 found, starting acquisition.\n");

    max30102_setup(&sensor, &g_sensor_cfg);
    show_init_progress(2);

    /* ---- 启用硬件看门狗（主循环停滞超时自动复位）---- */
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

#ifndef SINGLE_CORE
    /* ============================================================
     *  双核初始化
     * ============================================================ */

    /* 申请两个未使用的硬件 spinlock */
    spin_lock_t *lock_fifo   = spin_lock_instance(
                                   spin_lock_claim_unused(true));
    spin_lock_t *lock_result = spin_lock_instance(
                                   spin_lock_claim_unused(true));

    /* 初始化共享数据结构 */
    memset(&g_fifo,   0, sizeof(g_fifo));
    memset(&g_result, 0, sizeof(g_result));
    g_fifo.lock   = lock_fifo;
    g_result.lock = lock_result;

    /* 启动核心1 */
    multicore_launch_core1(core1_main);

    /* ---- 核心0 主循环：采集 + 显示 ---- */
    bool         have_result  = false;
    bool         finger_was   = false;
    spo2_result_t last_result;
    memset(&last_result, 0, sizeof(last_result));

    absolute_time_t next_display_update = make_timeout_time_ms(OLED_REFRESH_MS);

    while (true) {
        watchdog_update();

        /* 完全清空硬件 FIFO */
        while (max30102_check(&sensor) > 0) {
            /* 持续轮询直到读指针 == 写指针 */
        }

        /* 将所有新样本送入共享 FIFO */
        while (max30102_available(&sensor) > 0) {
            uint32_t red = max30102_get_red(&sensor);
            uint32_t ir  = max30102_get_ir(&sensor);

            {
                uint32_t state = spin_lock_blocking(g_fifo.lock);
                uint8_t  next  = (g_fifo.head + 1u) & SHARED_FIFO_MASK;
                if (next != g_fifo.tail) {          /* 未满 */
                    g_fifo.red[g_fifo.head] = red;
                    g_fifo.ir[g_fifo.head]  = ir;
                    g_fifo.head = next;
                }
                /* 否则：丢弃 — 在 25 Hz 下 64 槽缓冲区（2.56 秒余量）
                 *        理论上永远不会发生。  */
                spin_unlock(g_fifo.lock, state);
            }

            max30102_next_sample(&sensor);
        }

        /* 检查核心1 是否有新计算结果 */
        {
            uint32_t state = spin_lock_blocking(g_result.lock);
            if (g_result.updated) {
                last_result      = g_result.result;
                g_result.updated = false;
                have_result      = true;
            }
            spin_unlock(g_result.lock, state);
        }

        /* OLED 显示刷新 */
        if (absolute_time_diff_us(get_absolute_time(),
                                  next_display_update) <= 0) {
            if (have_result) {
                bool finger_now = last_result.finger_present;
                draw_dashboard(&last_result);

                /* 手指状态变化时调整轮询速率 */
                if (finger_now != finger_was) {
                    finger_was = finger_now;
                    DBG_PRINTF("Finger %s, switching poll rate\n",
                               finger_now ? "present" : "absent");
                }
            }
            next_display_update = make_timeout_time_ms(OLED_REFRESH_MS);
        }

        /* 自适应轮询：有手指 50Hz，无手指 10Hz 省电 */
        if (have_result && !last_result.finger_present) {
            sleep_ms(POLL_IDLE_MS);
        } else {
            sleep_ms(POLL_ACTIVE_MS);
        }
    }

#else  /* SINGLE_CORE — 原始单核行为 */
    spo2_algorithm_init();

    bool have_result = false;
    bool finger_was  = false;
    spo2_result_t result;
    memset(&result, 0, sizeof(result));

    absolute_time_t next_display_update = make_timeout_time_ms(OLED_REFRESH_MS);

    while (true) {
        watchdog_update();

        while (max30102_check(&sensor) > 0) {
            /* 持续轮询 */
        }

        while (max30102_available(&sensor) > 0) {
            uint32_t red = max30102_get_red(&sensor);
            uint32_t ir  = max30102_get_ir(&sensor);
            spo2_algorithm_add_sample(red, ir);
            max30102_next_sample(&sensor);
        }

        if (spo2_algorithm_ready()) {
            result = spo2_algorithm_compute();
            have_result = true;
        }

        if (absolute_time_diff_us(get_absolute_time(),
                                  next_display_update) <= 0) {
            if (have_result) {
                bool finger_now = result.finger_present;
                draw_dashboard(&result);
                if (finger_now != finger_was) {
                    finger_was = finger_now;
                }
            }
            next_display_update = make_timeout_time_ms(OLED_REFRESH_MS);
        }

        /* 自适应轮询 */
        if (have_result && !result.finger_present) {
            sleep_ms(POLL_IDLE_MS);
        } else {
            sleep_ms(POLL_ACTIVE_MS);
        }
    }
#endif /* !SINGLE_CORE */

    return 0;
}