/* ------------------------------------------------------------------
 * SpO2 / heart-rate monitor for the Raspberry Pi Pico / Pico 2
 *
 * Hardware:
 *   MAX30102 sensor -> I2C0  (GP4 = SDA, GP5 = SCL)
 *   SSD1306 OLED     -> I2C1 (GP6 = SDA, GP7 = SCL)
 * ------------------------------------------------------------------ */

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

/* SSD1306 OLED 配置（覆盖 ssd1306.h 默认值） */
#define SSD1306_I2C_PORT    i2c1
#define SSD1306_SDA_PIN     6
#define SSD1306_SCL_PIN     7
#define SSD1306_I2C_BAUD    100000

#include "max30102.h"
#include "ssd1306.h"
#include "algorithm.h"

#define MAX30102_I2C_PORT i2c0
#define MAX30102_SDA_PIN  4
#define MAX30102_SCL_PIN  5

/* Display the dashboard: SpO2, heart rate and a raw IR DC reading
 * (useful while tuning SPO2_FINGER_THRESHOLD). */
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

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    printf("SpO2 / heart-rate monitor (MAX30102 + SSD1306)\n");

    /* I2C0 -> MAX30102 sensor (fast mode, 400 kHz - within spec) */
    i2c_init(MAX30102_I2C_PORT, 400 * 1000);
    gpio_set_function(MAX30102_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(MAX30102_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(MAX30102_SDA_PIN);
    gpio_pull_up(MAX30102_SCL_PIN);

    /* I2C1 -> SSD1306 OLED (bus is initialised inside ssd1306_init) */
    ssd1306_init();
    ssd1306_draw_string_centered(3, "Initializing...");

    max30102_t sensor;
    if (!max30102_init(&sensor, MAX30102_I2C_PORT, MAX30102_I2C_ADDR)) {
        printf("MAX30102 not found at 0x%02X\n", MAX30102_I2C_ADDR);
        ssd1306_clear_page_range(0, 7);
        ssd1306_draw_string_centered(3, "Sensor not found");
        ssd1306_draw_string_centered(4, "Check wiring");
        while (true) tight_loop_contents();
    }
    printf("MAX30102 found, starting acquisition.\n");

    max30102_setup(&sensor, NULL); /* default: SPO2 mode, 100 Hz, avg=4 */
    spo2_algorithm_init();

    bool have_result = false;
    spo2_result_t result;
    memset(&result, 0, sizeof(result));

    absolute_time_t next_display_update = make_timeout_time_ms(1000);

    while (true) {
        /* Drain the device FIFO completely. max30102_check() reports
         * the device's read/write pointer gap; repeating until it
         * returns 0 guarantees rd_ptr == wr_ptr (FIFO empty), even
         * though a single call is capped to a 32-byte I2C burst. */
        while (max30102_check(&sensor) > 0) {
            /* keep polling */
        }

        while (max30102_available(&sensor) > 0) {
            uint32_t red = max30102_get_red(&sensor);
            uint32_t ir  = max30102_get_ir(&sensor);
            spo2_algorithm_add_sample(ir, red);
            max30102_next_sample(&sensor);
        }

        if (spo2_algorithm_ready()) {
            result = spo2_algorithm_compute();
            have_result = true;
        }

        if (absolute_time_diff_us(get_absolute_time(), next_display_update) <= 0) {
            if (have_result) {
                draw_dashboard(&result);
            }
            next_display_update = make_timeout_time_ms(1000);
        }

        sleep_ms(20); /* ~50 Hz poll rate; sensor FIFO fills at 25 Hz */
    }

    return 0;
}
