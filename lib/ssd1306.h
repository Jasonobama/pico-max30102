#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// 引脚配置（用户可覆盖）
// 默认使用 I2C0：GP8=SDA, GP9=SCL
// ============================================================

#ifndef SSD1306_I2C_PORT
#define SSD1306_I2C_PORT    i2c0
#endif

#ifndef SSD1306_SDA_PIN
#define SSD1306_SDA_PIN     8
#endif

#ifndef SSD1306_SCL_PIN
#define SSD1306_SCL_PIN     9
#endif

#ifndef SSD1306_I2C_ADDR
#define SSD1306_I2C_ADDR    0x3C
#endif

#ifndef SSD1306_I2C_BAUD
#define SSD1306_I2C_BAUD    400000
#endif

#ifndef SSD1306_WIDTH
#define SSD1306_WIDTH       128
#endif

#ifndef SSD1306_HEIGHT
#define SSD1306_HEIGHT      64
#endif

#define SSD1306_PAGE_HEIGHT 8
#define SSD1306_PAGES       (SSD1306_HEIGHT / SSD1306_PAGE_HEIGHT)
#define SSD1306_BUF_LEN     (SSD1306_PAGES * SSD1306_WIDTH)

// ============================================================
// 帧缓冲模式（默认关闭）
// 在 #include "ssd1306.h" 之前 #define SSD1306_USE_FRAMEBUFFER 启用
// ============================================================

#ifdef SSD1306_USE_FRAMEBUFFER
extern uint8_t oled_buf[SSD1306_BUF_LEN];
void ssd1306_render(void);
#endif

// ============================================================
// 公开 API（两种模式通用）
// ============================================================

void ssd1306_init(void);
void ssd1306_clear(void);
void ssd1306_draw_string(uint8_t page, uint8_t col, const char *str);
void ssd1306_draw_string_centered(uint8_t page, const char *str);
void ssd1306_clear_page(uint8_t page);
void ssd1306_clear_page_range(uint8_t start, uint8_t end);

#ifdef __cplusplus
}
#endif

#endif /* __SSD1306_H__ */
