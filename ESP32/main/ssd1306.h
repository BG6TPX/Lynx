#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#define SSD1306_W        128
#define SSD1306_H        64
#define SSD1306_FB_SIZE  (SSD1306_W * SSD1306_H / 8)   /* 1024 字节帧缓冲 */

/* SSD1306 0.96 寸 128x64 OLED (I2C 地址 0x3C) 最小驱动。
 * 帧缓冲布局与屏幕原生页结构一致: fb[页号*128 + 列], 每字节 8 个纵向像素, bit0 在最上 */
esp_err_t ssd1306_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev);

/* 把整幅帧缓冲刷到屏幕 */
esp_err_t ssd1306_flush(i2c_master_dev_handle_t dev, const uint8_t *fb);

void ssd1306_clear(uint8_t *fb);

/* 在 (x 列, page 页) 画一个 8x8 字符 / 一串 ASCII 文本。page 每页高 8 像素, 共 8 页 */
void ssd1306_draw_char(uint8_t *fb, int x, int page, char c);
void ssd1306_draw_text(uint8_t *fb, int x, int page, const char *s);
