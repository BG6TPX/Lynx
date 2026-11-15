/* SSD1306 0.96 寸 128x64 I2C OLED 最小驱动
 * I2C 数据格式: 每次传输首字节为控制字节
 *   0x00 = 后面全是命令, 0x40 = 后面全是显存数据 */
#include "ssd1306.h"

#include <string.h>
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "font8x8_basic.h"

static const char *TAG = "ssd1306";

#define OLED_ADDR 0x3C          /* 少数模块跳线为 0x3D, 扫描不到时改这里 */

/* 发一串命令(最长 31 字节) */
static esp_err_t ssd1306_cmd(i2c_master_dev_handle_t dev, const uint8_t *cmds, size_t len)
{
    uint8_t buf[32];
    if (len == 0 || len > sizeof(buf) - 1) {
        return ESP_ERR_INVALID_SIZE;
    }
    buf[0] = 0x00;
    memcpy(&buf[1], cmds, len);
    return i2c_master_transmit(dev, buf, len + 1, 100);
}

esp_err_t ssd1306_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev)
{
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = OLED_ADDR,
        .scl_speed_hz    = 400 * 1000,      /* OLED 用 400k, 刷屏快 */
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, dev), TAG, "add dev");

    static const uint8_t init_cmds[] = {
        0xAE,               /* display off */
        0xD5, 0x80,         /* 时钟分频 */
        0xA8, 0x3F,         /* 复用率: 64 行 */
        0xD3, 0x00,         /* 显示偏移 0 */
        0x40,               /* 显示起始行 0 */
        0x8D, 0x14,         /* 内部升压电荷泵开启 */
        0x20, 0x00,         /* 水平寻址模式 */
        0xA1,               /* 段重映射 (左右) */
        0xC8,               /* COM 扫描方向 (上下) */
        0xDA, 0x12,         /* COM 引脚配置, 128x64 用 0x12 */
        0x81, 0xCF,         /* 对比度 */
        0xD9, 0xF1,         /* 预充电周期 */
        0xDB, 0x40,         /* VCOMH 电压 */
        0xA4,               /* 显示 RAM 内容(不是全亮) */
        0xA6,               /* 正常显示(非反色) */
        0xAF,               /* display on */
    };
    ESP_RETURN_ON_ERROR(ssd1306_cmd(*dev, init_cmds, sizeof(init_cmds)), TAG, "init cmds");
    vTaskDelay(pdMS_TO_TICKS(50));

    /* 设定写入窗口为整屏(列 0-127, 页 0-7), 之后 flush 直接写数据 */
    const uint8_t range[] = { 0x21, 0x00, 0x7F, 0x22, 0x00, 0x07 };
    ESP_RETURN_ON_ERROR(ssd1306_cmd(*dev, range, sizeof(range)), TAG, "set range");

    /* 清屏一次, 避免 RAM 残留数据上电花屏 */
    static uint8_t blank[1 + SSD1306_FB_SIZE] = { 0x40 };
    return i2c_master_transmit(*dev, blank, sizeof(blank), 200);
}

esp_err_t ssd1306_flush(i2c_master_dev_handle_t dev, const uint8_t *fb)
{
    static uint8_t pkt[1 + SSD1306_FB_SIZE];
    const uint8_t range[] = { 0x21, 0x00, 0x7F, 0x22, 0x00, 0x07 };
    ESP_RETURN_ON_ERROR(ssd1306_cmd(dev, range, sizeof(range)), TAG, "set range");

    pkt[0] = 0x40;
    memcpy(&pkt[1], fb, SSD1306_FB_SIZE);
    return i2c_master_transmit(dev, pkt, sizeof(pkt), 200);
}

void ssd1306_clear(uint8_t *fb)
{
    memset(fb, 0, SSD1306_FB_SIZE);
}

/* font8x8 每字节是"一行的 8 个像素"(bit0 在左), SSD1306 每字节是"一列的 8 个像素"(bit0 在上),
 * 这里做一次转置 */
void ssd1306_draw_char(uint8_t *fb, int x, int page, char c)
{
    if (x < 0 || x > SSD1306_W - 8 || page < 0 || page > SSD1306_H / 8 - 1) {
        return;
    }
    const uint8_t *g = (const uint8_t *)font8x8_basic[(int)c & 0x7F];
    for (int col = 0; col < 8; col++) {
        uint8_t v = 0;
        for (int row = 0; row < 8; row++) {
            v |= (uint8_t)(((g[row] >> col) & 0x01) << row);
        }
        fb[page * SSD1306_W + x + col] = v;
    }
}

void ssd1306_draw_text(uint8_t *fb, int x, int page, const char *s)
{
    while (*s && x <= SSD1306_W - 8) {
        ssd1306_draw_char(fb, x, page, *s++);
        x += 8;
    }
}
