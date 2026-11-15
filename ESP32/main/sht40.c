/* SHT40 温湿度传感器驱动
 * 数据手册要点:
 *   - I2C 地址 0x44 (B 版为 0x45)
 *   - 0xFD = 高精度测量, 最长耗时 8.3ms, 期间读数据会 NACK
 *   - 返回 6 字节: T_MSB T_LSB CRC RH_MSB RH_LSB CRC
 *   - CRC-8: 多项式 0x31, 初值 0xFF
 */
#include "sht40.h"

#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "sht40";

#define SHT40_ADDR           0x44
#define SHT40_CMD_MEAS_HIGH  0xFD
#define SHT40_CMD_SOFT_RESET 0x94

static uint8_t sht40_crc8(const uint8_t *d, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

esp_err_t sht40_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev)
{
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = SHT40_ADDR,
        .scl_speed_hz    = 100 * 1000,      /* SHT40 用 100k, 稳 */
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, dev), TAG, "add dev");

    const uint8_t rst = SHT40_CMD_SOFT_RESET;
    esp_err_t err = i2c_master_transmit(*dev, &rst, 1, 100);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(2));       /* 软复位耗时约 1ms */
    }
    return err;
}

esp_err_t sht40_read(i2c_master_dev_handle_t dev, float *temp_c, float *rh)
{
    const uint8_t cmd = SHT40_CMD_MEAS_HIGH;
    uint8_t raw[6];

    ESP_RETURN_ON_ERROR(i2c_master_transmit(dev, &cmd, 1, 100), TAG, "send cmd");
    /* 测量最长 8.3ms, 必须等它做完再读, 否则传感器 NACK */
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_RETURN_ON_ERROR(i2c_master_receive(dev, raw, sizeof(raw), 100), TAG, "read data");

    if (sht40_crc8(&raw[0], 2) != raw[2] || sht40_crc8(&raw[3], 2) != raw[5]) {
        ESP_LOGE(TAG, "CRC check failed");
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t rt = ((uint16_t)raw[0] << 8) | raw[1];
    uint16_t rr = ((uint16_t)raw[3] << 8) | raw[4];

    /* 数据手册换算公式 */
    *temp_c = -45.0f + 175.0f * (float)rt / 65535.0f;
    *rh     =  -6.0f + 125.0f * (float)rr / 65535.0f;
    if (*rh > 100.0f) *rh = 100.0f;
    if (*rh < 0.0f)   *rh = 0.0f;
    return ESP_OK;
}
