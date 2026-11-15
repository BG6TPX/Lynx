#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/* SHT40 温湿度传感器驱动 (I2C 地址 0x44)
 * 用法: 先在 main 里创建 I2C 总线(i2c_new_master_bus), 再把总线句柄传进来 */
esp_err_t sht40_init(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev);

/* 读取温湿度, 读到的是校验过 CRC 的最终物理量 */
esp_err_t sht40_read(i2c_master_dev_handle_t dev, float *temp_c, float *rh);
