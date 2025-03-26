/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #include <stdint.h>
 #include <stdio.h>
 #include <math.h>
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "esp_log.h"
 #include "esp_err.h"
 #include "bmp280.h"
 
 static const char *TAG = "BMP280";
 
 bmp280_calib_data_t bmp280_cal;
 
 /**
  * @brief 在指定 I2C 总线上创建 BMP280 设备
  */
 i2c_master_dev_handle_t bmp280_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed)
 {
     i2c_device_config_t dev_cfg = {
         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
         .device_address = dev_addr,
         .scl_speed_hz = dev_speed,
     };
 
     i2c_master_dev_handle_t dev_handle;
     ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
     return dev_handle;
 }
 
 /**
  * @brief 初始化 BMP280 传感器，包括芯片 ID 校验、校准数据读取及配置传感器工作模式
  */
 esp_err_t bmp280_init_device(i2c_master_dev_handle_t dev_handle)
 {
     esp_err_t ret;
     uint8_t reg;
     uint8_t id;
 
     // 读取芯片 ID
     reg = BMP280_REG_ID;
     ret = i2c_master_transmit(dev_handle, &reg, 1, -1);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "写入 ID 寄存器失败");
         return ret;
     }
     ret = i2c_master_receive(dev_handle, &id, 1, -1);
     if (ret != ESP_OK || id != BMP280_CHIP_ID) {
         ESP_LOGE(TAG, "BMP280 芯片 ID 校验失败，读到 ID: 0x%02x", id);
         return ESP_FAIL;
     }
     ESP_LOGI(TAG, "BMP280 芯片 ID 验证通过: 0x%02x", id);
 
     // 读取校准数据，共 24 字节
     uint8_t calib_data[24];
     reg = BMP280_REG_CALIB;
     ret = i2c_master_transmit(dev_handle, &reg, 1, -1);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "写入校准数据寄存器失败");
         return ret;
     }
     ret = i2c_master_receive(dev_handle, calib_data, 24, -1);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "读取校准数据失败");
         return ret;
     }
 
     // 解析校准数据
     bmp280_cal.dig_T1 = (calib_data[1] << 8) | calib_data[0];
     bmp280_cal.dig_T2 = (calib_data[3] << 8) | calib_data[2];
     bmp280_cal.dig_T3 = (calib_data[5] << 8) | calib_data[4];
     bmp280_cal.dig_P1 = (calib_data[7] << 8) | calib_data[6];
     bmp280_cal.dig_P2 = (calib_data[9] << 8) | calib_data[8];
     bmp280_cal.dig_P3 = (calib_data[11] << 8) | calib_data[10];
     bmp280_cal.dig_P4 = (calib_data[13] << 8) | calib_data[12];
     bmp280_cal.dig_P5 = (calib_data[15] << 8) | calib_data[14];
     bmp280_cal.dig_P6 = (calib_data[17] << 8) | calib_data[16];
     bmp280_cal.dig_P7 = (calib_data[19] << 8) | calib_data[18];
     bmp280_cal.dig_P8 = (calib_data[21] << 8) | calib_data[20];
     bmp280_cal.dig_P9 = (calib_data[23] << 8) | calib_data[22];
 
     // 配置传感器：正常模式，温度和压力均采用 16 倍过采样
     uint8_t config[2] = { BMP280_REG_CONTROL, 0xB7 };  // 0xB7: 正常模式，温度 x16，压力 x16
     ret = i2c_master_transmit(dev_handle, config, 2, -1);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "配置 BMP280 传感器失败");
         return ret;
     }
     return ESP_OK;
 }
 
 /**
  * @brief 读取 BMP280 的温度和压力测量数据
  */
 esp_err_t bmp280_read_measurement(i2c_master_dev_handle_t dev_handle, float *temperature, float *pressure)
 {
     esp_err_t ret;
     uint8_t data[6];
     uint8_t reg = BMP280_REG_PRESSURE;
 
     // 请求从压力寄存器开始连续读取 6 字节数据
     ret = i2c_master_transmit(dev_handle, &reg, 1, -1);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "请求读取压力数据失败");
         return ret;
     }
     ret = i2c_master_receive(dev_handle, data, 6, -1);
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "读取温度和压力数据失败");
         return ret;
     }
 
     // 数据解析：\n"
     // 压力和温度数据均为 20 位，分别存储在 3 字节中
     int32_t adc_P = ((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4);
     int32_t adc_T = ((uint32_t)data[3] << 12) | ((uint32_t)data[4] << 4) | (data[5] >> 4);
 
     // 温度换算（单位：°C）
     int32_t var1, var2;
     var1 = ((((adc_T >> 3) - ((int32_t)bmp280_cal.dig_T1 << 1))) * ((int32_t)bmp280_cal.dig_T2)) >> 11;
     var2 = (((((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1))) >> 12) * ((int32_t)bmp280_cal.dig_T3)) >> 14;
     bmp280_cal.t_fine = var1 + var2;
     *temperature = (float)((bmp280_cal.t_fine * 5 + 128) >> 8) / 100.0f;
 
     // 压力换算（单位：hPa）
     var1 = (((int32_t)bmp280_cal.t_fine) >> 1) - 64000;
     var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)bmp280_cal.dig_P6);
     var2 = var2 + ((var1 * ((int32_t)bmp280_cal.dig_P5)) << 1);
     var2 = (var2 >> 2) + (((int32_t)bmp280_cal.dig_P4) << 16);
     var1 = (((bmp280_cal.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)bmp280_cal.dig_P2) * var1) >> 1)) >> 18;
     var1 = ((((32768 + var1)) * ((int32_t)bmp280_cal.dig_P1)) >> 15);
     if (var1 == 0) {
         ESP_LOGE(TAG, "除零错误");
         return ESP_FAIL;
     }
     uint32_t p;
     p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
     if (p < 0x80000000) {
         p = (p << 1) / ((uint32_t)var1);
     } else {
         p = (p / (uint32_t)var1) * 2;
     }
     var1 = (((int32_t)bmp280_cal.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
     var2 = (((int32_t)(p >> 2)) * ((int32_t)bmp280_cal.dig_P8)) >> 13;
     p = (uint32_t)((int32_t)p + ((var1 + var2 + bmp280_cal.dig_P7) >> 4));
     *pressure = (float)p / 100.0f;
 
     return ESP_OK;
 }
 
 /**
  * @brief 删除 BMP280 设备
  */
 esp_err_t bmp280_device_delete(i2c_master_dev_handle_t dev_handle)
 {
     return i2c_master_bus_rm_device(dev_handle);
 }
    