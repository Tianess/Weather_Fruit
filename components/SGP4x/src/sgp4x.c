/*
 * SPDX-FileCopyrightText: 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #include "sgp4x.h"
 #include <esp_log.h>
 #include <esp_err.h>
 
 static const char *TAG = "SGP4X";
 
 i2c_master_dev_handle_t sgp4x_device_create(i2c_master_bus_handle_t bus_handle, uint16_t dev_addr, uint32_t dev_speed)
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
 
 esp_err_t sgp4x_device_delete(i2c_master_dev_handle_t dev_handle)
 {
     return i2c_master_bus_rm_device(dev_handle);
 }
 
 esp_err_t sgp4x_start_measurement(i2c_master_dev_handle_t dev_handle, sgp4x_measurement_mode mode)
 {
     uint8_t cmd[2] = { (uint8_t)(mode >> 8), (uint8_t)mode };
     esp_err_t ret = i2c_master_transmit(dev_handle, cmd, sizeof(cmd), -1);
     
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Measurement command failed");
         return ret;
     }
     return ESP_OK;
 }
 
 esp_err_t sgp4x_read_measurement(i2c_master_dev_handle_t dev_handle, uint16_t *sraw_voc, uint16_t *sraw_nox)
 {
     uint8_t raw_data[6];
     esp_err_t ret = i2c_master_receive(dev_handle, raw_data, sizeof(raw_data), -1);
     
     if (ret != ESP_OK) {
         ESP_LOGE(TAG, "Read measurement failed");
         return ret;
     }
 
     // CRC校验和数据解析（示例）
     *sraw_voc = (raw_data[0] << 8) | raw_data[1];
     *sraw_nox = (raw_data[3] << 8) | raw_data[4];
     return ESP_OK;
 }