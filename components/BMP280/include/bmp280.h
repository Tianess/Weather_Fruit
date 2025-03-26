/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file bmp280.h
 * @brief BMP280 驱动头文件
 *
 * 本文件提供了 BMP280 传感器的设备创建、初始化、读取温度和压力测量数据、
 * 以及设备删除等接口，其风格与 SHT4x 驱动保持一致。
 */

 #pragma once

 #ifdef __cplusplus
 extern "C" {
 #endif
 
 #include "driver/i2c_master.h"
 #include "esp_err.h"
 
 // BMP280 I2C 地址（若 0x77 无效请使用 0x76）
 #define BMP280_I2C_ADDR         0x76
 
 // BMP280 寄存器定义
 #define BMP280_REG_ID           0xD0   // 芯片 ID 寄存器
 #define BMP280_CHIP_ID          0x58   // 预期的芯片 ID
 #define BMP280_REG_CALIB        0x88   // 校准数据起始地址
 #define BMP280_REG_CONTROL      0xF4   // 控制寄存器
 #define BMP280_REG_CONFIG       0xF5   // 配置寄存器
 #define BMP280_REG_PRESSURE     0xF7   // 压力数据起始地址
 
 /**
  * @brief BMP280 校准数据结构
  */
 typedef struct {
     uint16_t dig_T1;
     int16_t  dig_T2;
     int16_t  dig_T3;
     uint16_t dig_P1;
     int16_t  dig_P2;
     int16_t  dig_P3;
     int16_t  dig_P4;
     int16_t  dig_P5;
     int16_t  dig_P6;
     int16_t  dig_P7;
     int16_t  dig_P8;
     int16_t  dig_P9;
     int32_t  t_fine;
 } bmp280_calib_data_t;
 
 extern bmp280_calib_data_t bmp280_cal;
 
 /**
  * @brief 创建 BMP280 设备句柄
  *
  * 在指定的 I2C 总线上创建 BMP280 设备，并设置设备地址及通信速度。
  *
  * @param bus_handle I2C 总线句柄
  * @param dev_addr   BMP280 的 I2C 地址
  * @param dev_speed  I2C 通信速度（Hz）
  *
  * @return BMP280 设备句柄
  */
 i2c_master_dev_handle_t bmp280_device_create(i2c_master_bus_handle_t bus_handle, const uint16_t dev_addr, const uint32_t dev_speed);
 
 /**
  * @brief 初始化 BMP280 传感器
  *
  * 此函数完成芯片 ID 校验、校准数据读取，并配置传感器为正常模式，温度和压力均使用 16 倍过采样。
  *
  * @param dev_handle BMP280 设备句柄
  *
  * @return ESP_OK 表示成功；其他错误码表示失败
  */
 esp_err_t bmp280_init_device(i2c_master_dev_handle_t dev_handle);
 
 /**
  * @brief 读取 BMP280 测量数据
  *
  * 读取 BMP280 的温度（单位：°C）和压力（单位：hPa）数据。
  *
  * @param dev_handle  BMP280 设备句柄
  * @param temperature 存放温度值的指针
  * @param pressure    存放压力值的指针
  *
  * @return ESP_OK 表示成功；其他错误码表示失败
  */
 esp_err_t bmp280_read_measurement(i2c_master_dev_handle_t dev_handle, float *temperature, float *pressure);
 
 /**
  * @brief 删除 BMP280 设备
  *
  * 释放与 BMP280 设备相关的资源，并从 I2C 总线中移除设备。
  *
  * @param dev_handle BMP280 设备句柄
  *
  * @return ESP_OK 表示成功；其他错误码表示失败
  */
 esp_err_t bmp280_device_delete(i2c_master_dev_handle_t dev_handle);
 
 #ifdef __cplusplus
 }
 #endif
 