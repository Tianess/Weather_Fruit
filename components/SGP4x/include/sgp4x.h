/*
 * SPDX-FileCopyrightText: 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief SGP4X driver
 */

 #pragma once

 #ifdef __cplusplus
 extern "C" {
 #endif
 
#include "driver/i2c_master.h"
 
 #define SGP4X_I2C_ADDR                 0x59    // I2C address of SGP4X sensor
 #define SGP4X_CRC8_POLYNOM             0x31    // CRC8 polynomial
 
 /**
  * @enum sgp4x_measurement_mode
  * @brief Enumeration of SGP4x sensor commands
  */
 typedef enum {
     SGP4X_CMD_RESET                = 0x0006,  // Soft reset command
     SGP4X_CMD_SERIAL_NUMBER        = 0x3682,  // Serial number request
     SGP4X_CMD_EXEC_CONDITIONING    = 0x2612,  // Start conditioning
     SGP4X_CMD_MEAS_RAW_SIGNALS     = 0x2619,  // Measure raw signals
     SGP4X_CMD_EXEC_SELF_TEST       = 0x280E,  // Execute self-test
     SGP4X_CMD_TURN_HEATER_OFF      = 0x3615,  // Turn heater off
 } sgp4x_measurement_mode;
 

 
 /**
  * @brief Creates a handle for the SGP4X device
  *
  * @param bus_handle I2C bus handle
  * @param dev_addr Device I2C address
  * @param dev_speed I2C clock speed
  * @return sgp4x_dev_handle_t Device handle
  */
 i2c_master_dev_handle_t sgp4x_device_create(i2c_master_bus_handle_t bus_handle, uint16_t dev_addr, uint32_t dev_speed);
 
 /**
  * @brief Deletes the SGP4X device
  *
  * @param dev_handle Device handle
  * @return esp_err_t ESP_OK on success
  */
 esp_err_t sgp4x_device_delete(i2c_master_dev_handle_t dev_handle);
 
 /**
  * @brief Execute sensor measurement
  *
  * @param dev_handle Device handle
  * @param mode Measurement mode
  * @return esp_err_t ESP_OK on success
  */
 esp_err_t sgp4x_start_measurement(i2c_master_dev_handle_t dev_handle, sgp4x_measurement_mode mode);
 
 /**
  * @brief Read measurement results
  *
  * @param dev_handle Device handle
  * @param sraw_voc VOC raw signal output
  * @param sraw_nox NOX raw signal output
  * @return esp_err_t ESP_OK on success
  */
 esp_err_t sgp4x_read_measurement(i2c_master_dev_handle_t dev_handle, uint16_t *sraw_voc, uint16_t *sraw_nox);
 
 #ifdef __cplusplus
 }
 #endif