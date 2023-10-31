/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_BSP_IMU
#define INCLUDE_BSP_IMU

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float temp; // Unit: C
    struct { // Unit: G, dps
        float x, y, z;
    } accel, gyro;
} bsp_imu_data_t;

typedef struct {
    // range: 2, 4, 8, 16 (G)
    uint32_t acc_range; // default: 16
    // range: 250, 500, 1000, 2000 (dsp)
    uint32_t gyro_range; // default: 2000
// RoboBoard X4 v1.0 only (LSM6DS3)
    // refresh rate: 0, 12, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664 (Hz)
    uint32_t acc_rate; // default: 104
    // refresh rate: 0, 12, 26, 52, 104, 208, 416, 833, 1666 (Hz)
    uint32_t gyro_rate; // default: 104
    // filter: 50, 100, 200, 400 (Hz)
    uint32_t acc_filter; // default: 100
} bsp_imu_param_t;

/// @brief Helper function to initialize I2C peripheral. For custom parameters use your own initialization code
/// @param i2c_num [0:1] I2C port number
/// @param frequency [0:1000000] (Hz) frequency
/// @return ESP error
esp_err_t bsp_i2c_init(int i2c_num, uint32_t frequency);

/// @brief Initialize IMU sensor
/// @param i2c_num [0:1] I2C port number
/// @return ESP error
esp_err_t bsp_imu_init(int i2c_num);

/// @brief Get pointer to IMU parameter structure
/// @return pointer to bsp_imu_param_t
bsp_imu_param_t* bsp_imu_get_param();

/// @brief Apply parameters to IMU sensor
/// @param param pointer to bsp_imu_param_t
/// @return ESP error
esp_err_t bsp_imu_set_param(bsp_imu_param_t *param);

/// @brief Read measurements from IMU sensor
/// @param data pointer to bsp_imu_data_t
/// @return ESP error
esp_err_t bsp_imu_read(bsp_imu_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_BSP_IMU */
