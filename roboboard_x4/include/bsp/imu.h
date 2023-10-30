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

typedef struct {
    float temp; // Unit: C
    struct { // Unit: G, dps
        float x, y, z;
    } accel, gyro;
} bsp_imu_data_t;

/// @brief Initialize I2C peripheral
/// @param i2c_num [0:1] I2C port number
/// @param frequency [0:1000000] (Hz) frequency
/// @return ESP error
esp_err_t bsp_i2c_init(int i2c_num, uint32_t frequency);

esp_err_t bsp_imu_init();
// Set accelerometer maximum range of G force.
// Allowed values: 2, 4, 8, 16. Default: 16 (G)
esp_err_t bsp_imu_set_accel_range(uint16_t range);
// Set gyroscope maximum range of angle speed.
// Allowed values: 250, 500, 1000, 2000. Default 2000 (dps)
esp_err_t bsp_imu_set_gyro_range(uint16_t range);

esp_err_t bsp_imu_read(bsp_imu_data_t *data);

#endif /* INCLUDE_BSP_IMU */
