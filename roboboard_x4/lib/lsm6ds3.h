/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef LIB_LSM6DS3
#define LIB_LSM6DS3

#include "esp_err.h"
#include "imu_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t lsm6ds3_init(imu_i2c_t *i2c);

// range: 2, 4, 8, 16 (G)
// rate: 0, 12, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664 (Hz)
// filter: 50, 100, 200, 400 (Hz)
esp_err_t lsm6ds3_set_accel(imu_i2c_t *i2c, uint32_t *range, uint32_t *rate, uint32_t* filter);
// range: 125, 250, 500, 1000, 2000 (dps)
// rate: 0, 12, 26, 52, 104, 208, 416, 833, 1666 (Hz)
esp_err_t lsm6ds3_set_gyro(imu_i2c_t *i2c, uint32_t *range, uint32_t *rate);

esp_err_t lsm6ds3_read(imu_i2c_t *i2c, imu_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* LIB_LSM6DS3 */
