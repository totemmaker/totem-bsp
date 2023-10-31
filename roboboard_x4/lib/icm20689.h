/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef LIB_ICM20689
#define LIB_ICM20689

#include "esp_err.h"
#include "imu_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t icm20689_reset(imu_i2c_t *i2c);
esp_err_t icm20689_init(imu_i2c_t *i2c);

// range: 2, 4, 8, 16 (G)
esp_err_t icm20689_set_accel(imu_i2c_t *i2c, uint32_t *range);
// range: 250, 500, 1000, 2000 (dsp)
esp_err_t icm20689_set_gyro(imu_i2c_t *i2c, uint32_t *range);

esp_err_t icm20689_read(imu_i2c_t *i2c, imu_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* LIB_ICM20689 */
