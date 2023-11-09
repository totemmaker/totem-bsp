/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef LIB_QMI8658
#define LIB_QMI8658

#include "esp_err.h"
#include "imu_interface.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t qmi8658_init(imu_i2c_t *i2c);
// range: 2, 4, 8, 16 (G)
// rate: 31, 62, 125, 250, 500, 1000, 2000, 4000, 8000 (Hz)
esp_err_t qmi8658_set_accel(imu_i2c_t *i2c, uint32_t *range, uint32_t *rate);
// range: 16, 32, 64, 128, 256, 512, 1024, 2048 (dps)
// rate: 31, 62, 125, 250, 500, 1000, 2000, 4000, 8000 (Hz)
esp_err_t qmi8658_set_gyro(imu_i2c_t *i2c, uint32_t *range, uint32_t *rate);

esp_err_t qmi8658_read(imu_i2c_t *i2c, imu_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* LIB_QMI8658 */
