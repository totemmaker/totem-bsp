/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef PRIV_INCLUDE_LSM6DS3
#define PRIV_INCLUDE_LSM6DS3

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { // Accelerometer output data rate (sample rate)
    LSM6DS3_ACCEL_RATE_PowerDown,
    LSM6DS3_ACCEL_RATE_12_5Hz,
    LSM6DS3_ACCEL_RATE_26Hz,
    LSM6DS3_ACCEL_RATE_52Hz,
    LSM6DS3_ACCEL_RATE_104Hz,
    LSM6DS3_ACCEL_RATE_208Hz,
    LSM6DS3_ACCEL_RATE_416Hz,
    LSM6DS3_ACCEL_RATE_833Hz,
    LSM6DS3_ACCEL_RATE_1666Hz,
    LSM6DS3_ACCEL_RATE_3332Hz,
    LSM6DS3_ACCEL_RATE_6664Hz,
} lsm6ds3_accel_rate_t;

typedef enum { // Accelerometer Anti-aliasing filter bandwidth 
    LSM6DS3_ACCEL_FILTER_400Hz,
    LSM6DS3_ACCEL_FILTER_200Hz,
    LSM6DS3_ACCEL_FILTER_100Hz,
    LSM6DS3_ACCEL_FILTER_50Hz,
} lsm6ds3_accel_filter_t;

typedef enum { // Gyroscope output data rate (sample rate)
    LSM6DS3_GYRO_RATE_PowerDown,
    LSM6DS3_GYRO_RATE_12_5Hz,
    LSM6DS3_GYRO_RATE_26Hz,
    LSM6DS3_GYRO_RATE_52Hz,
    LSM6DS3_GYRO_RATE_104Hz,
    LSM6DS3_GYRO_RATE_208Hz,
    LSM6DS3_GYRO_RATE_416Hz,
    LSM6DS3_GYRO_RATE_833Hz,
    LSM6DS3_GYRO_RATE_1666Hz,
} lsm6ds3_gyro_rate_t;

typedef struct {
    float temp; // Unit: C
    struct { // Unit: G, dps
        float x, y, z;
    } accel, gyro;
} lsm6ds3_data_t;

esp_err_t lsm6ds3_init(void);

// range: 2, 4, 8, 16 (G)
esp_err_t lsm6ds3_set_accel(uint16_t range, lsm6ds3_accel_rate_t rate, lsm6ds3_accel_filter_t filter);
// range: 245, 500, 1000, 2000 (dsp)
esp_err_t lsm6ds3_set_gyro(uint16_t range, lsm6ds3_gyro_rate_t rate);

esp_err_t lsm6ds3_read(lsm6ds3_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* PRIV_INCLUDE_LSM6DS3 */
