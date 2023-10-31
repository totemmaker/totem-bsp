/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef LIB_IMU_INTERFACE
#define LIB_IMU_INTERFACE

#include <stdint.h>
#include "esp_err.h"

#if __cplusplus
extern "C" {
#endif

typedef struct {
    float temp; // Unit: C
    struct { // Unit: G, dps
        float x, y, z;
    } accel, gyro;
} imu_data_t;

typedef struct {
    int num;
    int (* const write_reg)(uint8_t addr, uint8_t reg, uint8_t value);
    int (* const read_reg)(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len);
} imu_i2c_t;

#if __cplusplus
}
#endif

#endif /* LIB_IMU_INTERFACE */
