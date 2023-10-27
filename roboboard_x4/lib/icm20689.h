/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef PRIV_INCLUDE_ICM20689
#define PRIV_INCLUDE_ICM20689

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

enum { // Gyroscope output data rate
    ICM20689_GYRO_RATE_8000Hz,
    ICM20689_GYRO_RATE_4000Hz,
    ICM20689_GYRO_RATE_2000Hz,
    ICM20689_GYRO_RATE_1000Hz,
    ICM20689_GYRO_RATE_500Hz,
    ICM20689_GYRO_RATE_250Hz,
    ICM20689_GYRO_RATE_125Hz,
    ICM20689_GYRO_RATE_62_5Hz,
    ICM20689_GYRO_RATE_31_25Hz,
};

typedef struct {
    float temp; // Unit: C
    struct { // Unit: G, dps
        float x, y, z;
    } accel, gyro;
} icm20689_data_t;

esp_err_t icm20689_reset(void);
esp_err_t icm20689_init(void);

// range: 2, 4, 8, 16 (G)
esp_err_t icm20689_set_accel(uint16_t range);
// range: 250, 500, 1000, 2000 (dsp)
esp_err_t icm20689_set_gyro(uint16_t range);

esp_err_t icm20689_read(icm20689_data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* PRIV_INCLUDE_ICM20689 */
