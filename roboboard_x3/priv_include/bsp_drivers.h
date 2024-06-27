/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef PRIV_INCLUDE_BSP_DRIVERS
#define PRIV_INCLUDE_BSP_DRIVERS

#include "esp_err.h"

// Motor states
enum {
    MOTOR_DECAY_SLOW,
    MOTOR_DECAY_FAST,
};
// Motor parameters
enum {
    MOTOR_DC_MODE_POWER,
    MOTOR_DC_MODE_BRAKE,
    MOTOR_DC_MODE_TONE,
    MOTOR_DC_FREQUENCY,
    MOTOR_DC_DECAY,
    MOTOR_DC_ENABLE,
    MOTOR_SERVO_PULSE = 0x10,
    MOTOR_SERVO_PERIOD,
    MOTOR_SERVO_ENABLE,
};
// Motor control
esp_err_t bsp_motor_init();
esp_err_t bsp_motor_set(int8_t portID, uint32_t param, int32_t value);
int32_t bsp_motor_get(uint8_t portID, uint32_t param);
// Error handler
#define BSP_ERR(bsp_func) { esp_err_t err = bsp_func; if (err) return err; }

#endif /* PRIV_INCLUDE_BSP_DRIVERS */
