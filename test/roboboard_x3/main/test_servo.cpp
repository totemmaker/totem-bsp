/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "totem_test.h"
#include "bsp/totem-bsp.h"

#include "driver/mcpwm.h"

TEST_CASE("Test Servo", "[Servo]") {
    // Validate parameters
    TEST_EQUAL(50, mcpwm_get_frequency(MCPWM_UNIT_1, MCPWM_TIMER_1));
    TEST_EQUAL(0, mcpwm_get_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A));
    TEST_EQUAL(0, mcpwm_get_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B));
    // Validate period update
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A, 30000));
    TEST_EQUAL(30000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(30000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_B));
    if (bsp_cmd_read(BSP_BOARD_REVISION, 0) > 30) {
        TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_C));
        TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_D));
    }
    // Change period and validate
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_ALL, 40000));
    TEST_EQUAL(25, mcpwm_get_frequency(MCPWM_UNIT_1, MCPWM_TIMER_1));
    if (bsp_cmd_read(BSP_BOARD_REVISION, 0) > 30) {
        TEST_EQUAL(25, mcpwm_get_frequency(MCPWM_UNIT_1, MCPWM_TIMER_2));
    }
    // Change duty cycle and validate
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_ALL, 39000));
    TEST_EQUAL(39000, mcpwm_get_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A));
    TEST_EQUAL(39000, mcpwm_get_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B));
    // Restore frequency and validate
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_ALL, 20000));
    TEST_EQUAL(50, mcpwm_get_frequency(MCPWM_UNIT_1, MCPWM_TIMER_1));
    // Restore duty cycle and validate
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_ALL, 1500));
    TEST_EQUAL(1500, mcpwm_get_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A));
    TEST_EQUAL(1500, mcpwm_get_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B));
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_ALL, 0));
    TEST_EQUAL(0, mcpwm_get_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A));
    TEST_EQUAL(0, mcpwm_get_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B));
}
