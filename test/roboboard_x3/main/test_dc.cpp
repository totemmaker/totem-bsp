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

TEST_CASE("Test DC MCPWM", "[DC]") {
    // Validate parameters
    TEST_EQUAL(20000, mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0));
    TEST_EQUAL(20000, mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_1));
    TEST_EQUAL(20000, mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_2));
    TEST_EQUAL(20000, mcpwm_get_frequency(MCPWM_UNIT_1, MCPWM_TIMER_0));
    
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B));
    // Change frequency and validate
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_ALL, 50));
    TEST_EQUAL(50, mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0));
    TEST_EQUAL(50, mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_1));
    TEST_EQUAL(50, mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_2));
    TEST_EQUAL(50, mcpwm_get_frequency(MCPWM_UNIT_1, MCPWM_TIMER_0));
    // [slow decay] Change duty cycle and validate brake 90%
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_BRAKE, BSP_PORT_ALL, 90));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B));
    // [slow decay] Change duty cycle and validate 50%
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_POWER, BSP_PORT_ALL, 50));
    TEST_EQUAL(100, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
    TEST_EQUAL(100, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B));
    TEST_EQUAL(100, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B));
    TEST_EQUAL(100, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B));
    // [slow decay] Change duty cycle and validate -50%
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_POWER, BSP_PORT_ALL, -50));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(100, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A));
    TEST_EQUAL(100, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A));
    TEST_EQUAL(100, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(100, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B));
    // Change to fast decay
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_CONFIG_DECAY, BSP_PORT_ALL, 1));
    // Should affect duty
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B));
    // [fast decay] Change duty cycle and validate brake 90%
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_BRAKE, BSP_PORT_ALL, 90));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(90, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B));
    // [fast decay] Change duty cycle and validate 50%
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_POWER, BSP_PORT_ALL, 50));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B));
    // Change back to slow decay
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_CONFIG_DECAY, BSP_PORT_ALL, 0));
    // Should affect duty
    TEST_EQUAL(100, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
    TEST_EQUAL(100, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B));
    TEST_EQUAL(100, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B));
    TEST_EQUAL(100, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(50, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B));
    // Restore frequency and validate
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_ALL, 20000));
    TEST_EQUAL(20000, mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0));
    TEST_EQUAL(20000, mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_1));
    TEST_EQUAL(20000, mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_2));
    TEST_EQUAL(20000, mcpwm_get_frequency(MCPWM_UNIT_1, MCPWM_TIMER_0));
    // Restore duty cycle and validate
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_POWER, BSP_PORT_ALL, 0));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A));
    TEST_EQUAL(0, mcpwm_get_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_B));
}
