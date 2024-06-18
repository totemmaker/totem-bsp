/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "totem_test.h"
#include "bsp/totem-bsp.h"

#include "driver/gpio.h"

bool pulseUpdate = false;
void on_bsp_event(uint32_t evt, uint32_t portID, uint32_t value, void *arg) {
    TEST_ASSERT((uint32_t)arg == 5);
    if (evt == BSP_EVT_SERVO_PULSE && value) pulseUpdate = true;
}

TEST_CASE("Test Board API", "[BOARD]") {
    // Should accept any port
    int revision = bsp_cmd_read(BSP_BOARD_REVISION, 55);
    int driver = bsp_cmd_read(BSP_DRIVER_FIRMWARE, 0);
    // Register event handler
    bsp_board_reg_event(on_bsp_event, (void*)5);
    // Check read results are valid
    TEST_VALUES(10, 11, revision);
    TEST_LIMIT(150, 300, driver);
    TEST_VALUES(0, 1, bsp_cmd_read(BSP_BUTTON_STATE, 0));
    TEST_EQUAL(1, bsp_cmd_read(BSP_USB_STATE, 0));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    if (revision == 11) TEST_EQUAL(0, bsp_cmd_read(BSP_POWER_STATE, 0));
    else TEST_VALUES(0, 1, bsp_cmd_read(BSP_POWER_STATE, 0));
    if (driver >= 160) { // Driver v1.60 by default initializes motors to 20kHz PWM
        TEST_EQUAL(20000, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 0));
        TEST_EQUAL(20000, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 1));
        TEST_EQUAL(20000, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 2));
        TEST_EQUAL(20000, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 3));
        // Slow decay selected
        TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_DECAY, 0));
        TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_DECAY, 1));
        TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_DECAY, 2));
        TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_DECAY, 3));
    }
    else { // Older drivers initialize motor to 50Hz
        TEST_EQUAL(50, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 0));
        TEST_EQUAL(50, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 1));
        TEST_EQUAL(50, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 2));
        TEST_EQUAL(50, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 3));
        // Fast decay selected
        TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_DECAY, 0));
        TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_DECAY, 1));
        TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_DECAY, 2));
        TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_DECAY, 3));
    }
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 1));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 2));
    // Test port validation
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 4));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 3));
    TEST_EQUAL(ESP_ERR_NOT_FOUND, bsp_cmd_write(BSP_RGB_COLOR, 4, 0));
    // Check if write is not allowed for unsupported commands
    uint8_t read_only[] = {
        BSP_BOARD_REVISION,
        BSP_DRIVER_FIRMWARE,
        BSP_BUTTON_STATE,
        BSP_POWER_STATE,
        BSP_USB_STATE,
        BSP_BATTERY_VOLTAGE,
    };
    uint8_t write_only[] = {
        BSP_LED_STATE,
        BSP_CAN_STATE,
        BSP_5V_STATE,
        BSP_DC_POWER,
        BSP_DC_BRAKE,
        BSP_DC_TONE,
        BSP_DC_CONFIG_ENABLE,
        BSP_SERVO_CONFIG_SPEED,
        BSP_SERVO_CONFIG_ENABLE,
        BSP_RGB_COLOR,
        BSP_RGB_FADE_COLOR,
        BSP_RGB_FADE_START,
        BSP_RGB_CONFIG_ENABLE,
    };
    // Try write read only commands
    for (int i=0; i<sizeof(read_only); i++) {
        TEST_MESSAGE("cmd=%d", read_only[i]);
        TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_cmd_write(read_only[i], 0, 1));
    }
    // Try read write only commands
    for (int i=0; i<sizeof(write_only); i++) {
        TEST_MESSAGE("cmd=%d", write_only[i]);
        TEST_EQUAL(0, bsp_cmd_read(write_only[i], 0));
    }
    // Test DC value validation
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_DC_POWER, 0, -101));
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_DC_POWER, 0, 101));
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_DC_BRAKE, 0, 101));
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_DC_BRAKE, 0, -1));
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_DC_TONE, 0, 20001));
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, 0, 0));
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, 0, 250001));
    if (driver >= 160) {
        TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_DC_CONFIG_DECAY, 0, 2));
    }
    else {
        TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_cmd_write(BSP_DC_CONFIG_DECAY, 0, 2));
        TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_cmd_write(BSP_SERVO_PULSE, 0, (0x8000 << 16) | 1500));
        TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_cmd_write(BSP_SERVO_CONFIG_SPEED, 0, 30));
    }
    // Test Servo value validation
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_SERVO_PULSE, 0, 20001));
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, 0, 0));
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, 0, 0xFFFF+1));
    // Test port validation
    TEST_ERROR(ESP_ERR_NOT_FOUND, bsp_cmd_write(BSP_DC_POWER, 4, 0));
    TEST_ERROR(ESP_ERR_NOT_FOUND, bsp_cmd_write(BSP_SERVO_PULSE, 3, 0));
    TEST_ERROR(ESP_ERR_NOT_FOUND, bsp_cmd_write(BSP_RGB_COLOR, 4, 0));
    // Test cmd validation
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_CMD_MAX, 0, 0));
    // Test motor values
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_DC_TONE, 0, (0xFFFF << 16) | 20001));
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_TONE, 0, (0xFFFF << 16) | 20000));
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_POWER, 0, 0));
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_SERVO_PULSE, 0, 20001));
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_SERVO_PULSE, 0, (0x7FFF << 16) | 20001));
    TEST_ASSERT(pulseUpdate == false);
    if (driver >= 160) {
        TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_PULSE, 0, ((0x8000|20000) << 16) | 20000));
        TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_PULSE, 0, ((0x8000|1) << 16) | 20000));
    }
    else {
        TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_cmd_write(BSP_SERVO_PULSE, 0, (0xFFFF << 16) | 20000));
    }
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_PULSE, 0, (0x7FFF << 16) | 19999));
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_PULSE, 0, (0x7FFF << 16) | 0));
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, 0, 25000));
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_PULSE, 0, 25000));
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, 0, 20000));
    if (driver >= 160) {
        TEST_DELAY(10); // Wait for periph update event
        TEST_ASSERT(pulseUpdate == true);
    }
    TEST_EQUAL(25000, bsp_cmd_read(BSP_SERVO_PULSE, 0));
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_PULSE, 0, 0));
    int timeoutMax = 6000; // 6000 us max
    if (driver < 160) timeoutMax = 12000;
    // Test spam write
    for (int i=0; i<500; i++) {
        TEST_TIME(timeoutMax,
        TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_PULSE, 0, 600));
        TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_PULSE, 0, 700));
        );
    }
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_SERVO_PULSE, 0, 0));
    // Test spam write
    for (int i=0; i<500; i++) {
        TEST_TIME(timeoutMax,
        TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_POWER, 0, 10));
        TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_POWER, 0, 20));
        );
    }
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_POWER, 0, 0));
}
