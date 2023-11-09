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

TEST_CASE("Test Board API", "[Board]") {
    // Should accept any port
    int revision = bsp_cmd_read(BSP_BOARD_REVISION, 55);
    // Check read results are valid
    TEST_VALUES(revision, 30);
    TEST_VALUES(0, 1, bsp_cmd_read(BSP_BUTTON_STATE, 0));
    TEST_EQUAL(1, bsp_cmd_read(BSP_USB_STATE, 0));
    // Voltage should read ~4.9V when USB is plugged in
    TEST_LIMIT(4750, 5100, bsp_cmd_read(BSP_BATTERY_VOLTAGE, 0));
    // Should be low current at idle
    TEST_LIMIT(-200, 450, bsp_cmd_read(BSP_BATTERY_CURRENT, 0));
    TEST_VALUES(0, 1, bsp_cmd_read(BSP_BATTERY_CHARGING, 0));
    if (bsp_cmd_read(BSP_BATTERY_CHARGING, 0)) TEST_MIN(1, bsp_cmd_read(BSP_BATTERY_CURRENT, 0));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 0));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 1));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 2));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 3));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_DECAY, 0));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_DECAY, 1));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_DECAY, 2));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_DECAY, 3));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 1));
    // Test port validation
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 4));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 2));
    // Check if write is not allowed for unsupported commands
    uint8_t read_only[] = {
        BSP_BOARD_REVISION,
        BSP_BUTTON_STATE,
        BSP_USB_STATE,
        BSP_BATTERY_VOLTAGE,
        BSP_BATTERY_CURRENT,
        BSP_BATTERY_CHARGING,
        BSP_DC_CONFIG_FREQUENCY,
        BSP_DC_CONFIG_DECAY,
        BSP_SERVO_CONFIG_PERIOD,
    };
    // Try read write only commands
    for (int cmd=0; cmd<BSP_CMD_MAX; cmd++) {
        bool skip = false;
        for (int i=0; i<sizeof(read_only); i++) {
            if (cmd == read_only[i]) { skip = true; break; }
        }
        if (skip) continue;
        TEST_MESSAGE("cmd=%d", cmd);
        TEST_EQUAL(0, bsp_cmd_read(cmd, 0));
    }
    // Try write read only commands
    for (int i=0; i<sizeof(read_only)-3; i++) {
        TEST_MESSAGE("cmd=%d", read_only[i]);
        TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_cmd_write(read_only[i], 0, 1));
    }
    // Test DC value validation
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_POWER, 0, -101));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_POWER, 0, 101));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_BRAKE, 0, 101));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_TONE, 0, 20001));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, 0, 0));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, 0, 250001));
    // Test Servo value validation
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_SERVO_PULSE, 0, 20001));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, 0, 0));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, 0, 1000000+1));
    // Test port validation
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_DC_POWER, 4, 0));
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_SERVO_PULSE, 2, 0));
    // Test cmd validation
    TEST_ERROR(ESP_ERR_NOT_FOUND, bsp_cmd_write(BSP_CMD_MAX, 0, 0));
}

TEST_CASE("Test Board API (GPIO)", "[Board]") {
    // Test button
    TEST_EQUAL(0, bsp_cmd_read(BSP_BUTTON_STATE, 0));
    TEST_EQUAL(1, gpio_get_level(BSP_IO_BUTTON));
    // Test USB
    TEST_EQUAL(1, bsp_cmd_read(BSP_USB_STATE, 0));
    TEST_EQUAL(1, gpio_get_level(BSP_IO_USB_DETECT));
    // Test charging
    TEST_EQUAL(!gpio_get_level(BSP_IO_BATTERY_CHARGE), bsp_cmd_read(BSP_BATTERY_CHARGING, 0));
}
