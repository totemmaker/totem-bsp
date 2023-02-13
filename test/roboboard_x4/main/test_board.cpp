/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "totem_test.h"
#include "bsp/roboboard_x4.h"
#include "bsp/roboboard_x4.hpp"

#include "driver/gpio.h"

TEST_CASE("Validate initial state", "[board][C]") {
    // Check if certain values are within expected range after board initialization
    int revision = bsp_cmd_read(BSP_BOARD_REVISION, 0);
    TEST_LIMIT(0, 0x7FFF, bsp_cmd_read(BSP_BOARD_SERIAL, 0));
    TEST_VALUES(10, 11, revision);
    TEST_LIMIT(100, 999, bsp_cmd_read(BSP_DRIVER_FIRMWARE, 0));
    TEST_EQUAL(0, bsp_cmd_read(BSP_BUTTON_STATE, 0));
    TEST_EQUAL(0, bsp_cmd_read(BSP_LED_STATE, 0));
    TEST_EQUAL(1, bsp_cmd_read(BSP_5V_STATE, 0));
    // In revision 1.1 DC check doesn't work (hardware issue). Should always 0
    if (revision == 11) TEST_EQUAL(0, bsp_cmd_read(BSP_DC_STATE, 0));
    if (revision == 10) TEST_LIMIT(0, 1, bsp_cmd_read(BSP_DC_STATE, 0));
    TEST_EQUAL(1, bsp_cmd_read(BSP_USB_STATE, 0));
    TEST_LIMIT(8400, 12600, bsp_cmd_read(BSP_BATTERY_VOLTAGE, 0));
    TEST_LIMIT(0, 1, bsp_cmd_read(BSP_BATTERY_STATE, 0));
}

TEST_CASE("Validate initial state", "[board][C++]") {
    // Check if certain values are within expected range after board initialization
    int revision = X4.getRevisionNum();
    int firmware = X4.getDriverVersionNum();
    TEST_VALUES(10, 11, revision);
    TEST_LIMIT(100, 999, firmware);
    TEST_STRING(X4.getRevision(), "%d.%d", (revision/10), (revision%10));
    TEST_STRING(X4.getDriverVersion(), "%d.%d%d", (firmware/100%10), (firmware/10%10), (firmware%10));
    TEST_LIMIT(0, 0x7FFF, X4.getSerial());
    // In revision 1.1 DC check doesn't work (hardware issue). Should always 0
    if (revision == 11) TEST_EQUAL(0, X4.isDC());
    if (revision == 10) TEST_LIMIT(0, 1, X4.isDC());
    TEST_EQUAL(1, X4.isUSB());
    TEST_LIMIT(8400, 12600, X4.getBatteryVoltage() * 1000);
}

TEST_CASE("Validate button state", "[board][C++]") {
    TEST_EQUAL(0, X4.button.isPressed());
    TEST_EQUAL(1, X4.button.isReleased());
    TEST_EQUAL(0, X4.button.isPressedFor(0));
    TEST_EQUAL(0, X4.button.isPressedFor(1000));
    TEST_EQUAL(1, X4.button.isReleasedFor(0));
    TEST_EQUAL(0, X4.button.isReleasedFor(1000));
    TEST_EQUAL(0, X4.button.wasPressed());
    TEST_EQUAL(0, X4.button.wasReleased());
    TEST_EQUAL(0, X4.button.wasPressedFor(0));
    TEST_EQUAL(0, X4.button.wasPressedFor(1000));
    TEST_EQUAL(1, X4.button.wasReleasedFor(0));
    TEST_EQUAL(0, X4.button.wasReleasedFor(1000));
    TEST_EQUAL(0, X4.button.wasDoubleClick());
    TEST_LIMIT(0, 2000, X4.button.lastChange());
}

TEST_CASE("Test write over limits", "[board][C]") {
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_LED_STATE, 0, -5)); TEST_EQUAL(1, bsp_cmd_read(BSP_LED_STATE, BSP_PORT_ALL));
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_LED_STATE, 0, 0)); TEST_EQUAL(0, bsp_cmd_read(BSP_LED_STATE, BSP_PORT_A));
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_5V_STATE, 0, 0)); TEST_EQUAL(0, bsp_cmd_read(BSP_5V_STATE, BSP_PORT_ALL));
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_5V_STATE, 0, 9)); TEST_EQUAL(1, bsp_cmd_read(BSP_5V_STATE, BSP_PORT_A));
}

TEST_CASE("Test LED API", "[board][C++]") {
    X4.led.on();
    TEST_EQUAL(1, X4.led.isOn());
    X4.led.set(0);
    TEST_EQUAL(0, X4.led.isOn());
    X4.led.set(1);
    TEST_EQUAL(1, X4.led.isOn());
    X4.led.off();
    TEST_EQUAL(0, X4.led.isOn());
    X4.led.toggle();
    TEST_EQUAL(1, X4.led.isOn());
    X4.led.toggle();
    TEST_EQUAL(0, X4.led.isOn());
    X4.led.toggle();
    TEST_EQUAL(1, X4.led.isOn());
}

TEST_CASE("Test LED function", "[board][C]") {
    // Turn on
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_LED_STATE, 1, 60));
    TEST_EQUAL(1, bsp_cmd_read(BSP_LED_STATE, 0));
    TEST_EQUAL(1, gpio_get_level(BSP_IO_LED));
    TEST_EQUAL(1, X4.led.isOn());
    // Turn off
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_LED_STATE, 0, 0));
    TEST_EQUAL(0, bsp_cmd_read(BSP_LED_STATE, 0));
    TEST_EQUAL(0, gpio_get_level(BSP_IO_LED));
    TEST_EQUAL(0, X4.led.isOn());
    // Turn on
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_LED_STATE, 2, 1));
    TEST_EQUAL(1, bsp_cmd_read(BSP_LED_STATE, 0));
    TEST_EQUAL(1, gpio_get_level(BSP_IO_LED));
    TEST_EQUAL(1, X4.led.isOn());
    // Turn off
    gpio_set_level(BSP_IO_LED, 0);
    TEST_EQUAL(0, bsp_cmd_read(BSP_LED_STATE, 0));
    TEST_EQUAL(0, gpio_get_level(BSP_IO_LED));
    TEST_EQUAL(0, X4.led.isOn());
    // Turn on
    gpio_set_level(BSP_IO_LED, 1);
    TEST_EQUAL(1, bsp_cmd_read(BSP_LED_STATE, 0));
    TEST_EQUAL(1, gpio_get_level(BSP_IO_LED));
    TEST_EQUAL(1, X4.led.isOn());
}

TEST_CASE("Test LED function", "[board][C++]") {
    // Turn on
    X4.led.on();
    TEST_EQUAL(1, bsp_cmd_read(BSP_LED_STATE, 0));
    TEST_EQUAL(1, gpio_get_level(BSP_IO_LED));
    TEST_EQUAL(1, X4.led.isOn());
    // Turn off
    X4.led.set(0);
    TEST_EQUAL(0, bsp_cmd_read(BSP_LED_STATE, 0));
    TEST_EQUAL(0, gpio_get_level(BSP_IO_LED));
    TEST_EQUAL(0, X4.led.isOn());
    // Turn on
    X4.led.set(50);
    TEST_EQUAL(1, bsp_cmd_read(BSP_LED_STATE, 0));
    TEST_EQUAL(1, gpio_get_level(BSP_IO_LED));
    TEST_EQUAL(1, X4.led.isOn());
    // Turn off
    X4.led.off();
    TEST_EQUAL(0, bsp_cmd_read(BSP_LED_STATE, 0));
    TEST_EQUAL(0, gpio_get_level(BSP_IO_LED));
    TEST_EQUAL(0, X4.led.isOn());
    // Turn on
    X4.led.toggle();
    TEST_EQUAL(1, bsp_cmd_read(BSP_LED_STATE, 0));
    TEST_EQUAL(1, gpio_get_level(BSP_IO_LED));
    TEST_EQUAL(1, X4.led.isOn());
    // Keep on
    X4.led.on();
    TEST_EQUAL(1, bsp_cmd_read(BSP_LED_STATE, 0));
    TEST_EQUAL(1, gpio_get_level(BSP_IO_LED));
    TEST_EQUAL(1, X4.led.isOn());
    // Turn off
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_LED_STATE, 0, 0));
    TEST_EQUAL(0, bsp_cmd_read(BSP_LED_STATE, 0));
    TEST_EQUAL(0, gpio_get_level(BSP_IO_LED));
    TEST_EQUAL(0, X4.led.isOn());
}
