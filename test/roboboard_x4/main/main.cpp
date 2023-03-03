/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "driver/gpio.h"

#include "totem_test.h"

#include "bsp/roboboard_x4.h"
#include "bsp/roboboard_x4.hpp"

static struct {
    bsp_cmd_t cmd;
    uint8_t port;
    int32_t value;
    uint8_t called;
} lastCall;

void bsp_cmd_change_callback(bsp_cmd_t cmd, uint8_t port, int32_t value, uint8_t isr) {
    lastCall.cmd = cmd;
    lastCall.port = port;
    lastCall.value = value;
    lastCall.called = 1;
}

TEST_CASE("Board init [board][C++]", "[board][C++]") {
    bsp_callback_register(bsp_cmd_change_callback);
    // Initialize board
    TEST_ERROR(ESP_OK, X4.begin());
}

#include "test_board.cpp"
#include "test_dc.cpp"
#include "test_imu.cpp"
#include "test_servo.cpp"
#include "test_rgb.cpp"

TEST_CASE("Test board configuration", "[config][C++]") {
    // Reset config before start
    X4.config.reset();
    // Make sure config is empty
    char *name = X4.config.getRobotName();
    TEST_STRING("RoboBoard X4", name);
    TEST_EQUAL(0, X4.config.getRobotModel());
    TEST_EQUAL(0, X4.config.getRobotColor());
    TEST_EQUAL(0, X4.config.getDCInvert());
    TEST_EQUAL(0, X4.config.getDCAutobrake());
    // Write test values
    X4.config.setRobotName("_123456789_123456789_123456789_abcd"); // Limit 29 chars
    X4.config.setRobotModel("abcdefgh"); // fnv1a16Hash hash
    X4.config.setRobotColor(0x13124578); // ignore alpha
    X4.config.setDCInvert(true, false, true, false);
    X4.config.setDCAutobrake(false, true, false, true);
    // Test stored values
    TEST_STRING("_123456789_123456789_12345678", X4.config.getRobotName());
    TEST_EQUAL(0x0000dc57, X4.config.getRobotModel());
    TEST_EQUAL(0x00124578, X4.config.getRobotColor());
    TEST_EQUAL(0x01000100, X4.config.getDCInvert());
    TEST_EQUAL(0x00640064, X4.config.getDCAutobrake());
}

TEST_CASE("Test Qwiic I2C scan", "[qwiic][C++]") {
    static int foundAddr = 0;
    int foundCnt = X4.qwiic.scan([](int addr) {
        foundAddr = addr;
    });
    // Always has to find IMU sensor
    TEST_EQUAL(1, foundCnt);
    TEST_EQUAL(X4.imu.getI2CAddr(), foundAddr);
    TEST_EQUAL(true, X4.qwiic.isConnected(X4.imu.getI2CAddr()));
    TEST_EQUAL(false, X4.qwiic.isConnected(0x5)); // Random address. Not connected
}

TEST_CASE("Test bsp_cmd_read does not trigger callback", "[bsp][C]") {
    lastCall.called = 0;
    // No callback should be called during read
    for (int i=0; i<BSP_CMD_MAX; i++) {
        bsp_cmd_read(i, 0);
        TEST_EQUAL(0, lastCall.called);
    }
}

TEST_CASE("Test bsp_cmd_write error", "[bsp][C]") {
    // Callback must not be called if write operation failed
    lastCall.called = 0;
    // Test port number validation
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_DC_POWER, 4, 5)); TEST_EQUAL(0, lastCall.called);
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_SERVO_PULSE, 3, 1000)); TEST_EQUAL(0, lastCall.called);
    // Test command value validation
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_POWER, 0, 5000)); TEST_EQUAL(0, lastCall.called);
    // Test command id validation
    TEST_ERROR(ESP_ERR_NOT_FOUND, bsp_cmd_write(BSP_CMD_MAX, 0, 5)); TEST_EQUAL(0, lastCall.called);
    TEST_ERROR(ESP_ERR_NOT_FOUND, bsp_cmd_write(-1, 0, 5)); TEST_EQUAL(0, lastCall.called);
    // Test read only validation
    TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_cmd_write(BSP_BOARD_SERIAL, 0, 5)); TEST_EQUAL(0, lastCall.called);
    TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_cmd_write(BSP_BOARD_REVISION, 0, 5)); TEST_EQUAL(0, lastCall.called);
    TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_cmd_write(BSP_DRIVER_FIRMWARE, 0, 5)); TEST_EQUAL(0, lastCall.called);
    TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_cmd_write(BSP_BUTTON_STATE, 0, 5)); TEST_EQUAL(0, lastCall.called);
    TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_cmd_write(BSP_DC_STATE, 0, 5)); TEST_EQUAL(0, lastCall.called);
    TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_cmd_write(BSP_USB_STATE, 0, 5)); TEST_EQUAL(0, lastCall.called);
    TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_cmd_write(BSP_BATTERY_VOLTAGE, 0, 5)); TEST_EQUAL(0, lastCall.called);
    TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_cmd_write(BSP_BATTERY_STATE, 0, 5)); TEST_EQUAL(0, lastCall.called);
}

TEST_CASE("Test bsp_cmd_write trigger callback", "[bsp][C]") {
    // Test if value change with bsp_cmd_write triggers callback
    bsp_cmd_write(BSP_DC_CONFIG_ENABLE, BSP_PORT_C, 1);
    lastCall.called = 0;
    bsp_cmd_write(BSP_DC_CONFIG_ENABLE, BSP_PORT_C, 1);
    // Value not changed. No trigger
    TEST_EQUAL(0, lastCall.called); lastCall.called = 0;
    bsp_cmd_write(BSP_DC_CONFIG_ENABLE, BSP_PORT_C, 0);
    // Value changed. Trigger
    TEST_EQUAL(0, lastCall.value);
    TEST_EQUAL(1, lastCall.called); lastCall.called = 0;
    bsp_cmd_write(BSP_DC_CONFIG_ENABLE, BSP_PORT_C, 1);
    TEST_EQUAL(1, lastCall.value);
    TEST_EQUAL(1, lastCall.called); lastCall.called = 0;
}

TEST_CASE("Test write time", "[bsp][C]") {
    // Clean UART TX FIFO before test
    uart_flush_input(UART_NUM_2);
    uart_wait_tx_done(UART_NUM_2, portMAX_DELAY);
    // UART FIFO length is 128 bytes. Each cmd packet is 6 bytes.
    // Should be able to write 21 unique commands in row between 15-70us.
    for (int i=0; i<=21; i++) {
        TEST_MESSAGE("i=%d", i);
        TEST_TIME(100, bsp_cmd_write(BSP_DC_POWER, BSP_PORT_A, i));
    }
    TEST_TIME(100, bsp_cmd_write(BSP_DC_POWER, BSP_PORT_A, 0));
}

TEST_CASE("Test read time", "[bsp][C]") {
    for (int cmd=0; cmd<BSP_CMD_MAX; cmd++) {
        int maxT = 20;
        if (cmd == BSP_BOARD_SERIAL) maxT = 250; // up to 200 us
        if (cmd == BSP_BATTERY_VOLTAGE) maxT = 1000; // up to 1ms
        if (cmd == BSP_SERVO_PULSE) maxT = 14000; // up to 14ms
        TEST_MESSAGE("cmd=%d", cmd);
        TEST_TIME(maxT, bsp_cmd_read(cmd, BSP_PORT_A));
    }
}

TEST_CASE("Test write time", "[bsp][C++]") {
    // Clean UART TX FIFO before test
    uart_flush_input(UART_NUM_2);
    uart_wait_tx_done(UART_NUM_2, portMAX_DELAY);

    for (int i=1; i<=21; i+=1) {
        TEST_MESSAGE("i=%d", i);
        TEST_TIME(150, X4.dcA.power(i));
    }
    X4.dcA.power(0);
}

void print_board_info() {
    TEST_PRINT("--- Printing board information ---");
    TEST_PRINT("DCin: %d", X4.isDC());
    TEST_PRINT("USBin: %d", X4.isUSB());
    TEST_PRINT("Rev: %s", X4.getRevision());
    TEST_PRINT("Driver: %s", X4.getDriverVersion());
    TEST_PRINT("Serial: %d", X4.getSerial());
    TEST_PRINT("Battery: %.3fV", X4.getBatteryVoltage());
}

void run_button_test() {
    TEST_PRINT("Button test is running...");
    X4.button.addEvent([]() {
        TEST_PRINT("ButtonEVT: state: %d", !gpio_get_level(BSP_IO_BUTTON));
        if (X4.button.isPressed()) TEST_PRINT("ButtonEVT: isPressed");
        if (X4.button.isReleased()) TEST_PRINT("ButtonEVT: isReleased");
    });
    while (1) {
        // Button test
        if (X4.button.wasPressedFor(1500)) TEST_PRINT("Button: wasPressedFor(1500)");
        if (X4.button.wasReleasedFor(2500)) TEST_PRINT("Button: wasReleasedFor(2500)");
        if (X4.button.wasPressed()) TEST_PRINT("Button: wasPressed");
        if (X4.button.wasReleased()) TEST_PRINT("Button: wasReleased");
        if (X4.button.wasLongPress()) TEST_PRINT("Button: wasLongPress");
        if (X4.button.wasDoubleClick()) TEST_PRINT("Button: wasDoubleClick()");
    }
}

#define WAIT_BUTTON() X4.button.waitPress()

// Manually test if features are actually working
void run_hardware_test() {
    TEST_PRINT("--- Running manual hardware test ---");
    TEST_PRINT("Resetting default parameters...");
    // Reset hardware to default settings
    X4.dc.enable();
    X4.dcAB.setFrequency(50);
    X4.dcCD.setFrequency(50);
    X4.dc.setInvert(0, 0, 0, 0);
    X4.dc.setAutobrake(0, 0, 0, 0);
    X4.dc.power(0, 0, 0, 0);
    X4.servo.enable();
    X4.servo.setPeriod(20000);
    X4.servoA.setPulseMinMax(500, 2500);
    X4.servoB.setPulseMinMax(500, 2500);
    X4.servoC.setPulseMinMax(500, 2500);
    X4.servoA.setInvert(0);
    X4.servoB.setInvert(0);
    X4.servoC.setInvert(0);
    X4.servoA.setSpeedRPM(0);
    X4.servoB.setSpeedRPM(0);
    X4.servoC.setSpeedRPM(0);
    X4.servo.pos(0, 0, 0);
    X4.rgb.enable();
    X4.rgb.off();
    TEST_PRINT("Running DC hardware test... Press button to continue.");
    for (int i=0; i<4; i++) {
        TEST_PRINT("Next test: DC %c", 'A'+i);
        WAIT_BUTTON();
        X4.dc[i].power(80); // Spin clockwise
        TEST_PRINT("Set DC %c power %d CW", 'A'+i, 80);
        WAIT_BUTTON();
        X4.dc[i].disable();
        X4.dc[i].power(100);
        TEST_PRINT("Disable DC %c", 'A'+i);
        WAIT_BUTTON();
        X4.dc[i].enable();
        X4.dc[i].power(-80); // Spin counter-clockwise
        TEST_PRINT("Enable.\nSet DC %c power %d CCW", 'A'+i, -80);
        WAIT_BUTTON();
        X4.dc[i].setInvert(1); // Spin clockwise
        TEST_PRINT("Invert DC %c CW", 'A'+i);
        WAIT_BUTTON();
        X4.dc[i].brake(); // Stop
        TEST_PRINT("Brake DC %c", 'A'+i);
    }
    TEST_PRINT("Next test: DC CD tone");
    WAIT_BUTTON();
    X4.dcCD.tone(1000, 500);
    TEST_DELAY(600);
    X4.dcCD.tone(2000, 500);
    TEST_DELAY(600);
    X4.dcCD.tone(3000, 500);
    TEST_DELAY(600);
    TEST_PRINT("Next test: DC AB tone");
    WAIT_BUTTON();
    X4.dcAB.tone(3000, 500);
    TEST_DELAY(600);
    X4.dcAB.tone(2000, 500);
    TEST_DELAY(600);
    X4.dcAB.tone(1000, 500);
    TEST_DELAY(600);
    TEST_PRINT("Running Servo hardware test... Press button to continue.");
    for (int i=0; i<3; i++) {
        TEST_PRINT("Next test: Servo %c", 'A'+i);
        WAIT_BUTTON();
        X4.servo[i].pos(-100);
        TEST_PRINT("Pos Servo %c to %d", 'A'+i, -100);
        WAIT_BUTTON();
        X4.servo[i].pos(100);
        TEST_PRINT("Pos Servo %c to %d", 'A'+i, 100);
        WAIT_BUTTON();
        X4.servo[i].setInvert(1);
        TEST_PRINT("Invert Servo %c", 'A'+i);
        WAIT_BUTTON();
        X4.servo[i].pos(50, 1000);
        TEST_PRINT("Pos Servo %c to %d in 1s", 'A'+i, 50);
        WAIT_BUTTON();
        X4.servo[i].disable();
        X4.servo[i].pos(-100);
        TEST_PRINT("Disabled Servo %c. Free spin", 'A'+i);
        WAIT_BUTTON();
        X4.servo[i].enable();
        TEST_PRINT("Enabled. Pos Servo %c to %d", 'A'+i, -100);
    }
    TEST_PRINT("Running RGB hardware test... Press button to continue.");
    X4.rgb.colorRGB(0, 255, 0); // Set green
    WAIT_BUTTON();
    X4.rgb[0].fadeColorRGB(255, 0, 0);
    X4.rgb[1].fadeColorRGB(0, 255, 0);
    X4.rgb[2].fadeColorRGB(0, 255, 255);
    X4.rgb[3].fadeColorRGB(255, 255, 0);
    X4.rgb[0].fadeStart(500);
    X4.rgb[1].fadeStart(1000);
    X4.rgb[2].fadeStart(1500);
    X4.rgb[3].fadeStart(2000);
    TEST_PRINT("Individual fade. A: 0.5s, B: 1.0s, C: 1.5s, D: 2.0s");
    for (int i=0; i<5; i++) {
        WAIT_BUTTON();
        if (i-1>=0) X4.rgb[i-1].enable();
        if (i < 4) {
            X4.rgb[i].disable();
            TEST_PRINT("Disabled RGB %c", 'A'+i);
        }
    }
    WAIT_BUTTON();
    X4.rgb.colorARGB(150, 125, 125, 125);
    TEST_PRINT("Dim RGB");
    TEST_PRINT("--- Test end ---");
}

extern "C" void app_main(void) {
    // Run unit tests
    TEST_RUN();
    // Print board info
    print_board_info();
    // Run button test
    run_hardware_test();
}
