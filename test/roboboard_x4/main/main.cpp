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

#include "bsp/totem-bsp.h"
#include "bsp/imu.h"
#include "bsp/can.h"

int button_irq;

void interrupt_func(void *arg) {
    if ((uint32_t)arg == BSP_BUTTON_STATE) button_irq = 1;
}

TEST_CASE("Board init", "[board]") {
    // Initialize board
    TEST_ERROR(ESP_OK, bsp_board_init());
    int revision = bsp_cmd_read(BSP_BOARD_REVISION, 0);
    TEST_ERROR(ESP_OK, bsp_register_interrupt_callback(BSP_BUTTON_STATE, interrupt_func, (uint32_t*)BSP_BUTTON_STATE));
    TEST_ERROR(revision == 11 ? ESP_ERR_NOT_SUPPORTED : ESP_OK, bsp_register_interrupt_callback(BSP_POWER_STATE, interrupt_func, 0));
    TEST_ERROR(ESP_OK, bsp_register_interrupt_callback(BSP_USB_STATE, interrupt_func, 0));
    TEST_ERROR(ESP_ERR_NOT_SUPPORTED, bsp_register_interrupt_callback(BSP_BOARD_REVISION, interrupt_func, 0));
}

#include "test_board.cpp"
#include "test_imu.cpp"
#include "test_can.cpp"

void print_board_info() {
    TEST_PRINT("--- Printing board information ---");
    TEST_PRINT("RoboBoard X4");
    TEST_PRINT("USBin: %d", bsp_cmd_read(BSP_USB_STATE, 0));
    TEST_PRINT("Rev: %d", bsp_cmd_read(BSP_BOARD_REVISION, 0));
    TEST_PRINT("Driver: %d", bsp_cmd_read(BSP_DRIVER_FIRMWARE, 0));
    TEST_PRINT("Battery: %d mV", bsp_cmd_read(BSP_BATTERY_VOLTAGE, 0));
    TEST_PRINT("DC frequency: %d Hz", bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 0));
    TEST_PRINT("Servo period: %d us", bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0));
}

void wait_button() {
    while (bsp_cmd_read(BSP_BUTTON_STATE, 0) == 1) { vTaskDelay(10); }
    while (bsp_cmd_read(BSP_BUTTON_STATE, 0) == 0) { vTaskDelay(10); }
}

// Manually test if features are actually working
void run_hardware_test() {
    TEST_PRINT("--- Running manual hardware test ---");
    // Test Button
    TEST_PRINT("--- Test Button ---");
    TEST_PRINT("waiting for button press...");
    button_irq = 0;
    while (bsp_cmd_read(BSP_BUTTON_STATE, 0) == 0) { vTaskDelay(10); }
    TEST_EQUAL(1, button_irq);
    button_irq = 0;
    while (bsp_cmd_read(BSP_BUTTON_STATE, 0) == 1) { vTaskDelay(10); }
    TEST_EQUAL(1, button_irq);
    TEST_PRINT("Done");
    // Test LED
    TEST_PRINT("--- Test LED ---");
    TEST_PRINT("set LED on"); wait_button();
    bsp_cmd_write(BSP_LED_STATE, 0, 1);
    TEST_PRINT("set LED off"); wait_button();
    bsp_cmd_write(BSP_LED_STATE, 0, 0);
    // Test RGB
    TEST_PRINT("--- Test RGB ---");
    TEST_PRINT("RGB on A"); wait_button();
    bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_ALL, 0);
    bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_A, 0xFFFFFFFF);
    TEST_PRINT("RGB on B"); wait_button();
    bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_ALL, 0);
    bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_B, 0xFFFFFFFF);
    TEST_PRINT("RGB on C"); wait_button();
    bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_ALL, 0);
    bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_C, 0xFFFFFFFF);
    TEST_PRINT("RGB on D"); wait_button();
    bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_ALL, 0);
    bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_D, 0xFFFFFFFF);
    TEST_PRINT("RGB set RED"); wait_button();
    bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_ALL, 0xFFFF0000);
    TEST_PRINT("RGB set GREEN"); wait_button();
    bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_ALL, 0xFF00FF00);
    TEST_PRINT("RGB set BLUE"); wait_button();
    bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_ALL, 0xFF0000FF);
    TEST_PRINT("RGB set BLUE alpha 50%"); wait_button();
    bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_ALL, 0x800000FF);
    TEST_PRINT("RGB fade to white in 3s"); wait_button();
    bsp_cmd_write(BSP_RGB_FADE_COLOR, BSP_PORT_ALL, 0xFFFFFFFF);
    bsp_cmd_write(BSP_RGB_FADE_START, BSP_PORT_ALL, 3000);
    TEST_PRINT("RGB disable A"); wait_button();
    bsp_cmd_write(BSP_RGB_CONFIG_ENABLE, BSP_PORT_A, 0);
    TEST_PRINT("RGB disable B"); wait_button();
    bsp_cmd_write(BSP_RGB_CONFIG_ENABLE, BSP_PORT_B, 0);
    TEST_PRINT("RGB disable C"); wait_button();
    bsp_cmd_write(BSP_RGB_CONFIG_ENABLE, BSP_PORT_C, 0);
    TEST_PRINT("RGB disable D"); wait_button();
    bsp_cmd_write(BSP_RGB_CONFIG_ENABLE, BSP_PORT_D, 0);
    // Test motor
    TEST_PRINT("--- Test Motor ---");
    TEST_PRINT("Spin A"); wait_button();
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_A, 50);
    TEST_PRINT("Brake A 100%"); wait_button();
    bsp_cmd_write(BSP_DC_BRAKE, BSP_PORT_A, 100);
    TEST_PRINT("Spin B"); wait_button();
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_B, 50);
    TEST_PRINT("Brake B 15%"); wait_button();
    bsp_cmd_write(BSP_DC_BRAKE, BSP_PORT_B, 15);
    TEST_PRINT("Spin C"); wait_button();
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_C, 50);
    TEST_PRINT("Stop C"); wait_button();
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_C, 0);
    TEST_PRINT("Spin D"); wait_button();
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_D, 50);
    TEST_PRINT("Beep CD"); wait_button();
    bsp_cmd_write(BSP_DC_TONE, BSP_PORT_D, 1000);
    TEST_PRINT("Beep AB"); wait_button();
    bsp_cmd_write(BSP_DC_TONE, BSP_PORT_B, 1000);
    bsp_cmd_write(BSP_DC_TONE, BSP_PORT_D, 0);
    TEST_PRINT("Stop"); wait_button();
    bsp_cmd_write(BSP_DC_TONE, BSP_PORT_A, 0);
    TEST_PRINT("AB spin frequency 50Hz"); wait_button();
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_A, 50);
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_B, 50);
    TEST_PRINT("AB spin frequency 2000Hz"); wait_button();
    bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_B, 2000);
    TEST_PRINT("DC spin frequency 50Hz"); wait_button();
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_ALL, 0);
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_C, 50);
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_D, 50);
    TEST_PRINT("DC spin frequency 2000Hz"); wait_button();
    bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_C, 2000);
    TEST_PRINT("DC spin all 50Hz"); wait_button();
    bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_ALL, 50);
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_ALL, 50);
    TEST_PRINT("DC brake all"); wait_button();
    bsp_cmd_write(BSP_DC_BRAKE, BSP_PORT_ALL, 100);
    TEST_PRINT("DC spin all"); wait_button();
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_ALL, 50);
    TEST_PRINT("DC A disable"); wait_button();
    bsp_cmd_write(BSP_DC_CONFIG_ENABLE, BSP_PORT_A, 0);
    TEST_PRINT("DC B disable"); wait_button();
    bsp_cmd_write(BSP_DC_CONFIG_ENABLE, BSP_PORT_B, 0);
    TEST_PRINT("DC C disable"); wait_button();
    bsp_cmd_write(BSP_DC_CONFIG_ENABLE, BSP_PORT_C, 0);
    TEST_PRINT("DC D disable"); wait_button();
    bsp_cmd_write(BSP_DC_CONFIG_ENABLE, BSP_PORT_D, 0);
    TEST_PRINT("Try spin disabled"); wait_button();
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_ALL, 100);
    TEST_PRINT("Done");
    // Test motor
    TEST_PRINT("--- Test Servo ---");
    TEST_PRINT("Spin A to 500"); wait_button();
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_A, 500);
    TEST_PRINT("Spin A to 1500"); wait_button();
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_A, 1500);
    TEST_PRINT("Spin B to 500"); wait_button();
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_B, 500);
    TEST_PRINT("Spin B to 1500"); wait_button();
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_B, 1500);
    TEST_PRINT("Spin C to 500"); wait_button();
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_C, 500);
    TEST_PRINT("Spin C to 1500"); wait_button();
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_C, 1500);
    TEST_PRINT("Spin All to 500"); wait_button();
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_ALL, 500);
    TEST_PRINT("Spin All to 1500"); wait_button();
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_ALL, 1500);
    TEST_PRINT("5V off"); wait_button();
    bsp_cmd_write(BSP_5V_STATE, 0, 0); // Motors should stop holding position
    TEST_PRINT("5V on"); wait_button();
    bsp_cmd_write(BSP_5V_STATE, 0, 1); // Motors should go back to position
    TEST_PRINT("Done");
    // Test servo speed
    TEST_PRINT("--- Test Servo Speed ---");
    TEST_PRINT("Spin A to 500"); wait_button();
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_A, 500);
    TEST_PRINT("Spin A to 1500 in 1s"); wait_button();
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_A, 1000 << 16 | 1500);
    TEST_PRINT("Spin B to 500"); wait_button();
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_B, 500);
    TEST_PRINT("Spin B to 1500 in 1s"); wait_button();
    bsp_cmd_write(BSP_SERVO_CONFIG_SPEED, BSP_PORT_B, 15 * 60); // Set 15 RPM
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_B, 1500);
    TEST_PRINT("Spin B to 500 in 2s"); wait_button();
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_B, 2000 << 16 | 500);
    TEST_PRINT("Done");
    // Test servo enable
    TEST_PRINT("--- Test Servo Enable ---");
    TEST_PRINT("Spin All to 500"); wait_button();
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_ALL, 500);
    TEST_PRINT("Disable A and Spin All to 1500"); wait_button();
    bsp_cmd_write(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_A, 0);
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_ALL, 1500);
    TEST_PRINT("Disable B and Spin All to 500"); wait_button();
    bsp_cmd_write(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_B, 0);
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_ALL, 500);
    TEST_PRINT("Disable C and Spin All to 1500"); wait_button();
    bsp_cmd_write(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_C, 0);
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_ALL, 1500);
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
