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

TEST_CASE("Validate initial state [servo][C]", "[servo][C]") {
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_INVERT, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_INVERT, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_INVERT, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_SPEED, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_SPEED, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_SPEED, BSP_PORT_C));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_B));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_C));
    TEST_EQUAL(500 << 16 | 2500, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_A));
    TEST_EQUAL(500 << 16 | 2500, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_B));
    TEST_EQUAL(500 << 16 | 2500, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_C));
    TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_A));
    TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_B));
    TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_C));
}

TEST_CASE("Test write over limits", "[servo][C]") {
    // BSP_SERVO_PULSE
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_ALL, 1000));
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_B, 900));
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_D, 900)); // servo port D not exist
    TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(900, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_ALL, 2501)); TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_ALL, 499)); TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    // BSP_SERVO_CONFIG_INVERT
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_INVERT, BSP_PORT_ALL, 6)); TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_INVERT, BSP_PORT_A));
    TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    // BSP_SERVO_CONFIG_SPEED
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_SPEED, BSP_PORT_ALL, 80*60)); TEST_EQUAL(80*60, bsp_cmd_read(BSP_SERVO_CONFIG_SPEED, BSP_PORT_A));
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_SPEED, BSP_PORT_ALL, 0)); TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_SPEED, BSP_PORT_A));
    // BSP_SERVO_CONFIG_PERIOD
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_ALL, 499)); TEST_EQUAL(499, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_ALL, 0)); TEST_EQUAL(499, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(499 << 16 | 499, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_A));
    TEST_EQUAL(499 << 16 | 499, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_B));
    TEST_EQUAL(499 << 16 | 499, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_C));
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_ALL, 20000)); TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(499 << 16 | 499, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_A));
    TEST_EQUAL(499 << 16 | 499, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_B));
    TEST_EQUAL(499 << 16 | 499, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_C));
    // BSP_SERVO_CONFIG_RANGE
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_ALL, 10000)); TEST_EQUAL(10000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_RANGE, BSP_PORT_A, 400 << 16 | 900)); TEST_EQUAL(400 << 16 | 900, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_A));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_SERVO_CONFIG_RANGE, BSP_PORT_ALL, 400 << 16 | 10001)); TEST_EQUAL(400 << 16 | 900, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_A));
    TEST_EQUAL(499 << 16 | 499, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_C));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_SERVO_CONFIG_RANGE, BSP_PORT_ALL, 400 << 16 | 399)); TEST_EQUAL(400 << 16 | 900, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_A));
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_RANGE, BSP_PORT_ALL, 400 << 16 | 400)); TEST_EQUAL(400 << 16 | 400, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_A));
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_ALL, 20000)); TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_RANGE, BSP_PORT_ALL, 500 << 16 | 2500));
    TEST_EQUAL(500 << 16 | 2500, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_A));
    TEST_EQUAL(500 << 16 | 2500, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_B));
    TEST_EQUAL(500 << 16 | 2500, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_C));
    // BSP_SERVO_CONFIG_ENABLE
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_ALL, 7)); TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_A));
}

TEST_CASE("Test write over limits", "[servo][C++]") {
    // X4.servo
    X4.servo.disable();
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_C));
    X4.servo.enable();
    TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_A));
    TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_B));
    TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_C));
    X4.servo.setPeriod(1000);
    X4.servoA.pulse(900);
    TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_B));
    TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_C));
    TEST_EQUAL(900, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    X4.servo.setPeriod(950);
    TEST_EQUAL(950, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(950, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_B));
    TEST_EQUAL(950, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_C));
    TEST_EQUAL(900, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    X4.servo.setPeriod(800);
    TEST_EQUAL(800, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(800, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_B));
    TEST_EQUAL(800, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_C));
    TEST_EQUAL(800, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(500 << 16 | 800, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_A));
    TEST_EQUAL(500 << 16 | 800, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_B));
    TEST_EQUAL(500 << 16 | 800, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_C));
    X4.servo.setPeriod(65535 + 1);
    TEST_EQUAL(0xFFFF, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(0xFFFF, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_B));
    TEST_EQUAL(0xFFFF, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_C));
    X4.servo.setPeriod(0);
    TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_B));
    TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_C));
    X4.servo.setPeriod(20000);
    X4.servoA.setPulseMinMax(500, 2500);
    X4.servoB.setPulseMinMax(500, 2500);
    X4.servoC.setPulseMinMax(500, 2500);
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_B));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_C));
    X4.servo.pos(-101, 0, 101);
    TEST_EQUAL(500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(1500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(2500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    X4.servoA.setPulseMinMax(400, 2600);
    X4.servoB.setPulseMinMax(400, 2600);
    X4.servoC.setPulseMinMax(400, 2600);
    X4.servo.pos(-100, 0, 100);
    TEST_EQUAL(400, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(1500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(2600, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    X4.servoA.setPulseMinMax(500, 2500);
    X4.servoB.setPulseMinMax(500, 2500);
    X4.servoC.setPulseMinMax(500, 2500);
    X4.servo.angle(181, 90, 0);
    TEST_EQUAL(2500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(1500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    X4.servo.pulse(499, 1234, 2501);
    TEST_EQUAL(500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(1234, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(2500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    // X4.servoX
    X4.servoA.enable();
    X4.servoB.disable();
    X4.servoC.enable();
    TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_B));
    TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_C));
    X4.servoB.pos(-101);
    TEST_EQUAL(500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    X4.servo.enable();
    X4.servoA.pos(-101);
    X4.servoB.pos(50, 600);
    X4.servoC.pos(101, 0);
    TEST_EQUAL(500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(2000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(2500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    X4.servoA.angle(181, 0);
    X4.servoB.angle(50, 600);
    X4.servoC.angle(0);
    TEST_EQUAL(2500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(1055, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    X4.servoA.pulse(499);
    X4.servoB.pulse(700, 600);
    X4.servoC.pulse(2501, 0);
    TEST_EQUAL(500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(700, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(2500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    X4.servoA.setInvert(10);
    X4.servoB.setInvert(0);
    X4.servoC.setInvert(true);
    TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_INVERT, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_INVERT, BSP_PORT_B));
    TEST_EQUAL(1, bsp_cmd_read(BSP_SERVO_CONFIG_INVERT, BSP_PORT_C));
    X4.servoA.setSpeedRPM(0);
    X4.servoB.setSpeedRPM(0.5);
    X4.servoC.setSpeedRPM(1000);
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_SPEED, BSP_PORT_A));
    TEST_EQUAL(30, bsp_cmd_read(BSP_SERVO_CONFIG_SPEED, BSP_PORT_B));
    TEST_EQUAL(1000*60, bsp_cmd_read(BSP_SERVO_CONFIG_SPEED, BSP_PORT_C));
    X4.servoA.setSpeedS60(0);
    X4.servoB.setSpeedS60(0.20); // (1 / (0.2 * 6)) * 60 = 50
    X4.servoC.setSpeedS60(1.5); // (1 / (1.5 * 6)) * 60 = 6.66666
    TEST_EQUAL(0, bsp_cmd_read(BSP_SERVO_CONFIG_SPEED, BSP_PORT_A));
    TEST_EQUAL(50*60, bsp_cmd_read(BSP_SERVO_CONFIG_SPEED, BSP_PORT_B));
    TEST_EQUAL(400, bsp_cmd_read(BSP_SERVO_CONFIG_SPEED, BSP_PORT_C));
    X4.servoA.setPulseMinMax(700, 600);
    X4.servoB.setPulseMinMax(465, 1235);
    X4.servoC.setPulseMinMax(0, 20001);
    TEST_EQUAL(600 << 16 | 600, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_A));
    TEST_EQUAL(465 << 16 | 1235, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_B));
    TEST_EQUAL(0 << 16 | 20000, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_C));
    X4.servoA.setPulseMinMax(500, 2500);
    X4.servoB.setPulseMinMax(500, 2500);
    X4.servoC.setPulseMinMax(500, 2500);
    TEST_EQUAL(500 << 16 | 2500, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_A));
    TEST_EQUAL(500 << 16 | 2500, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_B));
    TEST_EQUAL(500 << 16 | 2500, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, BSP_PORT_C));
}

TEST_CASE("Test related values update", "[C++]") {
    X4.servoA.setPulseMinMax(500, 2500);
    X4.servo.setPeriod(10000);
    TEST_EQUAL(10000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0));
    X4.servo.setPeriod(20000);
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0));
    TEST_EQUAL(500 << 16 | 2500, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, 0));
    X4.servoA.pulse(200);
    X4.servoB.pulse(500);
    X4.servoC.pulse(4000);
    TEST_EQUAL(500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(2500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    // Can't be higher than period
    X4.servo.setPeriod(1000);
    TEST_EQUAL(500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    TEST_EQUAL(500 << 16 | 1000, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, 0));
    X4.servo.setPeriod(20000);
    X4.servoB.pulse(1001);
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0));
    TEST_EQUAL(500 << 16 | 1000, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, 0));
    TEST_EQUAL(500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    X4.servoA.setPulseMinMax(500, 2500);
    X4.servoB.setPulseMinMax(500, 2500);
    X4.servoC.setPulseMinMax(500, 2500);
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0));
    TEST_EQUAL(500 << 16 | 2500, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, 0));
    TEST_EQUAL(500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    X4.servoA.pulse(300);
    X4.servoB.pulse(800);
    X4.servoC.pulse(2600);
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0));
    TEST_EQUAL(500 << 16 | 2500, bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, 0));
    TEST_EQUAL(500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(800, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(2500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
}

TEST_CASE("Test get servo position", "[servo][C++]") {
    X4.servoA.setPulseMinMax(500, 2500);
    X4.servoB.setPulseMinMax(500, 2500);
    X4.servoC.setPulseMinMax(500, 2500);

    for (int i = -100; i<100; i+=15) {
        X4.servoA.pos(i);
        X4.servoC.pos(i+1);
        TEST_EQUAL(i, X4.servoA.getPos());
        TEST_EQUAL(i+1, X4.servoC.getPos());
    }

    for (int i = 0; i<180; i+=15) {
        X4.servoA.angle(i);
        TEST_LIMIT(i-1, i, X4.servoA.getAngle());
    }
}

TEST_CASE("Test write channel group", "[servo][C]") {
    // Servo (single group ABC, ignores channels)
    TEST_EQUAL(0, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_ALL, 20000));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_B));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_C));
    TEST_EQUAL(20000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_D));

    TEST_EQUAL(0, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, 0, 15000));
    TEST_EQUAL(15000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0));
    TEST_EQUAL(15000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(15000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_B));
    TEST_EQUAL(15000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_C));
    TEST_EQUAL(15000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_D));

    TEST_EQUAL(0, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A, 13000));
    TEST_EQUAL(13000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0));
    TEST_EQUAL(13000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(13000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_B));
    TEST_EQUAL(13000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_C));
    TEST_EQUAL(13000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_D));

    TEST_EQUAL(0, bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_D, 11000));
    TEST_EQUAL(11000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0));
    TEST_EQUAL(11000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_A));
    TEST_EQUAL(11000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_B));
    TEST_EQUAL(11000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_C));
    TEST_EQUAL(11000, bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_D));
}

TEST_CASE("Test range update", "[servo][C++]") {
    X4.servo.setPeriod(20000);
    X4.servoA.setPulseMinMax(0, 20000);
    X4.servoB.setPulseMinMax(0, 20000);
    X4.servoC.setPulseMinMax(0, 20000);
    X4.servoA.pulse(4000);
    X4.servoB.pulse(3000);
    X4.servoC.pulse(1000);
    TEST_EQUAL(4000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(3000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    X4.servo.setPeriod(1500);
    TEST_EQUAL(1500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(1500, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(1000, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
}

TEST_CASE("Test speed", "[servo][C++]") {
    X4.servoA.angle(0);
    X4.servoA.setSpeedRPM(30);
    X4.servoA.angle(180);
    TEST_DELAY(30);
    TEST_MAX(180, X4.servoA.getAngle());
}

TEST_CASE("Acces C++ port index", "[servo][C++]") {
    // X4.servo[]
    X4.servo[0].pulse(510);
    X4.servo[1].pulse(620);
    X4.servo[2].pulse(730);
    TEST_EQUAL(510, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(620, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(730, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    X4.servo[3].pulse(840);
    TEST_EQUAL(510, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_A));
    TEST_EQUAL(620, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_B));
    TEST_EQUAL(840, bsp_cmd_read(BSP_SERVO_PULSE, BSP_PORT_C));
    X4.servo.pos(0, 0, 0);
}
