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

TEST_CASE("Validate initial state", "[dc][C]") {
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_INVERT, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_INVERT, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_INVERT, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_INVERT, BSP_PORT_D));
    TEST_EQUAL(50, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_A)); // AB group
    TEST_EQUAL(50, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_C)); // CD group
}

TEST_CASE("Test write over limits", "[dc][C]") {
    // BSP_DC_POWER
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_DC_POWER, BSP_PORT_ALL, 90));
    TEST_EQUAL(90, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(90, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_B));
    TEST_EQUAL(90, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(90, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_C));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_POWER, BSP_PORT_ALL, -150)); TEST_EQUAL(90, bsp_cmd_read(BSP_DC_POWER, 0));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_POWER, BSP_PORT_ALL, 3000)); TEST_EQUAL(90, bsp_cmd_read(BSP_DC_POWER, 0));
    // BSP_DC_BRAKE
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_DC_BRAKE, BSP_PORT_ALL, 15));
    TEST_EQUAL(15, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(15, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_B));
    TEST_EQUAL(15, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    TEST_EQUAL(15, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_C));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_BRAKE, BSP_PORT_ALL, -101)); TEST_EQUAL(15, bsp_cmd_read(BSP_DC_BRAKE, 0));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_BRAKE, BSP_PORT_ALL, 101)); TEST_EQUAL(15, bsp_cmd_read(BSP_DC_BRAKE, 0));
    // BSP_DC_TONE
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_DC_TONE, BSP_PORT_A, 4000));
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_DC_TONE, BSP_PORT_D, 1000 << 16 | 5000));
    TEST_EQUAL(4000, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_A));
    TEST_EQUAL(4000, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_B));
    TEST_EQUAL(5000, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_C));
    TEST_EQUAL(5000, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_D));
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_DC_TONE, BSP_PORT_ALL, 6000));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_TONE, BSP_PORT_ALL, 20001));
    TEST_EQUAL(6000, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_A));
    TEST_EQUAL(6000, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_B));
    TEST_EQUAL(6000, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_C));
    TEST_EQUAL(6000, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    // BSP_DC_CONFIG_INVERT
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_DC_CONFIG_INVERT, BSP_PORT_ALL, 5)); TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_INVERT, BSP_PORT_A));
    // BSP_DC_CONFIG_FREQUENCY
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_ALL, 250000)); TEST_EQUAL(250000, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_A));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_ALL, 250001)); TEST_EQUAL(250000, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_A));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_ALL, 0)); TEST_EQUAL(250000, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_A));
    // BSP_DC_CONFIG_BRAKE
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_DC_POWER, BSP_PORT_ALL, 0));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_DC_CONFIG_BRAKE, BSP_PORT_ALL, 80)); TEST_EQUAL(80, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_A));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_CONFIG_BRAKE, BSP_PORT_ALL, 101)); TEST_EQUAL(80, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_A));
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_cmd_write(BSP_DC_CONFIG_BRAKE, BSP_PORT_ALL, -1)); TEST_EQUAL(80, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_A));
    TEST_EQUAL(80, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(80, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_B));
    TEST_EQUAL(80, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    TEST_EQUAL(80, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_D));
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_DC_POWER, BSP_PORT_C, 10));
    TEST_EQUAL(80, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(80, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    TEST_EQUAL(80, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_D));
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_DC_POWER, BSP_PORT_C, 0));
    TEST_EQUAL(80, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    // BSP_DC_CONFIG_ENABLE
    TEST_EQUAL(ESP_OK, bsp_cmd_write(BSP_DC_CONFIG_ENABLE, BSP_PORT_ALL, 5)); TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_A));
}

TEST_CASE("Test write over limits", "[dc][C++]") {
    // X4.dc
    X4.dc.disable();
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_D));
    X4.dc.enable();
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_A));
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_B));
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_C));
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_D));
    X4.dc.power(101, -101, 0, 66);
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(-100, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(66, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_D));
    X4.dc.brake(101, -10, 50, 0);
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_B));
    TEST_EQUAL(50, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_D));
    X4.dc.setInvert(true, false, 0, 125);
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_INVERT, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_INVERT, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_INVERT, BSP_PORT_C));
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_INVERT, BSP_PORT_D));
    X4.dc.setAutobrake(0, 66, -101, 101);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_A));
    TEST_EQUAL(66, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_C));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_D));
    X4.dc.setAutobrake(false, false, (bool)90, (bool)0);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_B));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_D));
    X4.dc.setAutobrake(false, false, true, true);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_B));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_C));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_D));
    // X4.dcXX
    X4.dcAB.tone(500);
    X4.dcCD.tone(600);
    TEST_EQUAL(500, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_A)); // AB group
    TEST_EQUAL(500, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_B)); // AB group
    TEST_EQUAL(600, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_C)); // CD group
    TEST_EQUAL(600, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_D)); // CD group
    X4.dcAB.tone(400, 1500);
    X4.dcCD.tone(3000, 1333);
    TEST_EQUAL(400, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_A)); // AB group
    TEST_EQUAL(400, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_B)); // AB group
    TEST_EQUAL(3000, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_C)); // CD group
    TEST_EQUAL(3000, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_D)); // CD group
    X4.dcAB.setFreq(0);
    X4.dcCD.setFreq(250001);
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_A)); // AB group
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_B)); // AB group
    TEST_EQUAL(250000, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_C)); // AB group
    TEST_EQUAL(250000, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_D)); // AB group
    X4.dcAB.setFreq(60);
    X4.dcCD.setFreq(2560);
    TEST_EQUAL(60, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_A)); // AB group
    TEST_EQUAL(60, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_B)); // AB group
    TEST_EQUAL(2560, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_C)); // AB group
    TEST_EQUAL(2560, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_D)); // AB group
    // X4.dcX
    X4.dcA.enable();
    X4.dcB.disable();
    X4.dcC.enable();
    X4.dcD.disable();
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_B));
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_D));
    X4.dc.power(0, 0, 0, 0);
    X4.dcA.power(101);
    X4.dcC.power(-101);
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_B));
    TEST_EQUAL(-100, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_D));
    X4.dc.enable();
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_A));
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_B));
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_C));
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_ENABLE, BSP_PORT_D));
    X4.dcA.brake(-15);
    X4.dcB.brake();
    X4.dcC.coast();
    X4.dcD.brake(101);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_D));
    X4.dcA.setInvert(false);
    X4.dcB.setInvert(true);
    X4.dcC.setInvert(10);
    X4.dcD.setInvert(0);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_INVERT, BSP_PORT_A));
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_INVERT, BSP_PORT_B));
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_CONFIG_INVERT, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_INVERT, BSP_PORT_D));
    X4.dcA.setAutobrake(0);
    X4.dcB.setAutobrake(55);
    X4.dcC.setAutobrake(-101);
    X4.dcD.setAutobrake(101);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_A));
    TEST_EQUAL(55, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_C));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_D));
    X4.dcA.setAutobrake(false);
    X4.dcB.setAutobrake((bool)50);
    X4.dcC.setAutobrake(true);
    X4.dcD.setAutobrake(0);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_A));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_B));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_D));
}

TEST_CASE("Test related values update", "[dc][C++]") {
    // Changing value should affect it's related ones
    // X4.dc
    X4.dc.power(-50, 10, 0, 100);
    TEST_EQUAL(-50, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(10, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_D));
    X4.dc.brake(100, 10, 60, 3);
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(10, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_B));
    TEST_EQUAL(60, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    TEST_EQUAL(3, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_D));
    X4.dcC.power(44);
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(10, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    TEST_EQUAL(3, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_B));
    TEST_EQUAL(44, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_D));
    X4.dc.setAutobrake(15, 30, 40, 60);
    TEST_EQUAL(15, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_A));
    TEST_EQUAL(30, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_B));
    TEST_EQUAL(40, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_C));
    TEST_EQUAL(60, bsp_cmd_read(BSP_DC_CONFIG_BRAKE, BSP_PORT_D));
    TEST_EQUAL(15, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(30, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    TEST_EQUAL(60, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_B));
    TEST_EQUAL(44, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_D));
    X4.dcAB.tone(500);
    X4.dcCD.tone(600);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_D));
    TEST_EQUAL(500, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_A));
    TEST_EQUAL(500, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_B));
    TEST_EQUAL(600, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_C));
    TEST_EQUAL(600, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_D));
    X4.dcB.power(36);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(36, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_B));
    TEST_EQUAL(600, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_C));
    TEST_EQUAL(600, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_D));
    X4.dcC.brake(48);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_B));
    TEST_EQUAL(48, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(36, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_D));
    X4.dcAB.tone(400, 1500);
    X4.dcCD.tone(3000, 1333);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_B));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_D));
    TEST_EQUAL(400, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_A));
    TEST_EQUAL(400, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_B));
    TEST_EQUAL(3000, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_C));
    TEST_EQUAL(3000, bsp_cmd_read(BSP_DC_TONE, BSP_PORT_D));
    X4.dcC.power(1);
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    X4.dcC.setInvert(1);
    TEST_EQUAL(1, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    X4.dcC.brake();
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C)); // Power should be 0 when braking
}

TEST_CASE("Test brake/power update", "[dc][C++]") {
    X4.dcA.setAutobrake(0);
    X4.dcA.brake(0);
    X4.dcA.power(50);
    TEST_EQUAL(50, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    X4.dcA.brake(10);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(10, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    X4.dcA.brake(0);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    X4.dcA.brake(25);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(25, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    X4.dcA.power(15);
    TEST_EQUAL(15, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    X4.dcA.power(0);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    X4.dcA.setAutobrake(20);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(20, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    X4.dcA.power(30);
    X4.dcA.setAutobrake(20);
    TEST_EQUAL(30, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    X4.dcA.power(0);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(20, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
    X4.dcA.setAutobrake(0);
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(0, bsp_cmd_read(BSP_DC_BRAKE, BSP_PORT_A));
}

TEST_CASE("Test write channel group", "[dc][C++]") {
    // DC (groups AB & CD)
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_ALL, 50));
    TEST_ERROR(ESP_OK, bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_A, 100));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 0));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_A));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_B));
    TEST_EQUAL(50, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_C));
    TEST_EQUAL(50, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_D));
    TEST_EQUAL(0, bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_D, 200));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_A));
    TEST_EQUAL(100, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_B));
    TEST_EQUAL(200, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_C));
    TEST_EQUAL(200, bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, BSP_PORT_D));
}

TEST_CASE("Acces C++ port index", "[dc][C++]") {
    // X4.dc[]
    X4.dc[0].power(50);
    X4.dc[1].power(60);
    X4.dc[2].power(70);
    X4.dc[3].power(80);
    TEST_EQUAL(50, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(60, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_B));
    TEST_EQUAL(70, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(80, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_D));
    X4.dc[4].power(13);
    TEST_EQUAL(50, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_A));
    TEST_EQUAL(60, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_B));
    TEST_EQUAL(70, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_C));
    TEST_EQUAL(13, bsp_cmd_read(BSP_DC_POWER, BSP_PORT_D));
    X4.dc.power(0, 0, 0, 0);
}
