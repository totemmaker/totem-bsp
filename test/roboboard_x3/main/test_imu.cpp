/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <cmath>
#include "totem_test.h"
#include "bsp/totem-bsp.h"

TEST_CASE("Read IMU data", "[IMU]") {
    bsp_imu_data_t data;
    // Test if non-initialized state is validated
    TEST_ERROR(ESP_ERR_NOT_FOUND, bsp_imu_set_param(bsp_imu_get_param()));
    TEST_ERROR(ESP_ERR_NOT_FOUND, bsp_imu_read(&data));
    // Initialize I2C_NUM_0 at 400Khz
    TEST_ERROR(ESP_OK, bsp_i2c_init(0, 400000));
    // Initialize IMU sensor at I2C_NUM_0
    TEST_ERROR(ESP_OK, bsp_imu_init(0));
    // Wait for IMU to settle
    TEST_DELAY(500);
    // Read IMU data
    TEST_ERROR(ESP_OK, bsp_imu_read(&data));
    // Test temperature
    TEST_LIMIT(21, 32, (int)data.temp); // Should be around 25C
    // Test accelerometer
    TEST_FLOAT(0, 0.17, data.accel.x); // Should be around 0.0. May drift to 0.17
    TEST_FLOAT(0, 0.17, data.accel.y); // Should be around 0.0. May drift to 0.17
    TEST_FLOAT(1, 0.17, data.accel.z); // Should be around 1.0. May drift to 1.17
    // Test gyroscope
    TEST_FLOAT(0, 1.50, data.gyro.x); // Should be around 0.0. May drift to 1.50
    TEST_FLOAT(0, 1.50, data.gyro.y); // Should be around 0.0. May drift to 1.50
    TEST_FLOAT(0, 1.50, data.gyro.z); // Should be around 0.0. May drift to 1.50
}

TEST_CASE("Test IMU API", "[IMU]") {
    // Test if I2C number if validated
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_imu_init(2));
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_imu_init(-1));
    // Test if data ptr is validated
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_imu_read(NULL));
    // Get IMU parameters pointer
    bsp_imu_param_t *param = bsp_imu_get_param();
    TEST_ASSERT(param != NULL);
    // Check for default values
    TEST_EQUAL(16, param->acc_range);
    TEST_EQUAL(2048, param->gyro_range);
    TEST_EQUAL(125, param->acc_rate);
    TEST_EQUAL(125, param->gyro_rate);
    // Test parameter setting and limits
    TEST_ERROR(ESP_OK, bsp_imu_set_param(param));
    param->acc_range = 0;
    param->gyro_range = 0;
    param->acc_rate = 0;
    param->gyro_rate = 0;
    TEST_ERROR(ESP_OK, bsp_imu_set_param(param));
    TEST_EQUAL(2, param->acc_range);
    TEST_EQUAL(16, param->gyro_range);
    TEST_EQUAL(31, param->acc_rate);
    TEST_EQUAL(31, param->gyro_rate);
    param->acc_range = 17;
    param->gyro_range = 3000;
    param->acc_rate = 9000;
    param->gyro_rate = 8500;
    TEST_ERROR(ESP_OK, bsp_imu_set_param(param));
    TEST_EQUAL(16, param->acc_range);
    TEST_EQUAL(2048, param->gyro_range);
    TEST_EQUAL(8000, param->acc_rate);
    TEST_EQUAL(8000, param->gyro_rate);
    param->acc_range = 5;
    param->gyro_range = 800;
    param->acc_rate = 1400;
    param->gyro_rate = 550;
    TEST_ERROR(ESP_OK, bsp_imu_set_param(param));
    TEST_EQUAL(4, param->acc_range);
    TEST_EQUAL(1024, param->gyro_range);
    TEST_EQUAL(1000, param->acc_rate);
    TEST_EQUAL(500, param->gyro_rate);
    // Test if IMU still works
    bsp_imu_data_t data;
    TEST_ERROR(ESP_OK, bsp_imu_read(&data));
    TEST_LIMIT(21, 30, (int)data.temp); // Should be around 25C
}