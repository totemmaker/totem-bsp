/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <cmath>
#include "totem_test.h"
#include "bsp/roboboard_x4.h"
#include "bsp/roboboard_x4.hpp"

TEST_CASE("Read IMU data", "[IMU][C]") {
    // Wait for IMU to settle
    TEST_DELAY(500);
    // Read IMU data
    BspIMU_data_t data;
    TEST_ERROR(ESP_OK, bsp_imu_read(&data));
    // Test temperature
    TEST_LIMIT(21, 45, (int)data.temp); // Should be around 25C
    // Test accelerometer
    TEST_FLOAT(0, 0.07, data.accel.x); // Should be around 0.0. May drift to 0.07
    TEST_FLOAT(0, 0.07, data.accel.y); // Should be around 0.0. May drift to 0.07
    TEST_FLOAT(1, 0.07, data.accel.z); // Should be around 1.0. May drift to 1.07
    // Test gyroscope
    TEST_FLOAT(0, 3.00, data.gyro.x); // Should be around 0.0. May drift to 3.00
    TEST_FLOAT(0, 3.00, data.gyro.y); // Should be around 0.0. May drift to 3.00
    TEST_FLOAT(0, 3.00, data.gyro.z); // Should be around 0.0. May drift to 3.00
}

TEST_CASE("Read IMU data", "[IMU][C++]") {
    auto imu = X4.imu.read();
    // Test temperature
    TEST_EQUALF(imu.getTemp(), imu.getTempC());
    TEST_LIMIT(21, 45, (int)imu.getTemp()); // Should be around 25C
    TEST_LIMIT(70, 113, (int)imu.getTempF()); // Should be around 77F
    // Test accelerometer
    TEST_FLOAT(0, 0.07, imu.getX_G()); // Should be around 0.0. May drift to 0.07
    TEST_FLOAT(0, 0.07, imu.getY_G()); // Should be around 0.0. May drift to 0.07
    TEST_FLOAT(1, 0.07, imu.getZ_G()); // Should be around 1.0. May drift to 1.07
    TEST_FLOAT(0, 0.60, imu.getX_mss()); // Should be around 0.0. May drift to 0.6
    TEST_FLOAT(0, 0.60, imu.getY_mss()); // Should be around 0.0. May drift to 0.6
    TEST_FLOAT(9.8, 0.60, imu.getZ_mss()); // Should be around 9.8. May drift to 10.4
    // Test gyroscope
    TEST_FLOAT(0, 3.00, imu.getX_dps()); // Should be around 0.0. May drift to 3.00
    TEST_FLOAT(0, 3.00, imu.getY_dps()); // Should be around 0.0. May drift to 3.00
    TEST_FLOAT(0, 3.00, imu.getZ_dps()); // Should be around 0.0. May drift to 3.00
    TEST_FLOAT(0, 0.07, imu.getX_rads()); // Should be around 0.0. May drift to 0.07
    TEST_FLOAT(0, 0.07, imu.getY_rads()); // Should be around 0.0. May drift to 0.07
    TEST_FLOAT(0, 0.07, imu.getZ_rads()); // Should be around 0.0. May drift to 0.07
    TEST_FLOAT(0, 0.50, imu.getX_rpm()); // Should be around 0.0. May drift to 0.50
    TEST_FLOAT(0, 0.50, imu.getY_rpm()); // Should be around 0.0. May drift to 0.50
    TEST_FLOAT(0, 0.50, imu.getZ_rpm()); // Should be around 0.0. May drift to 0.50
    // Test pitch roll
    TEST_FLOAT(0, 4.0, imu.getPitch()); // Should be around 0.0. May drift to 4.0
    TEST_FLOAT(0, 4.0, imu.getRoll()); // Should be around 0.0. May drift to 4.0
    TEST_FLOAT(0, 4.0, imu.getOrientX()); // Should be around 0.0. May drift to 4.0
    TEST_FLOAT(0, 3.0, imu.getOrientY()); // Should be around 0.0. May drift to 3.0
}