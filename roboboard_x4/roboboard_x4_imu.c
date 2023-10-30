/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "driver/i2c.h"

#include "bsp/imu.h"
#include "bsp/roboboard_x4.h"

#include "lib/icm20689.h"
#include "lib/lsm6ds3.h"

static int bsp_board_revision;

esp_err_t bsp_i2c_init(int i2c_num, uint32_t frequency) {
    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_IO_I2C_SDA,
        .scl_io_num = BSP_IO_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = frequency,
        .clk_flags = 0
    };
    esp_err_t err;
    err = i2c_param_config(i2c_num, &conf);
    if (err) return err;
    err = i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);
    return err;
}
esp_err_t bsp_imu_init() {
    esp_err_t err = ESP_FAIL;
    bsp_board_revision = bsp_cmd_read(BSP_BOARD_REVISION, 0);
    // Initialize IMU
    if (bsp_board_revision == 11) {
        // Reset icm20689 IC
        err = icm20689_reset();
        if (err) return err;
        vTaskDelay(pdMS_TO_TICKS(100));
        err = icm20689_init();
    }
    else {
        err = lsm6ds3_init();
    }
    // Set measurement range to max
    bsp_imu_set_accel_range(16);
    bsp_imu_set_gyro_range(2000);
    return err;
}
esp_err_t bsp_imu_set_accel_range(uint16_t range) {
    if (bsp_board_revision == 11) {
        return icm20689_set_accel(range);
    }
    else {
        return lsm6ds3_set_accel(range, LSM6DS3_ACCEL_RATE_104Hz, LSM6DS3_ACCEL_FILTER_100Hz);
    }
}
esp_err_t bsp_imu_set_gyro_range(uint16_t range) {
    if (bsp_board_revision == 11) {
        return icm20689_set_gyro(range);
    }
    else {
        if (range == 250) range = 245;
        return lsm6ds3_set_gyro(range, LSM6DS3_GYRO_RATE_104Hz);
    }
}
esp_err_t bsp_imu_read(bsp_imu_data_t *data) {
    // Validate argument
    if (data == NULL) return ESP_ERR_INVALID_ARG;
    esp_err_t err;
    if (bsp_board_revision == 11) {
        err = icm20689_read((icm20689_data_t*)data);
        // Flip measurements relative to board perspective
        data->accel.x = -data->accel.x;
        data->accel.z = -data->accel.z;
        data->gyro.x = -data->gyro.x;
        data->gyro.z = -data->gyro.z;
    }
    else {
        err = lsm6ds3_read((lsm6ds3_data_t*)data);
        // Flip measurements relative to board perspective
        data->accel.y = -data->accel.y;
        data->accel.z = -data->accel.z;
        data->gyro.y = -data->gyro.y;
        data->gyro.z = -data->gyro.z;
    }
    return err;
}
