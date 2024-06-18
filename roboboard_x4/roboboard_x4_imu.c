/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "driver/i2c.h"

#include "lib/icm20689.h"
#include "lib/lsm6ds3.h"

#include "bsp/imu.h"
#include "bsp/roboboard_x4.h"

enum {
    IMU_DRIVER_NONE,
    IMU_DRIVER_LSM6DS3,
    IMU_DRIVER_ICM20689,
};

// I2C control functions
static imu_i2c_t i2c;
static int i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = { reg, value };
    return i2c_master_write_to_device(i2c.num, addr, buffer, sizeof(buffer), 50);
}
static int i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len) {
    return i2c_master_write_read_device(i2c.num, addr, &reg, 1, data, len, 50);
}
// I2C control interface
static imu_i2c_t i2c = {
    .write_reg = i2c_write_reg,
    .read_reg = i2c_read_reg,
};

static bsp_imu_param_t imu_param = { 16, 2000, 100, 104, 104 };
static uint8_t imu_type;

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
esp_err_t bsp_imu_init(int i2c_num) {
    if (i2c_num < 0 || i2c_num >= I2C_NUM_MAX) return ESP_ERR_INVALID_ARG;
    esp_err_t err = ESP_FAIL;
    i2c.num = i2c_num;
    // Find driver type
    switch (bsp_board_get_revision()) {
        case 11: imu_type = IMU_DRIVER_ICM20689; break;
        case 10: imu_type = IMU_DRIVER_LSM6DS3; break;
    }
    // Initialize IMU
    switch (imu_type) {
        case IMU_DRIVER_LSM6DS3: {
            err = lsm6ds3_init(&i2c);
            break;
        }
        case IMU_DRIVER_ICM20689: {
            icm20689_reset(&i2c);
            vTaskDelay(pdMS_TO_TICKS(100));
            err = icm20689_init(&i2c);
            break;
        }
        default: return ESP_ERR_NOT_FOUND;
    }
    if (err) return err;
    // Configure IMU
    return bsp_imu_set_param(&imu_param);
}
bsp_imu_param_t* bsp_imu_get_param() {
    return &imu_param;
}
esp_err_t bsp_imu_set_param(bsp_imu_param_t *param) {
    // Validate argument
    if (!param) return ESP_ERR_INVALID_ARG;
    esp_err_t err = ESP_FAIL;
    switch (imu_type) {
        case IMU_DRIVER_LSM6DS3: {
            err = lsm6ds3_set_accel(&i2c, &(param->acc_range), &(param->acc_rate), &(param->acc_filter));
            err = lsm6ds3_set_gyro(&i2c, &(param->gyro_range), &(param->gyro_rate));
            break;
        }
        case IMU_DRIVER_ICM20689: {
            param->acc_rate = 0;
            param->gyro_rate = 0;
            param->acc_filter = 0;
            err = icm20689_set_accel(&i2c, &(param->acc_range));
            err = icm20689_set_gyro(&i2c, &(param->gyro_range));
            break;
        }
        default: return ESP_ERR_NOT_FOUND;
    }
    return err;
}
esp_err_t bsp_imu_read(bsp_imu_data_t *data) {
    // Validate argument
    if (data == NULL) return ESP_ERR_INVALID_ARG;
    esp_err_t err = ESP_FAIL;
    switch (imu_type) {
        case IMU_DRIVER_LSM6DS3: {
            err = lsm6ds3_read(&i2c, (imu_data_t*)data);
            // Flip measurements relative to board perspective
            data->accel.y = -data->accel.y;
            data->accel.z = -data->accel.z;
            data->gyro.y = -data->gyro.y;
            data->gyro.z = -data->gyro.z;
            break;
        }
        case IMU_DRIVER_ICM20689: {
            err = icm20689_read(&i2c, (imu_data_t*)data);
            // Flip measurements relative to board perspective
            data->accel.x = -data->accel.x;
            data->accel.z = -data->accel.z;
            data->gyro.x = -data->gyro.x;
            data->gyro.z = -data->gyro.z;
            break;
        }
        default: return ESP_ERR_NOT_FOUND;
    }
    return err;
}
