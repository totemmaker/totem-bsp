/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "driver/i2c.h"

#include "lib/qmi8658.h"

#include "bsp/imu.h"
#include "bsp/roboboard_x3.h"

enum {
    IMU_DRIVER_NONE,
    IMU_DRIVER_QMI8658,
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

static bsp_imu_param_t imu_param = { 16, 2048, 125, 125 };
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
    if (bsp_board_get_revision() >= 30) {
        imu_type = IMU_DRIVER_QMI8658;
    }
    // Initialize IMU
    switch (imu_type) {
        case IMU_DRIVER_QMI8658: {
            err = qmi8658_init(&i2c);
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
        case IMU_DRIVER_QMI8658: {
            err = qmi8658_set_accel(&i2c, &(param->acc_range), &(param->acc_rate));
            err = qmi8658_set_gyro(&i2c, &(param->gyro_range), &(param->gyro_rate));
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
        case IMU_DRIVER_QMI8658: {
            err = qmi8658_read(&i2c, (imu_data_t*)data);
            // Flip measurements relative to board perspective
            data->accel.x = -data->accel.x;
            data->accel.y = -data->accel.y;
            data->gyro.x = -data->gyro.x;
            data->gyro.y = -data->gyro.y;
            break;
        }
        default: return ESP_ERR_NOT_FOUND;
    }
    return err;
}
