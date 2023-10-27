/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "lsm6ds3.h"

#include "driver/i2c.h"

enum { // Accelerometer full-scale
    LSM6DS3_ACC_SCALE_2g,
    LSM6DS3_ACC_SCALE_16g,
    LSM6DS3_ACC_SCALE_4g,
    LSM6DS3_ACC_SCALE_8g,
};

enum { // Gyroscope full-scale
    LSM6DS3_GYRO_SCALE_245dps,
    LSM6DS3_GYRO_SCALE_500dps,
    LSM6DS3_GYRO_SCALE_1000dps,
    LSM6DS3_GYRO_SCALE_2000dps,
};

#define LSM6DS3_ADDR    0x6A

#define REG_WHO_AM_I    0x0F
#define REG_CTRL1_XL    0x10
#define REG_CTRL2_G     0x11
#define REG_OUT_TEMP_L  0x20

static float accel_factor;
static float gyro_factor;

static esp_err_t write_reg(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = { reg, value };
    return i2c_master_write_to_device(CONFIG_BSP_I2C_NUM, LSM6DS3_ADDR, buffer, sizeof(buffer), 100);
}

static esp_err_t read_reg(uint8_t reg, uint8_t *data, uint32_t len) {
    return i2c_master_write_read_device(CONFIG_BSP_I2C_NUM, LSM6DS3_ADDR, &reg, 1, data, len, 100);
}

esp_err_t lsm6ds3_init(void) {
    esp_err_t err;
    uint8_t result = 0;
    // Read identification register
    err = read_reg(REG_WHO_AM_I, &result, 1);
    if (err) return err;
    // Validate identification
    if (result != 0x69) return ESP_ERR_NOT_SUPPORTED;
    return ESP_OK;
}

esp_err_t lsm6ds3_set_accel(uint16_t range, lsm6ds3_accel_rate_t rate, lsm6ds3_accel_filter_t filter) {
    uint8_t rBits;
    switch (range) {
        case 2: rBits = LSM6DS3_ACC_SCALE_2g; break;
        case 4: rBits = LSM6DS3_ACC_SCALE_4g; break;
        case 8: rBits = LSM6DS3_ACC_SCALE_8g; break;
        case 16: rBits = LSM6DS3_ACC_SCALE_16g; break;
        default: return ESP_ERR_INVALID_ARG;
    }
    accel_factor = (float)range / 32767;
    return write_reg(REG_CTRL1_XL, rate << 4 | rBits << 2 | filter);
}
esp_err_t lsm6ds3_set_gyro(uint16_t range, lsm6ds3_gyro_rate_t rate) {
    uint8_t rBits;
    switch (range) {
        case 245: rBits = LSM6DS3_GYRO_SCALE_245dps; break;
        case 500: rBits = LSM6DS3_GYRO_SCALE_500dps; break;
        case 1000: rBits = LSM6DS3_GYRO_SCALE_1000dps; break;
        case 2000: rBits = LSM6DS3_GYRO_SCALE_2000dps; break;
        default: return ESP_ERR_INVALID_ARG;
    }
    gyro_factor = (float)range / 32767;
    return write_reg(REG_CTRL2_G, rate << 4 | rBits << 2);
}

esp_err_t lsm6ds3_read(lsm6ds3_data_t *data) {
    esp_err_t err;
    int16_t buffer[7] = {0};
    err = read_reg(REG_OUT_TEMP_L, (uint8_t*)&buffer, sizeof(buffer));
    int16_t temp = buffer[0];
    int16_t gyroX = buffer[1];
    int16_t gyroY = buffer[2];
    int16_t gyroZ = buffer[3];
    int16_t accX = buffer[4];
    int16_t accY = buffer[5];
    int16_t accZ = buffer[6];

    data->temp = (temp / 16.0) + 25.0;
    data->gyro.x = gyroX * gyro_factor;
    data->gyro.y = gyroY * gyro_factor;
    data->gyro.z = gyroZ * gyro_factor;
    data->accel.x = accX * accel_factor;
    data->accel.y = accY * accel_factor;
    data->accel.z = accZ * accel_factor;
    return err;
}
