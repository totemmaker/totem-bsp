/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "icm20689.h"
#include "driver/i2c.h"

enum { // Accelerometer full-scale
    ICM20689_ACC_SCALE_2g,
    ICM20689_ACC_SCALE_4g,
    ICM20689_ACC_SCALE_8g,
    ICM20689_ACC_SCALE_16g,
};

enum { // Gyroscope full-scale
    ICM20689_GYRO_SCALE_250dps,
    ICM20689_GYRO_SCALE_500dps,
    ICM20689_GYRO_SCALE_1000dps,
    ICM20689_GYRO_SCALE_2000dps,
};


#define ICM20689_ADDR       0x68

#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

static float accel_factor;
static float gyro_factor;

static esp_err_t write_reg(uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = { reg, value };
    return i2c_master_write_to_device(CONFIG_BSP_I2C_NUM, ICM20689_ADDR, buffer, sizeof(buffer), 100);
}

static esp_err_t read_reg(uint8_t reg, uint8_t *data, uint32_t len) {
    return i2c_master_write_read_device(CONFIG_BSP_I2C_NUM, ICM20689_ADDR, &reg, 1, data, len, 100);
}

esp_err_t icm20689_reset(void) {
    // Perform soft reset
    return write_reg(REG_PWR_MGMT_1, 0x80);
}

esp_err_t icm20689_init(void) {
    esp_err_t err;
    uint8_t result = 0;

    write_reg(REG_PWR_MGMT_1, 0x00); // Wake up & select internal oscillator
    // Read identification register
    err = read_reg(REG_WHO_AM_I, &result, 1);
    // Check for communication error
    if (err) return err;
    // Validate identification
    if (result != 0x98) return ESP_ERR_NOT_SUPPORTED;
    return err;
}

esp_err_t icm20689_set_accel(uint16_t range) {
    uint8_t rBits;
    switch (range) {
        case 2: rBits = ICM20689_ACC_SCALE_2g; break;
        case 4: rBits = ICM20689_ACC_SCALE_4g; break;
        case 8: rBits = ICM20689_ACC_SCALE_8g; break;
        case 16: rBits = ICM20689_ACC_SCALE_16g; break;
        default: return ESP_ERR_INVALID_ARG;
    }
    accel_factor = (float)range / 32767;
    return write_reg(REG_ACCEL_CONFIG, rBits << 3);
}
esp_err_t icm20689_set_gyro(uint16_t range) {
    uint8_t rBits;
    switch (range) {
        case 250: rBits = ICM20689_GYRO_SCALE_250dps; break;
        case 500: rBits = ICM20689_GYRO_SCALE_500dps; break;
        case 1000: rBits = ICM20689_GYRO_SCALE_1000dps; break;
        case 2000: rBits = ICM20689_GYRO_SCALE_2000dps; break;
        default: return ESP_ERR_INVALID_ARG;
    }
    gyro_factor = (float)range / 32767;
    return write_reg(REG_GYRO_CONFIG, rBits << 3);
}

#define GET_REG_VALUE(buf, idx) ((((int16_t)buf[idx*2]) << 8) | buf[(idx*2)+1])

esp_err_t icm20689_read(icm20689_data_t *data) {
    esp_err_t err;
    uint8_t buffer[7*2];
    err = read_reg(REG_ACCEL_XOUT_H, (uint8_t*)&buffer, sizeof(buffer));
    int16_t accX = GET_REG_VALUE(buffer, 0);
    int16_t accY = GET_REG_VALUE(buffer, 1);
    int16_t accZ = GET_REG_VALUE(buffer, 2);
    int16_t temp = GET_REG_VALUE(buffer, 3);
    int16_t gyroX = GET_REG_VALUE(buffer, 4);
    int16_t gyroY = GET_REG_VALUE(buffer, 5);
    int16_t gyroZ = GET_REG_VALUE(buffer, 6);

    data->temp = (temp / 326.8f) + 25;
    data->gyro.x = gyroX * gyro_factor;
    data->gyro.y = gyroY * gyro_factor;
    data->gyro.z = gyroZ * gyro_factor;
    data->accel.x = accX * accel_factor;
    data->accel.y = accY * accel_factor;
    data->accel.z = accZ * accel_factor;
    return err;
}
