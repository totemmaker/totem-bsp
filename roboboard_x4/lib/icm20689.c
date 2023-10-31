/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "icm20689.h"

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

enum { // Gyroscope output data rate
    ICM20689_GYRO_RATE_8000Hz,
    ICM20689_GYRO_RATE_4000Hz,
    ICM20689_GYRO_RATE_2000Hz,
    ICM20689_GYRO_RATE_1000Hz,
    ICM20689_GYRO_RATE_500Hz,
    ICM20689_GYRO_RATE_250Hz,
    ICM20689_GYRO_RATE_125Hz,
    ICM20689_GYRO_RATE_62_5Hz,
    ICM20689_GYRO_RATE_31_25Hz,
};

#define ICM20689_ADDR       0x68

#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_XOUT_H    0x3B
#define REG_PWR_MGMT_1      0x6B
#define REG_WHO_AM_I        0x75

static const uint16_t accel_scale_list[] = { 2, 4, 8, 16 };
static const uint16_t gyro_scale_list[] = { 250, 500, 1000, 2000 };
static float accel_factor;
static float gyro_factor;

#define ARRAY_SIZE(array) (sizeof(array)/sizeof(array[0]))
static int findIdx(uint32_t value, const uint16_t list[], uint32_t size) {
    int i=0;
    for (; i<size; i++) { if (value <= list[i]) break; }
    int last = ((i-1) < 0) ? 0 : (i-1);
    return (list[i]-value) < (value-list[last]) ? i : last;
}

esp_err_t icm20689_reset(imu_i2c_t *i2c) {
    // Perform soft reset
    return i2c->write_reg(ICM20689_ADDR, REG_PWR_MGMT_1, 0x80);
}

esp_err_t icm20689_init(imu_i2c_t *i2c) {
    esp_err_t err;
    uint8_t result = 0;
    i2c->write_reg(ICM20689_ADDR, REG_PWR_MGMT_1, 0x00); // Wake up & select internal oscillator
    // Read identification register
    err = i2c->read_reg(ICM20689_ADDR, REG_WHO_AM_I, &result, 1);
    // Check for communication error
    if (err) return err;
    // Validate identification
    if (result != 0x98) return ESP_ERR_NOT_SUPPORTED;
    return err;
}

esp_err_t icm20689_set_accel(imu_i2c_t *i2c, uint32_t *range) {
    uint8_t scaleIdx = findIdx(*range, accel_scale_list, ARRAY_SIZE(accel_scale_list));
    *range = accel_scale_list[scaleIdx];
    accel_factor = (float)accel_scale_list[scaleIdx] / 32767;
    return i2c->write_reg(ICM20689_ADDR, REG_ACCEL_CONFIG, scaleIdx << 3);
}
esp_err_t icm20689_set_gyro(imu_i2c_t *i2c, uint32_t *range) {
    uint8_t scaleIdx = findIdx(*range, gyro_scale_list, ARRAY_SIZE(gyro_scale_list));
    *range = gyro_scale_list[scaleIdx];
    gyro_factor = (float)gyro_scale_list[scaleIdx] / 32767;
    return i2c->write_reg(ICM20689_ADDR, REG_GYRO_CONFIG, scaleIdx << 3);
}

#define GET_REG_VALUE(buf, idx) ((((int16_t)buf[idx*2]) << 8) | buf[(idx*2)+1])

esp_err_t icm20689_read(imu_i2c_t *i2c, imu_data_t *data) {
    esp_err_t err;
    uint8_t buffer[7*2];
    err = i2c->read_reg(ICM20689_ADDR, REG_ACCEL_XOUT_H, (uint8_t*)&buffer, sizeof(buffer));
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
