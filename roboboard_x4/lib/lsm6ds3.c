/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "lsm6ds3.h"

enum { // Accelerometer full-scale
    LSM6DS3_ACC_SCALE_2g,
    LSM6DS3_ACC_SCALE_16g,
    LSM6DS3_ACC_SCALE_4g,
    LSM6DS3_ACC_SCALE_8g,
};

enum { // Gyroscope full-scale
    LSM6DS3_GYRO_SCALE_250dps,
    LSM6DS3_GYRO_SCALE_500dps,
    LSM6DS3_GYRO_SCALE_1000dps,
    LSM6DS3_GYRO_SCALE_2000dps,
};

typedef enum { // Accelerometer output data rate (sample rate)
    LSM6DS3_ACCEL_RATE_PowerDown,
    LSM6DS3_ACCEL_RATE_12_5Hz,
    LSM6DS3_ACCEL_RATE_26Hz,
    LSM6DS3_ACCEL_RATE_52Hz,
    LSM6DS3_ACCEL_RATE_104Hz,
    LSM6DS3_ACCEL_RATE_208Hz,
    LSM6DS3_ACCEL_RATE_416Hz,
    LSM6DS3_ACCEL_RATE_833Hz,
    LSM6DS3_ACCEL_RATE_1666Hz,
    LSM6DS3_ACCEL_RATE_3332Hz,
    LSM6DS3_ACCEL_RATE_6664Hz,
} lsm6ds3_accel_rate_t;

typedef enum { // Gyroscope output data rate (sample rate)
    LSM6DS3_GYRO_RATE_PowerDown,
    LSM6DS3_GYRO_RATE_12_5Hz,
    LSM6DS3_GYRO_RATE_26Hz,
    LSM6DS3_GYRO_RATE_52Hz,
    LSM6DS3_GYRO_RATE_104Hz,
    LSM6DS3_GYRO_RATE_208Hz,
    LSM6DS3_GYRO_RATE_416Hz,
    LSM6DS3_GYRO_RATE_833Hz,
    LSM6DS3_GYRO_RATE_1666Hz,
} lsm6ds3_gyro_rate_t;

typedef enum { // Accelerometer Anti-aliasing filter bandwidth 
    LSM6DS3_ACCEL_FILTER_400Hz,
    LSM6DS3_ACCEL_FILTER_200Hz,
    LSM6DS3_ACCEL_FILTER_100Hz,
    LSM6DS3_ACCEL_FILTER_50Hz,
} lsm6ds3_accel_filter_t;

#define LSM6DS3_ADDR    0x6A

#define REG_WHO_AM_I    0x0F
#define REG_CTRL1_XL    0x10
#define REG_CTRL2_G     0x11
#define REG_OUT_TEMP_L  0x20

static const uint16_t accel_scale_list[] = { 2, 4, 8, 16 };
static const uint16_t gyro_scale_list[] = { 125, 250, 500, 1000, 2000 };
static const uint16_t data_rate_list[] = { 0, 12, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664 };
static const uint16_t filter_list[] = { 50, 100, 200, 400 };
static float accel_factor;
static float gyro_factor;

#define ARRAY_SIZE(array) (sizeof(array)/sizeof(array[0]))
static int findIdx(uint32_t value, const uint16_t list[], uint32_t size) {
    int i=0;
    for (; i<size; i++) { if (value <= list[i]) break; }
    int last = ((i-1) < 0) ? 0 : (i-1);
    return (list[i]-value) < (value-list[last]) ? i : last;
}

esp_err_t lsm6ds3_init(imu_i2c_t *i2c) {
    esp_err_t err;
    uint8_t result = 0;
    // Read identification register
    err = i2c->read_reg(LSM6DS3_ADDR, REG_WHO_AM_I, &result, 1);
    if (err) return err;
    // Validate identification
    if (result != 0x69) return ESP_ERR_NOT_SUPPORTED;
    return ESP_OK;
}
// range: 2, 4, 8, 16 (G)
// rate: 0, 12, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664 (Hz)
// filter: 50, 100, 200, 400 (Hz)
esp_err_t lsm6ds3_set_accel(imu_i2c_t *i2c, uint32_t *range, uint32_t *rate, uint32_t* filter) {
    uint8_t scaleIdx = findIdx(*range, accel_scale_list, ARRAY_SIZE(accel_scale_list));
    uint8_t odrIdx = findIdx(*rate, data_rate_list, ARRAY_SIZE(data_rate_list));
    uint8_t filerIdx = findIdx(*filter, filter_list, ARRAY_SIZE(filter_list));
    *range = accel_scale_list[scaleIdx];
    *rate = data_rate_list[odrIdx];
    *filter = filter_list[filerIdx];
    accel_factor = (float)accel_scale_list[scaleIdx] / 32767;
    const uint8_t remap[] = { 0b00, 0b10, 0b11, 0b01 }; // 2, 4, 8, 16
    return i2c->write_reg(LSM6DS3_ADDR, REG_CTRL1_XL, odrIdx << 4 | remap[scaleIdx] << 2 | (3-filerIdx));
}
// range: 125, 250, 500, 1000, 2000 (dps)
// rate: 0, 12, 26, 52, 104, 208, 416, 833, 1666 (Hz)
esp_err_t lsm6ds3_set_gyro(imu_i2c_t *i2c, uint32_t *range, uint32_t *rate) {
    uint8_t scaleIdx = findIdx(*range, gyro_scale_list, ARRAY_SIZE(gyro_scale_list));
    uint8_t odrIdx = findIdx(*rate, data_rate_list, ARRAY_SIZE(data_rate_list)-2);
    *range = gyro_scale_list[scaleIdx];
    *rate = data_rate_list[odrIdx];
    gyro_factor = (float)gyro_scale_list[scaleIdx] / 32767;
    if (scaleIdx == 0) scaleIdx = 1;
    else scaleIdx = (scaleIdx-1) << 1;
    return i2c->write_reg(LSM6DS3_ADDR, REG_CTRL2_G, odrIdx << 4 | scaleIdx << 1);
}

esp_err_t lsm6ds3_read(imu_i2c_t *i2c, imu_data_t *data) {
    esp_err_t err;
    int16_t buffer[7] = {0};
    err = i2c->read_reg(LSM6DS3_ADDR, REG_OUT_TEMP_L, (uint8_t*)&buffer, sizeof(buffer));
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
