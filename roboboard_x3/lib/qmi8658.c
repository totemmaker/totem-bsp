/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "qmi8658.h"

enum { // Accelerometer full-scale
    QMI8658_ACC_SCALE_2g,
    QMI8658_ACC_SCALE_4g,
    QMI8658_ACC_SCALE_8g,
    QMI8658_ACC_SCALE_16g,
};

enum { // Accelerometer output data rate
    QMI8658_ACC_RATE_8000Hz,
    QMI8658_ACC_RATE_4000Hz,
    QMI8658_ACC_RATE_2000Hz,
    QMI8658_ACC_RATE_1000Hz,
    QMI8658_ACC_RATE_500Hz,
    QMI8658_ACC_RATE_250Hz,
    QMI8658_ACC_RATE_125Hz,
    QMI8658_ACC_RATE_62_5Hz,
    QMI8658_ACC_RATE_31_25Hz,
    // Low power
    QMI8658_ACC_RATE_LP_128Hz = 12,
    QMI8658_ACC_RATE_LP_21Hz,
    QMI8658_ACC_RATE_LP_11Hz,
    QMI8658_ACC_RATE_LP_3Hz,
};

enum { // Gyroscope full-scale
    QMI8658_GYRO_SCALE_16dps,
    QMI8658_GYRO_SCALE_32dps,
    QMI8658_GYRO_SCALE_64dps,
    QMI8658_GYRO_SCALE_128dps,
    QMI8658_GYRO_SCALE_256dps,
    QMI8658_GYRO_SCALE_512dps,
    QMI8658_GYRO_SCALE_1024dps,
    QMI8658_GYRO_SCALE_2048dps,
};

enum { // Gyroscope output data rate
    QMI8658_GYRO_RATE_8000Hz,
    QMI8658_GYRO_RATE_4000Hz,
    QMI8658_GYRO_RATE_2000Hz,
    QMI8658_GYRO_RATE_1000Hz,
    QMI8658_GYRO_RATE_500Hz,
    QMI8658_GYRO_RATE_250Hz,
    QMI8658_GYRO_RATE_125Hz,
    QMI8658_GYRO_RATE_62_5Hz,
    QMI8658_GYRO_RATE_31_25Hz,
};

#define QMI8658_ADDR 0x6B

#define REG_WHO_AM_I 0x00
#define REG_CTRL1    0x02 // Accelerometer settings
#define REG_CTRL2    0x03 // Accelerometer settings
#define REG_CTRL3    0x04 // Gyroscope settings
#define REG_CTRL5    0x06 // Sensor Data Processing Settings
#define REG_CTRL7    0x08 // Enable Sensors and Configure Data Reads
#define REG_TEMP_L   0x33
// #define REG_AX_L     0x35
// #define REG_GX_L     0x3B

static const uint16_t accel_scale_list[] = { 2, 4, 8, 16 };
static const uint16_t gyro_scale_list[] = { 16, 32, 64, 128, 256, 512, 1024, 2048 };
static const uint16_t data_rate_list[] = { 31, 62, 125, 250, 500, 1000, 2000, 4000, 8000 };
static float accel_factor;
static float gyro_factor;

#define ARRAY_SIZE(array) (sizeof(array)/sizeof(array[0]))
static int findIdx(uint32_t value, const uint16_t list[], uint32_t size) {
    int i=0;
    for (; i<size; i++) { if (value <= list[i]) break; }
    int last = ((i-1) < 0) ? 0 : (i-1);
    return (list[i]-value) < (value-list[last]) ? i : last;
}

esp_err_t qmi8658_init(imu_i2c_t *i2c) {
    esp_err_t err;
    uint8_t result = 0;
    // Read identification register
    err = i2c->read_reg(QMI8658_ADDR, REG_WHO_AM_I, &result, 1);
    // Check for communication error
    if (err) return err;
    // Validate identification
    if (result != 0x05) return ESP_ERR_NOT_SUPPORTED;
    // Enable gyroscope and accelerometer
    i2c->write_reg(QMI8658_ADDR, REG_CTRL1, 0x60);
    i2c->write_reg(QMI8658_ADDR, REG_CTRL7, 0x3);
    i2c->write_reg(QMI8658_ADDR, REG_CTRL5, 0x71);
    return ESP_OK;
}

esp_err_t qmi8658_set_accel(imu_i2c_t *i2c, uint32_t *range, uint32_t *rate) {
    uint8_t scaleIdx = findIdx(*range, accel_scale_list, ARRAY_SIZE(accel_scale_list));
    uint8_t odrIdx = findIdx(*rate, data_rate_list, ARRAY_SIZE(data_rate_list));
    *range = accel_scale_list[scaleIdx];
    *rate = data_rate_list[odrIdx];
    odrIdx = ARRAY_SIZE(data_rate_list)-1-odrIdx;
    accel_factor = (float)accel_scale_list[scaleIdx] / 32767;
    return i2c->write_reg(QMI8658_ADDR, REG_CTRL2, scaleIdx << 4 | odrIdx);
}

esp_err_t qmi8658_set_gyro(imu_i2c_t *i2c, uint32_t *range, uint32_t *rate) {
    uint8_t scaleIdx = findIdx(*range, gyro_scale_list, ARRAY_SIZE(gyro_scale_list));
    uint8_t odrIdx = findIdx(*rate, data_rate_list, ARRAY_SIZE(data_rate_list));
    *range = gyro_scale_list[scaleIdx];
    *rate = data_rate_list[odrIdx];
    odrIdx = ARRAY_SIZE(data_rate_list)-1-odrIdx;
    gyro_factor = (float)gyro_scale_list[scaleIdx] / 32767;
    return i2c->write_reg(QMI8658_ADDR, REG_CTRL3, scaleIdx << 4 | odrIdx);
}

esp_err_t qmi8658_read(imu_i2c_t *i2c, imu_data_t *data) {
    esp_err_t err;
    int16_t buffer[7];
    err = i2c->read_reg(QMI8658_ADDR, REG_TEMP_L, (uint8_t*)&buffer, sizeof(buffer));
    int16_t temp = buffer[0];
    int16_t accX = buffer[1];
    int16_t accY = buffer[2];
    int16_t accZ = buffer[3];
    int16_t gyroX = buffer[4];
    int16_t gyroY = buffer[5];
    int16_t gyroZ = buffer[6];

    data->temp = (temp / 256.0);
    data->gyro.x = gyroX * gyro_factor;
    data->gyro.y = gyroY * gyro_factor;
    data->gyro.z = gyroZ * gyro_factor;
    data->accel.x = accX * accel_factor;
    data->accel.y = accY * accel_factor;
    data->accel.z = accZ * accel_factor;
    return err;
}