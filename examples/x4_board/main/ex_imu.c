#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bsp/totem-bsp.h"
#include "bsp/imu.h"

/**
 * RoboBoard X4 has Internal Measurement Unit (Accelerometer and Gyroscope) to detect board movements.
 * BSP supports: reading G force, reading rotation DPS, config sample rate.
*/

void run_example_imu(void) {
    ESP_LOGI("Board", "--- Initialize board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Turn LED on
    ESP_ERROR_CHECK(bsp_board_set_led(1));
    // Read accelerometer and gyroscope measurements
    ESP_LOGI("Board", "--- IMU reading ---");
    // Initialize I2C at 400kHz speed
    ESP_ERROR_CHECK(bsp_i2c_init(0, 400000));
    // Initialize IMU driver at I2C_NUM_0
    ESP_ERROR_CHECK(bsp_imu_init(0));
    bsp_imu_data_t data;
    while (1) {
        // Read IMU
        ESP_ERROR_CHECK(bsp_imu_read(&data));
        ESP_LOGI("IMU", "Acc: X:%2.2f, Y:%2.2f, Z:%2.2f Gyro: X:%3.2f, Y:%3.2f, Z:%3.2f, Temp: %.2f",
            data.accel.x, data.accel.y, data.accel.z, data.gyro.x, data.gyro.y, data.gyro.z, data.temp);
        // Wait 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
