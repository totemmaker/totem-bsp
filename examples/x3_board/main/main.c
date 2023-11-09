#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/gpio.h"

#include "bsp/totem-bsp.h"
#include "bsp/imu.h"
#include "bsp/rgb.h"

void on_button_press(void *arg) {
    // Read button state
    // bsp_cmd_read(BSP_BUTTON_STATE, 0);
}

void app_main(void) {
    ESP_LOGI("Board", "--- Initializing board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Enable external 3V LDO regulator
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_3V_STATE, 0, 1));
    // Register button interrupt
    ESP_ERROR_CHECK(bsp_register_interrupt_callback(BSP_BUTTON_STATE, on_button_press, NULL));

    // Configure GPIO pins
    ESP_LOGI("GPIO", "--- Setup GPIO pins ---");
    gpio_config_t pGPIOConfig = {0};
    // Configure IO32 and IO33 to output
    pGPIOConfig.mode = GPIO_MODE_OUTPUT;
    pGPIOConfig.pin_bit_mask = 1ULL << BSP_IO_IO32 | 1ULL << BSP_IO_IO33;
    ESP_ERROR_CHECK(gpio_config(&pGPIOConfig));
    // Configure IO26 to input
    pGPIOConfig.mode = GPIO_MODE_INPUT;
    pGPIOConfig.pin_bit_mask = 1ULL << BSP_IO_IO26;
    ESP_ERROR_CHECK(gpio_config(&pGPIOConfig));
    // Set IO32 to LOW, IO33 to HIGH
    gpio_set_level(BSP_IO_IO32, 0);
    gpio_set_level(BSP_IO_IO33, 1);
    // Print pin states:
    ESP_LOGI("GPIO", "IO32: 0, IO33: 1, IO26: %" PRId32, (int32_t)gpio_get_level(BSP_IO_IO26));

    // Print board information
    ESP_LOGI("Board", "--- Printing board information ---");
    ESP_LOGI("Board", "RoboBoard X3");
    ESP_LOGI("Board", "USBin: %" PRId32, bsp_cmd_read(BSP_USB_STATE, 0));
    ESP_LOGI("Board", "Revision: %" PRId32, bsp_cmd_read(BSP_BOARD_REVISION, 0));
    ESP_LOGI("Board", "Battery voltage: %" PRId32 " mV", bsp_cmd_read(BSP_BATTERY_VOLTAGE, 0));
    ESP_LOGI("Board", "Battery current: %" PRId32 " mA", bsp_cmd_read(BSP_BATTERY_CURRENT, 0));
    ESP_LOGI("Board", "Battery charging: %" PRId32, bsp_cmd_read(BSP_BATTERY_CHARGING, 0));
    ESP_LOGI("Board", "DC frequency: %" PRId32 " Hz", bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 0));
    ESP_LOGI("Board", "Servo period: %" PRId32 " us", bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0));

    // Turn RGB lights on
    ESP_LOGI("RGB", "--- Turn on RGB lights ---");
    // Initialize RGB library
    ESP_ERROR_CHECK(bsp_rgb_init());
    ESP_ERROR_CHECK(bsp_rgb_hex(BSP_PORT_A, 0x400000)); // Set A to red
    ESP_ERROR_CHECK(bsp_rgb_hex(BSP_PORT_B, 0x004000)); // Set B to green
    ESP_ERROR_CHECK(bsp_rgb_hex(BSP_PORT_C, 0x000040)); // Set C to blue
    ESP_ERROR_CHECK(bsp_rgb_hex(BSP_PORT_D, 0x404040)); // Set D to white
    ESP_ERROR_CHECK(bsp_rgb_update()); // Write configured colors to LED

    // Turn motors on
    ESP_LOGI("Motor", "--- Spin motors ---");
    // Spin DC A at 50% power
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_DC_POWER, BSP_PORT_A, 50));
    // Set DC B to brake at 100% power
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_DC_BRAKE, BSP_PORT_B, 100));
    // Emit beep 1000Hz at C and D motors
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_DC_TONE, BSP_PORT_C, 1000));
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_DC_TONE, BSP_PORT_D, 1000));
    // Spin Servo A to 500us position
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_A, 500));
    // Spin Servo B to 1500us position
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_B, 1500));

    // Read accelerometer and gyroscope measurements
    ESP_LOGI("IMU", "--- Testing IMU reading ---");
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
