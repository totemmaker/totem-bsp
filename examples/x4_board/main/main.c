#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/gpio.h"

#include "bsp/totem-bsp.h"
#include "bsp/imu.h"
#include "bsp/can.h"

// CAN message received registered in "bsp_can_init"
void on_can_msg(void *args, uint32_t id, uint8_t *data, uint8_t len) {
    // Print message start
    ESP_LOGI("CAN", "MSG: id(%s): %" PRIX32, (id & BSP_CAN_FlagExt) ? "29b" : "11b", id & BSP_CAN_id);
    // Print message end
    if (id & BSP_CAN_FlagRTR) {
        ESP_LOGI("Data", "remote request. datalen: %" PRIu8, len);
    }
    else {
        ESP_LOG_BUFFER_HEX("Data", data, len);
    }
}

void on_button_press(void *arg) {
    // Turn LED on when button is pressed
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_LED_STATE, 0, bsp_cmd_read(BSP_BUTTON_STATE, 0)));
}

void app_main(void) {
    ESP_LOGI("Board", "--- Initializing board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Register button interrupt
    ESP_ERROR_CHECK(bsp_register_interrupt_callback(BSP_BUTTON_STATE, on_button_press, NULL));

    // Configure GPIO pins
    ESP_LOGI("GPIO", "--- Setup GPIO pins ---");
    gpio_config_t pGPIOConfig = {0};
    // Configure GPIOA and GPIOB to output
    pGPIOConfig.mode = GPIO_MODE_OUTPUT;
    pGPIOConfig.pin_bit_mask = 1ULL << BSP_IO_GPIOA | 1ULL << BSP_IO_GPIOB;
    ESP_ERROR_CHECK(gpio_config(&pGPIOConfig));
    // Configure GPIOC and GPIOD to input
    pGPIOConfig.mode = GPIO_MODE_INPUT;
    pGPIOConfig.pin_bit_mask = 1ULL << BSP_IO_GPIOC | 1ULL << BSP_IO_GPIOD;
    ESP_ERROR_CHECK(gpio_config(&pGPIOConfig));
    // Set GPIOA to LOW, GPIOB to HIGH
    gpio_set_level(BSP_IO_GPIOA, 0);
    gpio_set_level(BSP_IO_GPIOB, 1);
    // Print pin states:
    ESP_LOGI("GPIO", "GPIOA: 0, GPIOB: 1, GPIOC: %" PRId32 ", GPIOD: %" PRId32, (int32_t)gpio_get_level(BSP_IO_GPIOC), (int32_t)gpio_get_level(BSP_IO_GPIOD));

    // Print board information
    ESP_LOGI("Board", "--- Printing board information ---");
    ESP_LOGI("Board", "RoboBoard X4");
    ESP_LOGI("Board", "USBin: %" PRId32, bsp_cmd_read(BSP_USB_STATE, 0));
    ESP_LOGI("Board", "Revision: %" PRId32, bsp_cmd_read(BSP_BOARD_REVISION, 0));
    ESP_LOGI("Board", "Driver fw: %" PRId32, bsp_cmd_read(BSP_DRIVER_FIRMWARE, 0));
    ESP_LOGI("Board", "Battery voltage: %" PRId32 " mV", bsp_cmd_read(BSP_BATTERY_VOLTAGE, 0));
    ESP_LOGI("Board", "DC frequency: %" PRId32 " Hz", bsp_cmd_read(BSP_DC_CONFIG_FREQUENCY, 0));
    ESP_LOGI("Board", "Servo period: %" PRId32 " us", bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0));

    // Turn RGB lights on
    ESP_LOGI("RGB", "--- Turn on RGB lights ---");
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_A, 0xFFFF0000)); // Set A to red
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_B, 0xFF00FF00)); // Set B to green
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_C, 0xFF0000FF)); // Set C to blue
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_D, 0xFFFFFFFF)); // Set D to white

    // Turn motors on
    ESP_LOGI("Motor", "--- Spin motors ---");
    // Spin DC A at 50% power
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_DC_POWER, BSP_PORT_A, 50));
    // Set DC B to brake at 100% power
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_DC_BRAKE, BSP_PORT_B, 100));
    // Emit beep 1000Hz at C and D motors for 3000ms (and stop)
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_DC_TONE, BSP_PORT_C, 3000 << 16 | 1000));
    // Spin Servo A to 500us position
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_A, 500));
    // Spin Servo B to 500us position in 2 seconds
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_B, 2000 << 16 | 500));
    // Spin Servo C to 500us position at 10 RPM speed
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_SERVO_CONFIG_SPEED, BSP_PORT_C, 10 * 60));
    ESP_ERROR_CHECK(bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_C, 500));

    // Test CAN messaging
    ESP_LOGI("CAN", "--- Testing CAN reception ---");
    // Initialize TWAI driver in loopback mode
    // NOTE: call bsp_twai_init(0) when working with real CAN network.
    // parameter "1" is a loopback mode for testing end example code
    ESP_ERROR_CHECK(bsp_twai_init(1));
    // Initialize and enable CAN library
    ESP_ERROR_CHECK(bsp_can_init(on_can_msg, NULL));
    ESP_ERROR_CHECK(bsp_can_enable(1));
    // Send fake CAN messages that are received in "on_can_msg"
    // function due to enabled loopback mode
    uint8_t msg[] = {1, 2, 3};
    ESP_LOGI("CAN", "Sending...");
    ESP_ERROR_CHECK(bsp_can_send_std(0xAB, msg, sizeof(msg) - 1)); // Send 11b ID CAN message
    ESP_ERROR_CHECK(bsp_can_send_ext(0xABCDEF, msg, sizeof(msg))); // Send 29b ID CAN message
    ESP_ERROR_CHECK(bsp_can_send_std_rtr(0x10, 5)); // Send RTR CAN message with data len 5
    // Wait 100ms
    vTaskDelay(pdMS_TO_TICKS(100));

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
