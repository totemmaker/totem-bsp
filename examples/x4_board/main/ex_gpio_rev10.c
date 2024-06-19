#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bsp/totem-bsp.h"

/**
 * RoboBoard X4 (revision 1.0) has 4 (limited) GPIO pins for external circuitry.
 * Can be used for simple input / output and ADC measurements (pins are not connected to ESP32).
*/

void run_example_gpio_rev10(void) {
    ESP_LOGI("Board", "--- Initialize board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Turn LED on
    ESP_ERROR_CHECK(bsp_board_set_led(1));
    // Configure GPIO pins
    ESP_LOGI("Board", "--- Setup GPIO pins ---");
    const int GPIOA = 0;
    const int GPIOB = 1;
    const int GPIOC = 2;
    const int GPIOD = 3;
    // 0-pd, 1-pu, 2-float, 3-out, 4-analog input
    bsp_gpio_mode(GPIOA, 1); // Set GPIOA to INPUT_PULLUP
    bsp_gpio_mode(GPIOB, 2); // Set GPIOB to floating INPUT
    bsp_gpio_mode(GPIOC, 4); // Set GPIOC to floating ANALOG input
    bsp_gpio_mode(GPIOD, 3); // Set GPIOD to floating OUTPUT
    // Interact with GPIO pins
    int pinState = 0;
    while (1) {
        // Toggle GPIOD output pin
        pinState = !pinState;
        bsp_gpio_digital_write(GPIOD, pinState);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Print GPIOA, GPIOB, GPIOC pin states
        ESP_LOGI("GPIO", "GPIOA: %d, GPIOB: %d, GPIOC: %d(ADC)",
            (int)bsp_gpio_digital_read(GPIOA),
            (int)bsp_gpio_digital_read(GPIOB),
            (int)bsp_gpio_analog_read(GPIOC)
        );
    }
}
