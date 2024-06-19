#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/gpio.h"

#include "bsp/totem-bsp.h"

/**
 * RoboBoard X4 has 4 GPIO pins for external circuitry.
 * Can be used for simple input / output or communications I2C, UART, SPI, ...
*/

void run_example_gpio(void) {
    ESP_LOGI("Board", "--- Initialize board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Turn LED on
    ESP_ERROR_CHECK(bsp_board_set_led(1));
    // Configure GPIO pins
    ESP_LOGI("GPIO", "--- Setup GPIO pins ---");
    gpio_config_t pGPIOConfig = {0};
    // Configure GPIOA to INPUT PULLUP
    pGPIOConfig.mode = GPIO_MODE_INPUT;
    pGPIOConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    pGPIOConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pGPIOConfig.pin_bit_mask = 1ULL << BSP_IO_GPIOA;
    ESP_ERROR_CHECK(gpio_config(&pGPIOConfig));
    // Configure GPIOB to INPUT PULLDOWN
    pGPIOConfig.mode = GPIO_MODE_INPUT;
    pGPIOConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    pGPIOConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
    pGPIOConfig.pin_bit_mask = 1ULL << BSP_IO_GPIOB;
    ESP_ERROR_CHECK(gpio_config(&pGPIOConfig));
    // Configure GPIOC to INPUT floating
    pGPIOConfig.mode = GPIO_MODE_INPUT;
    pGPIOConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    pGPIOConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pGPIOConfig.pin_bit_mask = 1ULL << BSP_IO_GPIOC;
    ESP_ERROR_CHECK(gpio_config(&pGPIOConfig));
    // Configure GPIOD to OUTPUT
    pGPIOConfig.mode = GPIO_MODE_OUTPUT;
    pGPIOConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    pGPIOConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pGPIOConfig.pin_bit_mask = 1ULL << BSP_IO_GPIOD;
    ESP_ERROR_CHECK(gpio_config(&pGPIOConfig));
    // Interact with GPIO pins
    int pinState = 0;
    while (1) {
        // Toggle GPIOD output pin
        pinState = !pinState;
        gpio_set_level(BSP_IO_GPIOD, pinState);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Print GPIOA, GPIOB, GPIOC pin states
        ESP_LOGI("GPIO", "GPIOA: %d, GPIOB: %d GPIOC: %d",
            (int)gpio_get_level(BSP_IO_GPIOA),
            (int)gpio_get_level(BSP_IO_GPIOB),
            (int)gpio_get_level(BSP_IO_GPIOC)
        );
    }
}
