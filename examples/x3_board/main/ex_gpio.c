#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/gpio.h"

#include "bsp/totem-bsp.h"

/**
 * RoboBoard X3 has 3 GPIO pins for external circuitry. Servo motor (SIG) pins can be also used as GPIO.
 * Can be used for simple input / output or communications I2C, UART, SPI, ...
*/

void run_example_gpio(void) {
    ESP_LOGI("Board", "--- Initialize board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Enable external 3V LDO regulator
    ESP_ERROR_CHECK(bsp_board_set_3V(1));
    // Configure GPIO pins
    ESP_LOGI("GPIO", "--- Setup GPIO pins ---");
    gpio_config_t pGPIOConfig = {0};
    // Configure IO32 to INPUT PULLUP
    pGPIOConfig.mode = GPIO_MODE_INPUT;
    pGPIOConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    pGPIOConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pGPIOConfig.pin_bit_mask = 1ULL << BSP_IO_IO32;
    ESP_ERROR_CHECK(gpio_config(&pGPIOConfig));
    // Configure IO33 to INPUT PULLDOWN
    pGPIOConfig.mode = GPIO_MODE_INPUT;
    pGPIOConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    pGPIOConfig.pull_down_en = GPIO_PULLDOWN_ENABLE;
    pGPIOConfig.pin_bit_mask = 1ULL << BSP_IO_IO33;
    ESP_ERROR_CHECK(gpio_config(&pGPIOConfig));
    // Configure IO26 to OUTPUT
    pGPIOConfig.mode = GPIO_MODE_OUTPUT;
    pGPIOConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    pGPIOConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pGPIOConfig.pin_bit_mask = 1ULL << BSP_IO_IO26;
    ESP_ERROR_CHECK(gpio_config(&pGPIOConfig));
    // Configure SERVOA (SIG pin) to OUTPUT
    pGPIOConfig.mode = GPIO_MODE_OUTPUT;
    pGPIOConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    pGPIOConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pGPIOConfig.pin_bit_mask = 1ULL << BSP_IO_SERVOA_IN;
    ESP_ERROR_CHECK(gpio_config(&pGPIOConfig));
    // Interact with GPIO pins
    int pinState = 0;
    while (1) {
        // Toggle IO26 and SERVOA output pins
        gpio_set_level(BSP_IO_IO26, pinState);
        pinState = !pinState;
        gpio_set_level(BSP_IO_SERVOA_IN, pinState);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Print GPIOA, GPIOB, GPIOC pin states
        ESP_LOGI("GPIO", "IO32: %d, IO33: %d",
            (int)gpio_get_level(BSP_IO_IO32),
            (int)gpio_get_level(BSP_IO_IO33)
        );
    }
}
