#include <stdio.h>
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "bsp/roboboard_x4.h"
#include "bsp/roboboard_x4.hpp"

extern "C" void app_main(void)
{
    // Initialize board components
    X4.begin();

    while (1) {
        // Turn LED on using C++ API
        X4.led.on();
        // Set RGB A, B ports using C++ API
        X4.rgbA.color(0xFF, esp_random());
        X4.rgbB.color(0xFF, esp_random());
        // Set RGB C, D ports using C API
        bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_C, esp_random());
        bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_D, esp_random());
        // Wait 1000ms
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Turn LED off using ESP-IDF GPIO driver
        gpio_set_level(BSP_IO_LED, 0);
        // Turn RGB off using C++ API
        X4.rgb.off();
        // Wait 1000ms
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
