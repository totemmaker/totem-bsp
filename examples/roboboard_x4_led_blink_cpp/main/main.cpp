#include <stdio.h>
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "bsp/roboboard_x4.hpp"

extern "C" void app_main(void)
{
    // Initialize board components
    X4.begin();

    while (1) {
        // Turn LED on
        X4.led.on();
        // Turn RGB on with random colors
        X4.rgbA.color(0xFF, esp_random());
        X4.rgbB.color(0xFF, esp_random());
        X4.rgbC.color(0xFF, esp_random());
        X4.rgbD.color(0xFF, esp_random());
        // Wait 1000ms
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Turn LED on
        X4.led.off();
        // Turn RGB off
        X4.rgb.off();
        // Wait 1000ms
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
