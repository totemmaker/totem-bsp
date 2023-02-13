#include <stdio.h>
#include "esp_random.h"
#include "freertos/FreeRTOS.h"
#include "bsp/roboboard_x4.h"

void app_main(void)
{
    // Initialize board components
    bsp_board_init();

    while (1) {
        // Turn LED on
        bsp_cmd_write(BSP_LED_STATE, 0, 1);
        // Turn RGB on with random colors
        bsp_cmd_write(BSP_RGB_COLOR, BSP_CHANNEL_A, esp_random() | 0xFF000000);
        bsp_cmd_write(BSP_RGB_COLOR, BSP_CHANNEL_B, esp_random() | 0xFF000000);
        bsp_cmd_write(BSP_RGB_COLOR, BSP_CHANNEL_C, esp_random() | 0xFF000000);
        bsp_cmd_write(BSP_RGB_COLOR, BSP_CHANNEL_D, esp_random() | 0xFF000000);
        // Wait 1000ms
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Turn LED off
        bsp_cmd_write(BSP_LED_STATE, 0, 0);
        // Turn RGB off
        bsp_cmd_write(BSP_RGB_COLOR, BSP_CHANNEL_All, 0);
        // Wait 1000ms
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
