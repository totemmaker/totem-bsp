#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bsp/totem-bsp.h"

/**
 * RoboBoard X4 has 4 RGB lights for indications and styling.
 * BSP supports: Settings individual light color (HEX), fade animations.
*/

// Set LED color in RGB value
static void bsp_rgb_color_rgb(int8_t ledID, uint8_t red, uint8_t green, uint8_t blue) {
    ESP_ERROR_CHECK(bsp_rgb_color(ledID, 0xFF << 24 | red << 16 | green << 8 | blue));
}
// Set LED fade color in RGB value
static void bsp_rgb_fade_color_rgb(int8_t ledID, uint8_t red, uint8_t green, uint8_t blue) {
    ESP_ERROR_CHECK(bsp_rgb_fade_color(ledID, 0xFF << 24 | red << 16 | green << 8 | blue));
}

static void on_bsp_event(uint32_t evt, uint32_t portID, uint32_t value, void *arg) {
    // Interrupt called on button press (and release)
    if (evt == BSP_EVT_BUTTON) {
        // Set to white on button press
        if (value) { // "value" contains button state
            bsp_rgb_color_rgb(BSP_PORT_ALL, 255, 255, 255);
        }
    }
}

void run_example_rgb(void) {
    ESP_LOGI("Board", "--- Initialize board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Turn LED on
    ESP_ERROR_CHECK(bsp_board_set_led(1));
    // Register button interrupt
    ESP_ERROR_CHECK(bsp_board_reg_event(on_bsp_event, NULL));
    // Control RGB lights
    ESP_LOGI("Board", "--- Control RGB lights ---");
    // Interact with RGB
    while (1) {
        bsp_rgb_color_rgb(BSP_PORT_A, 255, 0, 0); // Set A to red
        bsp_rgb_color_rgb(BSP_PORT_B, 0, 255, 0); // Set B to green
        bsp_rgb_color_rgb(BSP_PORT_C, 0, 0, 255); // Set C to blue
        bsp_rgb_color(BSP_PORT_D, 0xFFFFFFFF); // Set D to white
        vTaskDelay(pdMS_TO_TICKS(2000));
        // Fade to color
        bsp_rgb_fade_color_rgb(BSP_PORT_A, 0, 255, 0); // Fade A to green
        bsp_rgb_fade_color_rgb(BSP_PORT_B, 0, 0, 255); // Fade B to blue
        bsp_rgb_fade_color_rgb(BSP_PORT_C, 255, 0, 0); // Fade C to red
        bsp_rgb_fade_color_rgb(BSP_PORT_D, 50, 50, 50); // Dim light D
        bsp_rgb_fade_start(BSP_PORT_ALL, 1000); // Fade all in 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
