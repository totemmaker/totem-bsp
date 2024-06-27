#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bsp/totem-bsp.h"
#include "bsp/rgb.h"

/**
 * RoboBoard X3 has 4 RGB lights for indications and styling (addressable, pin 13).
 * BSP supports: Settings individual light color (RGB, HEX, HSV).
*/

static void on_bsp_event(uint32_t evt, uint32_t portID, uint32_t value, void *arg) {
    // Interrupt called on button press (and release)
    if (evt == BSP_EVT_BUTTON) {
        // Brake motors on button press
        if (value) { // "value" contains button state
            bsp_rgb_hex(BSP_PORT_ALL, 0xFFFFFF);
            bsp_rgb_update();
        }
    }
}

void run_example_rgb(void) {
    ESP_LOGI("Board", "--- Initialize board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Enable external 3V LDO regulator
    ESP_ERROR_CHECK(bsp_board_set_3V(1));
    // Register button interrupt
    ESP_ERROR_CHECK(bsp_board_reg_event(on_bsp_event, NULL));
    // Control RGB lights
    ESP_LOGI("Board", "--- Control RGB lights ---");
    // Initialize RGB library
    ESP_ERROR_CHECK(bsp_rgb_init());
    // Interact with RGB
    while (1) {
        ESP_ERROR_CHECK(bsp_rgb_rgb(BSP_PORT_A, 255, 0, 0)); // Set A to red
        ESP_ERROR_CHECK(bsp_rgb_rgb(BSP_PORT_B, 0, 255, 0)); // Set B to green
        ESP_ERROR_CHECK(bsp_rgb_rgb(BSP_PORT_C, 0, 0, 255)); // Set C to blue
        ESP_ERROR_CHECK(bsp_rgb_hex(BSP_PORT_D, 0xFFFFFF));  // Set D to white
        ESP_ERROR_CHECK(bsp_rgb_update()); // Output configured colors
        vTaskDelay(pdMS_TO_TICKS(500));
        // Change color
        ESP_ERROR_CHECK(bsp_rgb_rgb(BSP_PORT_A, 0, 255, 0)); // Set A to green
        ESP_ERROR_CHECK(bsp_rgb_rgb(BSP_PORT_B, 0, 0, 255)); // Set B to blue
        ESP_ERROR_CHECK(bsp_rgb_rgb(BSP_PORT_C, 255, 0, 0)); // Set C to red
        ESP_ERROR_CHECK(bsp_rgb_rgb(BSP_PORT_D, 50, 50, 50)); // Set D dim white
        ESP_ERROR_CHECK(bsp_rgb_update()); // Output configured colors
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
