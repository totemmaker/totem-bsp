#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bsp/totem-bsp.h"

/**
 * RoboBoard X4 has some useful on board features.
 * BSP supports: USB detect, button state, LED on/off, CAN on/off, battery voltage
*/

static void on_bsp_event(uint32_t evt, uint32_t portID, uint32_t value, void *arg) {
    // Interrupt called on button press (and release)
    if (evt == BSP_EVT_BUTTON) {
        // Turn LED off when button is pressed
        bsp_board_set_led(!value); // "value" contains button state
    }
}

void run_example_board(void) {
    ESP_LOGI("Board", "--- Initialize board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Turn LED on
    ESP_ERROR_CHECK(bsp_board_set_led(1));
    // Register button interrupt
    ESP_ERROR_CHECK(bsp_board_reg_event(on_bsp_event, NULL));
    // Interact with board info
    while (1) {
        // Print board information
        ESP_LOGI("Board", "--- Printing board information ---");
        ESP_LOGI("Board", "RoboBoard X4");
        ESP_LOGI("Board", "USBin: %d", (int)bsp_board_get_usb());
        ESP_LOGI("Board", "Revision: %d", (int)bsp_board_get_revision());
        ESP_LOGI("Board", "Driver fw: %d", (int)bsp_board_get_firmware());
        ESP_LOGI("Board", "Battery voltage: %d mV", (int)bsp_battery_get_voltage());
        ESP_LOGI("Board", "DC frequency: %d Hz", (int)bsp_dc_get_frequency_AB());
        ESP_LOGI("Board", "DC decay: %s", bsp_dc_get_decay(0)==0 ? "slow" : "fast");
        ESP_LOGI("Board", "Servo period: %d us", (int)bsp_servo_get_period());
        // Wait 2s
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
