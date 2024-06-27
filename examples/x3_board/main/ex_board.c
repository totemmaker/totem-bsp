#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bsp/totem-bsp.h"

/**
 * RoboBoard X3 has some useful on board features.
 * BSP supports: USB detect, button state, 3V regulator on/off
 * battery voltage, battery current, battery charging state.
*/

void run_example_board(void) {
    ESP_LOGI("Board", "--- Initialize board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Enable external 3V LDO regulator
    ESP_ERROR_CHECK(bsp_board_set_3V(1));
    // Interact with board info
    while (1) {
        // Print board information
        ESP_LOGI("Board", "--- Printing board information ---");
        ESP_LOGI("Board", "RoboBoard X3");
        ESP_LOGI("Board", "USBin: %d", (int)bsp_board_get_usb());
        ESP_LOGI("Board", "Revision: %d", (int)bsp_board_get_revision());
        ESP_LOGI("Board", "Battery voltage: %d mV", (int)bsp_battery_get_voltage());
        ESP_LOGI("Board", "Battery current: %d mA", (int)bsp_battery_get_current());
        ESP_LOGI("Board", "Battery charging: %s", bsp_battery_get_charging() ? "Yes" : "No");
        ESP_LOGI("Board", "DC frequency: %d Hz", (int)bsp_dc_get_frequency(0));
        ESP_LOGI("Board", "DC decay: %s", bsp_dc_get_decay(0)==0 ? "slow" : "fast");
        ESP_LOGI("Board", "Servo period: %d us", (int)bsp_servo_get_period_AB());
        ESP_LOGI("Board", "Servo ports: %d", (int)bsp_servo_get_port_cnt());
        // Wait 2s
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
