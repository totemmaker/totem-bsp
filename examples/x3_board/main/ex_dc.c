#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bsp/totem-bsp.h"

/**
 * RoboBoard X3 has 4 individual motor ports for connecting brushed motors.
 * BSP supports: spinning, braking, tone output, PWM frequency config, decay config.
*/

static void on_bsp_event(uint32_t evt, uint32_t portID, uint32_t value, void *arg) {
    // Interrupt called on button press (and release)
    if (evt == BSP_EVT_BUTTON) {
        // Brake motors on button press
        if (value) { // "value" contains button state
            bsp_dc_brake(BSP_PORT_ALL, 100); // brake ALL motors with 100% power
        }
    }
}

void run_example_dc(void) {
    ESP_LOGI("Board", "--- Initialize board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Enable external 3V LDO regulator
    ESP_ERROR_CHECK(bsp_board_set_3V(1));
    // Register button interrupt
    ESP_ERROR_CHECK(bsp_board_reg_event(on_bsp_event, NULL));
    // Control motors
    // Default A,B,C,D port settings: slow decay, 20kHz
    ESP_LOGI("Board", "--- Spin motors ---");
    // Interact with motors
    while (1) {
        // Play 45kHz tone on all motors
        ESP_LOGI("Motor", "Tone 4000Hz");
        bsp_dc_tone(BSP_PORT_ALL, 4000);
        vTaskDelay(pdMS_TO_TICKS(1000));
        // Spin all motors forward at 30% power
        ESP_LOGI("Motor", "Spin forward 30%%");
        bsp_dc_spin(BSP_PORT_ALL, 30);
        vTaskDelay(pdMS_TO_TICKS(2000));
        // Spin all motors backward at 50% power
        ESP_LOGI("Motor", "Spin backward -50%%");
        bsp_dc_spin(BSP_PORT_ALL, -50);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
