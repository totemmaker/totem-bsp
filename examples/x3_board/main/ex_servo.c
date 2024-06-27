#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bsp/totem-bsp.h"

/**
 * RoboBoard X3 has 4 individual servo motor ports (revisions before v3.1 has 2).
 * BSP supports: Position control with pulse (us), period config.
*/

void run_example_servo(void) {
    ESP_LOGI("Board", "--- Initialize board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Enable external 3V LDO regulator
    ESP_ERROR_CHECK(bsp_board_set_3V(1));
    // Control servo motors
    ESP_LOGI("Board", "--- Spin motors ---");
    // Set start position
    bsp_servo_spin(BSP_PORT_ALL, 2000); // Right
    // Interact with motors
    while (1) {
        ESP_LOGI("Servo", "Move all servos");
        bsp_servo_spin(BSP_PORT_ALL, 1500); // Move center
        vTaskDelay(pdMS_TO_TICKS(500));
        bsp_servo_spin(BSP_PORT_ALL, 1000); // Move left
        vTaskDelay(pdMS_TO_TICKS(500));
        bsp_servo_spin(BSP_PORT_ALL, 2000); // Move right
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI("Servo", "Move Servo B only");
        bsp_servo_spin(BSP_PORT_B, 1000); // Move to right
        vTaskDelay(pdMS_TO_TICKS(1000));
        bsp_servo_spin(BSP_PORT_B, 2000); // Move to left
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
