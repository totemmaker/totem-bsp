#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bsp/totem-bsp.h"

/**
 * RoboBoard X4 has 3 individual servo motor ports.
 * BSP supports: Position control with pulse (us), speed control, period config.
*/

// BSP library uses PPP (Pulse-Per-Period) unit for speed input.
// Use provided functions to convert RPM values to PPP:

// Spin servo to microseconds with specified speed (RPM) (override speed)
static void bsp_servo_spin_rpm(int8_t portID, uint32_t pulse, uint32_t rpm) {
    const int motorUsMin = 500;
    const int motorUsMax = 2500;
    const int motorAngle = 180;
    int ppp = (rpm * 6 * (motorUsMax - motorUsMin)) / motorAngle / 50;
    ESP_ERROR_CHECK(bsp_servo_spin_ppp(portID, pulse, ppp));
}
// Set constant servo port speed (RPM)
static void bsp_servo_set_speed_rpm(int8_t portID, uint32_t rpm) {
    const int motorUsMin = 500;
    const int motorUsMax = 2500;
    const int motorAngle = 180;
    int ppp = (rpm * 6 * (motorUsMax - motorUsMin)) / motorAngle / 50;
    ESP_ERROR_CHECK(bsp_servo_set_speed_ppp(portID, ppp));
}

void run_example_servo(void) {
    ESP_LOGI("Board", "--- Initialize board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Turn LED on
    ESP_ERROR_CHECK(bsp_board_set_led(1));
    // Control servo motors
    ESP_LOGI("Board", "--- Spin motors ---");
    // Set start position
    bsp_servo_spin(BSP_PORT_ALL, 2000); // Right
    // Interact with motors
    while (1) {
        // Spin slow
        ESP_LOGI("Servo", "Move all servos (slow)");
        bsp_servo_set_speed_rpm(BSP_PORT_ALL, 10); // Set to 10 RPM
        bsp_servo_spin(BSP_PORT_ALL, 1500); // Move center (10 RPM)
        vTaskDelay(pdMS_TO_TICKS(2000));
        bsp_servo_spin(BSP_PORT_ALL, 1000); // Move left (10 RPM)
        vTaskDelay(pdMS_TO_TICKS(2000));
        bsp_servo_spin(BSP_PORT_ALL, 2000); // Move right (10 RPM)
        vTaskDelay(pdMS_TO_TICKS(3000));
        // Spin fast
        ESP_LOGI("Servo", "Move all servos (fast)");
        bsp_servo_set_speed_rpm(BSP_PORT_ALL, 0); // Disable speed control
        bsp_servo_spin(BSP_PORT_ALL, 1500); // Move center (max speed)
        vTaskDelay(pdMS_TO_TICKS(500));
        bsp_servo_spin_rpm(BSP_PORT_ALL, 1000, 30); // Move left at 30 RPM
        vTaskDelay(pdMS_TO_TICKS(500));
        bsp_servo_spin(BSP_PORT_ALL, 2000); // Move right (max speed)
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI("Servo", "Move Servo B (slow and fast)");
        // Move Servo B to left in exactly 2 seconds
        bsp_servo_spin_duration(BSP_PORT_B, 1000, 2000);
        vTaskDelay(pdMS_TO_TICKS(3000));
        // Move back at maximum speed
        bsp_servo_spin(BSP_PORT_B, 2000); // Move to left (max speed)
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
