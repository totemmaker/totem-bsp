#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/i2c.h"

#include "bsp/totem-bsp.h"

/**
 * RoboBoard X3 has Qwiic connector for external I2C modules.
 * IMU is also connected to same SDA, SCL pins as Qwiic connector. It will have (IMU) tag.
*/

esp_err_t i2c_master_is_device(i2c_port_t i2c_num, uint8_t device_address, TickType_t ticks_to_wait) {
    uint8_t buffer[300] = { 0 };
    i2c_cmd_handle_t handle = i2c_cmd_link_create_static(buffer, sizeof(buffer));
    i2c_master_start(handle);
    i2c_master_write_byte(handle, device_address << 1 | I2C_MASTER_WRITE, true);
    i2c_master_stop(handle);
    esp_err_t err = i2c_master_cmd_begin(i2c_num, handle, ticks_to_wait);
    i2c_cmd_link_delete_static(handle);
    return err;
}

void run_example_qwiic(void) {
    ESP_LOGI("Board", "--- Initialize board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Enable external 3V LDO regulator
    ESP_ERROR_CHECK(bsp_board_set_3V(1));
    // Initialize I2C on Qwiic port pins
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_IO_I2C_SDA,
        .scl_io_num = BSP_IO_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = 100000, // Set to 100kHz
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
    // Scan for connected modules
    uint8_t imuAddr = 0x6B; // QMI8658
    while (1) {
        ESP_LOGI("Board", "Run scan...");
        bool found = false;
        for (uint8_t address=1; address<126; address++) {
            // Try to communicate with a device
            if (i2c_master_is_device(I2C_NUM_0, address, pdTICKS_TO_MS(50)) == ESP_OK) {
                // Device has responded. Print address
                if (address == imuAddr) ESP_LOGI("Board", "Found: 0x%X (IMU)", address);
                else ESP_LOGI("Board", "Found: 0x%X", address);
                found = true;
            }
        }
        if (!found) ESP_LOGW("Board", "No devices found.");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
