#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/twai.h"

#include "bsp/totem-bsp.h"

/**
 * RoboBoard X4 has CAN bus. For wiring view: https://docs.totemmaker.net/roboboard/api/can/
*/

// CAN is started in loopback mode which receives all sent packets. This will not require any CAN bus wires connected.
// In order to connect to real CAN network - comment out 'CAN_SELFTEST' definition
#define CAN_SELFTEST

// Task for message listening
static void twai_receive_task(void *context) {
    twai_message_t message;
    for (;;) {
        // Loop to receive CAN packets and forward to callback
        if (twai_receive(&message, portMAX_DELAY) == ESP_OK) {
            char dataStr[20], *strPtr = dataStr;
            // Format data string
            if (message.flags & TWAI_MSG_FLAG_RTR) {
                strPtr += sprintf(strPtr, " remote request");
            }
            else {
                for (int i=0; i<message.data_length_code; i++) {
                    strPtr += sprintf(strPtr, " %02X", message.data[i]);
                }
            }
            // Print received message
            ESP_LOGI("CAN", "MSG: id(%s): %x, Data(%d):%s",
                (message.flags & TWAI_MSG_FLAG_EXTD) ? "29b" : "11b",
                (int)message.identifier,
                (int)message.data_length_code,
                dataStr
            );
        }
    }
}

static void on_bsp_event(uint32_t evt, uint32_t portID, uint32_t value, void *arg) {
    // Interrupt called on button press (and release)
    if (evt == BSP_EVT_BUTTON) {
        // Send button state on press
        twai_message_t message = {0};
        // Set identifier to 0xABCD
        message.identifier = 0xABCD;
        message.data_length_code = 1;
        message.extd = 1; // Send extended packet
        message.data[0] = value; // "value" contains button state
#ifdef CAN_SELFTEST
        message.self = 1;
#endif
        // Put message to queue
        twai_transmit(&message, 0);
    }
}

void run_example_can(void) {
    ESP_LOGI("Board", "--- Initialize board ---");
    // Initialize board components
    ESP_ERROR_CHECK(bsp_board_init());
    // Turn LED on
    ESP_ERROR_CHECK(bsp_board_set_led(1));
    // Register button interrupt
    ESP_ERROR_CHECK(bsp_board_reg_event(on_bsp_event, NULL));
    // Test CAN messaging
    ESP_LOGI("Board", "--- Testing CAN reception ---");
    // Configure TWAI parameters
#ifdef CAN_SELFTEST      // Setup TWAI for loopback mode
    const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(BSP_IO_CAN_TX, BSP_IO_CAN_RX, TWAI_MODE_NO_ACK);
#else                     // Setup TWAI for real CAN network
    const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(BSP_IO_CAN_TX, BSP_IO_CAN_RX, TWAI_MODE_NORMAL);
#endif
    // Select 500kbits speed
    const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    // Disable hardware filters
    const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    // Initialize TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    // Start receive task
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_receive", 2048, NULL, 9, NULL, tskNO_AFFINITY);
    // Turn CAN transceiver on
    bsp_board_set_can(1);
    // Start twai driver
    ESP_ERROR_CHECK(twai_start());
    // Send messages
    int counter = 0;
    while (1) {
        // Prepare message
        twai_message_t message = {0};
        // Set identifier
        message.identifier = counter++;
        message.data_length_code = 3;
        message.data[0] = 1;
        message.data[1] = 2;
        message.data[2] = 3;
#ifdef CAN_SELFTEST
        message.self = 1;
#endif
        // Put message to queue
        ESP_ERROR_CHECK(twai_transmit(&message, pdMS_TO_TICKS(50)));
        // Wait 300ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
