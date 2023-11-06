/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "periph_driver.h"
#include "bsp/roboboard_x4.h"

#define GPIO_SEL(pin)   ((uint64_t)(((uint64_t)1)<<(pin)))

#define PERIPH_DRIVER_ID    0x98
#define UART_PORT_NUM       CONFIG_BSP_UART_NUM // UART_NUM_2 by default

static TimerHandle_t tickTimer;
static StaticTimer_t tickTimerBuffer;
static uint8_t initialized = 0;
uint32_t bsp_board_revision;
uint32_t bsp_driver_version;

// Initialize UART interface to communicate with peripheral driver
static esp_err_t UART_init(void) {
    if (!initialized) {
        // Configure UART
        uart_config_t uart_config = {
            .baud_rate = 115200,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_EVEN,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
            .source_clk = UART_SCLK_DEFAULT,
#endif
        };
        uart_param_config(UART_PORT_NUM, &uart_config);
        // Set pins
        uart_set_pin(UART_PORT_NUM, BSP_IO_DRIVER_UART_RX, BSP_IO_DRIVER_UART_TX, 0, 0);
        // Start driver
        if (uart_driver_install(UART_PORT_NUM, 1024 * 2, 0, 0, NULL, 0) != ESP_OK)
            return ESP_FAIL;
        initialized = 1;
    }
    return ESP_OK;
}
// Deinitialize UART interface
static void UART_deinit(void) {
    uart_driver_delete(UART_PORT_NUM);
    initialized = 0;
}

static void vTimerCallback(TimerHandle_t xTimer) {
    // Change period to intensive check
    if (xTimerGetPeriod(xTimer) != pdMS_TO_TICKS(100))
        xTimerChangePeriod(xTimer, pdMS_TO_TICKS(100), 0);
    // Send keep alive time
    periph_driver_write(PERIPH_DRIVER_SET_ALIVE, 250); // Maintain peripheral driver keep alive status
}

esp_err_t periph_driver_init(void) {
    // Initialize pins
    gpio_set_level(BSP_IO_DRIVER_DFU, 0);
    gpio_set_level(BSP_IO_DRIVER_RESET, 0);
    gpio_config_t io_conf = {0};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_SEL(BSP_IO_DRIVER_DFU)|GPIO_SEL(BSP_IO_DRIVER_RESET);
    gpio_config(&io_conf);
    // Try to establish communication
    gpio_set_level(BSP_IO_DRIVER_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(BSP_IO_DRIVER_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    // Initialize UART driver
    if (UART_init() != ESP_OK)
        return ESP_ERR_NOT_FOUND;
    // Try to communicate over UART interface
    uint32_t chip_id = 0;
    for (int i=0; i<5; i++) {
        chip_id = periph_driver_read(PERIPH_DRIVER_GET_ID);
        if (chip_id == PERIPH_DRIVER_ID)
            break;
        vTaskDelay(pdMS_TO_TICKS(11));
    }
    // Validate if communication established
    if (chip_id != PERIPH_DRIVER_ID) {
        UART_deinit();
        return ESP_ERR_TIMEOUT;
    }
    // Read information
    bsp_board_revision = periph_driver_read(PERIPH_DRIVER_GET_BOARD);
    bsp_driver_version = periph_driver_read(PERIPH_DRIVER_GET_VERSION);
    // Check if data is valid
    if (bsp_board_revision == 0 || bsp_driver_version == 0)
        return ESP_ERR_INVALID_RESPONSE;
    // Convert version to correct format
    if (bsp_driver_version < 100) bsp_driver_version *= 10;
    if (bsp_driver_version > 10000) {
        uint32_t v = (bsp_driver_version >> 24) & 0xF;
        v *= 10;
        v += (bsp_driver_version >> 16) & 0xFF;
        v *= 10;
        v += (bsp_driver_version >> 8) & 0xFF;
        bsp_driver_version = v;
    }
    // Reinitialize chip to ignore previous configuration
    periph_driver_write(PERIPH_DRIVER_SET_INIT, 0);
    // Keep alive chip for few time until ESP32 is fully initialized
    periph_driver_write(PERIPH_DRIVER_SET_ALIVE, 1700);
    // Start keep alive timer.
    // First time start after 1300ms to give time for ESP32 to initialize Bluetooth stack, which is blocking core
    // for a few hundred miliseconds. Later timer will roll back to 100ms.
    tickTimer = xTimerCreateStatic("Driver_alive", pdMS_TO_TICKS(1300), pdTRUE, 0, vTimerCallback, &tickTimerBuffer);
    if (tickTimer == NULL) return ESP_ERR_NO_MEM;
    xTimerStart(tickTimer, 0);
    return ESP_OK;
}
// Write peripheral register value
void periph_driver_write(PeriphRegMap reg, uint32_t value) {
    if (!initialized) return;
    uint8_t data[6];
    memcpy(data, &reg, 2);
    memcpy(&data[2], &value, 4);
    uart_write_bytes(UART_PORT_NUM, (uint8_t*)data, sizeof(data));
}
// Read peripheral register value
uint32_t periph_driver_read(PeriphRegMap reg) {
    if (!initialized) return 0;
    uint32_t result = 0;
    uint16_t data = reg | 0x8000;
    uart_write_bytes(UART_PORT_NUM, (uint8_t*)&data, sizeof(data));
    uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(100));
    int num = uart_read_bytes(UART_PORT_NUM, (uint8_t*)&result, sizeof(result), pdMS_TO_TICKS(100));
    return num == 4 ? result : 0;
}
