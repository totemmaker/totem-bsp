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
#define TASK_SIZE 2048

static uint8_t initialized = 0;
static uint8_t comm_mode_full_duplex = 0;
static volatile uint32_t request_reg;
static volatile TaskHandle_t request_task;
static volatile TimerHandle_t alive_timer;
static StaticTimer_t alive_timer_buffer;
static StackType_t receive_task_stack[TASK_SIZE];
static StaticTask_t receive_task_buffer;
static periph_driver_update_handler_t stream_func;

uint32_t bsp_driver_version;
uint32_t bsp_board_revision;

// Initialize UART interface to communicate with peripheral driver
static esp_err_t UART_init(uint32_t baud) {
    if (!initialized) {
        // Configure UART
        uart_config_t uart_config = {
            .baud_rate = baud,
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
        if (uart_driver_install(UART_PORT_NUM, UART_FIFO_LEN+1, 0, 0, NULL, 0) != ESP_OK)
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
// Encode driver command to serial buffer
static int frameEncode(uint16_t reg, uint32_t value, uint8_t buffer[8]) {
    if (reg & 0x7000) return -1;
    // Pack data to buffer
    int len = 2;
    uint8_t data[7];
    data[0] = reg >> 8;
    data[1] = reg;
    while (value) { data[len++] = value & 0xFF; value >>= 8; }
    // Compute checksum
    uint8_t sum = 0;
    for (int i=0; i<len; i++) {
        sum += data[i];
    }
    data[len++] = sum;
    // Byte stuffing
    uint8_t pos = 1;
    uint8_t firstPos = 0;
    uint8_t *posPtr = &firstPos;
    for (int i=0; i<len; i++) {
        if (data[i] != 0xFE) { buffer[i] = data[i]; ++pos; }
        else { *posPtr = pos; pos = 1; posPtr = &buffer[i]; }
    }
    *posPtr = pos;
    buffer[0] |= (firstPos-1) << 4;
    buffer[len++] = 0xFE;
    return len;
}
// Decode driver command from serial buffer
static int frameDecode(uint8_t buffer[], uint32_t length, uint16_t *reg, uint32_t *value) {
    if (length > 7) return -1;
    if (length < 3) return -1;
    uint8_t data[7];
    // Byte stuffing
    uint8_t pos = (buffer[0] >> 4) & 0x7; buffer[0] &= 0x8F;
    for (int i=0; i<length; i++, pos--) {
        if (pos) { data[i] = buffer[i]; }
        else {
            pos = buffer[i];
            if (pos == 0xFE) return -1;
            data[i] = 0xFE;
        }
    }
    // Validate frame size
    if (pos != 0) return -1;
    // Validate checksum
    uint8_t sum = 0;
    length--;
    for (int i=0; i<length; i++) {
        sum += data[i];
    }
    if (sum != data[length]) return -1;
    int frameSize = length;
    // Unpack frame
    *reg = 0;
    *reg |= data[0] << 8;
    *reg |= data[1];
    *value = 0;
    while (length > 2) { *value <<= 8; *value |= data[--length]; }
    return frameSize;
}
// ESP32 heart beat
static void alive_timer_handler(TimerHandle_t xTimer) {
    // Change period to intensive check
    if (xTimerGetPeriod(xTimer) != pdMS_TO_TICKS(100)) {
        xTimerChangePeriod(xTimer, pdMS_TO_TICKS(100), pdMS_TO_TICKS(50));
        alive_timer = xTimer;
    }
    // Send keep alive time
    periph_driver_write(PERIPH_DRIVER_SET_ALIVE, 250); // Maintain peripheral driver keep alive status
}
// UART receive task
static void receive_task_handler(void *context) {
    uint8_t data;
    uint8_t frame[8];
    int frameLen = 0;
    int num = 0;
    for (;;) {
        num = uart_read_bytes(UART_PORT_NUM, &data, sizeof(data), portMAX_DELAY);
        if (num != 1) continue;
        if (frameLen >= sizeof(frame)) { frameLen = 0; }
        if (data == 0xFE) {
            uint16_t reg; uint32_t value;
            int result = frameDecode(frame, frameLen, &reg, &value);
            if (result != -1) {
                if (request_reg && request_reg == reg) {
                    request_reg = 0;
                    xTaskNotify(request_task, value, eSetValueWithOverwrite);
                }
                stream_func(reg, value);
            }
            frameLen = 0;
            continue;
        }
        frame[frameLen++] = data;
    }
}
// Connect to peripheral driver
static esp_err_t periph_connect() {
    // Try to communicate over UART interface
    uint32_t chip_id = 0;
    for (int i=0; i<5; i++) {
        chip_id = periph_driver_read(PERIPH_DRIVER_GET_ID);
        if (chip_id == PERIPH_DRIVER_ID)
            break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Validate if communication established
    if (chip_id != PERIPH_DRIVER_ID) {
        UART_deinit();
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}
// Establish communication with peripheral driver
esp_err_t periph_driver_init(periph_driver_update_handler_t func) {
    esp_err_t err = ESP_FAIL;
    stream_func = func;
    // Initialize pins
    gpio_set_level(BSP_IO_DRIVER_DFU, 0);
    gpio_set_level(BSP_IO_DRIVER_RESET, 0);
    gpio_config_t io_conf = {0};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_SEL(BSP_IO_DRIVER_DFU)|GPIO_SEL(BSP_IO_DRIVER_RESET);
    gpio_config(&io_conf);
    // Restart driver
    gpio_set_level(BSP_IO_DRIVER_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(BSP_IO_DRIVER_RESET, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    // Initialize UART driver
    if (UART_init(115200) != ESP_OK)
        return ESP_ERR_NOT_FOUND;
    // Try to connect over UART interface
    err = periph_connect();
    if (err != ESP_OK) return err;
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
    // Initialize chip peripherals
    periph_driver_write(PERIPH_DRIVER_SET_INIT, 0 << 16 | 20000); // Slow decay, 20kHz
    // Keep alive chip for few time until ESP32 is fully initialized
    periph_driver_write(PERIPH_DRIVER_SET_ALIVE, 1700);
    // Start keep alive timer.
    // First time start after 1300ms to give time for ESP32 to initialize Bluetooth stack, which is blocking core
    // for a few hundred miliseconds. Later timer will roll back to 250ms.
    TimerHandle_t xTimer = xTimerCreateStatic("Driver_alive", pdMS_TO_TICKS(1300), pdTRUE, 0, alive_timer_handler, &alive_timer_buffer);
    if (xTimer == NULL) return ESP_ERR_NO_MEM;
    xTimerStart(xTimer, pdMS_TO_TICKS(50));
    // Increase communication speed (if driver firmware supports it)
    if (bsp_driver_version >= 160) {
        // Inform driver to change baudrate
        periph_driver_write(PERIPH_DRIVER_SET_BAUD, 460800);
        uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(50));
        vTaskDelay(pdMS_TO_TICKS(100));
        // Change baud rate
        UART_deinit();
        if (UART_init(460800) != ESP_OK) {
            return ESP_ERR_INVALID_STATE;
        }
        // Try to connect over UART interface
        err = periph_connect();
        if (err != ESP_OK) return err;
        // Start receiver task and enable full duplex mode
        if (stream_func) {
            xTaskCreateStatic(receive_task_handler, "Driver_receive", TASK_SIZE, NULL, 21, receive_task_stack, &receive_task_buffer);
            // Switch to full-duplex communication mode
            periph_driver_write(PERIPH_DRIVER_SET_COMM, 1);
            uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(50));
            comm_mode_full_duplex = 1;
        }
    }
    return err;
}
// Subscribe to streaming of driver register
esp_err_t periph_driver_subscribe(PeriphRegMap reg) {
    if (comm_mode_full_duplex == 0) return ESP_ERR_INVALID_STATE;
    return periph_driver_write(PERIPH_DRIVER_SET_STREAM, reg);
}
// Write value to peripheral driver register
esp_err_t periph_driver_write(PeriphRegMap reg, uint32_t value) {
    if (!initialized) return ESP_ERR_INVALID_STATE;
    uint8_t data[8]; int size;
    // Prepare data buffer
    if (comm_mode_full_duplex) {
        size = frameEncode(reg, value, data);
        if (size == -1) { return ESP_ERR_INVALID_ARG; }
    }
    else {
        memcpy(data, &reg, 2);
        memcpy(&data[2], &value, 4);
        size = 6;
    }
    // Write data to UART
    int num = uart_write_bytes(UART_PORT_NUM, (uint8_t*)data, size);
    if (num != size) return ESP_ERR_NOT_FINISHED;
    // Delay heart beat if command was sent
    if (bsp_driver_version >= 160 && alive_timer) {
        xTimerChangePeriod(alive_timer, pdMS_TO_TICKS(100), 0);
    }
    return ESP_OK;
}
// Read value from peripheral driver register
uint32_t periph_driver_read(PeriphRegMap reg) {
    if (!initialized) return 0;
    uint32_t result = 0;
    if (comm_mode_full_duplex) {
        // Request register read
        uint8_t data[8]; int size;
        size = frameEncode(reg | 0x8000, 0, data);
        if (size == -1) return 0;
        // In full-duplex mode wait for response in receive task
        request_task = xTaskGetCurrentTaskHandle();
        request_reg = reg;
        uart_write_bytes(UART_PORT_NUM, data, size);
        xTaskNotifyWait(0, 0, &result, pdMS_TO_TICKS(100));
        return result;
    }
    else {
        uint16_t data = reg | 0x8000;
        // Discard any unread bytes
        uart_flush_input(UART_PORT_NUM);
        // Request register read
        uart_write_bytes(UART_PORT_NUM, (uint8_t*)&data, sizeof(data));
        uart_wait_tx_done(UART_PORT_NUM, pdMS_TO_TICKS(10));
        // In half-duplex mode wait for response right away
        int num = uart_read_bytes(UART_PORT_NUM, (uint8_t*)&result, sizeof(result), pdMS_TO_TICKS(10));
        return num == 4 ? result : 0;
    }
}
