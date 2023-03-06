/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#include "tbus.h"

#define TASK_SIZE 10240
#define TASK_CORE 1

static TaskHandle_t xHandle;
static StaticTask_t xTaskBuffer;
static StackType_t xStack[TASK_SIZE];

static void twai_receive_task_callback(void *context);

static tbus_receive_handle_t tbus_callback;
static void *tbus_callback_args;
static uint8_t tbus_running;
static uint32_t tbus_enable_pin;

esp_err_t tbus_init(uint32_t pin_en, uint32_t pin_tx, uint32_t pin_rx) {
    // Initialize transceiver enable pin
    tbus_enable_pin = pin_en;
    // Initialize CAN transceiver enable pin
    gpio_set_level(tbus_enable_pin, 1); // Turn transceiver Off
    gpio_config_t pGPIOConfig = {0};
    pGPIOConfig.mode = GPIO_MODE_OUTPUT;
    pGPIOConfig.pin_bit_mask = ((uint64_t)1)<<tbus_enable_pin;
    gpio_config(&pGPIOConfig);
    // Configure TWAI parameters
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(pin_tx, pin_rx, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 25;
    g_config.intr_flags = 0;
    const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    // Initialize TWAI driver
    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) return err;
    // Start receive task
    xHandle = xTaskCreateStaticPinnedToCore(twai_receive_task_callback, "TWAI_receive", TASK_SIZE, NULL, 20, xStack, &xTaskBuffer, TASK_CORE);
    return xHandle != NULL ? ESP_OK : ESP_FAIL;
}

void tbus_callback_register(tbus_receive_handle_t receive_handler, void *args) {
    tbus_callback = receive_handler;
    tbus_callback_args = args;
}

void tbus_enable(uint8_t state) {
    if (!tbus_callback) return;
    // Start / stop TWAI driver
    state ? twai_start() : twai_stop();
    // Turn On / Off transceiver
    gpio_set_level(tbus_enable_pin, state ? 0 : 1);
    // Resume / suspend receive task
    state ? vTaskResume(xHandle) : vTaskSuspend(xHandle);
    tbus_running = state;
}

uint8_t tbus_is_enabled(void) {
    return tbus_running;
}

esp_err_t tbus_send(uint32_t id, uint8_t *data, uint8_t len) {
    if (!tbus_running) return ESP_ERR_INVALID_STATE;
    // Prepare message
    twai_message_t message;
    message.flags = TWAI_MSG_FLAG_NONE;
    message.flags |= (id & BSP_TBUS_FlagExt) ? TWAI_MSG_FLAG_EXTD : 0;
    message.flags |= (id & BSP_TBUS_FlagRTR) ? TWAI_MSG_FLAG_RTR : 0;
    // Set identifier
    message.identifier = id & ((message.flags & TWAI_MSG_FLAG_EXTD) ? TWAI_EXTD_ID_MASK : TWAI_STD_ID_MASK);
    message.data_length_code = len;
    // Copy data
    memcpy(message.data, data, len);
    // Put message to queue
    return twai_transmit(&message, 0);
}

static void twai_receive_task_callback(void *context) {
    twai_message_t message;
    for (;;) {
        if (twai_receive(&message, portMAX_DELAY) == ESP_OK) {
            uint32_t id = message.identifier;
            id |= (message.flags & TWAI_MSG_FLAG_EXTD) ? BSP_TBUS_FlagExt : 0;
            id |= (message.flags & TWAI_MSG_FLAG_RTR) ? BSP_TBUS_FlagRTR : 0;
            tbus_callback(tbus_callback_args, id, message.data, message.data_length_code);
        }
    }
}
