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

#include "bsp/can.h"
#include "bsp/roboboard_x4.h"

#define TASK_SIZE 2048
#define TASK_CORE 0

static uint8_t twai_selftest = 0;
static TaskHandle_t xHandle;
static StaticTask_t xTaskBuffer;
static StackType_t xStack[TASK_SIZE];

static void twai_receive_task_callback(void *context);

static bsp_can_receive_handle_t can_callback;
static void *can_callback_args;
static uint8_t can_running;

esp_err_t bsp_twai_init(int selftest) {
    // Configure TWAI parameters
    twai_selftest = !!selftest;
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(BSP_IO_CAN_TX, BSP_IO_CAN_RX, (twai_selftest) ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 25;
    g_config.intr_flags = 0;
    const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    // Initialize TWAI driver
    return twai_driver_install(&g_config, &t_config, &f_config);
}
esp_err_t bsp_can_init(bsp_can_receive_handle_t receive_handler, void *args) {
    can_callback = receive_handler;
    can_callback_args = args;
    // Start receive task
    xHandle = xTaskCreateStaticPinnedToCore(twai_receive_task_callback, "TWAI_receive", TASK_SIZE, NULL, 21, xStack, &xTaskBuffer, TASK_CORE);
    return xHandle != NULL ? ESP_OK : ESP_FAIL;
}

esp_err_t bsp_can_enable(uint8_t state) {
    esp_err_t err = ESP_FAIL;
    if (state) {
        bsp_cmd_write(BSP_CAN_STATE, 0, 1); // On transceiver
        vTaskResume(xHandle); // Resume RX task
        err = twai_start(); // Start twai driver
    }
    else {
        err = twai_stop(); // Stop twai driver
        vTaskSuspend(xHandle); // Stop RX task
        bsp_cmd_write(BSP_CAN_STATE, 0, 0); // Off transceiver
    }
    can_running = state;
    return err;
}

esp_err_t bsp_can_send(uint32_t id, uint8_t *data, uint8_t len) {
    if (!can_running) return ESP_ERR_INVALID_STATE;
    if (len > 8) return ESP_ERR_INVALID_SIZE;
    // Prepare message
    twai_message_t message;
    message.flags = TWAI_MSG_FLAG_NONE;
    message.flags |= (id & BSP_CAN_FlagExt) ? TWAI_MSG_FLAG_EXTD : 0;
    message.flags |= (id & BSP_CAN_FlagRTR) ? TWAI_MSG_FLAG_RTR : 0;
    // Set identifier
    message.identifier = id & ((message.flags & TWAI_MSG_FLAG_EXTD) ? TWAI_EXTD_ID_MASK : TWAI_STD_ID_MASK);
    message.data_length_code = len;
    message.self = twai_selftest;
    // Copy data
    if (!(id & BSP_CAN_FlagRTR)) {
        if (data == NULL && len > 0) return ESP_ERR_INVALID_ARG;
        memcpy(message.data, data, len);
    }
    // Put message to queue
    return twai_transmit(&message, 50);
}

esp_err_t bsp_can_send_rtr(uint32_t id, uint8_t len) {
    return bsp_can_send(BSP_CAN_FlagRTR | id, NULL, len);
}

static void twai_receive_task_callback(void *context) {
    twai_message_t message;
    for (;;) {
        if (twai_receive(&message, portMAX_DELAY) == ESP_OK) {
            uint32_t id = message.identifier;
            id |= (message.flags & TWAI_MSG_FLAG_EXTD) ? BSP_CAN_FlagExt : 0;
            id |= (message.flags & TWAI_MSG_FLAG_RTR) ? BSP_CAN_FlagRTR : 0;
            if (can_callback) can_callback(can_callback_args, id, message.data, message.data_length_code);
        }
    }
}
