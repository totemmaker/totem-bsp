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
#include "driver/twai.h"
#include "totem_test.h"
#include "bsp/totem-bsp.h"

#define BSP_CAN_FlagExt 0x80000000U
#define BSP_CAN_FlagRTR 0x40000000U
#define BSP_CAN_id      0x3FFFFFFFU

struct CanMsg {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
};

static CanMsg can_rcv_msg[15];
static int can_rcv_cnt;

static void twai_receive_task(void *context) {
    twai_message_t message;
    for (;;) {
        // Loop to receive CAN packets and forward to callback
        if (twai_receive(&message, portMAX_DELAY) == ESP_OK) {
            // Store received message
            TEST_ASSERT(can_rcv_cnt < (sizeof(can_rcv_msg)/sizeof(can_rcv_msg[0])));
            TEST_ASSERT(message.data_length_code <= 8);
            can_rcv_msg[can_rcv_cnt].id = message.identifier;
            if (message.extd) can_rcv_msg[can_rcv_cnt].id |= BSP_CAN_FlagExt;
            if (message.rtr) can_rcv_msg[can_rcv_cnt].id |= BSP_CAN_FlagRTR;
            can_rcv_msg[can_rcv_cnt].len = message.data_length_code;
            memcpy(can_rcv_msg[can_rcv_cnt++].data, message.data, message.data_length_code);
        }
    }
}

TEST_CASE("Test CAN bus", "[CAN]") {
    // Initialize TWAI driver (in self test mode)
    const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(BSP_IO_CAN_TX, BSP_IO_CAN_RX, TWAI_MODE_NO_ACK);
    const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    // Initialize TWAI driver
    TEST_ERROR(ESP_OK, twai_driver_install(&g_config, &t_config, &f_config));
    xTaskCreatePinnedToCore(twai_receive_task, "TWAI_receive", 2048, NULL, 8, NULL, tskNO_AFFINITY);
    // Initialize CAN library
    bsp_cmd_write(BSP_CAN_STATE, 0, 1); // On transceiver
    TEST_ERROR(ESP_OK, twai_start()); // Start twai driver
    // Prepare test packets
    CanMsg msgList[] = {
        /*0*/{0xFF, 2, {0xA, 0xB}},
        /*1*/{0xE7, 8, {0, 1, 2, 3, 4, 5, 6, 7}},
        /*2*/{0xAA|BSP_CAN_FlagExt, 2, {3, 5}},
        /*3*/{0xAA|BSP_CAN_FlagRTR, 3, {1, 2, 3}},
        /*4*/{0x1B, 0, {}},
        /*5*/{0x7FF, 1, {3}},
        /*6*/{0x1FFFFFFF, 1, {3}},
        /*7*/{0x1FFFFFFF|BSP_CAN_FlagRTR, 1, {3}},
    };
    // Send all test packets
    for (int i=0; i<(sizeof(msgList)/sizeof(msgList[0])); i++) {
        TEST_MESSAGE("i=%d", i);
        // Prepare message
        twai_message_t message;
        memset(&message, 0, sizeof(message));
        message.data_length_code = msgList[i].len;
        message.identifier = msgList[i].id & BSP_CAN_id;
        message.extd = !!(msgList[i].id & BSP_CAN_FlagExt);
        message.rtr = !!(msgList[i].id & BSP_CAN_FlagRTR);
        memcpy(message.data, msgList[i].data, msgList[i].len);
        message.self = 1;
        // Put message to queue
        TEST_ERROR(ESP_OK, twai_transmit(&message, 50));
    }
    // Wait for all packets to receive in "on_can_msg" handler
    TEST_DELAY(100);
    TEST_EQUAL(sizeof(msgList)/sizeof(msgList[0]), can_rcv_cnt);
    // Test received packets if data match
    for (int i=0; i<can_rcv_cnt; i++) {
        // Validate message ID (6 and 7 are special cases)
        TEST_MESSAGE("i=%d", i);
        if (i == 6) TEST_EQUAL(0x7FF, can_rcv_msg[i].id);
        else if (i == 7) TEST_EQUAL(0x7FF|BSP_CAN_FlagRTR, can_rcv_msg[i].id);
        else TEST_EQUAL(msgList[i].id, can_rcv_msg[i].id);
        // Validate message data lenght
        TEST_MESSAGE("i=%d", i);
        TEST_EQUAL(msgList[i].len, can_rcv_msg[i].len);
        // Validate message data (RTR should not contain data)
        TEST_MESSAGE("i=%d", i);
        if (msgList[i].id & BSP_CAN_FlagRTR)
            TEST_EQUAL_ARRAY(0, can_rcv_msg[i].data, msgList[i].len);
        else
            TEST_EQUAL_ARRAYS(msgList[i].data, can_rcv_msg[i].data, msgList[i].len);
    }
}
