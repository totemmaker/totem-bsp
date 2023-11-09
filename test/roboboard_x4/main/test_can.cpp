/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "totem_test.h"
#include "bsp/totem-bsp.h"

struct CanMsg {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
};

static CanMsg can_rcv_msg[15];
static int can_rcv_cnt;
void on_can_msg(void *args, uint32_t id, uint8_t *data, uint8_t len) {
    // Store received message
    TEST_ASSERT(can_rcv_cnt < (sizeof(can_rcv_msg)/sizeof(can_rcv_msg[0])));
    TEST_ASSERT(len <= 8);
    can_rcv_msg[can_rcv_cnt].id = id;
    can_rcv_msg[can_rcv_cnt].len = len;
    memcpy(can_rcv_msg[can_rcv_cnt++].data, data, len);
}

TEST_CASE("Test CAN bus", "[CAN]") {
    // Check if initialization state is validated
    uint8_t dummy[9];
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
    TEST_ERROR(ESP_ERR_INVALID_STATE, bsp_can_send(0, dummy, 2));
#else
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_can_send(0, dummy, 2));
#endif
    // Initialize TWAI driver (in self test mode)
    TEST_ERROR(ESP_OK, bsp_twai_init(1));
    // Initialize CAN library
    TEST_ERROR(ESP_OK, bsp_can_init(on_can_msg, NULL));
    TEST_ERROR(ESP_OK, bsp_can_enable(1));
    // Try sending more than 8 bytes
    TEST_ERROR(ESP_ERR_INVALID_SIZE, bsp_can_send(0, dummy, 9));
    // Error if data has null pointer
    TEST_ERROR(ESP_ERR_INVALID_ARG, bsp_can_send(0, NULL, 7));
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
        TEST_ERROR(ESP_OK, bsp_can_send(msgList[i].id, msgList[i].data, msgList[i].len));
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
