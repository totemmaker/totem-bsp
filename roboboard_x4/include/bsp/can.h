/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_BSP_CAN
#define INCLUDE_BSP_CAN

#include "esp_err.h"

#define BSP_CAN_FlagExt 0x80000000U
#define BSP_CAN_FlagRTR 0x40000000U
#define BSP_CAN_id      0x3FFFFFFFU

/**
 * TBUS (CAN) packets receive handler
 * `args` - context
 * `id`   - identifier
 * `data` - buffer (8 bytes)
 * `len`  - length of data array
*/
typedef void (*bsp_can_receive_handle_t)(void *args, uint32_t id, uint8_t *data, uint8_t len);
/**
 * Register TBUS (CAN) packets receiver
 * `receive_handler` - function to receive CAN messages
 * `args`            - context passed to handler (NULL if not required)
*/
esp_err_t bsp_twai_init(int selftest);
esp_err_t bsp_can_init(bsp_can_receive_handle_t receive_handler, void *args);

esp_err_t bsp_can_enable(uint8_t state);
/**
 * Send (CAN) packet to TBUS network
 * `id`   - identifier
 * `data` - buffer (8 bytes)
 * `len`  - length of data array
* Returns:
 *  `0`  - success
 * error - not enabled or TWAI error
*/
esp_err_t bsp_can_send(uint32_t id, uint8_t *data, uint8_t len);
esp_err_t bsp_can_send_rtr(uint32_t id, uint8_t len);

#endif /* INCLUDE_BSP_CAN */
