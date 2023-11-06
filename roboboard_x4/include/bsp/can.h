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

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Extended identifier flag
#define BSP_CAN_FlagExt 0x80000000U
/// @brief Remote request flag
#define BSP_CAN_FlagRTR 0x40000000U
/// @brief Mask of identifier (strip away flags)
#define BSP_CAN_id      0x3FFFFFFFU

/// @brief CAN packet receive handler
/// @param args context pointer passed to "bsp_can_init"
/// @param id message identifier (use Flags to check if Ext or RTR)
/// @param data data array sent
/// @param len [0:8] data length
typedef void (*bsp_can_receive_handle_t)(void *args, uint32_t id, uint8_t *data, uint8_t len);

/// @brief Helper function to initialize TWAI driver at 500kbps. For custom parameters use your own initialization code
/// @param selftest 0 - normal operation, 1 - loopback mode
/// @return ESP error
esp_err_t bsp_twai_init(int selftest);

/// @brief Initialize CAN library
/// @param receive_handler received packets callback
/// @param args pointer to context (NULL if not required)
/// @return ESP error
esp_err_t bsp_can_init(bsp_can_receive_handle_t receive_handler, void *args);

/// @brief Turn CAN functionality on / off
/// @param state 0 - off, 1 - on
/// @return ESP error
esp_err_t bsp_can_enable(uint8_t state);

/// @brief Send CAN packet
/// @param id identifier. Use Flags for Extended or RTR packet
/// @param data data array
/// @param len [0:8] data length
/// @return ESP error
esp_err_t bsp_can_send(uint32_t id, uint8_t *data, uint8_t len);

/// @brief Send standard CAN packet
/// @param id [0:0x7FF] (11b) identifier
/// @param data data array
/// @param len [0:8] data length
/// @return ESP error
inline esp_err_t bsp_can_send_std(uint32_t id, uint8_t *data, uint8_t len) {
    return bsp_can_send(id, data, len);
}

/// @brief Send extended CAN packet
/// @param id [0:0x1FFFFFFF] (29b) identifier
/// @param data data array
/// @param len [0:8] data length
/// @return ESP error
inline esp_err_t bsp_can_send_ext(uint32_t id, uint8_t *data, uint8_t len) {
    return bsp_can_send(id | BSP_CAN_FlagExt, data, len);
}

/// @brief Send standard remote request frame (no data)
/// @param id [0:0x7FF] (11b) identifier
/// @param data_len [0:8] data length
/// @return ESP error
inline esp_err_t bsp_can_send_std_rtr(uint32_t id, uint8_t data_len) {
    return bsp_can_send(id | BSP_CAN_FlagRTR, NULL, data_len);
}

/// @brief Send extended remote request frame (no data)
/// @param id [0:0x1FFFFFFF] (29b) identifier
/// @param data_len [0:8] data length
/// @return ESP error
inline esp_err_t bsp_can_send_ext_rtr(uint32_t id, uint8_t data_len) {
    return bsp_can_send(id | BSP_CAN_FlagExt | BSP_CAN_FlagRTR, NULL, data_len);
}

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_BSP_CAN */
