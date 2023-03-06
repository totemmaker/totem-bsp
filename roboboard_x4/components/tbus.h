/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef PRIV_INCLUDE_TBUS
#define PRIV_INCLUDE_TBUS

#include "esp_err.h"

#define BSP_TBUS_FlagExt 0x80000000U
#define BSP_TBUS_FlagRTR 0x40000000U

typedef void (*tbus_receive_handle_t)(void *args, uint32_t id, uint8_t *data, uint8_t len);

esp_err_t tbus_init(uint32_t pin_en, uint32_t pin_tx, uint32_t pin_rx);
void tbus_callback_register(tbus_receive_handle_t receive_handler, void *args);
void tbus_enable(uint8_t state);
uint8_t tbus_is_enabled(void);
esp_err_t tbus_send(uint32_t id, uint8_t *data, uint8_t len);


#endif /* PRIV_INCLUDE_TBUS */
