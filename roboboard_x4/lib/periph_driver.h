/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef PRIV_INCLUDE_PERIPH_DRIVER
#define PRIV_INCLUDE_PERIPH_DRIVER

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "periph_regmap.h"

// Contains X4 board revision
extern uint32_t bsp_board_revision;
// Contains X4 driver firmware version
extern uint32_t bsp_driver_version;

/**
 * Establish communication with peripheral driver
 * Returns:
 * ESP_ERR_NOT_FOUND - failed to initialize UART_NUM_2
 * ESP_ERR_NO_MEM - failed to create FreeRTOS task
 * ESP_ERR_TIMEOUT - peripheral driver is not responding
 * ESP_ERR_INVALID_RESPONSE - failed to read peripheral registers
*/
esp_err_t periph_driver_init(void);

/**
 * Write value to peripheral driver register
 * `reg` - register from PeriphRegMap list
 * `value` - 32 bit value
*/
void periph_driver_write(PeriphRegMap reg, uint32_t value);
/**
 * Read value from peripheral driver register
 * `reg` - register from PeriphRegMap list
 * Returns: 32 bit value
*/
uint32_t periph_driver_read(PeriphRegMap reg);

#ifdef __cplusplus
}
#endif

#endif /* PRIV_INCLUDE_PERIPH_DRIVER */
