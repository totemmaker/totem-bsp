/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef PRIV_INCLUDE_BSP_ADC
#define PRIV_INCLUDE_BSP_ADC

#include "hal/adc_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize specified ADC1 channel
*/
void bsp_adc1_init(adc_channel_t channel);
/**
 * Read raw ADC measurement
 * Returns: 12 bit measurement
*/
uint32_t bsp_adc1_get_raw();
/**
 * Convert raw ADC measurement to pin voltage
 * `adc` - raw 12 bit measurement
 * Returns: pin millivolts
*/
uint32_t bsp_adc1_raw_to_voltage(uint32_t adc);

#ifdef __cplusplus
}
#endif

#endif /* PRIV_INCLUDE_BSP_ADC */
