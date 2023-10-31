/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static adc_channel_t adc1_channel;
static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle;

void bsp_adc1_init(adc_channel_t channel) {
    adc1_channel = channel;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_2_5,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);
    adc_oneshot_config_channel(adc1_handle, adc1_channel, &config);

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_2_5,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle) == ESP_OK)
        return;
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_2_5,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle) == ESP_OK)
        return;
#endif
}

uint32_t bsp_adc1_get_raw() {
    int adc = 0;
    adc_oneshot_read(adc1_handle, adc1_channel, &adc);
    return adc;
}

uint32_t bsp_adc1_raw_to_voltage(uint32_t adc) {
    int voltage = 0;
    adc_cali_raw_to_voltage(adc1_cali_handle, adc, &voltage);
    return voltage;
}