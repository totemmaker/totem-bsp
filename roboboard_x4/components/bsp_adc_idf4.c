
/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "driver/adc.h"
#include "esp_adc_cal.h"

static adc_channel_t adc1_channel;
static esp_adc_cal_characteristics_t adc1_chars;

void bsp_adc1_init(adc_channel_t channel) {
    adc1_channel = channel;
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten((adc1_channel_t)adc1_channel, ADC_ATTEN_DB_2_5);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_2_5, ADC_WIDTH_BIT_12, 1100, &(adc1_chars));
}

uint32_t bsp_adc1_get_raw() {
    return adc1_get_raw((adc1_channel_t)adc1_channel);
}

uint32_t bsp_adc1_raw_to_voltage(uint32_t adc) {
    return esp_adc_cal_raw_to_voltage(adc, &(adc1_chars));
}