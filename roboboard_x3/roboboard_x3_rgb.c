/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "led_strip.h"
#include "include/bsp/rgb.h"

#include "include/bsp/roboboard_x3.h"

// GPIO assignment
#define RGB_GPIO  BSP_IO_RGB
// Numbers of the LED in the strip
#define RGB_CNT 4
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RGB_RMT_RES_HZ  (10 * 1000 * 1000)
// "led_strip" handler
static led_strip_handle_t led_strip;
// Initialize library
esp_err_t bsp_rgb_init() {
    // LED strip general initialization
    led_strip_config_t strip_config = {
        .strip_gpio_num = RGB_GPIO,               // The GPIO that connected to the LED strip's data line
        .max_leds = RGB_CNT,                      // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = 0,                    // whether to invert the output signal
    };
    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
#else
        .clk_src = RMT_CLK_SRC_DEFAULT,  // different clock source can lead to different power consumption
        .resolution_hz = RGB_RMT_RES_HZ, // RMT counter clock frequency
        .flags.with_dma = 0,             // DMA feature is available on ESP target like ESP32-S3
#endif
    };
    // LED Strip object handle
    return led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
}
// Write color to LED
esp_err_t bsp_rgb_rgb(int8_t port, uint8_t r, uint8_t g, uint8_t b) {
    if (port == -1) {
        for (int i=0; i<RGB_CNT; i++) { 
            esp_err_t err;
            err = led_strip_set_pixel(led_strip, i, r, g, b);
            if (err) return err;
        }
        return ESP_OK;
    }
    else return led_strip_set_pixel(led_strip, port, r, g, b);
}
// Write color to LED
esp_err_t bsp_rgb_hex(int8_t port, uint32_t hex) {
    return bsp_rgb_rgb(port, (hex >> 16)&0xFF, (hex >> 8)&0xFF, hex&0xFF);
}
// Write color to LED
esp_err_t bsp_rgb_hsv(int8_t port, uint16_t hue, uint8_t saturation, uint8_t value) {
    if (port == -1) {
        for (int i=0; i<RGB_CNT; i++) { 
            esp_err_t err;
            err = led_strip_set_pixel_hsv(led_strip, i, hue, saturation, value);
            if (err) return err;
        }
        return ESP_OK;
    }
    else return led_strip_set_pixel_hsv(led_strip, port, hue, saturation, value);
}
// Clear strip
esp_err_t bsp_rgb_clear() {
    return led_strip_clear(led_strip);
}
// Update strip
esp_err_t bsp_rgb_update() {
    return led_strip_refresh(led_strip);
}
