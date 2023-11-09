/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_BSP_RGB_H
#define INCLUDE_BSP_RGB_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @brief Initialize library
/// @return ESP error
esp_err_t bsp_rgb_init();
/// @brief Prepare color for RGB light
/// @param port [0:3] LED number. All -1
/// @param r [0:255] red color
/// @param g [0:255] green color
/// @param b [0:255] blue color
/// @return ESP error
esp_err_t bsp_rgb_rgb(int8_t port, uint8_t r, uint8_t g, uint8_t b);
/// @brief Prepare color for RGB light
/// @param port [0:3] LED number. All -1
/// @param hex [0:0xFFFFFF] HEX color
/// @return ESP error
esp_err_t bsp_rgb_hex(int8_t port, uint32_t hex);
/// @brief Prepare color for RGB light
/// @param port [0:3] LED number. All -1
/// @param hue [0:360] hue part of color
/// @param saturation [0:255] saturation part of color
/// @param value [0:255] value part of color
/// @return ESP error
esp_err_t bsp_rgb_hsv(int8_t port, uint16_t hue, uint8_t saturation, uint8_t value);
/// @brief Clear prepared colors
/// @return ESP error
esp_err_t bsp_rgb_clear();
/// @brief Write prepared colors to RGB lights
/// @return ESP error
esp_err_t bsp_rgb_update();

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_BSP_RGB_H */
