/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_BSP_ROBOBOARD_X4_H
#define INCLUDE_BSP_ROBOBOARD_X4_H

#include <stdint.h>
#include "esp_err.h"
#include "pin_map.h"
#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize board components
 * - LED
 * - Button
 * - Battery
 * - DC/USB detect
 * Returns:
 *  `0` - success
 * `-1` - failed to communicate peripheral driver
*/
int bsp_board_init(void);

typedef uint32_t bsp_cmd_t;
// List of common RoboBoard X4 commands
enum {
    // Disabling port will make it inactive, even when state is changed.
    // Some commands has ports combined in a group. Writing to either of
    // port will affect both. E.g., write to A of group AB affects A and B.
    //
    // General board information. "port" is ignored
    BSP_BOARD_SERIAL, // Board serial [0:0x7FFF]
    BSP_BOARD_REVISION, // Board revision [0:990]
    BSP_DRIVER_FIRMWARE, // Driver firmware version [0:999]
    BSP_BUTTON_STATE, // Button state [0:1]
    BSP_LED_STATE, // LED state [0:any]
    BSP_5V_STATE, // Disable / Enable 5V power rail [0:any]
    BSP_DC_STATE, // Is DC jack plugged? [0:1] (v1.0 only)
    BSP_USB_STATE, // Is USB cable plugged? [0:1]
    BSP_BATTERY_VOLTAGE, // Battery voltage [8400:12600]
    BSP_BATTERY_STATE, // Battery state [0 - none, 1 - charging]
    // DC ports control. [0:3] (A, B, C, D). -1 - ALL
    BSP_DC_POWER, // Write power to motor [-100:100]. 0 - stop
    BSP_DC_BRAKE, // Apply braking power [0:100] (0 - coast)
    BSP_DC_TONE, // AB, CD group. Play tone on motor [0:20000]. ((duration << 16) | freq)
    // BSP_DC_POWER_ABCD, // Single command write (A | B | C | D). "port" ignored
    // BSP_DC_BRAKE_ABCD, // Single command write (A | B | C | D). "port" ignored
    BSP_DC_CONFIG_INVERT, // Invert port no / yes [0:any]
    BSP_DC_CONFIG_FREQUENCY, // AB, CD group. Set PWM frequency [0:250000]
    BSP_DC_CONFIG_BRAKE, // Set autobrake power. (Braking when power = 0) [0:100]
    BSP_DC_CONFIG_ENABLE, // Disabled / Enabled [0:any]
    // Servo ports control. [0:2] (A, B, C). -1 - ALL
    BSP_SERVO_PULSE, // Write pulse [0:period]. ((duration << 16) | pulse)
    BSP_SERVO_CONFIG_INVERT, // Invert port no / yes [0:any]
    BSP_SERVO_CONFIG_SPEED, // Limit servo speed [0:(rpm*60)]
    BSP_SERVO_CONFIG_PERIOD, // ABC group. PWM period [1:0xFFFF] (default 20000us)
    BSP_SERVO_CONFIG_RANGE, // Microseconds range of 180 degrees (min << 16 | max)
    BSP_SERVO_CONFIG_ENABLE, // Disabled / Enabled [0:any]
    // RGB LED control. [0:3] (A, B, C, D). -1 - ALL
    BSP_RGB_COLOR, // HEX color with alpha [0:0xFFFFFFFF]
    BSP_RGB_FADE_COLOR, // Fade HEX color with alpha [0:0xFFFFFFFF]
    BSP_RGB_FADE_START, // Start fade animation [0:(duration ms)]
    BSP_RGB_CONFIG_ENABLE, // Disabled / Enabled [0:any]

    BSP_CMD_MAX
};

// Names for A, B, C, D ports.
// Cal also use integers (0, 1, 2, 3).
// Negative port -1 (ALL) can be only used with bsp_cmd_write().
enum {
    BSP_PORT_ALL = -1,
    BSP_PORT_A = 0,
    BSP_PORT_B = 1,
    BSP_PORT_C = 2,
    BSP_PORT_D = 3,
};

/**
 * Command value change callback handler
 * `cmd`   - command name from `bsp_cmd_t` list
 * `port`  - port number (A, B, C, D)
 * `value` - value to write
 * `isr`   - called from interrupt
*/
typedef void (*bsp_cmd_change_func_t)(bsp_cmd_t cmd, uint8_t port, int32_t value, uint8_t isr);

/**
 * Write value to component
 * `cmd`   - command name from `bsp_cmd_t` list
 * `port`  - port number (A, B, C, D). [-1] - all
 * `value` - value to write
 * Returns:
 * 0x000   (0) ESP_OK                - success
 * 0x102 (258) ESP_ERR_INVALID_ARG   - port is incorrect
 * 0x104 (260) ESP_ERR_INVALID_SIZE  - value is out of range
 * 0x105 (261) ESP_ERR_NOT_FOUND     - cmd not found
 * 0x106 (262) ESP_ERR_NOT_SUPPORTED - cmd is read-only
*/
esp_err_t bsp_cmd_write(bsp_cmd_t cmd, int8_t port, int32_t value);
/**
 * Read value from component
 * `cmd`  - command name from `bsp_cmd_t` list
 * `port` - port number (A, B, C, D)
 * Returns: value received from component. `0` - if error
*/
int32_t bsp_cmd_read(bsp_cmd_t cmd, uint8_t port);
/**
 * Register global event receiver. Will be called on each value change
 * Up to 3 callbacks can be registered
 * NOTE: not all commands supported at the moment
 * `callback` - function to call on cmd value change
*/
void bsp_callback_register(bsp_cmd_change_func_t callback);

/**************************************************************************************************
 * IMU
 **************************************************************************************************/

typedef struct {
    float temp; // Unit: C
    struct { // Unit: G, dps
        float x, y, z;
    } accel, gyro;
} BspIMU_data_t;

// Set accelerometer maximum range of G force.
// Allowed values: 2, 4, 8, 16. Default: 16 (G)
esp_err_t bsp_imu_set_accel_range(uint16_t range);
// Set gyroscope maximum range of angle speed.
// Allowed values: 250, 500, 1000, 2000. Default 2000 (dps)
esp_err_t bsp_imu_set_gyro_range(uint16_t range);

esp_err_t bsp_imu_read(BspIMU_data_t *data);

/*******************************
 * GPIO pin control functions. RoboBoard X4 v1.0 only!
 * Pins are connected to peripheral driver (STM32) so
 * additional functions are required for interaction.
 * For later versions use standard GPIO API!
 ******************************/
/**
 * Read digital state of GPIO pin
 * `port` - pin number (A, B, C, D)
 * Returns: [0] - LOW, [1] - HIGH
*/
uint32_t bsp_gpio_digital_read(uint8_t port);
/**
 * Write digital state to GPIO pin
 * `port` - pin number (A, B, C, D)
 * `pull` - pull mode: [1] - pull-up, [0] - pull-down
*/
void bsp_gpio_digital_write(uint8_t port, uint8_t state);
/**
 * Configure GPIO pin to input mode with pull resistor
 * `port` - pin number (A, B, C, D)
 * `pull` - pull mode: [1] - pull-up, [0] - pull-down
*/
void bsp_gpio_digital_input(uint8_t port, uint8_t pull);
/**
 * Read analog value of GPIO pin
 * `port` - pin number (A, B, C, D)
 * Returns: [0:1023]
*/
uint32_t bsp_gpio_analog_read(uint8_t port);
/**
 * Write analog value (PWM) to GPIO pin
 * `port` - pin number (A, B, C, D)
 * `value` - duty cycle [0:20]
*/
void bsp_gpio_analog_write(uint8_t port, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_BSP_ROBOBOARD_X4_H */
