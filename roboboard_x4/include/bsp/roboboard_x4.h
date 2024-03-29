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
#include "hal/gpio_types.h"

/**************************************************************************************************
 * Totem RoboBoard X4 pinout
 **************************************************************************************************/

/* I2C */
#define BSP_IO_I2C_SDA           (GPIO_NUM_21)
#define BSP_IO_I2C_SCL           (GPIO_NUM_22)

/* CAN */
#define BSP_IO_CAN_TX            (GPIO_NUM_17)
#define BSP_IO_CAN_RX            (GPIO_NUM_34)
#define BSP_IO_CAN_EN            (GPIO_NUM_5)  // v1.1 only

/* GPIO */
#define BSP_IO_GPIOA             (GPIO_NUM_14) // v1.1 only
#define BSP_IO_GPIOB             (GPIO_NUM_23) // v1.1 only
#define BSP_IO_GPIOC             (GPIO_NUM_25) // v1.1 only
#define BSP_IO_GPIOD             (GPIO_NUM_26) // v1.1 only

/* Driver */
#define BSP_IO_DRIVER_DFU        (GPIO_NUM_4)
#define BSP_IO_DRIVER_RESET      (GPIO_NUM_15)
#define BSP_IO_DRIVER_UART_RX    (GPIO_NUM_16)
#define BSP_IO_DRIVER_UART_TX    (GPIO_NUM_27)

/* Others */
#define BSP_IO_LED               (GPIO_NUM_13)
#define BSP_IO_BUTTON            (GPIO_NUM_18)
#define BSP_IO_POWER_DETECT      (GPIO_NUM_19)
#define BSP_IO_USB_DETECT        (GPIO_NUM_39)
#define BSP_IO_BATTERY_VOLTAGE   (GPIO_NUM_36)
#define BSP_IO_ACCEL_INT         (GPIO_NUM_35) // v1.1 only
#define BSP_IO_ACCEL_INT_v10     (GPIO_NUM_14) // v1.0 only

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 * Totem RoboBoard X4 board control
 **************************************************************************************************/

typedef uint32_t bsp_cmd_t;
// List of common RoboBoard X4 commands
enum {
    // Disabling port will make it inactive, even when state is changed.
    // Some commands has ports combined in a group. Writing to either of
    // port will affect both. E.g., write to A of group AB affects A and B.
    // Tags [read],[write] indicates if command should be used with bsp_cmd_read or bsp_cmd_write
    // Tag [irq] indicated that command can be registered in bsp_register_interrupt_callback

    // [read] board revision [0:990] -> 9.9
    BSP_BOARD_REVISION,
    // [read] driver firmware [0:999] -> 9.99
    BSP_DRIVER_FIRMWARE,
    // [read|irq] button [0:1] release/press
    BSP_BUTTON_STATE,
    // [write] LED [0:1] off/on
    BSP_LED_STATE,
    // [write] CAN bus transceiver [0:1] off/on
    BSP_CAN_STATE,
    // [write] 5V power rail [0:1] off/on
    BSP_5V_STATE,
    // [read|irq] DC power jack [0:1] unconnected/connected (v1.0 only)
    BSP_POWER_STATE,
    // [read|irq] USB cable [0:1] unconnected/connected
    BSP_USB_STATE,
    // [read] battery voltage [8400:12600]mV
    BSP_BATTERY_VOLTAGE,

    // DC motor ports control

    // [write] spin power [-100:100]%. 0:stop, negative:reverse
    BSP_DC_POWER,
    // [write] braking power [0:100]%. 0:coast
    BSP_DC_BRAKE,
    // [write] beep tone [0:20000]Hz. Duration [0:0xFFFF]ms (auto stop). ((duration << 16) | freq). Ports AB, CD are grouped together
    BSP_DC_TONE,
    // [write|read] PWM [0:250000]Hz. Ports AB, CD are grouped together
    BSP_DC_CONFIG_FREQUENCY,
    // [write] motor port [0:1] off/on
    BSP_DC_CONFIG_ENABLE,

    // Servo motor ports control

    // [write] spin to [0:period]us. Duration [0:0xFFFF]ms. ((duration << 16) | pulse). Overrides spin speed if "duration" is specified.
    BSP_SERVO_PULSE,
    // [write] servo speed. Format: (RPM * 60)
    BSP_SERVO_CONFIG_SPEED,
    // [write|read] PWM [1:0xFFFF]us. Ports ABC are grouped together
    BSP_SERVO_CONFIG_PERIOD,
    // [write] motor port [0:1] off/on
    BSP_SERVO_CONFIG_ENABLE,

    // RGB lights control

    // [write] color to RGB light [0:0xFFFFFFFF] (HEX with alpha)
    BSP_RGB_COLOR,
    // [write] fade color for RGB light [0:0xFFFFFFFF] (HEX with alpha)
    BSP_RGB_FADE_COLOR,
    // [write] start fade animation with duration
    BSP_RGB_FADE_START,
    // [write] RGB light [0:1] off/on
    BSP_RGB_CONFIG_ENABLE,

    BSP_CMD_MAX
};

// Names for A, B, C, D ports.
// Can also use integers (0, 1, 2, 3).
enum {
    /// @brief Affect all available ports
    BSP_PORT_ALL = -1,
    /// @brief Affect port A
    BSP_PORT_A = 0,
    /// @brief Affect port B
    BSP_PORT_B = 1,
    /// @brief Affect port C
    BSP_PORT_C = 2,
    /// @brief Affect port D
    BSP_PORT_D = 3,
};

/// @brief Callback of bsp_register_interrupt_callback function
/// @param arg pointer passed to bsp_register_interrupt_callback
typedef void (*bsp_interrupt_func_t)(void *arg);

/// @brief Initialize board
/// @return ESP error
esp_err_t bsp_board_init(void);

/// @brief Write value to command
/// @note Only commands with [write] tag can be used
/// @param cmd command name from `bsp_cmd_t` list
/// @param port port number (A, B, C, D). [-1] - all
/// @param value value to write
/// @return
///         ESP_OK                - success
///         ESP_ERR_INVALID_ARG   - invalid port
///         ESP_ERR_INVALID_STATE - BSP not initialized
///         ESP_ERR_INVALID_SIZE  - value is out of range
///         ESP_ERR_NOT_FOUND     - invalid cmd
///         ESP_ERR_NOT_SUPPORTED - action not suported
esp_err_t bsp_cmd_write(bsp_cmd_t cmd, int8_t port, int32_t value);

/// @brief Read command value
/// @note Only commands with [read] tag can be used
/// @param cmd command name from `bsp_cmd_t` list
/// @param port port number (A, B, C, D)
/// @return value received from command. 0 if error
int32_t bsp_cmd_read(bsp_cmd_t cmd, uint8_t port);

/// @brief Register global event receiver. Will be called on each value change
/// @note Only commands with [irq] tag can be registered
/// @param cmd command name from `bsp_cmd_t` list
/// @param func interrupt function callback
/// @param arg pointer passed to interrupt function
/// @return
///         ESP_OK                - success
///         ESP_ERR_INVALID_STATE - ISR init fail or BSP not initialized
///         ESP_ERR_NOT_SUPPORTED - cmd not supported
esp_err_t bsp_register_interrupt_callback(bsp_cmd_t cmd, bsp_interrupt_func_t func, void *arg);

/*******************************
 * GPIO pin control functions. RoboBoard X4 v1.0 only!
 * Pins are connected to peripheral driver (STM32) so
 * additional functions are required for interaction.
 * For later versions use standard GPIO API!
 ******************************/

/// @brief Read digital state of GPIO pin
/// @param port pin number (A, B, C, D)
/// @return [0] - LOW, [1] - HIGH
uint32_t bsp_gpio_digital_read(uint8_t port);

/// @brief Write digital state to GPIO pin
/// @param port pin number (A, B, C, D)
/// @param state pin state: [0] - LOW, [1] - HIGH
void bsp_gpio_digital_write(uint8_t port, uint8_t state);

/// @brief Configure GPIO pin to input mode with pull resistor
/// @param port pin number (A, B, C, D)
/// @param pull enable [1] - pull-up, [0] - pull-down
void bsp_gpio_digital_input(uint8_t port, uint8_t pull);

/// @brief Read analog value of GPIO pin
/// @param port pin number (A, B, C, D)
/// @return [0:1023]
uint32_t bsp_gpio_analog_read(uint8_t port);

/// @brief Write analog value (PWM) to GPIO pin
/// @param port pin number (A, B, C, D)
/// @param value duty cycle [0:20]
void bsp_gpio_analog_write(uint8_t port, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_BSP_ROBOBOARD_X4_H */
