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
 * Totem RoboBoard X4 low level control API
 **************************************************************************************************/
// Board events list
enum {
    // USB cable plug/unplug event (called from ISR)
    BSP_EVT_USB,
    // DC power adapter plug/unplug event (called from ISR)
    BSP_EVT_POWER,
    // Button press/release event (called from ISR)
    BSP_EVT_BUTTON,
    // Servo position pulse change (during speed control)
    BSP_EVT_SERVO_PULSE,
};
// Board events function
typedef void (*bsp_evt_func_t)(uint32_t evt, uint32_t portID, uint32_t value, void *arg);
// Names for A, B, C, D ports
enum {
    BSP_PORT_ALL = -1, // All ports
    BSP_PORT_A = 0, // Port A
    BSP_PORT_B = 1, // Port B
    BSP_PORT_C = 2, // Port C
    BSP_PORT_D = 3, // Port D
};

/// @brief Initialize BSP driver
/// @return ESP error
esp_err_t bsp_board_init();
/// @brief Register board events function
/// @param func event function handler
/// @param arg pointer passed to handler
/// @return ESP error
esp_err_t bsp_board_reg_event(bsp_evt_func_t func, void *arg);

/*******************************
 * Board control functions
 ******************************/

/// @brief Turn LED (pin 13) (default - off)
/// @param state [0] off, [1] on
/// @return ESP error
esp_err_t bsp_board_set_led(uint32_t state);
/// @brief Turn CAN bus transceiver (default - off)
/// @param state [0] off, [1] on
/// @return ESP error
esp_err_t bsp_board_set_can(uint32_t state);
/// @brief Turn 5V power rail (default - on) (v1.1 only)
/// @param state [0] off, [1] on
/// @note  Affects servo (+) and RGB power
/// @return ESP error
esp_err_t bsp_board_set_5V(uint32_t state);
/// @brief Get board revision
/// @return Format: [12] -> v1.2
int bsp_board_get_revision();
/// @brief Get motor driver firmware version
/// @return Format: [123] -> v1.23
int bsp_board_get_firmware();
/// @brief Is button pressed
/// @return [0] released, [1] pressed
int bsp_board_get_button();
/// @brief Is power adapter plugged in (v1.0 only)
/// @return [0] unplugged, [1] plugged in
int bsp_board_get_power();
/// @brief Is USB cable plugged in
/// @return [0] unplugged, [1] plugged in
int bsp_board_get_usb();

/*******************************
 * Battery control functions
 ******************************/

/// @brief Read battery voltage
/// @return [8400:12600]mV (millivolts)
int bsp_battery_get_voltage();

/*******************************
 * DC control functions
 ******************************/

/// @brief Spin motor
/// @param portID [0:3] port number, [-1] all ports
/// @param power [-100:100]% power and direction, [0] coast
/// @return ESP error
esp_err_t bsp_dc_spin(int8_t portID, int32_t power);
/// @brief Brake motor
/// @param portID [0:3] port number, [-1] all ports
/// @param power [0:100]% power and direction, [0] coast
/// @return ESP error
esp_err_t bsp_dc_brake(int8_t portID, uint32_t power);
/// @brief Output audible tone to AB ports
/// @param frequency [0:20000]Hz tone frequency
/// @param duration duration of time play (ms), [0] indefinitely
/// @return ESP error
esp_err_t bsp_dc_tone_AB(uint32_t frequency, uint32_t duration);
/// @brief Output audible tone to CD ports
/// @param frequency [0:20000]Hz tone frequency
/// @param duration duration of time play (ms), [0] indefinitely
/// @return ESP error
esp_err_t bsp_dc_tone_CD(uint32_t frequency, uint32_t duration);
/// @brief Output audible tone to ABCD ports
/// @param frequency [0:20000]Hz tone frequency
/// @param duration duration of time play (ms), [0] indefinitely
/// @return ESP error
esp_err_t bsp_dc_tone_ABCD(uint32_t frequency, uint32_t duration);
/// @brief Toggle port output (default - enabled)
/// @param portID [0:3] port number, [-1] all ports
/// @param enable [0] disable output, [1] enable output
/// @return ESP error
esp_err_t bsp_dc_set_enable(int8_t portID, uint32_t enable);
/// @brief Change decay mode (default - slow)
/// @param portID [0:3] port number, [-1] all ports
/// @param decay [0] slow decay, [1] fast decay
/// @return ESP error
esp_err_t bsp_dc_set_decay(int8_t portID, uint32_t decay);
/// @brief Change PWM frequency for AB ports (default - 20000)
/// @param frequency [0:250000]Hz PWM frequency
/// @return ESP error
esp_err_t bsp_dc_set_frequency_AB(uint32_t frequency);
/// @brief Change PWM frequency for CD ports (default - 20000)
/// @param frequency [0:250000]Hz PWM frequency
/// @return ESP error
esp_err_t bsp_dc_set_frequency_CD(uint32_t frequency);
/// @brief Change PWM frequency for ABCD ports (default - 20000)
/// @param frequency [0:250000]Hz PWM frequency
/// @return ESP error
esp_err_t bsp_dc_set_frequency_ABCD(uint32_t frequency);
/// @brief Change group AB mode (default - individual)
/// @param mode [0] individual control, [1] group control
/// @return ESP error
esp_err_t bsp_dc_set_mode_AB(uint32_t mode);
/// @brief Change group CD mode (default - individual)
/// @param mode [0] individual control, [1] group control
/// @return ESP error
esp_err_t bsp_dc_set_mode_CD(uint32_t mode);
/// @brief Get selected decay mode
/// @param portID [0:3] port number
/// @return [0] slow decay, [1] fast decay
int bsp_dc_get_decay(uint8_t portID);
/// @brief Get configured PWM frequency of AB ports
/// @return [0:250000]Hz PWM frequency
int bsp_dc_get_frequency_AB();
/// @brief Get configured PWM frequency of CD ports
/// @return [0:250000]Hz PWM frequency
int bsp_dc_get_frequency_CD();

/*******************************
 * Servo control functions
 ******************************/

/// @brief Spin servo motor to position
/// @param portID [0:2] port number, [-1] all ports
/// @param pulse [0:period]us position
/// @return ESP error
esp_err_t bsp_servo_spin(int8_t portID, uint32_t pulse);
/// @brief Spin servo motor to position
/// @param portID [0:2] port number, [-1] all ports
/// @param pulse [0:period]us position
/// @param duration [0] max speed, spin speed in duration (ms) (overrides default speed)
/// @note  Duration: amount of time to spin from current to target position
/// @return ESP error
esp_err_t bsp_servo_spin_duration(int8_t portID, uint32_t pulse, uint32_t duration);
/// @brief Spin servo motor to position
/// @param portID [0:2] port number, [-1] all ports
/// @param pulse [0:period]us position
/// @param ppp [0] max speed, spin speed in PPP unit (overrides default speed)
/// @note  Pulse-Per-Period: ppp = (RPM * 6 * (motorUsMax - motorUsMin)) / motorAngle / 50
/// @return ESP error
esp_err_t bsp_servo_spin_ppp(int8_t portID, uint32_t pulse, uint32_t ppp);
/// @brief Toggle port output (default - enabled)
/// @param portID [0:2] port number, [-1] all ports
/// @param enable [0] disable output, [1] enable output
/// @return ESP error
esp_err_t bsp_servo_set_enable(int8_t portID, uint32_t enable);
/// @brief Configure constant motor speed (default - disabled)
/// @param portID [0:2] port number, [-1] all ports
/// @param ppp [0] disabled (max speed), spin speed in PPP unit
/// @note  Pulse-Per-Period: ppp = (RPM * 6 * (motorUsMax - motorUsMin)) / motorAngle / 50
/// @return ESP error
esp_err_t bsp_servo_set_speed_ppp(int8_t portID, uint32_t ppp);
/// @brief Change PWM period for ABC ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t bsp_servo_set_period(uint32_t period);
/// @brief Get configured PWM period of ABC ports
/// @return [1:65535]us PWM period
int bsp_servo_get_period();
/// @brief Read servo motor position
/// @param portID [0:2] port number
/// @return [0:period]us position pulse
int bsp_servo_get_pulse(uint8_t portID);

/*******************************
 * RGB control functions
 ******************************/

/// @brief Change RGB light color
/// @param ledID [0:3] LED number, [-1] all LEDs
/// @param hex 32bit color | Alpha | Red | Green | Blue |
/// @return ESP error
esp_err_t bsp_rgb_color(int8_t ledID, uint32_t hex);
/// @brief Prepare RGB light fade color
/// @param ledID [0:3] LED number, [-1] all LEDs
/// @param hex 32bit fade color | Alpha | Red | Green | Blue |
/// @return ESP error
esp_err_t bsp_rgb_fade_color(int8_t ledID, uint32_t hex);
/// @brief Run fade animation
/// @param ledID [0:3] LED number, [-1] all LEDs
/// @param duration animation duration (ms)
/// @return ESP error
esp_err_t bsp_rgb_fade_start(int8_t ledID, uint32_t duration);
/// @brief Toggle LED output (default - enabled)
/// @param ledID [0:3] LED number, [-1] all LEDs
/// @param enable [0] LED is disabled, [1] LED is enabled
/// @return ESP error
esp_err_t bsp_rgb_set_enable(int8_t ledID, uint32_t enable);

/*******************************
 * GPIO pin control functions. RoboBoard X4 v1.0 only!
 * Pins are connected to peripheral driver (STM32) so
 * additional functions are required for interaction.
 * For later versions use standard GPIO API!
 ******************************/

/// @brief Read digital state of GPIO pin
/// @param pinID [0:3] pin number (A, B, C, D)
/// @return [0] LOW, [1] HIGH
uint32_t bsp_gpio_digital_read(uint8_t pinID);
/// @brief Write digital state to GPIO pin
/// @param pinID [0:3] pin number (A, B, C, D)
/// @param state [0] LOW, [1] HIGH
void bsp_gpio_digital_write(uint8_t pinID, uint8_t state);
/// @brief Configure GPIO mode
/// @param pinID [0:3] pin number (A, B, C, D)
/// @param mode [0] pd, [1] pu, [2] float, [3] out, [4] analog
void bsp_gpio_mode(uint8_t pinID, uint8_t mode);
/// @brief Read analog value of GPIO pin
/// @param pinID [0:3] pin number (A, B, C, D)
/// @return [0:1023] ADC measurement
uint32_t bsp_gpio_analog_read(uint8_t pinID);
/// @brief Write analog value (PWM) to GPIO pin
/// @param pinID [0:3] pin number (A, B, C, D)
/// @param value [0:20] duty cycle
void bsp_gpio_analog_write(uint8_t pinID, uint16_t value);

/**************************************************************************************************
 * Legacy command oriented control API. Keeping for backwards compatibility (will be removed)
 **************************************************************************************************/
/// @brief Callback of bsp_register_interrupt_callback function
/// @param arg pointer passed to bsp_register_interrupt_callback
typedef void (*bsp_interrupt_func_t)(void *arg);

typedef uint32_t bsp_cmd_t;
// List of common RoboBoard X4 commands
enum {
    // Disabling port will make it inactive, even when state is changed.
    // Some commands has ports combined in a group. Writing to either of
    // port will affect both. E.g., write to A of group AB affects A and B.
    // Tags [read],[write] indicates if command should be used with bsp_cmd_read or bsp_cmd_write
    // Tag [irq] indicate that command can be registered in bsp_register_interrupt_callback

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
    // [write] beep tone [0:20000]Hz. Duration [0:0xFFFF]ms (auto stop).
    // ((duration << 16) | freq). Ports AB, CD are grouped together
    BSP_DC_TONE,
    // [write|read] PWM [0:250000]Hz. Ports AB, CD are grouped together
    BSP_DC_CONFIG_FREQUENCY,
    // [write|read] decay [0-slow, 1-fast]
    BSP_DC_CONFIG_DECAY,
    // [write] motor port [0:1] off/on
    BSP_DC_CONFIG_ENABLE,

    // Servo motor ports control

    // [write|read] spin to [0:period]us. Optional speed control:
    // Duration [0:0x7FFF]ms. ((duration << 16) | pulse). Overrides BSP_SERVO_CONFIG_SPEED.
    // PPP 0x8000 | [0:0x7FFF]. (0x8000 | (ppp << 16) | pulse). Overrides BSP_SERVO_CONFIG_SPEED.
    BSP_SERVO_PULSE,
    // [write] servo speed. PPP (Pulse-Per-Period) (amount of microseconds to move each period)
    BSP_SERVO_CONFIG_SPEED,
    // [write|read] PWM period [1:0xFFFF]us. Ports ABC are grouped together
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
esp_err_t bsp_cmd_write(bsp_cmd_t cmd, int8_t port, int32_t value) __attribute__ ((deprecated));

/// @brief Read command value
/// @note Only commands with [read] tag can be used
/// @param cmd command name from `bsp_cmd_t` list
/// @param port port number (A, B, C, D)
/// @return value received from command. 0 if error
int32_t bsp_cmd_read(bsp_cmd_t cmd, uint8_t port) __attribute__ ((deprecated));

/// @brief Register global interrupt receiver. Will be called on each value change
/// @note Only commands with [irq] tag can be registered
/// @param cmd command name from `bsp_cmd_t` list
/// @param func interrupt function callback
/// @param arg pointer passed to interrupt function
/// @return
///         ESP_OK                - success
///         ESP_ERR_INVALID_STATE - ISR init fail or BSP not initialized
///         ESP_ERR_NOT_SUPPORTED - cmd not supported
esp_err_t bsp_register_interrupt_callback(bsp_cmd_t cmd, bsp_interrupt_func_t func, void *arg) __attribute__ ((deprecated));

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_BSP_ROBOBOARD_X4_H */
