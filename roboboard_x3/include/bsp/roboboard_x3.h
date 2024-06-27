/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_BSP_ROBOBOARD_X3_H
#define INCLUDE_BSP_ROBOBOARD_X3_H

#include <stdint.h>
#include "esp_err.h"
#include "hal/gpio_types.h"

/**************************************************************************************************
 * Totem RoboBoard X3 pinout
 **************************************************************************************************/

/* I2C */
#define BSP_IO_I2C_SDA           (GPIO_NUM_15) //MTDO
#define BSP_IO_I2C_SCL           (GPIO_NUM_5)

/* GPIO */
#define BSP_IO_IO26              (GPIO_NUM_26)
#define BSP_IO_IO32              (GPIO_NUM_32)
#define BSP_IO_IO33              (GPIO_NUM_33)

/* DC motor */
#define BSP_IO_MOTORA_INA        (GPIO_NUM_22)
#define BSP_IO_MOTORA_INB        (GPIO_NUM_12) // MTDI
#define BSP_IO_MOTORB_INA        (GPIO_NUM_23)
#define BSP_IO_MOTORB_INB        (GPIO_NUM_19)
#define BSP_IO_MOTORC_INA        (GPIO_NUM_18)
#define BSP_IO_MOTORC_INB        (GPIO_NUM_21)
#define BSP_IO_MOTORD_INA        (GPIO_NUM_4)
#define BSP_IO_MOTORD_INB        (GPIO_NUM_2)

/* Servo motor */
#define BSP_IO_SERVOA_IN         (GPIO_NUM_25)
#define BSP_IO_SERVOB_IN         (GPIO_NUM_14) // MTMS
#define BSP_IO_SERVOC_IN         (GPIO_NUM_16) // v3.1 only
#define BSP_IO_SERVOD_IN         (GPIO_NUM_17) // v3.1 only

/* Others */
#define BSP_IO_BUTTON            (GPIO_NUM_0)
#define BSP_IO_RGB               (GPIO_NUM_13) // MTCK
#define BSP_IO_3V3_EN            (GPIO_NUM_27)
#define BSP_IO_VERSION_DETECT    (GPIO_NUM_34) // VDET_1
#define BSP_IO_BATTERY_CHARGE    (GPIO_NUM_35) // VDET_2
#define BSP_IO_BATTERY_CURRENT   (GPIO_NUM_36) // SENSOR_VP
#define BSP_IO_BATTERY_VOLTAGE   (GPIO_NUM_37) // SENSOR_CAPP
#define BSP_IO_USB_DETECT        (GPIO_NUM_38) // SENSOR_CAPN
#define BSP_IO_IMU_INT           (GPIO_NUM_39) // SENSOR_VN

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
 * Totem RoboBoard X3 low level control API
 **************************************************************************************************/
// Board events list
enum {
    // USB cable plug/unplug event (called from ISR)
    BSP_EVT_USB,
    // Button press/release event (called from ISR)
    BSP_EVT_BUTTON,
    // Charging start/stop event (called from ISR)
    BSP_EVT_CHARGING,
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

/// @brief Turn 3.3V power rail (default - off)
/// @param state [0] off, [1] on
/// @note  Affects 3v3 pin and Qwiic
/// @return ESP error
esp_err_t bsp_board_set_3V(uint32_t state);
/// @brief Get board revision
/// @return Format: [12] -> v1.2
int bsp_board_get_revision();
/// @brief Is button pressed
/// @return [0] released, [1] pressed
int bsp_board_get_button();
/// @brief Is USB cable plugged in
/// @return [0] unplugged, [1] plugged in
int bsp_board_get_usb();

/*******************************
 * Battery control functions
 ******************************/

/// @brief Read battery voltage
/// @return [2600:4200]mV (millivolts)
int bsp_battery_get_voltage();
/// @brief Read battery current
/// @return [-2000:2000]mA (milliAmps). [-] discharging
int bsp_battery_get_current();
/// @brief Is battery charging
/// @return [0] not charging, [1] charging
int bsp_battery_get_charging();

/*******************************
 * DC control functions
 ******************************/

/// @brief Spin motor
/// @param portID [0:3] port number, [-1] all ports
/// @param power [-100:100]% power and direction, [0] no power
/// @return ESP error
esp_err_t bsp_dc_spin(int8_t portID, int32_t power);
/// @brief Brake motor
/// @param portID [0:3] port number, [-1] all ports
/// @param power [0:100]% braking power, [0] coast
/// @return ESP error
esp_err_t bsp_dc_brake(int8_t portID, uint32_t power);
/// @brief Output audible tone to motor
/// @param portID [0:3] port number, [-1] all ports
/// @param frequency [0:20000]Hz tone frequency
/// @return ESP error
esp_err_t bsp_dc_tone(int8_t portID, uint32_t frequency);
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
/// @brief Change PWM frequency for motor port (default - 20000)
/// @param portID [0:3] port number, [-1] all ports
/// @param frequency [1:250000]Hz PWM frequency
/// @return ESP error
esp_err_t bsp_dc_set_frequency(int8_t portID, uint32_t frequency);
/// @brief Get selected decay mode
/// @param portID [0:3] port number
/// @return [0] slow decay, [1] fast decay
int bsp_dc_get_decay(uint8_t portID);
/// @brief Get configured PWM frequency of motor port
/// @param portID [0:3] port number
/// @return [1:250000]Hz PWM frequency
int bsp_dc_get_frequency(uint8_t portID);

/*******************************
 * Servo control functions
 ******************************/

/// @brief Spin servo motor to position
/// @param portID [0:3] port number, [-1] all ports
/// @param pulse [0:period]us position
/// @return ESP error
esp_err_t bsp_servo_spin(int8_t portID, uint32_t pulse);
/// @brief Toggle port output (default - enabled)
/// @param portID [0:3] port number, [-1] all ports
/// @param enable [0] disable output, [1] enable output
/// @return ESP error
esp_err_t bsp_servo_set_enable(int8_t portID, uint32_t enable);
/// @brief Change PWM period for AB ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t bsp_servo_set_period_AB(uint32_t period);
/// @brief Change PWM period for CD ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t bsp_servo_set_period_CD(uint32_t period);
/// @brief Change PWM period for ABCD ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t bsp_servo_set_period_ABCD(uint32_t period);
/// @brief Get configured PWM period of AB ports
/// @return [1:65535]us PWM period
int bsp_servo_get_period_AB();
/// @brief Get configured PWM period of CD ports
/// @return [1:65535]us PWM period
int bsp_servo_get_period_CD();
/// @brief Read servo motor position
/// @param portID [0:3] port number
/// @return [0:period]us position pulse
int bsp_servo_get_pulse(uint8_t portID);
/// @brief Get number of servo ports
/// @return [2,4] servo port count
int bsp_servo_get_port_cnt();

/**************************************************************************************************
 * Legacy command oriented control API. Keeping for backwards compatibility (will be removed)
 **************************************************************************************************/
/// @brief Callback of bsp_register_interrupt_callback function
/// @param arg pointer passed to bsp_register_interrupt_callback
typedef void (*bsp_interrupt_func_t)(void *arg);

typedef uint32_t bsp_cmd_t;
// List of common RoboBoard X3 commands
enum {
    // Disabling port will make it inactive, even when state is changed.
    // Some commands has ports combined in a group. Writing to either of
    // port will affect both. E.g., write to A of group AB affects A and B.
    // Tags [read],[write] indicates if command should be used with bsp_cmd_read or bsp_cmd_write
    // Tag [irq] indicate that command can be registered in bsp_register_interrupt_callback

    // [read] board revision [0:990] -> 9.9
    BSP_BOARD_REVISION,
    // [read|irq] button state [0:1] release/press
    BSP_BUTTON_STATE,
    // [write] 3V power rail [0:1] off/on
    BSP_3V_STATE,
    // [read|irq] USB cable [0:1] unconnected/connected
    BSP_USB_STATE,
    // [read] battery voltage [2600:4200]mV
    BSP_BATTERY_VOLTAGE,
    // [read] battery current [-2000:2000]mA. negative:discharging
    BSP_BATTERY_CURRENT,
    // [read|irq] battery state [0:1] not charging/charging
    BSP_BATTERY_CHARGING,

    // DC motor ports control

    // [write] spin power [-100:100]%. 0:stop, negative:reverse
    BSP_DC_POWER,
    // [write] braking power [0:100]%. 0:coast
    BSP_DC_BRAKE,
    // [write] beep tone [0:20000]Hz
    BSP_DC_TONE,
    // [write|read] PWM [1:250000]Hz
    BSP_DC_CONFIG_FREQUENCY,
    // [write|read] decay [0-slow, 1-fast]
    BSP_DC_CONFIG_DECAY,
    // [write] motor port [0:1] off/on
    BSP_DC_CONFIG_ENABLE,

    // Servo motor ports control
    
    // [write] spin to [0:period]us ([500:2500] typical)
    BSP_SERVO_PULSE,
    // [write|read] PWM [1:1000000]us. Ports AB grouped together
    BSP_SERVO_CONFIG_PERIOD,
    // [write] motor port [0:1] off/on
    BSP_SERVO_CONFIG_ENABLE,

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

#endif /* INCLUDE_BSP_ROBOBOARD_X3_H*/
