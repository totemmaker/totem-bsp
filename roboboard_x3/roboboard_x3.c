/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_mac.h"

#include "driver/gpio.h"
#include "hal/adc_types.h"

#include "bsp/roboboard_x3.h"
#include "bsp_drivers.h"
// Macros to return peripheral port command and GPIO pin setup
#define RegPort(cmd, port) (cmd + ((port+1)*0x10))
#define GPIO_SEL(pin)   ((uint64_t)(((uint64_t)1)<<(pin)))
// Amount of peripheral ports available
#define DC_CNT 4
#define SERVO_CNT bsp_servo_cnt
// IDF version dependent ADC setup functions
void bsp_adc1_init(adc_channel_t channel);
uint32_t bsp_adc1_get_raw(adc_channel_t channel);
uint32_t bsp_adc1_raw_to_voltage(uint32_t adc);
// Runtime states
static bool bsp_initialized = false;
static uint8_t bsp_board_revision = 0;
static uint8_t bsp_servo_cnt = 2;
static struct { bsp_evt_func_t func; void *arg; } bsp_evt_handler;
// Board interrupt handler
static void bsp_on_isr(void *arg) {
    if (bsp_evt_handler.func == NULL) return;
    uint32_t value = 0;
    switch ((uint32_t)arg) {
    case BSP_EVT_USB: value = bsp_board_get_usb(); break;
    case BSP_EVT_BUTTON: value = bsp_board_get_button(); break;
    case BSP_EVT_CHARGING: value = bsp_battery_get_charging(); break;
    }
    bsp_evt_handler.func((uint32_t)arg, 0, value, bsp_evt_handler.arg);
}

/**************************************************************************************************
 * Totem RoboBoard X3 low level control API
 **************************************************************************************************/

/// @brief Initialize BSP driver
/// @return ESP error
esp_err_t bsp_board_init() {
    gpio_config_t pGPIOConfig = {0};
    // Initialize 3V3 en pin
    pGPIOConfig.mode = GPIO_MODE_OUTPUT;
    pGPIOConfig.intr_type = GPIO_INTR_ANYEDGE;
    pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_3V3_EN);
    gpio_config(&pGPIOConfig);
    // Initialize USB detect pin
    pGPIOConfig.mode = GPIO_MODE_INPUT;
    pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_USB_DETECT);
    gpio_config(&pGPIOConfig);
    // Initialize Button and charge detect pin
    pGPIOConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_BUTTON)|GPIO_SEL(BSP_IO_BATTERY_CHARGE);
    gpio_config(&pGPIOConfig);
    // Initialize interrupts
    BSP_ERR(gpio_install_isr_service(0));
    BSP_ERR(gpio_isr_handler_add(BSP_IO_BUTTON, bsp_on_isr, (void*)BSP_EVT_BUTTON));
    BSP_ERR(gpio_isr_handler_add(BSP_IO_USB_DETECT, bsp_on_isr, (void*)BSP_EVT_USB));
    if (bsp_board_revision != 11) { // Revision 1.1 does not support DC power jack detection
        BSP_ERR(gpio_isr_handler_add(BSP_IO_BATTERY_CHARGE, bsp_on_isr, (void*)BSP_EVT_CHARGING))
    }
    // Initialize motors
    BSP_ERR(bsp_motor_init());
    // Initialize battery analog read
    bsp_adc1_init(ADC_CHANNEL_0); // (BSP_IO_BATTERY_CURRENT) GPIO36 (SENSOR_VP) of ADC1
    bsp_adc1_init(ADC_CHANNEL_1); // (BSP_IO_BATTERY_VOLTAGE) GPIO37 (SENSOR_CAPP) of ADC1
    bsp_adc1_init(ADC_CHANNEL_6); // (BSP_IO_VERSION_DETECT) GPIO34 (VDET_1) of ADC1
    // Read board revision
    int verAdc = bsp_adc1_get_raw(ADC_CHANNEL_6);
    int verVolt = bsp_adc1_raw_to_voltage(verAdc);
    if (verVolt < 300) bsp_board_revision = 30;
    else if (verVolt > 3000) bsp_board_revision = 30;
    else if (verVolt < 700) {
        bsp_board_revision = 31;
        bsp_servo_cnt = 4;
    }
    // Return initialization state
    bsp_initialized = true;
    return ESP_OK;
}
/// @brief Register board events function
/// @param func event function handler
/// @param arg pointer passed to handler
/// @return ESP error
esp_err_t bsp_board_reg_event(bsp_evt_func_t func, void *arg) {
    bsp_evt_handler.func = func;
    bsp_evt_handler.arg = arg;
    return ESP_OK;
}

/*******************************
 * Board control functions
 ******************************/

/// @brief Turn 3.3V power rail (default - off)
/// @param state [0] off, [1] on
/// @note  Affects 3v3 pin and Qwiic
/// @return ESP error
esp_err_t bsp_board_set_3V(uint32_t state) {
    return gpio_set_level(BSP_IO_3V3_EN, !!state);
}
/// @brief Get board revision
/// @return Format: [12] -> v1.2
int bsp_board_get_revision() {
    return bsp_board_revision;
}
/// @brief Is button pressed
/// @return [0] released, [1] pressed
int bsp_board_get_button() {
    return !gpio_get_level(BSP_IO_BUTTON);
}
/// @brief Is USB cable plugged in
/// @return [0] unplugged, [1] plugged in
int bsp_board_get_usb() {
    return gpio_get_level(BSP_IO_USB_DETECT);
}

/*******************************
 * Battery control functions
 ******************************/

/// @brief Read battery voltage
/// @return [2600:4200]mV (millivolts)
int bsp_battery_get_voltage() {
    // Read pin voltage
    uint32_t adcReading = 0;
    // Oversample
    for (int i=0; i<16; i++) {
        adcReading += bsp_adc1_get_raw(ADC_CHANNEL_1);
    }
    adcReading /= 16;
    // Convert adcReading to pin voltage in mV
    int32_t pinVoltage = bsp_adc1_raw_to_voltage(adcReading);
    // Covert pin voltage to battery voltage
    return pinVoltage * 2;
}
/// @brief Read battery current
/// @return [-2000:2000]mA (milliAmps). [-] discharging
int bsp_battery_get_current() {
    // Read pin voltage
    int32_t adcReading = 0;
    // Oversample
    for (int i=0; i<16; i++) {
        adcReading += bsp_adc1_get_raw(ADC_CHANNEL_0);
    }
    adcReading /= 16;
    return (adcReading - 1400) * -2;
}
/// @brief Is battery charging
/// @return [0] not charging, [1] charging
int bsp_battery_get_charging() {
    return !gpio_get_level(BSP_IO_BATTERY_CHARGE);
}

/*******************************
 * DC control functions
 ******************************/

/// @brief Spin motor
/// @param portID [0:3] port number, [-1] all ports
/// @param power [-100:100]% power and direction, [0] no power
/// @return ESP error
esp_err_t bsp_dc_spin(int8_t portID, int32_t power) {
    return bsp_motor_set(portID, MOTOR_DC_MODE_POWER, power);
}
/// @brief Brake motor
/// @param portID [0:3] port number, [-1] all ports
/// @param power [0:100]% braking power, [0] coast
/// @return ESP error
esp_err_t bsp_dc_brake(int8_t portID, uint32_t power) {
    if (portID < -1 || portID >= DC_CNT) return ESP_ERR_NOT_FOUND;
    if (power > 100) return ESP_ERR_INVALID_ARG;
    return bsp_motor_set(portID, MOTOR_DC_MODE_BRAKE, power);
}
/// @brief Output audible tone to motor
/// @param portID [0:3] port number, [-1] all ports
/// @param frequency [0:20000]Hz tone frequency
/// @return ESP error
esp_err_t bsp_dc_tone(int8_t portID, uint32_t frequency) {
    return bsp_motor_set(portID, MOTOR_DC_MODE_TONE, frequency);
}
/// @brief Toggle port output (default - enabled)
/// @param portID [0:3] port number, [-1] all ports
/// @param enable [0] disable output, [1] enable output
/// @return ESP error
esp_err_t bsp_dc_set_enable(int8_t portID, uint32_t enable) {
    return bsp_motor_set(portID, MOTOR_DC_ENABLE, !!enable);
}
/// @brief Change decay mode (default - slow)
/// @param portID [0:3] port number, [-1] all ports
/// @param decay [0] slow decay, [1] fast decay
/// @return ESP error
esp_err_t bsp_dc_set_decay(int8_t portID, uint32_t decay) {
    return bsp_motor_set(portID, MOTOR_DC_DECAY, decay);
}
/// @brief Change PWM frequency for motor port (default - 20000)
/// @param portID [0:3] port number, [-1] all ports
/// @param frequency [1:250000]Hz PWM frequency
/// @return ESP error
esp_err_t bsp_dc_set_frequency(int8_t portID, uint32_t frequency) {
    return bsp_motor_set(portID, MOTOR_DC_FREQUENCY, frequency);
}
/// @brief Get selected decay mode
/// @param portID [0:3] port number
/// @return [0] slow decay, [1] fast decay
int bsp_dc_get_decay(uint8_t portID) {
    return bsp_motor_get(portID, MOTOR_DC_DECAY);
}
/// @brief Get configured PWM frequency of motor port
/// @param portID [0:3] port number
/// @return [1:250000]Hz PWM frequency
int bsp_dc_get_frequency(uint8_t portID) {
    return bsp_motor_get(portID, MOTOR_DC_FREQUENCY);
}

/*******************************
 * Servo control functions
 ******************************/

/// @brief Spin servo motor to position
/// @param portID [0:3] port number, [-1] all ports
/// @param pulse [0:period]us position
/// @return ESP error
esp_err_t bsp_servo_spin(int8_t portID, uint32_t pulse) {
    return bsp_motor_set(portID, MOTOR_SERVO_PULSE, pulse);
}
/// @brief Toggle port output (default - enabled)
/// @param portID [0:3] port number, [-1] all ports
/// @param enable [0] disable output, [1] enable output
/// @return ESP error
esp_err_t bsp_servo_set_enable(int8_t portID, uint32_t enable) {
    return bsp_motor_set(portID, MOTOR_SERVO_ENABLE, !!enable);
}
/// @brief Change PWM period for AB ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t bsp_servo_set_period_AB(uint32_t period) {
    return bsp_motor_set(0, MOTOR_SERVO_PERIOD, period);
}
/// @brief Change PWM period for CD ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t bsp_servo_set_period_CD(uint32_t period) {
    return bsp_motor_set(2, MOTOR_SERVO_PERIOD, period);
}
/// @brief Change PWM period for ABCD ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t bsp_servo_set_period_ABCD(uint32_t period) {
    BSP_ERR(bsp_servo_set_period_AB(period));
    BSP_ERR(bsp_servo_set_period_CD(period));
    return ESP_OK;
}
/// @brief Get configured PWM period of AB ports
/// @return [1:65535]us PWM period
int bsp_servo_get_period_AB() {
    return bsp_motor_get(0, MOTOR_SERVO_PERIOD);
}
/// @brief Get configured PWM period of CD ports
/// @return [1:65535]us PWM period
int bsp_servo_get_period_CD() {
    return bsp_motor_get(2, MOTOR_SERVO_PERIOD);
}
/// @brief Read servo motor position
/// @param portID [0:3] port number
/// @return [0:period]us position pulse
int bsp_servo_get_pulse(uint8_t portID) {
    return bsp_motor_get(portID, MOTOR_SERVO_PULSE);
}
/// @brief Get number of servo ports
/// @return [2,4] servo port count
int bsp_servo_get_port_cnt() {
    return bsp_servo_cnt;
}

/**************************************************************************************************
 * Legacy command oriented control API. Keeping for backwards compatibility (will be removed)
 **************************************************************************************************/

// Global command write function
esp_err_t bsp_cmd_write(bsp_cmd_t cmd, int8_t port, int32_t value) {
    if (!bsp_initialized) { return ESP_ERR_INVALID_STATE; }
    // Validate ports. X3 has maximum of 4 ports
    if (port < -1 || port > 3) { return ESP_ERR_NOT_FOUND; }
    // Validate commands range
    if (cmd >= BSP_CMD_MAX) { return ESP_ERR_INVALID_ARG; }
    // Handle command
    switch (cmd) {
    // Board
    case BSP_3V_STATE: { return bsp_board_set_3V(value); }
    // DC
    case BSP_DC_POWER: { return bsp_dc_spin(port, value); }
    case BSP_DC_BRAKE: { return bsp_dc_brake(port, value); }
    case BSP_DC_TONE: { return bsp_dc_tone(port, value); }
    case BSP_DC_CONFIG_FREQUENCY: { return bsp_dc_set_frequency(port, value); }
    case BSP_DC_CONFIG_DECAY: { return bsp_dc_set_decay(port, value); }
    case BSP_DC_CONFIG_ENABLE: { return bsp_dc_set_enable(port, value); }
    // Servo
    case BSP_SERVO_PULSE: { return bsp_servo_spin(port, value); }
    case BSP_SERVO_CONFIG_PERIOD: {
        if (port == -1) return bsp_servo_set_period_ABCD(value);
        else if (port < 2) return bsp_servo_set_period_AB(value);
        else return bsp_servo_set_period_CD(value);
    }
    case BSP_SERVO_CONFIG_ENABLE: { return bsp_servo_set_enable(port, value); }
    default: return ESP_ERR_NOT_SUPPORTED;
    }
}
// Global command read function
int32_t bsp_cmd_read(bsp_cmd_t cmd, uint8_t port) {
    if (!bsp_initialized) { return 0; }
    // Handle read supported commands
    switch (cmd) {
    // Board
    case BSP_BOARD_REVISION: { return bsp_board_get_revision(); }
    case BSP_BUTTON_STATE: { return bsp_board_get_button(); }
    case BSP_USB_STATE: { return bsp_board_get_usb(); }
    // Battery
    case BSP_BATTERY_VOLTAGE: { return bsp_battery_get_voltage(); }
    case BSP_BATTERY_CURRENT: { return bsp_battery_get_current(); }
    case BSP_BATTERY_CHARGING: { return bsp_battery_get_charging(); }
    // DC
    case BSP_DC_CONFIG_FREQUENCY: { return bsp_dc_get_frequency(port); }
    case BSP_DC_CONFIG_DECAY: { return bsp_dc_get_decay(port); }
    // Servo
    case BSP_SERVO_CONFIG_PERIOD: {
        if (port < 2) return bsp_servo_get_period_AB();
        else if (port < 4 && bsp_servo_cnt == 4) return bsp_servo_get_period_CD();
        else return 0;
    }
    }
    return 0;
}
// Global command interrupt register fuction
esp_err_t bsp_register_interrupt_callback(bsp_cmd_t cmd, bsp_interrupt_func_t func, void *arg) {
    if (!bsp_initialized) { return ESP_ERR_INVALID_STATE; }
    switch (cmd) {
    case BSP_USB_STATE: { return gpio_isr_handler_add(BSP_IO_USB_DETECT, func, arg); }
    case BSP_BUTTON_STATE: { return gpio_isr_handler_add(BSP_IO_BUTTON, func, arg); }
    case BSP_BATTERY_CHARGING: { return gpio_isr_handler_add(BSP_IO_BATTERY_CHARGE, func, arg); }
    default: return ESP_ERR_NOT_SUPPORTED;
    }
}
