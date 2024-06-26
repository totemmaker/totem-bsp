/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "hal/adc_types.h"

#include "bsp/roboboard_x4.h"
#include "lib/periph_driver.h"
// Macros to return peripheral port command and GPIO pin setup
#define RegPort(cmd, port) (cmd + (((port+1)&0xF)*0x10))
#define GPIO_SEL(pin)   ((uint64_t)(((uint64_t)1)<<(pin)))
// Amount of peripheral ports available
#define DC_CNT 4
#define SERVO_CNT 3
#define RGB_CNT 4
// Error handler
#define BSP_ERR(bsp_func) { esp_err_t err = bsp_func; if (err) return err; }
// IDF version dependent ADC setup functions
void bsp_adc1_init(adc_channel_t channel);
uint32_t bsp_adc1_get_raw();
uint32_t bsp_adc1_raw_to_voltage(uint32_t adc);
// Runtime states
static bool bsp_initialized = false;
static bool bsp_isV15x = false;
// Store values
static uint32_t bsp_dc_decay[DC_CNT];
static uint32_t bsp_dc_frequency[2] = { 20000, 20000 }; 
static uint32_t bsp_servo_pulse[SERVO_CNT];
static uint32_t bsp_servo_period = 20000;
static struct { bsp_evt_func_t func; void *arg; } bsp_evt_handler;
// Peripheral value update handler
static void periph_on_value_update(PeriphRegMap reg, uint32_t value) {
    if (bsp_evt_handler.func == NULL) return;
    // Servo motor position update
    if ((reg & PERIPH_SERVO_X_GET_PULSE) == PERIPH_SERVO_X_GET_PULSE) {
        uint32_t port = PeriphRegMap_channel(reg);
        if (port >= SERVO_CNT) return;
        bsp_servo_pulse[port] = value;
        bsp_evt_handler.func(BSP_EVT_SERVO_PULSE, port, value, bsp_evt_handler.arg);
    }
}
// Board interrupt handler
static void bsp_on_isr(void *arg) {
    if (bsp_evt_handler.func == NULL) return;
    uint32_t value = 0;
    switch ((uint32_t)arg) {
    case BSP_EVT_USB: value = bsp_board_get_usb(); break;
    case BSP_EVT_POWER: value = bsp_board_get_power(); break;
    case BSP_EVT_BUTTON: value = bsp_board_get_button(); break;
    default: return;
    }
    bsp_evt_handler.func((uint32_t)arg, 0, value, bsp_evt_handler.arg);
}

/**************************************************************************************************
 * Totem RoboBoard X4 low level control API
 **************************************************************************************************/

/// @brief Initialize BSP driver
/// @return ESP error
esp_err_t bsp_board_init() {
    gpio_config_t pGPIOConfig = {0};
    pGPIOConfig.intr_type = GPIO_INTR_ANYEDGE;
    // Initialize LED & CAN transceiver enable pin
    gpio_set_level(BSP_IO_CAN_EN, 0); // Leave CAN transceiver always On
    pGPIOConfig.mode = GPIO_MODE_OUTPUT;
    pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_LED)|GPIO_SEL(BSP_IO_CAN_EN);
    gpio_config(&pGPIOConfig);
    // Initialize DC & USB detect pins
    pGPIOConfig.mode = GPIO_MODE_INPUT;
    pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_POWER_DETECT)|GPIO_SEL(BSP_IO_USB_DETECT);
    gpio_config(&pGPIOConfig);
    // Initialize Button pin
    pGPIOConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_BUTTON);
    gpio_config(&pGPIOConfig);
    // Initialize battery analog read
    bsp_adc1_init(ADC_CHANNEL_0); // (BSP_IO_BATTERY_VOLTAGE) GPIO36 (SENSOR_VP) of ADC1
    // Establish connection to peripheral driver
    BSP_ERR(periph_driver_init(periph_on_value_update));
    // Initialize pin interrupts
    BSP_ERR(gpio_install_isr_service(0));
    BSP_ERR(gpio_isr_handler_add(BSP_IO_BUTTON, bsp_on_isr, (void*)BSP_EVT_BUTTON));
    BSP_ERR(gpio_isr_handler_add(BSP_IO_USB_DETECT, bsp_on_isr, (void*)BSP_EVT_USB));
    if (bsp_board_revision != 11) { // Revision 1.1 does not support DC power jack detection
        BSP_ERR(gpio_isr_handler_add(BSP_IO_POWER_DETECT, bsp_on_isr, (void*)BSP_EVT_POWER));
    }
    // Load default configuration values
    if (bsp_driver_version < 160) {
        // Disable internal servo range limit
        for (int i=0; i<SERVO_CNT; i++) {
            periph_driver_write(RegPort(PERIPH_SERVO_X_PULSE_MIN, i), 0);
            periph_driver_write(RegPort(PERIPH_SERVO_X_PULSE_MAX, i), bsp_servo_period);
        }
        // Set all motors to coast (instead of 1500 position)
        periph_driver_write(PERIPH_SERVO_SET_ABC_PULSE, 0);
        // Older driver version defaults to 50Hz, fast decay
        for (int i=0; i<2; i++) { bsp_dc_frequency[i] = 50; }
        for (int i=0; i<DC_CNT; i++) { bsp_dc_decay[i] = 1; }
        bsp_isV15x = true;
        // Wait for driver to process commands
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    else {
        // Enable RGB
        bsp_rgb_set_enable(BSP_PORT_ALL, 1);
        // Request servo position updates
        BSP_ERR(periph_driver_subscribe(PERIPH_SERVO_X_GET_PULSE));
    }
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

/// @brief Turn LED (pin 13) (default - off)
/// @param state [0] off, [1] on
/// @return ESP error
esp_err_t bsp_board_set_led(uint32_t state) {
    return gpio_set_level(BSP_IO_LED, !!state);
}
/// @brief Turn 5V power rail (default - on) (v1.1 only)
/// @param state [0] off, [1] on
/// @note  Affects servo (+) and RGB power
/// @return ESP error
esp_err_t bsp_board_set_5V(uint32_t state) {
    if (bsp_board_revision == 10) return ESP_ERR_NOT_SUPPORTED;
    return periph_driver_write(PERIPH_DRIVER_CTRL_POWER_5V, !!state);
}
/// @brief Get board revision
/// @return Format: [12] -> v1.2
int bsp_board_get_revision() {
    return bsp_board_revision;
}
/// @brief Get motor driver firmware version
/// @return Format: [123] -> v1.23
int bsp_board_get_firmware() {
    return bsp_driver_version;
}
/// @brief Is button pressed
/// @return [0] released, [1] pressed
int bsp_board_get_button() {
    return !gpio_get_level(BSP_IO_BUTTON);
}
/// @brief Is power adapter plugged in (v1.0 only)
/// @return [0] unplugged, [1] plugged in
int bsp_board_get_power() {
    if (bsp_board_revision == 11) return 0; // Not supported on v1.1
    return !gpio_get_level(BSP_IO_POWER_DETECT);
}
/// @brief Is USB cable plugged in
/// @return [0] unplugged, [1] plugged in
int bsp_board_get_usb() {
    return !gpio_get_level(BSP_IO_USB_DETECT);
}

/*******************************
 * Battery control functions
 ******************************/

/// @brief Read battery voltage
/// @return [8400:12600]mV (millivolts)
int bsp_battery_get_voltage() {
    // Read pin voltage
    uint32_t adcReading = 0;
    // Oversample
    for (int i=0; i<16; i++) {
        adcReading += bsp_adc1_get_raw();
    }
    adcReading /= 16;
    // Convert adcReading to pin voltage in mV
    int32_t pinVoltage = bsp_adc1_raw_to_voltage(adcReading);
    // Covert pin voltage to battery voltage
    return pinVoltage * 1124 / 124; // R1: 1M, R2: 124k
}

/*******************************
 * DC control functions
 ******************************/

/// @brief Spin motor
/// @param portID [0:3] port number, [-1] all ports
/// @param power [-100:100]% power and direction, [0] no power
/// @return ESP error
esp_err_t bsp_dc_spin(int8_t portID, int32_t power) {
    if (portID < -1 || portID >= DC_CNT) return ESP_ERR_NOT_FOUND;
    if (power < -100 || power > 100) return ESP_ERR_INVALID_ARG;
    // Write single port
    PeriphRegMap reg = RegPort(PERIPH_DC_X_POWER, portID);
    // Write all ports
    if (portID == -1) {
        reg = PERIPH_DC_SET_ABCD_POWER;
        int8_t *valuePtr = (int8_t*)&power;
        valuePtr[0]=valuePtr[1]=valuePtr[2]=valuePtr[3] = power;
    }
    return periph_driver_write(reg, power);
}
/// @brief Brake motor
/// @param portID [0:3] port number, [-1] all ports
/// @param power [0:100]% power and direction, [0] coast
/// @return ESP error
esp_err_t bsp_dc_brake(int8_t portID, uint32_t power) {
    if (portID < -1 || portID >= DC_CNT) return ESP_ERR_NOT_FOUND;
    if (power > 100) return ESP_ERR_INVALID_ARG;
    // Write single port
    PeriphRegMap reg = RegPort(PERIPH_DC_X_BRAKE, portID);
    // Write all ports
    if (portID == -1) {
        reg = PERIPH_DC_SET_ABCD_BRAKE;
        uint8_t *valuePtr = (uint8_t*)&power;
        valuePtr[0]=valuePtr[1]=valuePtr[2]=valuePtr[3] = power;
    }
    return periph_driver_write(reg, power);
}
/// @brief Output audible tone to AB ports
/// @param frequency [0:20000]Hz tone frequency
/// @param duration duration of time play (ms), [0] indefinitely
/// @return ESP error
esp_err_t bsp_dc_tone_AB(uint32_t frequency, uint32_t duration) {
    if (frequency > 20000) return ESP_ERR_INVALID_ARG;
    if (duration > 0xFFFF) return ESP_ERR_INVALID_ARG;
    return periph_driver_write(PERIPH_DC_SET_AB_TONE, (duration << 16) | frequency);
}
/// @brief Output audible tone to CD ports
/// @param frequency [0:20000]Hz tone frequency
/// @param duration duration of time play (ms), [0] indefinitely
/// @return ESP error
esp_err_t bsp_dc_tone_CD(uint32_t frequency, uint32_t duration) {
    if (frequency > 20000) return ESP_ERR_INVALID_ARG;
    if (duration > 0xFFFF) return ESP_ERR_INVALID_ARG;
    return periph_driver_write(PERIPH_DC_SET_CD_TONE, (duration << 16) | frequency);
}
/// @brief Output audible tone to ABCD ports
/// @param frequency [0:20000]Hz tone frequency
/// @param duration duration of time play (ms), [0] indefinitely
/// @return ESP error
esp_err_t bsp_dc_tone_ABCD(uint32_t frequency, uint32_t duration) {
    if (frequency > 20000) return ESP_ERR_INVALID_ARG;
    if (duration > 0xFFFF) return ESP_ERR_INVALID_ARG;
    if (bsp_isV15x) {
        periph_driver_write(PERIPH_DC_SET_AB_TONE, (duration << 16) | frequency);
        return periph_driver_write(PERIPH_DC_SET_CD_TONE, (duration << 16) | frequency);
    }
    return periph_driver_write(PERIPH_DC_SET_ABCD_TONE, (duration << 16) | frequency);
}
/// @brief Toggle port output (default - enabled)
/// @param portID [0:3] port number, [-1] all ports
/// @param enable [0] disable output, [1] enable output
/// @return ESP error
esp_err_t bsp_dc_set_enable(int8_t portID, uint32_t enable) {
    if (portID < -1 || portID >= DC_CNT) return ESP_ERR_NOT_FOUND;
    // Write single port
    PeriphRegMap reg = RegPort(PERIPH_DC_X_ENABLE, portID);
    enable = !!enable;
    // Write all ports
    if (portID == -1) {
        reg = PERIPH_DC_SET_ABCD_ENABLE;
        enable = enable ? 0x01010101 : 0;
    }
    return periph_driver_write(reg, enable);
}
/// @brief Change decay mode (default - slow)
/// @param portID [0:3] port number, [-1] all ports
/// @param decay [0] slow decay, [1] fast decay
/// @return ESP error
esp_err_t bsp_dc_set_decay(int8_t portID, uint32_t decay) {
    // Decay mode change is not available in versions prior v1.60
    if (bsp_isV15x) return ESP_ERR_NOT_SUPPORTED;
    if (portID < -1 || portID >= DC_CNT) return ESP_ERR_NOT_FOUND;
    if (decay != 0 && decay != 1) return ESP_ERR_INVALID_ARG;
    // Write single port
    PeriphRegMap reg = RegPort(PERIPH_DC_X_DECAY, portID);
    // Write all ports
    if (portID == -1) {
        reg = PERIPH_DC_SET_ABCD_DECAY;
        for (int i=0; i<DC_CNT; i++) { bsp_dc_decay[i] = decay; }
        uint8_t *valuePtr = (uint8_t*)&decay;
        valuePtr[0]=valuePtr[1]=valuePtr[2]=valuePtr[3] = decay;
    }
    else bsp_dc_decay[portID] = decay;
    return periph_driver_write(reg, decay);
}
/// @brief Change PWM frequency for AB ports (default - 20000)
/// @param frequency [1:250000]Hz PWM frequency
/// @return ESP error
esp_err_t bsp_dc_set_frequency_AB(uint32_t frequency) {
    if (frequency < 1 || frequency > 250000) return ESP_ERR_INVALID_ARG;
    // Write frequency to AB group
    bsp_dc_frequency[0] = frequency;
    return periph_driver_write(PERIPH_DC_SET_AB_FREQUENCY, frequency);
}
/// @brief Change PWM frequency for CD ports (default - 20000)
/// @param frequency [1:250000]Hz PWM frequency
/// @return ESP error
esp_err_t bsp_dc_set_frequency_CD(uint32_t frequency) {
    if (frequency < 1 || frequency > 250000) return ESP_ERR_INVALID_ARG;
    // Write frequency to CD group
    bsp_dc_frequency[1] = frequency;
    return periph_driver_write(PERIPH_DC_SET_CD_FREQUENCY, frequency);
}
/// @brief Change PWM frequency for ABCD ports (default - 20000)
/// @param frequency [1:250000]Hz PWM frequency
/// @return ESP error
esp_err_t bsp_dc_set_frequency_ABCD(uint32_t frequency) {
    if (frequency < 1 || frequency > 250000) return ESP_ERR_INVALID_ARG;
    // Write frequency to AB and CD groups
    bsp_dc_frequency[0] = frequency;
    bsp_dc_frequency[1] = frequency;
    if (bsp_isV15x) { // Single frequency set is not available in versions prior v1.60
        bsp_dc_set_frequency_AB(frequency);
        bsp_dc_set_frequency_CD(frequency);
        return ESP_OK;
    }
    return periph_driver_write(PERIPH_DC_SET_ABCD_FREQUENCY, frequency);
}
/// @brief Change group AB mode (default - individual)
/// @param mode [0] individual control, [1] group control
/// @return ESP error
esp_err_t bsp_dc_set_mode_AB(uint32_t mode) {
    // Group mode is not available in versions prior v1.60
    if (bsp_isV15x) return ESP_ERR_NOT_SUPPORTED;
    if (mode > 1) return ESP_ERR_INVALID_ARG;
    return periph_driver_write(PERIPH_DC_SET_AB_MODE, mode);
}
/// @brief Change group CD mode (default - individual)
/// @param mode [0] individual control, [1] group control
/// @return ESP error
esp_err_t bsp_dc_set_mode_CD(uint32_t mode) {
    // Group mode is not available in versions prior v1.60
    if (bsp_isV15x) return ESP_ERR_NOT_SUPPORTED;
    if (mode > 1) return ESP_ERR_INVALID_ARG;
    return periph_driver_write(PERIPH_DC_SET_CD_MODE, mode);
}
/// @brief Get selected decay mode
/// @param portID [0:3] port number
/// @return [0] slow decay, [1] fast decay
int bsp_dc_get_decay(uint8_t portID) {
    if (portID >= DC_CNT) return 0;
    return bsp_dc_decay[portID];
}
/// @brief Get configured PWM frequency of AB ports
/// @return [1:250000]Hz PWM frequency
int bsp_dc_get_frequency_AB() {
    return bsp_dc_frequency[0];
}
/// @brief Get configured PWM frequency of CD ports
/// @return [1:250000]Hz PWM frequency
int bsp_dc_get_frequency_CD() {
    return bsp_dc_frequency[1];
}

/*******************************
 * Servo control functions
 ******************************/

/// @brief Spin servo motor to position
/// @param portID [0:2] port number, [-1] all ports
/// @param pulse [0:period]us position
/// @return ESP error
esp_err_t bsp_servo_spin(int8_t portID, uint32_t pulse) {
    return bsp_servo_spin_duration(portID, pulse, 0);
}
/// @brief Spin servo motor to position
/// @param portID [0:2] port number, [-1] all ports
/// @param pulse [0:period]us position
/// @param duration [0] max speed, spin speed in duration (ms) (overrides default speed)
/// @note  Duration: amount of time to spin from current to target position
/// @return ESP error
esp_err_t bsp_servo_spin_duration(int8_t portID, uint32_t pulse, uint32_t duration) {
    if (portID < -1 || portID >= SERVO_CNT) return ESP_ERR_NOT_FOUND;
    if (pulse > bsp_servo_period) return ESP_ERR_INVALID_ARG;
    if (duration > 0x7FFF) return ESP_ERR_INVALID_ARG;
    // Bugfix: Workaround for mixed servo ports in older RoboBoard X4 v1.0 revision firmware
    if (bsp_board_revision == 10 && bsp_isV15x) { portID = portID == 2 ? 1 : (portID == 1 ? 2 : portID); }
    // Write single port
    PeriphRegMap reg = RegPort(PERIPH_SERVO_X_PULSE, portID);
    // Write all ports
    if (portID == -1) {
        reg = PERIPH_SERVO_SET_ABC_PULSE;
        // Workaround: For firmware prior v1.60, update pulse value directly
        if (bsp_isV15x) for (int i=0; i<SERVO_CNT; i++) {
            bsp_servo_pulse[i] = pulse;
            bsp_evt_handler.func(BSP_EVT_SERVO_PULSE, i, pulse, bsp_evt_handler.arg);
        }
    }
    // Workaround: For firmware prior v1.60, update pulse value directly
    else if (bsp_isV15x) {
        bsp_servo_pulse[portID] = pulse;
        bsp_evt_handler.func(BSP_EVT_SERVO_PULSE, portID, pulse, bsp_evt_handler.arg);
    }
    return periph_driver_write(reg, (duration << 16) | pulse);
}
/// @brief Spin servo motor to position
/// @param portID [0:2] port number, [-1] all ports
/// @param pulse [0:period]us position
/// @param ppp [0] max speed, spin speed in PPP unit (overrides default speed)
/// @note  Pulse-Per-Period: ppp = (RPM * 6 * (motorUsMax - motorUsMin)) / motorAngle / 50
/// @return ESP error
esp_err_t bsp_servo_spin_ppp(int8_t portID, uint32_t pulse, uint32_t ppp) {
    // PPP speed unit was not available in versions prior v1.60
    if (bsp_isV15x) return ESP_ERR_NOT_SUPPORTED;
    if (portID < -1 || portID >= SERVO_CNT) return ESP_ERR_NOT_FOUND;
    if (pulse > bsp_servo_period) return ESP_ERR_INVALID_ARG;
    if (ppp > bsp_servo_period) return ESP_ERR_INVALID_ARG;
    // Bugfix: Workaround for mixed servo ports in older RoboBoard X4 v1.0 revision firmware
    if (bsp_board_revision == 10 && bsp_isV15x) { portID = portID == 2 ? 1 : (portID == 1 ? 2 : 0); }
    // Write single port
    PeriphRegMap reg = RegPort(PERIPH_SERVO_X_PULSE, portID);
    // Write all ports
    if (portID == -1) {
        reg = PERIPH_SERVO_SET_ABC_PULSE;
        // Workaround: For firmware prior v1.60, update pulse value directly
        if (bsp_isV15x) for (int i=0; i<SERVO_CNT; i++) {
            bsp_servo_pulse[i] = pulse;
            bsp_evt_handler.func(BSP_EVT_SERVO_PULSE, i, pulse, bsp_evt_handler.arg);
        }
    }
    // Workaround: For firmware prior v1.60, update pulse value directly
    else if (bsp_isV15x) {
        bsp_servo_pulse[portID] = pulse;
        bsp_evt_handler.func(BSP_EVT_SERVO_PULSE, portID, pulse, bsp_evt_handler.arg);
    }
    return periph_driver_write(reg, ((0x8000|ppp) << 16) | pulse);
}
/// @brief Toggle port output (default - enabled)
/// @param portID [0:2] port number, [-1] all ports
/// @param enable [0] disable output, [1] enable output
/// @return ESP error
esp_err_t bsp_servo_set_enable(int8_t portID, uint32_t enable) {
    if (portID < -1 || portID >= SERVO_CNT) return ESP_ERR_NOT_FOUND;
    // Write single port
    PeriphRegMap reg = RegPort(PERIPH_SERVO_X_ENABLE, portID);
    enable = !!enable;
    // Write all ports
    if (portID == -1) {
        reg = PERIPH_SERVO_SET_ABC_ENABLE;
        enable = enable ? 0x01010101 : 0;
    }
    return periph_driver_write(reg, enable);
}
/// @brief Configure constant motor speed (default - disabled)
/// @param portID [0:2] port number, [-1] all ports
/// @param ppp [0] disabled (max speed), spin speed in PPP unit
/// @note  Pulse-Per-Period: ppp = (RPM * 6 * (motorUsMax - motorUsMin)) / motorAngle / 50
/// @return ESP error
esp_err_t bsp_servo_set_speed_ppp(int8_t portID, uint32_t ppp) {
    // Constant speed control is no more supported for old firmware
    if (bsp_isV15x) return ESP_ERR_NOT_SUPPORTED;
    if (portID < -1 || portID >= SERVO_CNT) return ESP_ERR_NOT_FOUND;
    if (ppp > bsp_servo_period) return ESP_ERR_INVALID_ARG;
    // Write single port
    PeriphRegMap reg = RegPort(PERIPH_SERVO_X_SPEED, portID);
    // Write all ports
    if (portID == -1) { reg = PERIPH_SERVO_SET_ABC_SPEED; }
    return periph_driver_write(reg, ppp);
}
/// @brief Change PWM period for ABC ports (default - 20000)
/// @param period [1:65535]us PWM period
/// @return ESP error
esp_err_t bsp_servo_set_period_ABC(uint32_t period) {
    if (period < 1 || period > 0xFFFF) return ESP_ERR_INVALID_ARG;
    // Write servo period (us) to all ports
    bsp_servo_period = period;
    esp_err_t err = periph_driver_write(PERIPH_SERVO_SET_ABC_PERIOD, period);
    // Workaround: ignore internal servo limit and inversion. Allow full range control prior v1.60
    if (bsp_isV15x) {
        for (int i=0; i<SERVO_CNT; i++) {
            periph_driver_write(RegPort(PERIPH_SERVO_X_PULSE_MIN, i), 0);
            periph_driver_write(RegPort(PERIPH_SERVO_X_PULSE_MAX, i), period);
        }
    }
    return err;
}
/// @brief Get configured PWM period of ABC ports
/// @return [1:65535]us PWM period
int bsp_servo_get_period_ABC() {
    return bsp_servo_period;
}
/// @brief Read servo motor position
/// @param portID [0:2] port number
/// @return [0:period]us position pulse
int bsp_servo_get_pulse(uint8_t portID) {
    if (portID >= SERVO_CNT) return 0;
    return bsp_servo_pulse[portID];
}

/*******************************
 * RGB control functions
 ******************************/

/// @brief Change RGB light color
/// @param ledID [0:3] LED number, [-1] all LEDs
/// @param hex 32bit color | Alpha | Red | Green | Blue |
/// @return ESP error
esp_err_t bsp_rgb_color(int8_t ledID, uint32_t hex) {
    if (ledID < -1 || ledID >= RGB_CNT) return ESP_ERR_NOT_FOUND;
    // Write single LED
    PeriphRegMap reg = RegPort(PERIPH_RGB_X_SET, ledID);
    // Write all LEDs
    if (ledID == -1) { reg = PERIPH_RGB_SET_ABCD_SET; }
    return periph_driver_write(reg, hex);
}
/// @brief Prepare RGB light fade color
/// @param ledID [0:3] LED number, [-1] all LEDs
/// @param hex 32bit fade color | Alpha | Red | Green | Blue |
/// @return ESP error
esp_err_t bsp_rgb_fade_color(int8_t ledID, uint32_t hex) {
    if (ledID < -1 || ledID >= RGB_CNT) return ESP_ERR_NOT_FOUND;
    // Write single LED
    PeriphRegMap reg = RegPort(PERIPH_RGB_X_SET_FADE, ledID);
    // Write all LEDs
    if (ledID == -1) { reg = PERIPH_RGB_SET_ABCD_FADE; }
    return periph_driver_write(reg, hex);
}
/// @brief Run fade animation
/// @param ledID [0:3] LED number, [-1] all LEDs
/// @param duration animation duration (ms)
/// @return ESP error
esp_err_t bsp_rgb_fade_start(int8_t ledID, uint32_t duration) {
    if (ledID < -1 || ledID >= RGB_CNT) return ESP_ERR_NOT_FOUND;
    // Write single LED
    PeriphRegMap reg = RegPort(PERIPH_RGB_X_START_FADE, ledID);
    // Write all LEDs
    if (ledID == -1) { reg = PERIPH_RGB_SET_ABCD_START_FADE; }
    return periph_driver_write(reg, duration);
}
/// @brief Toggle LED output (default - enabled)
/// @param ledID [0:3] LED number, [-1] all LEDs
/// @param enable [0] LED is disabled, [1] LED is enabled
/// @return ESP error
esp_err_t bsp_rgb_set_enable(int8_t ledID, uint32_t enable) {
    // Individual RGB disable is not supported in firmware prior v1.60
    if (bsp_isV15x && ledID != -1) return ESP_ERR_NOT_SUPPORTED;
    if (ledID < -1 || ledID >= RGB_CNT) return ESP_ERR_NOT_FOUND;
    // Write single LED
    PeriphRegMap reg = RegPort(PERIPH_RGB_X_ENABLE, ledID);
    enable = !!enable;
    // Write all LEDs
    if (ledID == -1) {
        reg = PERIPH_RGB_SET_ABCD_ENABLE;
        enable = enable ? 0x01010101 : 0;
    }
    return periph_driver_write(reg, enable);
}

/*******************************
 * GPIO pin control functions. RoboBoard X4 v1.0 only!
 * (not connected to ESP32 directly)
 ******************************/

/// @brief Read digital state of GPIO pin
/// @param pinID pin number [0:3] (A, B, C, D)
/// @return [0] - LOW, [1] - HIGH
uint32_t bsp_gpio_digital_read(uint8_t pinID) {
    return periph_driver_read(RegPort(PERIPH_GPIO_X_DIGITAL_READ, pinID));
}
/// @brief Write digital state to GPIO pin
/// @param pinID pin number [0:3] (A, B, C, D)
/// @param state pin state: [0] - LOW, [1] - HIGH
void bsp_gpio_mode(uint8_t pinID, uint8_t mode) {
    periph_driver_write(RegPort(PERIPH_GPIO_X_MODE, pinID), mode);
}
/// @brief Configure GPIO mode
/// @param pinID pin number [0:3] (A, B, C, D)
/// @param mode 0-pd, 1-pu, 2-float, 3-out, 4-analog
void bsp_gpio_digital_write(uint8_t pinID, uint8_t state) {
    periph_driver_write(RegPort(PERIPH_GPIO_X_DIGITAL_WRITE, pinID), state);
}
/// @brief Read analog value of GPIO pin
/// @param pinID pin number [0:3] (A, B, C, D)
/// @return [0:1023]
uint32_t bsp_gpio_analog_read(uint8_t pinID) {
    return periph_driver_read(RegPort(PERIPH_GPIO_X_ANALOG_READ, pinID));
}
/// @brief Write analog value (PWM) to GPIO pin
/// @param pinID pin number [0:3] (A, B, C, D)
/// @param value duty cycle [0:20]
void bsp_gpio_analog_write(uint8_t pinID, uint16_t value) {
    periph_driver_write(RegPort(PERIPH_GPIO_X_ANALOG_WRITE, pinID), value);
}

/**************************************************************************************************
 * Legacy command oriented control API. Keeping for backwards compatibility (will be removed)
 **************************************************************************************************/

// Global command write function
esp_err_t bsp_cmd_write(bsp_cmd_t cmd, int8_t port, int32_t value) {
    if (!bsp_initialized) { return ESP_ERR_INVALID_STATE; }
    // Validate ports. X4 has maximum of 4 ports
    if (port < -1 || port > 3) { return ESP_ERR_NOT_FOUND; }
    // Validate commands range
    if (cmd >= BSP_CMD_MAX) { return ESP_ERR_INVALID_ARG; }
    // Handle command
    switch (cmd) {
    // Board
    case BSP_LED_STATE: { return bsp_board_set_led(value); }
    case BSP_CAN_STATE: { return gpio_set_level(BSP_IO_CAN_EN, !value); }
    case BSP_5V_STATE: { return bsp_board_set_5V(value); }
    // DC
    case BSP_DC_POWER: { return bsp_dc_spin(port, value); }
    case BSP_DC_BRAKE: { return bsp_dc_brake(port, value); }
    case BSP_DC_TONE: {
        if (port == -1) return bsp_dc_tone_ABCD(value & 0xFFFF, (value >> 16) & 0xFFFF);
        else if (port < 2) return bsp_dc_tone_AB(value & 0xFFFF, (value >> 16) & 0xFFFF);
        else return bsp_dc_tone_CD(value & 0xFFFF, (value >> 16) & 0xFFFF);
    }
    case BSP_DC_CONFIG_FREQUENCY: {
        if (port == -1) return bsp_dc_set_frequency_ABCD(value);
        else if (port < 2) return bsp_dc_set_frequency_AB(value);
        else return bsp_dc_set_frequency_CD(value);
    }
    case BSP_DC_CONFIG_DECAY: { return bsp_dc_set_decay(port, value); }
    case BSP_DC_CONFIG_ENABLE: { return bsp_dc_set_enable(port, value); }
    // Servo
    case BSP_SERVO_PULSE: {
        if (value & 0x80000000) return bsp_servo_spin_ppp(port, value & 0xFFFF, (value >> 16) & 0x7FFF);
        else return bsp_servo_spin_duration(port, value & 0xFFFF, (value >> 16) & 0x7FFF);
    }
    case BSP_SERVO_CONFIG_SPEED: { return bsp_servo_set_speed_ppp(port, value); }
    case BSP_SERVO_CONFIG_PERIOD: { return bsp_servo_set_period_ABC(value); }
    case BSP_SERVO_CONFIG_ENABLE: { return bsp_servo_set_enable(port, value); }
    // RGB
    case BSP_RGB_COLOR: { return bsp_rgb_color(port, value); }
    case BSP_RGB_FADE_COLOR: { return bsp_rgb_fade_color(port, value); }
    case BSP_RGB_FADE_START: { return bsp_rgb_fade_start(port, value); }
    case BSP_RGB_CONFIG_ENABLE: { return bsp_rgb_set_enable(port, value); }
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
    case BSP_DRIVER_FIRMWARE: { return bsp_board_get_firmware(); }
    case BSP_BUTTON_STATE: { return bsp_board_get_button(); }
    case BSP_POWER_STATE: { return bsp_board_get_power(); }
    case BSP_USB_STATE: { return bsp_board_get_usb(); }
    case BSP_BATTERY_VOLTAGE: { return bsp_battery_get_voltage(); }
    // DC
    case BSP_DC_CONFIG_FREQUENCY: {
        if (port < 2) return bsp_dc_get_frequency_AB();
        else if (port < 4) return bsp_dc_get_frequency_CD();
        else return 0;
    }
    case BSP_DC_CONFIG_DECAY: { return bsp_dc_get_decay(port); }
    // Servo
    case BSP_SERVO_PULSE: { return bsp_servo_get_pulse(port); }
    case BSP_SERVO_CONFIG_PERIOD: {
        if (port >= SERVO_CNT) return 0;
        return bsp_servo_get_period_ABC();
    }
    }
    return 0;
}
// Register interrupt receiver
esp_err_t bsp_register_interrupt_callback(bsp_cmd_t cmd, bsp_interrupt_func_t func, void *arg) {
    if (!bsp_initialized) { return ESP_ERR_INVALID_STATE; }
    switch (cmd) {
    case BSP_BUTTON_STATE: { return gpio_isr_handler_add(BSP_IO_BUTTON, func, arg); }
    case BSP_POWER_STATE: {
        if (bsp_board_revision == 11) return ESP_ERR_NOT_SUPPORTED;
        return gpio_isr_handler_add(BSP_IO_POWER_DETECT, func, arg);
    }
    case BSP_USB_STATE: { return gpio_isr_handler_add(BSP_IO_USB_DETECT, func, arg); }
    default: return ESP_ERR_NOT_SUPPORTED;
    }
}
