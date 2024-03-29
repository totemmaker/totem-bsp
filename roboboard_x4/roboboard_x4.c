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

#include "bsp/roboboard_x4.h"
#include "lib/periph_driver.h"
// Macros to return peripheral port command and GPIO pin setup
#define RegPort(cmd, port) (cmd + ((port+1)*0x10))
#define GPIO_SEL(pin)   ((uint64_t)(((uint64_t)1)<<(pin)))
// Amount of peripheral ports available
#define DC_CNT 4
#define SERVO_CNT 3
#define RGB_CNT 4
// IDF version dependent ADC setup functions
void bsp_adc1_init(adc_channel_t channel);
uint32_t bsp_adc1_get_raw();
uint32_t bsp_adc1_raw_to_voltage(uint32_t adc);
// Runtime states
static bool bsp_initialized = false;
static uint32_t bsp_servo_period = 20000;
static uint32_t bsp_dc_frequency[2] = { 50, 50 };
// Board initialization function
esp_err_t bsp_board_init(void) {
    esp_err_t err = ESP_FAIL;
    gpio_config_t pGPIOConfig = {0};
    // Initialize LED & CAN transceiver enable pin
    gpio_set_level(BSP_IO_CAN_EN, 1); // Turn transceiver Off
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
    err = periph_driver_init();
    // Return initialization state
    if (err == ESP_OK) bsp_initialized = true;;
    return err;
}
// Handler for "board" command write
static int bsp_board_cmd_write(bsp_cmd_t cmd, uint32_t value) {
    // Handle commands
    switch (cmd) {
    case BSP_LED_STATE: { gpio_set_level(BSP_IO_LED, !!value); break; }
    case BSP_CAN_STATE: { gpio_set_level(BSP_IO_CAN_EN, !value); break; }
    case BSP_5V_STATE: { periph_driver_write(PERIPH_DRIVER_CTRL_POWER_5V, !!value); break; }
    default: return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_OK;
}
// Handler for "DC" commands write
static int bsp_dc_cmd_write(bsp_cmd_t cmd, int8_t port, int32_t value) {
    PeriphRegMap reg;
    // Bugfix: DC power is not reset if tone is applied. Reset power manually before writing tone.
    static int8_t dc_power[DC_CNT] = {0};
    // Handle commands
    switch (cmd) {
    case BSP_DC_POWER: {
        // Limit to [-100:100]
        if (value < -100 || value > 100) return ESP_ERR_INVALID_SIZE;
        if (port == -1) { // Write power to all ports
            reg = PERIPH_DC_SET_ABCD_POWER;
            for (int i=0; i<DC_CNT; i++) { dc_power[i] = value; }
            value |= (((int8_t)value)<<8)|(((int8_t)value)<<16)|(((int8_t)value)<<24);
        }
        else { // Write power to single port
            reg = RegPort(PERIPH_DC_X_POWER, port);
            dc_power[port] = value;
        }
        break;
    }
    case BSP_DC_BRAKE: {
        // Limit to [0:100]
        if (value < 0 || value > 100) return ESP_ERR_INVALID_SIZE;
        if (port == -1) { // Write brake to all ports
            reg = PERIPH_DC_SET_ABCD_BRAKE;
            for (int i=0; i<DC_CNT; i++) { dc_power[i] = 0; }
            value |= (((uint8_t)value)<<8)|(((uint8_t)value)<<16)|(((uint8_t)value)<<24);
        }
        else { // Write brake to single port
            reg = RegPort(PERIPH_DC_X_BRAKE, port);
            dc_power[port] = value;
        }
        break;
    }
    case BSP_DC_TONE: {
        // Limit to [0:20000]
        if ((value & 0xFFFF) > 20000) return ESP_ERR_INVALID_SIZE;
        if (port == -1) { // Write tone to all ports
            for (int i=0; i<DC_CNT; i++) { if (dc_power[i]) { bsp_dc_cmd_write(BSP_DC_POWER, -1, 0); break; }}
            periph_driver_write(PERIPH_DC_SET_AB_TONE, value);
            periph_driver_write(PERIPH_DC_SET_CD_TONE, value);
            return ESP_OK;
        }
        else if (port < 2) { // Write tone to AB group
            if (dc_power[0]) { bsp_dc_cmd_write(BSP_DC_POWER, 0, 0); }
            if (dc_power[1]) { bsp_dc_cmd_write(BSP_DC_POWER, 1, 0); }
            reg = PERIPH_DC_SET_AB_TONE;
        }
        else { // Write tone to CD group
            if (dc_power[2]) { bsp_dc_cmd_write(BSP_DC_POWER, 2, 0); }
            if (dc_power[3]) { bsp_dc_cmd_write(BSP_DC_POWER, 3, 0); }
            reg = PERIPH_DC_SET_CD_TONE;
        }
        break;
    }
    case BSP_DC_CONFIG_FREQUENCY: {
        // Limit to [1:250000]
        if (value < 1 || value > 250000) return ESP_ERR_INVALID_SIZE;
        if (port == -1) { // Write frequency to all ports
            periph_driver_write(PERIPH_DC_SET_AB_FREQUENCY, value);
            periph_driver_write(PERIPH_DC_SET_CD_FREQUENCY, value);
            bsp_dc_frequency[0] = value;
            bsp_dc_frequency[1] = value;
            return ESP_OK;
        }
        else { // Write frequency to AB or CD group
            reg = port < 2 ? PERIPH_DC_SET_AB_FREQUENCY : PERIPH_DC_SET_CD_FREQUENCY;
            bsp_dc_frequency[port < 2 ? 0 : 1] = value;
        }
        break;
    }
    case BSP_DC_CONFIG_ENABLE: {
        if (port == -1) { // Switch all ports
            reg = PERIPH_DC_SET_ABCD_ENABLE;
            value = value ? 0x01010101 : 0;
        }
        else { // Switch single port
            reg = RegPort(PERIPH_DC_X_ENABLE, port);
            value = !!value;
        }
        break;
    }
    default: return ESP_ERR_NOT_FOUND;
    }
    // Send command to peripheral driver
    periph_driver_write(reg, value);
    return ESP_OK;
}
// Handler for "servo" commands write
static int bsp_servo_cmd_write(bsp_cmd_t cmd, int8_t port, uint32_t value) {
    // Validate port. X4 has 3 servo ports
    if (port >= SERVO_CNT) { return ESP_ERR_INVALID_ARG; }
    PeriphRegMap reg;
    // Bugfix: Workaround for mixed servo ports in older RoboBoard X4 v1.0 revision
    if (bsp_board_revision == 10) {
        if (port == 1) port = 2;
        else if (port == 2) port = 1;
    }
    // Handle commands
    switch (cmd) {
    case BSP_SERVO_PULSE: {
        // Limit to [0:bsp_servo_period]
        if ((value & 0xFFFF) > bsp_servo_period) return ESP_ERR_INVALID_SIZE;
        // Write pulse (us) to all ports or single one
        reg = port == -1 ? PERIPH_SERVO_SET_ABC_PULSE : RegPort(PERIPH_SERVO_X_PULSE, port);
        periph_driver_write(reg, value);
        break;
    }
    case BSP_SERVO_CONFIG_SPEED: {
        if (port == -1) { // Write servo speed to all ports
            for (int i=0; i<SERVO_CNT; i++) {
                periph_driver_write(RegPort(PERIPH_SERVO_X_SPEED, i), value);
            }
        }
        else { // Write servo speed to single port
            reg = RegPort(PERIPH_SERVO_X_SPEED, port);
            periph_driver_write(reg, value);
        }
        break;
    }
    case BSP_SERVO_CONFIG_PERIOD: {
        // Limit to [1:0xFFFF]
        if (value < 1 || value > 0xFFFF) return ESP_ERR_INVALID_SIZE;
        bsp_servo_period = value;
        // Write servo period (us) to all ports
        periph_driver_write(PERIPH_SERVO_SET_ABC_PERIOD, value);
        // Bugfix: ignore internal servo limit and inversion. Allow full range control
        for (int i=0; i<SERVO_CNT; i++) {
            periph_driver_write(RegPort(PERIPH_SERVO_X_PULSE_MIN, i), 0);
            periph_driver_write(RegPort(PERIPH_SERVO_X_PULSE_MAX, i), value);
        }
        break;
    }
    case BSP_SERVO_CONFIG_ENABLE: {
        // Switch all servo ports or only single one
        reg = port == -1 ? PERIPH_SERVO_SET_ABC_ENABLE : RegPort(PERIPH_SERVO_X_ENABLE, port);
        periph_driver_write(reg, !!value);
        break;
    }
    default: return ESP_ERR_NOT_FOUND;
    };
    
    return ESP_OK;
}
// Check if all RGB lights are enabled
#define RGB_ENABLED_ALL() (rgb_en[0] && rgb_en[1] && rgb_en[2] && rgb_en[3])
// Handler for "rgb" commands write
static int bsp_rgb_cmd_write(bsp_cmd_t cmd, int8_t port, uint32_t value) {
    // Workaround: hold states to mimic individual RGB enable control
    static uint8_t rgb_en[RGB_CNT] = {1,1,1,1};
    static uint32_t rgb_last[RGB_CNT];
    static uint32_t rgb_fade[RGB_CNT];
    // Handle commands
    switch (cmd) {
    case BSP_RGB_COLOR: {
        if (port == -1 && RGB_ENABLED_ALL()) { // Write color to all LED
            periph_driver_write(PERIPH_RGB_SET_ABCD_SET, value);
            for (int i=0; i<RGB_CNT; i++) { rgb_last[i] = value; }
        }
        else if (rgb_en[port]) { // Write color to single LED
            periph_driver_write(RegPort(PERIPH_RGB_X_SET, port), value);
            rgb_last[port] = value;
        }
        break; 
    }
    case BSP_RGB_FADE_COLOR: {
        if (port == -1 && RGB_ENABLED_ALL()) { // Set fade color to all LED
            periph_driver_write(PERIPH_RGB_SET_ABCD_FADE, value);
            for (int i=0; i<RGB_CNT; i++) { rgb_fade[i] = value; }
        }
        else if (rgb_en[port]) { // Set fade color to single LED
            periph_driver_write(RegPort(PERIPH_RGB_X_SET_FADE, port), value);
            rgb_fade[port] = value;
        }
        break; 
    }
    case BSP_RGB_FADE_START: {
        if (port == -1 && RGB_ENABLED_ALL()) { // Start fade for all LED
            periph_driver_write(PERIPH_RGB_SET_ABCD_START_FADE, value);
            for (int i=0; i<RGB_CNT; i++) { rgb_last[i] = rgb_fade[i]; }
        }
        else if (rgb_en[port]) { // Start fade for single LED
            periph_driver_write(RegPort(PERIPH_RGB_X_START_FADE, port), value);
            rgb_last[port] = rgb_fade[port];
        }
        break;
    }
    case BSP_RGB_CONFIG_ENABLE: {
        if (port == -1) { // Switch all LED
            for (int i=0; i<RGB_CNT; i++) {
                rgb_en[i] = value;
                periph_driver_write(RegPort(PERIPH_RGB_X_SET, i), value ? rgb_last[i] : 0);
            }
        }
        else { // Switch single LED
            rgb_en[port] = value;
            periph_driver_write(RegPort(PERIPH_RGB_X_SET, port), value ? rgb_last[port] : 0);
        }
        break;
    }
    default: return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}
// Global command write function
esp_err_t bsp_cmd_write(bsp_cmd_t cmd, int8_t port, int32_t value) {
    if (!bsp_initialized) { return ESP_ERR_INVALID_STATE; }
    esp_err_t err = ESP_FAIL;
    // Validate ports. X4 has maximum of 4 ports
    if (port < -1 || port > 3) { return ESP_ERR_INVALID_ARG; }
    // Validate commands range
    if (cmd >= BSP_CMD_MAX) {
        return ESP_ERR_NOT_FOUND;
    }
    else if (cmd >= BSP_RGB_COLOR) { // Call RGB command handler
        err = bsp_rgb_cmd_write(cmd, port, value);
    }
    else if (cmd >= BSP_SERVO_PULSE) { // Call Servo command handler
        err = bsp_servo_cmd_write(cmd, port, value);
    }
    else if (cmd >= BSP_DC_POWER) { // Call DC command handler
        err = bsp_dc_cmd_write(cmd, port, value);
    }
    else { // Call board command handler
        err = bsp_board_cmd_write(cmd, value);
    }
    return err;
}
// Global command read function
int32_t bsp_cmd_read(bsp_cmd_t cmd, uint8_t port) {
    if (!bsp_initialized) { return 0; }
    // Handle read supported commands
    switch (cmd) {
    case BSP_BOARD_REVISION: { return bsp_board_revision; }
    case BSP_DRIVER_FIRMWARE: { return bsp_driver_version; }
    case BSP_BUTTON_STATE: { return !gpio_get_level(BSP_IO_BUTTON); }
    case BSP_POWER_STATE: {
        if (bsp_board_revision == 11) return 0; // Not supported on v1.1
        return !gpio_get_level(BSP_IO_POWER_DETECT);
    }
    case BSP_USB_STATE: { return !gpio_get_level(BSP_IO_USB_DETECT); }
    case BSP_BATTERY_VOLTAGE: {
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
    case BSP_DC_CONFIG_FREQUENCY: {
        if (port >= DC_CNT) return 0;
        return bsp_dc_frequency[port < 2 ? 0 : 1];
    }
    case BSP_SERVO_CONFIG_PERIOD: {
        if (port >= SERVO_CNT) return 0;
        return bsp_servo_period;
    }
    }
    return 0;
}
// Global command interrupt register fuction
esp_err_t bsp_register_interrupt_callback(bsp_cmd_t cmd, bsp_interrupt_func_t func, void *arg) {
    if (!bsp_initialized) { return ESP_ERR_INVALID_STATE; }
    // Check if ISR service is installed
    static bool isr_installed = false;
    if (!isr_installed) {
        esp_err_t err = gpio_install_isr_service(0);
        isr_installed = (err == ESP_OK) || (err == ESP_ERR_INVALID_STATE);
    }
    if (!isr_installed) return ESP_ERR_INVALID_STATE;
    // Setup GPIO pin for interrupt
    gpio_config_t pGPIOConfig = {0};
    pGPIOConfig.mode = GPIO_MODE_INPUT;
    pGPIOConfig.intr_type = GPIO_INTR_ANYEDGE;
    switch (cmd) {
    case BSP_BUTTON_STATE: {
        pGPIOConfig.pull_up_en = GPIO_PULLUP_ENABLE;
        pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_BUTTON);
        gpio_isr_handler_add(BSP_IO_BUTTON, func, arg);
        break;
    }
    case BSP_POWER_STATE: {
        if (bsp_board_revision == 11) return ESP_ERR_NOT_SUPPORTED; // Not supported on v1.1
        pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_POWER_DETECT);
        gpio_isr_handler_add(BSP_IO_POWER_DETECT, func, arg);
        break;
    }
    case BSP_USB_STATE: {
        pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_USB_DETECT);
        gpio_isr_handler_add(BSP_IO_USB_DETECT, func, arg);
        break;
    }
    default: return ESP_ERR_NOT_SUPPORTED;
    }
    gpio_config(&pGPIOConfig);
    return ESP_OK;
}
// GPIO pin control for older RoboBoard X4 v1.0 revision (not connected to ESP32 directly)
uint32_t bsp_gpio_digital_read(uint8_t port) {
    return periph_driver_read(RegPort(PERIPH_GPIO_X_DIGITAL_READ, port));
}
void bsp_gpio_digital_input(uint8_t port, uint8_t pull) {
    periph_driver_write(RegPort(PERIPH_GPIO_X_PULL, port), pull);
}
void bsp_gpio_digital_write(uint8_t port, uint8_t state) {
    periph_driver_write(RegPort(PERIPH_GPIO_X_DIGITAL_WRITE, port), state);
}
uint32_t bsp_gpio_analog_read(uint8_t port) {
    return periph_driver_read(RegPort(PERIPH_GPIO_X_ANALOG_READ, port));
}
void bsp_gpio_analog_write(uint8_t port, uint16_t value) {
    periph_driver_write(RegPort(PERIPH_GPIO_X_ANALOG_WRITE, port), value);
}
