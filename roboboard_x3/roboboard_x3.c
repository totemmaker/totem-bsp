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
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "hal/adc_types.h"

#include "bsp/roboboard_x3.h"
// Macros to return peripheral port command and GPIO pin setup
#define RegPort(cmd, port) (cmd + ((port+1)*0x10))
#define GPIO_SEL(pin)   ((uint64_t)(((uint64_t)1)<<(pin)))
// Amount of peripheral ports available
#define DC_CNT 4
// IDF version dependent ADC setup functions
void bsp_adc1_init(adc_channel_t channel);
uint32_t bsp_adc1_get_raw(adc_channel_t channel);
uint32_t bsp_adc1_raw_to_voltage(uint32_t adc);
// Runtime states
static bool bsp_initialized = false;
static uint8_t bsp_board_revision = 0;
static uint8_t bsp_servo_cnt = 2;
// MCPWM states
enum {
    MCPWM_DECAY_UNDEFINED,
    MCPWM_DECAY_SLOW,
    MCPWM_DECAY_FAST,
};
enum {
    MCPWM_MODE_UNDEFINED,
    MCPWM_MODE_POWER,
    MCPWM_MODE_BRAKE,
    MCPWM_MODE_TONE,
};
static struct MCPWMOutput {
    const mcpwm_unit_t unit;
    const mcpwm_timer_t tim;
    
    uint32_t frequency;
    uint32_t decay; 
    uint32_t mode;
    int32_t value;
    uint32_t frequencySet;

} mcpwm_dc[DC_CNT] = { // Default DC motors configuration
    {MCPWM_UNIT_0, MCPWM_TIMER_0, 20000, MCPWM_DECAY_SLOW, MCPWM_MODE_POWER, 0, 0},
    {MCPWM_UNIT_0, MCPWM_TIMER_1, 20000, MCPWM_DECAY_SLOW, MCPWM_MODE_POWER, 0, 0},
    {MCPWM_UNIT_0, MCPWM_TIMER_2, 20000, MCPWM_DECAY_SLOW, MCPWM_MODE_POWER, 0, 0},
    {MCPWM_UNIT_1, MCPWM_TIMER_0, 20000, MCPWM_DECAY_SLOW, MCPWM_MODE_POWER, 0, 0},
};
static struct MCPWMServo {
    const mcpwm_timer_t tim;
    const mcpwm_generator_t gen;
    uint32_t period;
} mcpwm_servo[4] = { // Default Servo motors configuration
    {MCPWM_TIMER_1, MCPWM_GEN_A, 20000},
    {MCPWM_TIMER_1, MCPWM_GEN_B, 20000},
    {MCPWM_TIMER_2, MCPWM_GEN_A, 20000},
    {MCPWM_TIMER_2, MCPWM_GEN_B, 20000},
};
// MCPWM initialization
static void mcpwm_initialize() {
    // Init MCPWM units with motor pins
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, BSP_IO_MOTORA_INA);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, BSP_IO_MOTORA_INB);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, BSP_IO_MOTORB_INA);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, BSP_IO_MOTORB_INB);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, BSP_IO_MOTORC_INA);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, BSP_IO_MOTORC_INB);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, BSP_IO_MOTORD_INA);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0B, BSP_IO_MOTORD_INB);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, BSP_IO_SERVOA_IN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, BSP_IO_SERVOB_IN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM2A, BSP_IO_SERVOC_IN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM2B, BSP_IO_SERVOD_IN);
    // Configure MCPWM units
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000; // 20kHz for DC
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
    pwm_config.frequency = 50;   // 50Hz for servo
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_2, &pwm_config);
}
// MCPWM reconfigure
static esp_err_t mcpwm_configure(struct MCPWMOutput *mcpwm, uint32_t frequency, uint32_t mode, int32_t value) {
    esp_err_t err = ESP_FAIL;
    // Update frequency
    if (mcpwm->frequencySet != frequency) {
        mcpwm->frequencySet = frequency;
        // Prevent invalid frequency value
        if (frequency < 1) frequency = 1;
        err = mcpwm_set_frequency(mcpwm->unit, mcpwm->tim, frequency);
    }
    // Apply control parameters
    if (mode == MCPWM_MODE_POWER) {
        int A = 0, B = 0; // Select pin state depending on decay mode
        if (mcpwm->decay == MCPWM_DECAY_SLOW) {
            if (value) A = (value < 0) ? 100+value : 100;
            if (value) B = (value < 0) ? 100 : 100-value;
        }
        else if (mcpwm->decay == MCPWM_DECAY_FAST) {
            A = (value < 0) ? 0 : value;
            B = (value < 0) ? -value : 0;
        }
              mcpwm_set_duty(mcpwm->unit, mcpwm->tim, MCPWM_OPR_A, A);
        err = mcpwm_set_duty(mcpwm->unit, mcpwm->tim, MCPWM_OPR_B, B);
    }
    else if (mode == MCPWM_MODE_BRAKE) {
              mcpwm_set_duty(mcpwm->unit, mcpwm->tim, MCPWM_OPR_A, value);
        err = mcpwm_set_duty(mcpwm->unit, mcpwm->tim, MCPWM_OPR_B, value);
    }
    else if (mode == MCPWM_MODE_TONE && mcpwm->mode != MCPWM_MODE_TONE) {
              mcpwm_set_duty(mcpwm->unit, mcpwm->tim, MCPWM_OPR_A, 50);
              mcpwm_set_duty(mcpwm->unit, mcpwm->tim, MCPWM_OPR_B, 50);
              mcpwm_set_duty_type(mcpwm->unit, mcpwm->tim, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        err = mcpwm_set_duty_type(mcpwm->unit, mcpwm->tim, MCPWM_OPR_B, MCPWM_DUTY_MODE_1);
    }
    else {
        return ESP_FAIL;
    }
    // Reset duty type back to normal after tone was activated
    if (mode != MCPWM_MODE_TONE && mcpwm->mode == MCPWM_MODE_TONE) {
              mcpwm_set_duty_type(mcpwm->unit, mcpwm->tim, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
        err = mcpwm_set_duty_type(mcpwm->unit, mcpwm->tim, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
    mcpwm->mode = mode;
    return err;
}
// Board initialization function
esp_err_t bsp_board_init(void) {
    gpio_config_t pGPIOConfig = {0};
    // Initialize 3V3 en pin
    pGPIOConfig.mode = GPIO_MODE_OUTPUT;
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
    // Initialize motor pins
    mcpwm_initialize();
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
// Handler for "board" command write
static int bsp_board_cmd_write(bsp_cmd_t cmd, uint32_t value) {
    // Handle commands
    switch (cmd) {
    case BSP_3V_STATE: { gpio_set_level(BSP_IO_3V3_EN, value); break; }
    default: return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_OK;
}
// Handler for "DC" commands write
static int bsp_dc_cmd_write(bsp_cmd_t cmd, uint8_t port, int32_t value) {
    if (port >= DC_CNT) { return ESP_ERR_INVALID_ARG; }
    // Load mcpwm handler of selected port
    struct MCPWMOutput *mcpwm = &mcpwm_dc[port];
    // Handle commands
    switch (cmd) {
    case BSP_DC_POWER: {
        // Limit to [-100:100]
        if (value < -100 || value > 100) return ESP_ERR_INVALID_SIZE;
        // Configure port to spin
        mcpwm->value = value;
        return mcpwm_configure(mcpwm, mcpwm->frequency, MCPWM_MODE_POWER, mcpwm->value);
    }
    case BSP_DC_BRAKE: {
        // Limit to [0:100]
        if (value < 0 || value > 100) return ESP_ERR_INVALID_SIZE;
        // Configure port to braking
        mcpwm->value = value;
        return mcpwm_configure(mcpwm, mcpwm->frequency, MCPWM_MODE_BRAKE, mcpwm->value);
    }
    case BSP_DC_TONE: {
        value &= 0xFFFF;
        // Limit to [0:20000]
        if (value > 20000) return ESP_ERR_INVALID_SIZE;
        // Configure port to beep tone
        return mcpwm_configure(mcpwm, value, (value == 0) ? MCPWM_MODE_POWER : MCPWM_MODE_TONE, 0);
    }
    case BSP_DC_CONFIG_FREQUENCY: {
        // Limit to [1:250000]
        if (value < 1 || value > 250000) return ESP_ERR_INVALID_SIZE;
        // Configure motor PWM frequency
        mcpwm->frequency = value;
        if (mcpwm->mode != MCPWM_MODE_TONE) {
            return mcpwm_configure(mcpwm, mcpwm->frequency, mcpwm->mode, mcpwm->value);
        }
        break;
    }
    case BSP_DC_CONFIG_DECAY: {
        // Limit to [0-slow, 1-fast]
        if (value != 0 && value != 1) return ESP_ERR_INVALID_SIZE;
        // Configure motor decay mode
        mcpwm->decay = (value == 0) ? MCPWM_DECAY_SLOW : MCPWM_DECAY_FAST;
        if (mcpwm->mode != MCPWM_MODE_TONE) {
            return mcpwm_configure(mcpwm, mcpwm->frequency, mcpwm->mode, mcpwm->value);
        }
        break;
    }
    case BSP_DC_CONFIG_ENABLE: {
        // enable / disable port
        if (value) return mcpwm_start(mcpwm->unit, mcpwm->tim);
        return mcpwm_stop(mcpwm->unit, mcpwm->tim);
    }
    default: return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}
// Handler for "servo" commands write
static int bsp_servo_cmd_write(bsp_cmd_t cmd, uint8_t port, uint32_t value) {
    if (port >= bsp_servo_cnt) { return ESP_ERR_INVALID_ARG; }
    // Handle commands
    switch (cmd) {
    case BSP_SERVO_PULSE: {
        // Limit to [0:period]
        if (value > mcpwm_servo[port].period) return ESP_ERR_INVALID_SIZE;
        // Configure spin position
        return mcpwm_set_duty_in_us(MCPWM_UNIT_1, mcpwm_servo[port].tim, mcpwm_servo[port].gen, value);
    }
    case BSP_SERVO_CONFIG_PERIOD: {
        // Limit to [1:1000000]
        if (value < 1 || value > 1000000) return ESP_ERR_INVALID_SIZE;
        // Configure ports A and B PWM period (us)
        if (port < 2) {
            mcpwm_servo[0].period = value;
            mcpwm_servo[1].period = value;
            return mcpwm_set_frequency(MCPWM_UNIT_1, MCPWM_TIMER_1, 1000000/value);
        }
        else {
            mcpwm_servo[2].period = value;
            mcpwm_servo[3].period = value;
            return mcpwm_set_frequency(MCPWM_UNIT_1, MCPWM_TIMER_2, 1000000/value);
        }
    }
    case BSP_SERVO_CONFIG_ENABLE: {
        // enable / disable port
        if (value) return mcpwm_start(MCPWM_UNIT_1, mcpwm_servo[port].tim);
        else return mcpwm_stop(MCPWM_UNIT_1, mcpwm_servo[port].tim);
    }
    default: return ESP_ERR_NOT_FOUND;
    };
    return ESP_OK;
}
// Global command write function
esp_err_t bsp_cmd_write(bsp_cmd_t cmd, int8_t port, int32_t value) {
    if (!bsp_initialized) { return ESP_ERR_INVALID_STATE; }
    esp_err_t err = ESP_FAIL;
    // Validate ports. X3 has maximum of 4 ports
    if (port < -1 || port > 3) { return ESP_ERR_INVALID_ARG; }
    // Validate commands range
    if (cmd >= BSP_CMD_MAX) {
        return ESP_ERR_NOT_FOUND;
    }
    else if (cmd >= BSP_SERVO_PULSE) { // Call Servo command handler
        if (port == -1) { // Apply to all Servo ports
            for (int i=0; i<bsp_servo_cnt; i++) {
                err = bsp_servo_cmd_write(cmd, i, value);
                if (err) return err;
            }
        }
        else { // Apply to single servo port
            err = bsp_servo_cmd_write(cmd, port, value);
        }
    }
    else if (cmd >= BSP_DC_POWER) { // Call DC command handler
        if (port == -1) { // Apply to all DC ports
            for (int i=0; i<DC_CNT; i++) {
                err = bsp_dc_cmd_write(cmd, i, value);
                if (err) return err;
            }
        }
        else { // Apply to single DC port
            err = bsp_dc_cmd_write(cmd, port, value);
        }
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
    case BSP_BUTTON_STATE: { return gpio_get_level(BSP_IO_BUTTON) == 0; }
    case BSP_USB_STATE: { return gpio_get_level(BSP_IO_USB_DETECT) != 0; }
    case BSP_BATTERY_VOLTAGE: {
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
    case BSP_BATTERY_CURRENT: {
        // Read pin voltage
        int32_t adcReading = 0;
        // Oversample
        for (int i=0; i<16; i++) {
            adcReading += bsp_adc1_get_raw(ADC_CHANNEL_0);
        }
        adcReading /= 16;
        return (adcReading - 1400) * -2;
    }
    case BSP_BATTERY_CHARGING: { return !gpio_get_level(BSP_IO_BATTERY_CHARGE); }
    case BSP_DC_CONFIG_FREQUENCY: {
        if (port >= DC_CNT) return 0;
        return mcpwm_dc[port].frequency;
    }
    case BSP_DC_CONFIG_DECAY: {
        if (port >= DC_CNT) return 0;
        return mcpwm_dc[port].decay == MCPWM_DECAY_SLOW ? 0 : 1;
    }
    case BSP_SERVO_CONFIG_PERIOD: {
        if (port >= bsp_servo_cnt) return 0;
        return mcpwm_servo[port].period;
    }
    }
    return ESP_OK;
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
    case BSP_USB_STATE: {
        pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_USB_DETECT);
        gpio_isr_handler_add(BSP_IO_USB_DETECT, func, arg);
        break;
    }
    case BSP_BATTERY_CHARGING: {
        pGPIOConfig.pull_up_en = GPIO_PULLUP_ENABLE;
        pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_BATTERY_CHARGE);
        gpio_isr_handler_add(BSP_IO_BATTERY_CHARGE, func, arg);
        break;
    }
    default: return ESP_ERR_NOT_SUPPORTED;
    }
    gpio_config(&pGPIOConfig);
    return ESP_OK;
}
