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
#include "driver/i2c.h"

#include "esp_timer.h"
#include "esp_mac.h"

#include "bsp/roboboard_x4.h"
#include "components/bsp_adc.h"
#include "components/icm20689.h"
#include "components/lsm6ds3.h"
#include "components/periph_driver.h"
#include "components/tbus.h"

#define RegPort(cmd, port) (cmd + ((port+1)*0x10))
#define GPIO_SEL(pin)   ((uint64_t)(((uint64_t)1)<<(pin)))

static int32_t bsp_cmd_data[BSP_CMD_MAX][4];
static const int32_t bsp_cmd_limit[BSP_CMD_MAX] = {
    [BSP_BOARD_SERIAL]         = 0, // Board serial [0:0x7FFF]
    [BSP_BOARD_REVISION]       = 0, // Board revision [0:990]
    [BSP_DRIVER_FIRMWARE]      = 0, // Driver firmware version [0:999]
    [BSP_BUTTON_STATE]         = 0, // Button state [0:1]
    [BSP_LED_STATE]            =-1, // LED state [0:any]
    [BSP_TBUS_STATE]           = 1, // Disable / Enable TBUS [0:1]
    [BSP_5V_STATE]             =-1, // Disable / Enable 5V power rail [0:any]
    [BSP_DC_STATE]             = 0, // Is DC jack plugged? [0:1] (v1.0 only)
    [BSP_USB_STATE]            = 0, // Is USB cable plugged? [0:1]
    [BSP_BATTERY_VOLTAGE]      = 0, // Battery voltage [8400:12600]
    [BSP_BATTERY_STATE]        = 0, // Battery state [0 - none, 1 - charging]

    [BSP_DC_POWER]             =   -100, // Write power to motor [-100:100]. 0 - stop
    [BSP_DC_BRAKE]             =    100, // Apply braking power [0:100] (0 - coast)
    [BSP_DC_TONE]              =      0, // AB, CD group. Play tone on motor [0:20000]. ((duration << 16) | freq)
    [BSP_DC_CONFIG_INVERT]     =     -1, // Invert port no / yes [0:any]
    [BSP_DC_CONFIG_FREQUENCY]  = 250000, // AB, CD group. Set PWM frequency [0:250000]
    [BSP_DC_CONFIG_BRAKE]      =    100, // Set autobrake power. (Braking when power = 0) [0:100]
    [BSP_DC_CONFIG_ENABLE]     =     -1, // Disabled / Enabled [0:any]

    [BSP_SERVO_PULSE]          = 0, // Write pulse [0:period]. ((duration << 16) | pulse)
    [BSP_SERVO_CONFIG_INVERT]  =-1, // Invert port no / yes [0:any]
    [BSP_SERVO_CONFIG_SPEED]   = 0, // Limit servo speed [0:(rpm*60)]
    [BSP_SERVO_CONFIG_PERIOD] = 0xFFFF, // ABC group. PWM period [1:0xFFFF] (default 20000us)
    [BSP_SERVO_CONFIG_RANGE]   = 0, // Microseconds range of 180 degrees (min << 16 | max)
    [BSP_SERVO_CONFIG_ENABLE]  =-1, // Disabled / Enabled [0:any]

    [BSP_RGB_COLOR]            = 0, // HEX color with alpha [0:0xFFFFFFFF]
    [BSP_RGB_FADE_COLOR]       = 0, // Fade HEX color with alpha [0:0xFFFFFFFF]
    [BSP_RGB_FADE_START]       = 0, // Start fade animation [0:(duration ms)]
    [BSP_RGB_CONFIG_ENABLE]    =-1, // Disabled / Enabled [0:any]
};

static bsp_cmd_change_func_t bsp_cmd_callback[3];
static volatile uint32_t bsp_battery_stateTime = 0;

static void execute_bsp_callback(bsp_cmd_t cmd, uint8_t port, int32_t value, uint8_t isr) {
    for (int i=0; i<3; i++) {
        if (bsp_cmd_callback[i] == 0) break;
        bsp_cmd_callback[i](cmd, port, value, isr);
    }
}

static uint32_t IRAM_ATTR getUptime() {
    return (uint32_t) (esp_timer_get_time() / 1000ULL);
}

static void IRAM_ATTR bsp_button_irq(void *arg) {
    // Call update handler on button state change
    execute_bsp_callback(BSP_BUTTON_STATE, 0, !gpio_get_level(BSP_IO_BUTTON), 1);
}

static void IRAM_ATTR bsp_battery_charging_irq(void *arg) {
    // Store timestamp on charging LED state change
    bsp_battery_stateTime = getUptime();
}

int bsp_board_init(void) {
    esp_err_t err;
    // Install ISR service
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    // Initialize LED pin
    gpio_config_t pGPIOConfig = {0};
    pGPIOConfig.mode = GPIO_MODE_INPUT_OUTPUT;
    pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_LED);
    gpio_config(&pGPIOConfig);
    // Initialize DC & USB detect pins
    pGPIOConfig.mode = GPIO_MODE_INPUT;
    pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_DC_DETECT)|GPIO_SEL(BSP_IO_USB_DETECT);
    gpio_config(&pGPIOConfig);
    // Initialize Button pin
    pGPIOConfig.intr_type = GPIO_INTR_ANYEDGE;
    pGPIOConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_BUTTON);
    gpio_config(&pGPIOConfig);
    gpio_isr_handler_add(BSP_IO_BUTTON, bsp_button_irq, NULL);
    // Initialize Battery charging indication pin
    pGPIOConfig.intr_type = GPIO_INTR_ANYEDGE;
    pGPIOConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    pGPIOConfig.pin_bit_mask = GPIO_SEL(BSP_IO_BATTERY_CHARGE);
    gpio_config(&pGPIOConfig);
    gpio_isr_handler_add(BSP_IO_BATTERY_CHARGE, bsp_battery_charging_irq, NULL);
    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = BSP_IO_I2C_SDA,
        .scl_io_num = BSP_IO_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_DISABLE,
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = CONFIG_BSP_I2C_FREQUENCY,
        .clk_flags = 0
    };
    err = i2c_param_config(CONFIG_BSP_I2C_NUM, &conf);
    if (err) return err;
    err = i2c_driver_install(CONFIG_BSP_I2C_NUM, conf.mode, 0, 0, 0);
    if (err) return err;
    // Reset icm20689 IC
    icm20689_reset();
    // Initialize battery analog read
    bsp_adc1_init(ADC_CHANNEL_0); // (BSP_IO_BATTERY_VOLTAGE) GPIO36 (SENSOR_VP) of ADC1
    // Establish connection to peripheral driver
    err = periph_driver_init();
    if (err) return err;
    // Load default peripheral driver configuration
    bsp_cmd_data[BSP_5V_STATE][0] = 1;
    bsp_cmd_data[BSP_DC_CONFIG_FREQUENCY][0] = 50;
    bsp_cmd_data[BSP_DC_CONFIG_FREQUENCY][2] = 50;
    bsp_cmd_data[BSP_SERVO_CONFIG_PERIOD][0] = 20000;
    for (int port=0; port<4; port++) {
        bsp_cmd_data[BSP_DC_CONFIG_ENABLE][port] = 1;
        bsp_cmd_data[BSP_RGB_CONFIG_ENABLE][port] = 1;
        bsp_cmd_data[BSP_SERVO_CONFIG_ENABLE][port] = 1;
        bsp_cmd_data[BSP_SERVO_CONFIG_RANGE][port] = (500 << 16 | 2500);
    }
    // Initialize IMU
    err = (bsp_boardRevision == 11) ? icm20689_init() : lsm6ds3_init();
    bsp_imu_set_accel_range(16);
    bsp_imu_set_gyro_range(2000);
    // Initialize TBUS component
    err = tbus_init(BSP_IO_TWAI_EN, BSP_IO_TWAI_TX, BSP_IO_TWAI_RX);
    // Return initialization state
    return err;
}

static int32_t bsp_board_cmd_read(bsp_cmd_t cmd) {
    switch (cmd) {
    case BSP_BOARD_SERIAL: {
        // Generate 15 bit serial number from MAC
        uint64_t chipmacid = 0LL;
        esp_efuse_mac_get_default((uint8_t*) (&chipmacid));
        return chipmacid & 0x7FFF;
        break;
    }
    case BSP_BOARD_REVISION: { return bsp_boardRevision; }
    case BSP_DRIVER_FIRMWARE: { return bsp_driverVersion; }
    case BSP_BUTTON_STATE: { return !gpio_get_level(BSP_IO_BUTTON); }
    case BSP_LED_STATE: { return gpio_get_level(BSP_IO_LED); }
    case BSP_TBUS_STATE: { return tbus_is_enabled(); }
    case BSP_5V_STATE: { return bsp_cmd_data[BSP_5V_STATE][0]; }
    case BSP_DC_STATE: { 
        if (bsp_boardRevision == 11) return 0; // Not supported
        return gpio_get_level(BSP_IO_DC_DETECT) == 0;
    }
    case BSP_USB_STATE: { return gpio_get_level(BSP_IO_USB_DETECT) == 0; }
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
    case BSP_BATTERY_STATE: {
        // Read battery charging state
        if (gpio_get_level(BSP_IO_BATTERY_CHARGE) == 1 
        && (getUptime() - bsp_battery_stateTime) > 1500) return 1;
        else return 0;
        break;
    }
    default: return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

static int bsp_board_cmd_write(bsp_cmd_t cmd, int8_t port, uint32_t value) {
    switch (cmd) {
    case BSP_LED_STATE: { gpio_set_level(BSP_IO_LED, value); break; }
    case BSP_TBUS_STATE: { tbus_enable(value); break; }
    case BSP_5V_STATE: { periph_driver_write(PERIPH_DRIVER_CTRL_POWER_5V, value); break; }
    default: return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_OK;
}

static void cmd_reset_value(bsp_cmd_t cmd, uint8_t port, uint32_t value) {
    if (bsp_cmd_data[cmd][port] != value) {
        bsp_cmd_data[cmd][port] = value;
        execute_bsp_callback(cmd, port, value, 0);
    }
}

static void dc_cmd_update(bsp_cmd_t cmd, uint8_t port, uint32_t value) {

    switch (cmd) {
    case BSP_DC_POWER: {
        uint32_t brake_power = (value == 0) ? bsp_cmd_data[BSP_DC_CONFIG_BRAKE][port] : 0;
        cmd_reset_value(BSP_DC_BRAKE, port, brake_power);
        cmd_reset_value(BSP_DC_TONE, port < 2 ? 0 : 2, 0);
        break;
    }
    case BSP_DC_BRAKE: {
        cmd_reset_value(BSP_DC_POWER, port, 0);
        cmd_reset_value(BSP_DC_TONE, port < 2 ? 0 : 2, 0);
        break;
    }
    case BSP_DC_TONE: {
        for (int i=port; i<=(port+1); i++) {
            // Reset related values
            cmd_reset_value(BSP_DC_POWER, i, 0);
            cmd_reset_value(BSP_DC_BRAKE, i, 0);
        }
        break;
    }
    }
}

int bsp_dc_cmd_write(bsp_cmd_t cmd, int8_t port, uint32_t value) {
    if (port < 0 || port > 3) { return ESP_ERR_INVALID_ARG; }

    PeriphRegMap reg;

    switch (cmd) {
    case BSP_DC_POWER: {
        // Write power to DC port
        reg = RegPort(PERIPH_DC_X_POWER, port);
        periph_driver_write(reg, value);
        // Reset related values
        dc_cmd_update(cmd, port, value);
        break;
    }
    case BSP_DC_BRAKE: {
        // Write brake to DC port
        reg = RegPort(PERIPH_DC_X_BRAKE, port);
        periph_driver_write(reg, value);
        // Reset related values
        dc_cmd_update(cmd, port, value);
        break;
    }
    case BSP_DC_TONE: {
        if (port == 1 || port == 3) return ESP_ERR_INVALID_ARG;
        if ((value & 0xFFFF) > 20000) return ESP_ERR_INVALID_SIZE;
        // Write tone to DC motor group
        reg = port < 2 ? PERIPH_DC_SET_AB_TONE : PERIPH_DC_SET_CD_TONE;
        periph_driver_write(reg, value);
        // Reset both channels
        dc_cmd_update(cmd, port, value);
        break;
    }
    case BSP_DC_CONFIG_INVERT: {
        // Write DC port pin swap
        reg = RegPort(PERIPH_DC_X_INVERT, port);
        periph_driver_write(reg, value);
        break;
    }
    case BSP_DC_CONFIG_FREQUENCY: {
        if (port == 1 || port == 3) break;
        // Write DC ports AB or CD PWM frequency
        reg = port < 2 ? PERIPH_DC_SET_AB_FREQUENCY : PERIPH_DC_SET_CD_FREQUENCY;
        periph_driver_write(reg, value);
        break;
    }
    case BSP_DC_CONFIG_BRAKE: {
        // Write DC port autobrake if power == 0
        reg = RegPort(PERIPH_DC_X_AUTOBRAKE, port);
        periph_driver_write(reg, value);
        // Reset brake value
        if (bsp_cmd_data[BSP_DC_POWER][port] == 0) {
            cmd_reset_value(BSP_DC_BRAKE, port, value);
        }
        break;
    }
    case BSP_DC_CONFIG_ENABLE: {
        // Write DC port enable
        reg = RegPort(PERIPH_DC_X_ENABLE, port);
        periph_driver_write(reg, value);
        break;
    }
    default: return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

int bsp_servo_cmd_write(bsp_cmd_t cmd, int8_t port, uint32_t value) {
    if (port < 0 || port > 2) { return ESP_ERR_INVALID_ARG; }
    
    PeriphRegMap reg;

    switch (cmd) {
    case BSP_SERVO_PULSE: {
        // Write servo port pulse (us)
        uint32_t pulseMin = bsp_cmd_data[BSP_SERVO_CONFIG_RANGE][port] >> 16;
        uint32_t pulseMax = bsp_cmd_data[BSP_SERVO_CONFIG_RANGE][port] & 0xFFFF;
        uint32_t duration = value >> 16;
        uint32_t pulse = value & 0xFFFF;
        if (pulse > pulseMax) return ESP_ERR_INVALID_SIZE;
        if (pulse < pulseMin) return ESP_ERR_INVALID_SIZE;
        // Swap pulse value if servo direction is inverted
        if (bsp_cmd_data[BSP_SERVO_CONFIG_INVERT][port]) {
            pulse = (pulseMin + pulseMax) - pulse;
        }
        reg = RegPort(PERIPH_SERVO_X_PULSE, port);
        periph_driver_write(reg, duration << 16 | pulse);
        break;
    }
    case BSP_SERVO_CONFIG_INVERT: {
        // In v1.52 internal driver invert doesn't work if writing with duration
        // Apply manual workaround
        bsp_cmd_data[BSP_SERVO_CONFIG_INVERT][port] = value ? 1 : 0;
        return bsp_servo_cmd_write(BSP_SERVO_PULSE, port, bsp_cmd_data[BSP_SERVO_PULSE][port]);
    }
    case BSP_SERVO_CONFIG_SPEED: {
        // Write servo port spin speed
        reg = RegPort(PERIPH_SERVO_X_SPEED, port);
        periph_driver_write(reg, value);
        break;
    }
    case BSP_SERVO_CONFIG_PERIOD: {
        // Write servo ports ABC PWM period (us)
        periph_driver_write(PERIPH_SERVO_SET_ABC_PERIOD, value);
        for (int i=0; i<3; i++) {
            // Reset range value
            uint32_t pulseMin = bsp_cmd_data[BSP_SERVO_CONFIG_RANGE][i] >> 16;
            uint32_t pulseMax = bsp_cmd_data[BSP_SERVO_CONFIG_RANGE][i] & 0xFFFF;
            if (pulseMin > value) pulseMin = value;
            if (pulseMax > value) pulseMax = value;
            cmd_reset_value(BSP_SERVO_CONFIG_RANGE, i, pulseMin << 16 | pulseMax);
            // Reset pulse value
            if (bsp_cmd_data[BSP_SERVO_PULSE][i] > value) {
                cmd_reset_value(BSP_SERVO_PULSE, i, value);
                periph_driver_write(RegPort(PERIPH_SERVO_X_PULSE, i), value);
            }
        }
        return ESP_OK;
    }
    case BSP_SERVO_CONFIG_RANGE: {
        // Write servo port min, max pulse range
        uint32_t pulseMin = value >> 16;
        uint32_t pulseMax = value & 0xFFFF;
        if (pulseMin > pulseMax) return ESP_ERR_INVALID_SIZE;
        if (pulseMax > bsp_cmd_data[BSP_SERVO_CONFIG_PERIOD][0]) return ESP_ERR_INVALID_SIZE;
        periph_driver_write(RegPort(PERIPH_SERVO_X_PULSE_MIN, port), pulseMin);
        periph_driver_write(RegPort(PERIPH_SERVO_X_PULSE_MAX, port), pulseMax);
        return ESP_OK;
    }
    case BSP_SERVO_CONFIG_ENABLE: {
        // Write servo port enable
        reg = RegPort(PERIPH_SERVO_X_ENABLE, port);
        periph_driver_write(reg, value);
        break;
    }
    default: return ESP_ERR_NOT_FOUND;
    };
    
    return ESP_OK;
}

#define RGB_ENABLED_ALL() (bsp_cmd_data[BSP_RGB_CONFIG_ENABLE][0] && bsp_cmd_data[BSP_RGB_CONFIG_ENABLE][1] && bsp_cmd_data[BSP_RGB_CONFIG_ENABLE][2] && bsp_cmd_data[BSP_RGB_CONFIG_ENABLE][3])

int bsp_rgb_cmd_write(bsp_cmd_t cmd, int8_t port, uint32_t value) {

    if (port == -1) {
        // Copy value to all channels
        for (int i=0; i<4; i++) { cmd_reset_value(cmd, i, value); }
    }

    switch (cmd) {
    case BSP_RGB_COLOR: {
        // Write RGB LED color
        if (port == -1 && RGB_ENABLED_ALL()) periph_driver_write(PERIPH_RGB_SET_ABCD_SET, value);
        else if (bsp_cmd_data[BSP_RGB_CONFIG_ENABLE][port]) periph_driver_write(RegPort(PERIPH_RGB_X_SET, port), value);
        break; 
    }
    case BSP_RGB_FADE_COLOR: {
        // Write RGB LED next fade color
        if (port == -1 && RGB_ENABLED_ALL()) periph_driver_write(PERIPH_RGB_SET_ABCD_FADE, value);
        else if (bsp_cmd_data[BSP_RGB_CONFIG_ENABLE][port]) periph_driver_write(RegPort(PERIPH_RGB_X_SET_FADE, port), value);
        break; 
    }
    case BSP_RGB_FADE_START: {
        // Write RGB LED fade duration & start
        if (port == -1 && RGB_ENABLED_ALL()) {
            periph_driver_write(PERIPH_RGB_SET_ABCD_START_FADE, value);
            for (int i=0; i<4; i++) { bsp_cmd_data[BSP_RGB_COLOR][i] = bsp_cmd_data[BSP_RGB_FADE_COLOR][i]; }
        }
        else if (bsp_cmd_data[BSP_RGB_CONFIG_ENABLE][port]) {
            periph_driver_write(RegPort(PERIPH_RGB_X_START_FADE, port), value);
            bsp_cmd_data[BSP_RGB_COLOR][port] = bsp_cmd_data[BSP_RGB_FADE_COLOR][port];
        }
        break;
    }
    case BSP_RGB_CONFIG_ENABLE: {
        // Write RGB LED enable
        if (port == -1) {
            for (int i=0; i<4; i++) {
                periph_driver_write(RegPort(PERIPH_RGB_X_SET, i), value ? bsp_cmd_data[BSP_RGB_COLOR][i] : 0);
            }
        }
        else {
            periph_driver_write(RegPort(PERIPH_RGB_X_SET, port), value ? bsp_cmd_data[BSP_RGB_COLOR][port] : 0);
        }
        break;
    }
    default: return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

static esp_err_t execute_write_cmd(int (*func)(bsp_cmd_t cmd, int8_t port, uint32_t value), bsp_cmd_t cmd, int8_t port, uint32_t value) {
    esp_err_t err = ESP_OK;
    // Validate if value has changed
    if (bsp_cmd_data[cmd][port] == value
        && cmd != BSP_DC_TONE)
        return ESP_OK;
    // Call command handler
    err = func(cmd, port, value);
    // Store latest value and call update handlers
    if (err == ESP_OK && port != -1) {
        if (cmd == BSP_DC_TONE) { value &= 0xFFFF; }
        if (cmd == BSP_SERVO_PULSE) { value &= 0xFFFF; }
        bsp_cmd_data[cmd][port] = value;
        execute_bsp_callback(cmd, port, value, 0);
    }
    return err;
}
esp_err_t bsp_cmd_write(bsp_cmd_t cmd, int8_t port, int32_t value) {
    esp_err_t err = ESP_OK;
    // Validate commands range
    if (cmd >= BSP_CMD_MAX) { return ESP_ERR_NOT_FOUND; }
    // Validate port range
    if (port < -1 || port > 3) { return ESP_ERR_INVALID_ARG; }
    // Validate value range
    int32_t limit = bsp_cmd_limit[cmd];
    if (limit == 0) { } // No limit
    else if (limit == -1) { value = value ? 1 : 0; } // Logic level
    else if (limit < 0) { // Limit with negative value
        if (value < limit || value > -limit) return ESP_ERR_INVALID_SIZE;
    }
    else if (value > limit) return ESP_ERR_INVALID_SIZE; // Positive limit
    else if (value < 0) return ESP_ERR_INVALID_SIZE; // Only positive allowed
    // Perform generic feature write
    if (cmd <= BSP_BATTERY_STATE) { // BOARD commands
        // Call board command handler
        err = execute_write_cmd(bsp_board_cmd_write, cmd, 0, value);
    }
    else if (cmd <= BSP_DC_CONFIG_ENABLE) { // DC commands
        // Validate special input cases
        if (cmd == BSP_DC_CONFIG_FREQUENCY && value == 0) return ESP_ERR_INVALID_SIZE;
        if (port != -1 && (cmd == BSP_DC_CONFIG_FREQUENCY || cmd == BSP_DC_TONE)) { port = port < 2 ? 0 : 2; }
        // Write command or loop if -1 passed
        int start, end;
        if (port == -1) { start = 0; end = 3; }
        else { start = port; end = port; }
        for (port=start; port<=end; port++) {
            // Only call A and C ports on these commands
            if (cmd == BSP_DC_CONFIG_FREQUENCY || cmd == BSP_DC_TONE) { if ((port % 2) != 0) continue; }
            // Call dc command handler
            err = execute_write_cmd(bsp_dc_cmd_write, cmd, port, value);
            if (err != ESP_OK) break;
        }
    }
    else if (cmd <= BSP_SERVO_CONFIG_ENABLE) { // SERVO commands
        // Validate special input cases
        if (cmd == BSP_SERVO_CONFIG_PERIOD) {
            if (value == 0) return ESP_ERR_INVALID_SIZE;
            port = 0;
        }
        // Write command or loop if -1 passed
        int start, end;
        if (port == -1) { start = 0; end = 2; }
        else { start = port; end = port; }
        for (port=start; port<=end; port++) {
            // Call servo command handler
            err = execute_write_cmd(bsp_servo_cmd_write, cmd, port, value);
            if (err != ESP_OK) break;
        }
    }
    else if (cmd <= BSP_RGB_CONFIG_ENABLE) { // RGB commands
        // Call RGB command handler
        err = execute_write_cmd(bsp_rgb_cmd_write, cmd, port, value);
    }
    return err;
}

int32_t bsp_cmd_read(bsp_cmd_t cmd, uint8_t port) {
    // Call board command handler
    if (cmd <= BSP_BATTERY_STATE) {
        return bsp_board_cmd_read(cmd);
    }
    // Validate command range
    if (cmd >= BSP_CMD_MAX) return 0;
    // Validate port range
    if (port > 3) return 0;
    // Validate special input cases
         if (cmd == BSP_SERVO_CONFIG_PERIOD) port = 0;
    else if (cmd == BSP_DC_CONFIG_FREQUENCY) port = port < 2 ? 0 : 2;
    else if (cmd == BSP_DC_TONE) port = port < 2 ? 0 : 2;
    // Return latest value
    return bsp_cmd_data[cmd][port];
}

void bsp_callback_register(bsp_cmd_change_func_t callback) {
    // Find empty slot and register new function
    for (int i=0; i<3; i++) {
        if (bsp_cmd_callback[i] == 0) {
            bsp_cmd_callback[i] = callback;
            break;
        }
    }
}

/**************************************************************************************************
 * IMU
 **************************************************************************************************/
esp_err_t bsp_imu_set_accel_range(uint16_t range) {
    if (bsp_boardRevision == 11) {
        return icm20689_set_accel(range);
    }
    else {
        return lsm6ds3_set_accel(range, LSM6DS3_ACCEL_RATE_104Hz, LSM6DS3_ACCEL_FILTER_100Hz);
    }
}
esp_err_t bsp_imu_set_gyro_range(uint16_t range) {
    if (bsp_boardRevision == 11) {
        return icm20689_set_gyro(range);
    }
    else {
        if (range == 250) range = 245;
        return lsm6ds3_set_gyro(range, LSM6DS3_GYRO_RATE_104Hz);
    }
}
esp_err_t bsp_imu_read(BspIMU_data_t *data) {
    // Validate argument
    if (data == NULL) return ESP_ERR_INVALID_ARG;
    esp_err_t err;
    if (bsp_boardRevision == 11) {
        err = icm20689_read((icm20689_data_t*)data);
        // Flip measurements relative to board perspective
        data->accel.x = -data->accel.x;
        data->accel.z = -data->accel.z;
        data->gyro.x = -data->gyro.x;
        data->gyro.z = -data->gyro.z;
    }
    else {
        err = lsm6ds3_read((lsm6ds3_data_t*)data);
        // Flip measurements relative to board perspective
        data->accel.y = -data->accel.y;
        data->accel.z = -data->accel.z;
        data->gyro.y = -data->gyro.y;
        data->gyro.z = -data->gyro.z;
    }
    return err;
}

/**************************************************************************************************
 * TBUS
 **************************************************************************************************/
void bsp_tbus_callback_register(bsp_tbus_receive_func_t receive_handler, void *ctx) {
    // Register receive handler
    tbus_callback_register(receive_handler, ctx);
}

int bsp_tbus_send(uint32_t id, uint8_t *data, uint8_t len) {
    // Send CAN packet
    return tbus_send(id, data, len);
}

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
