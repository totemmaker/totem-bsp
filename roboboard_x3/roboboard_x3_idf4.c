
/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "esp_idf_version.h"
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_struct.h"
#include "hal/mcpwm_ll.h"
#include "hal/misc.h"

#include "bsp/roboboard_x3.h"
#include "bsp_drivers.h"

#define DC_CNT 4
#define SERVO_CNT bsp_servo_get_port_cnt()

static esp_adc_cal_characteristics_t adc1_chars;

void bsp_adc1_init(adc_channel_t channel) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten((adc1_channel_t)channel, ADC_ATTEN_DB_12);
    if (adc1_chars.adc_num == 0)
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &(adc1_chars));
}

uint32_t bsp_adc1_get_raw(adc_channel_t channel) {
    return adc1_get_raw((adc1_channel_t)channel);
}

uint32_t bsp_adc1_raw_to_voltage(uint32_t adc) {
    return esp_adc_cal_raw_to_voltage(adc, &(adc1_chars));
}

// Motor status
static struct {
    struct MotorDC_t {
        const mcpwm_unit_t unit;
        const mcpwm_timer_t tim;
        mcpwm_duty_type_t typeA;
        mcpwm_duty_type_t typeB;
        uint32_t frequencySet;
        uint32_t frequency;
        uint32_t period;
        uint32_t decay;
        uint8_t enable;
        uint8_t mode;
        int32_t value;
        int32_t duty;
    } dc[4];
    struct MotorServoGroup_t {
        const mcpwm_timer_t tim;
        uint32_t period;
        struct MotorServo_t {
            const mcpwm_generator_t gen;
            uint32_t pulse;
            // uint8_t enable;
        } servo[2];
    } servo_group[2];
} motors = {
    {
        {MCPWM_UNIT_0, MCPWM_TIMER_0, 0, 0, 20000, 20000, 50, MOTOR_DECAY_SLOW, 1, 0, 0, 0},
        {MCPWM_UNIT_0, MCPWM_TIMER_1, 0, 0, 20000, 20000, 50, MOTOR_DECAY_SLOW, 1, 0, 0, 0},
        {MCPWM_UNIT_0, MCPWM_TIMER_2, 0, 0, 20000, 20000, 50, MOTOR_DECAY_SLOW, 1, 0, 0, 0},
        {MCPWM_UNIT_1, MCPWM_TIMER_0, 0, 0, 20000, 20000, 50, MOTOR_DECAY_SLOW, 1, 0, 0, 0},
    },
    {
        {MCPWM_TIMER_1, 20000, {{MCPWM_GEN_A, 0}, {MCPWM_GEN_B, 0}}},
        {MCPWM_TIMER_2, 20000, {{MCPWM_GEN_A, 0}, {MCPWM_GEN_B, 0}}},
    }
};;

/*******************************
 * DC control functions
 ******************************/

// Private DC ports initialization
esp_err_t bsp_motor_init() {
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
    // Configure timers
    for (int i=0; i<DC_CNT; i++) {
        mcpwm_ll_timer_enable_sync_input(MCPWM_LL_GET_HW(motors.dc[i].unit), motors.dc[i].tim, true);
        mcpwm_ll_timer_update_period_at_once(MCPWM_LL_GET_HW(motors.dc[i].unit), motors.dc[i].tim);
        mcpwm_ll_operator_update_compare_at_once(MCPWM_LL_GET_HW(motors.dc[i].unit), motors.dc[i].tim, MCPWM_GEN_A);
        mcpwm_ll_operator_update_compare_at_once(MCPWM_LL_GET_HW(motors.dc[i].unit), motors.dc[i].tim, MCPWM_GEN_B);
    }
    return ESP_OK;
}
// Motor duty cycle update
static esp_err_t motor_set_duty(struct MotorDC_t *motor, int gen, int duty) {
    if (duty < 0) duty = -duty;
    int pulse = duty * motor->period / 100;
    mcpwm_ll_operator_set_compare_value(MCPWM_LL_GET_HW(motor->unit), motor->tim, gen, pulse);
    return ESP_OK;
}
// Motor duty change
static esp_err_t motor_set_duty_type(struct MotorDC_t *motor, mcpwm_duty_type_t typeA, mcpwm_duty_type_t typeB) {
    if (motor->typeA != typeA) {
        motor->typeA = typeA;
        if (motor->enable) BSP_ERR(mcpwm_set_duty_type(motor->unit, motor->tim, MCPWM_GEN_A, typeA));
    }
    if (motor->typeB != typeB) {
        motor->typeB = typeB;
        if (motor->enable) BSP_ERR(mcpwm_set_duty_type(motor->unit, motor->tim, MCPWM_GEN_B, typeB));
    }
    return ESP_OK;
}
// Change timer period
static esp_err_t motor_set_period(struct MotorDC_t *motor, uint32_t period) {
    if (period > 0xFFFF) period = 0xFFFF;
    if (motor->period == period) return ESP_OK;
    motor->period = period;
    mcpwm_ll_timer_set_peak(MCPWM_LL_GET_HW(motor->unit), motor->tim, period, false);
    return ESP_OK;
}
// Change timer frequency
static esp_err_t motor_set_frequency(struct MotorDC_t *motor, uint32_t frequency) {
    if (motor->frequencySet == frequency) return ESP_OK;
    motor->frequencySet = frequency;
    if (frequency < 15) frequency = 15;
    return motor_set_period(motor, 1000000 / frequency);
}
// Process servo parameter
static esp_err_t bsp_motor_set_servo(uint8_t portID, uint32_t param, int32_t value) {
    struct MotorServoGroup_t *group = &(motors.servo_group[portID/2]);
    struct MotorServo_t *motor = &(group->servo[portID%2]);
    // Handle Servo
    switch (param) {
        case MOTOR_SERVO_PULSE: {
            if (value > group->period) return ESP_ERR_INVALID_ARG;
            motor->pulse = value;
            return mcpwm_set_duty_in_us(MCPWM_UNIT_1, group->tim, motor->gen, value);
        }
        case MOTOR_SERVO_PERIOD: {
            if (value < 1 || value > 0xFFFF) return ESP_ERR_INVALID_ARG;
            group->period = value;
            return mcpwm_set_frequency(MCPWM_UNIT_1, motors.servo_group[1].tim, 1000000/value);
        }
        case MOTOR_SERVO_ENABLE: {
            return mcpwm_set_duty_type(MCPWM_UNIT_1, group->tim, motor->gen,
            value ? MCPWM_DUTY_MODE_0 : MCPWM_HAL_GENERATOR_MODE_FORCE_LOW);
        }
        default: return ESP_ERR_INVALID_ARG;
    }
}

// Process DC parameter
static esp_err_t bsp_motor_set_dc(struct MotorDC_t *motor, uint32_t param, int32_t value) {
    // Setting
    if (param == MOTOR_DC_FREQUENCY) {
        if (value < 1 || value > 250000) return ESP_ERR_INVALID_ARG;
        motor->frequency = value;
        if (motor->mode == MOTOR_DC_MODE_TONE) return ESP_OK;
        param = motor->mode;
        value = motor->value;
    }
    else if (param == MOTOR_DC_DECAY) {
        if (value != 0 && value != 1) return ESP_ERR_INVALID_ARG;
        motor->decay = value;
        if (motor->mode != MOTOR_DC_MODE_POWER) return ESP_OK;
        param = motor->mode;
        value = motor->value;
    }
    // Parameter
    switch (param) {
        case MOTOR_DC_MODE_POWER: {
            if (value < -100 || value > 100) return ESP_ERR_INVALID_ARG;
            motor->mode = param;
            motor->value = value;
            int A = 0, B = 0; // Select pin state depending on decay mode
            if (motor->decay == MOTOR_DECAY_SLOW) {
                if (value) A = (value < 0) ? 100+value : 100;
                if (value) B = (value < 0) ? 100 : 100-value;
            }
            else if (motor->decay == MOTOR_DECAY_FAST) {
                A = (value < 0) ? 0 : value;
                B = (value < 0) ? -value : 0;
            }
            BSP_ERR(motor_set_duty_type(motor, MCPWM_DUTY_MODE_0, MCPWM_DUTY_MODE_0));
            BSP_ERR(motor_set_frequency(motor, motor->frequency));
            BSP_ERR(motor_set_duty(motor, MCPWM_GEN_A, A));
            BSP_ERR(motor_set_duty(motor, MCPWM_GEN_B, B));
            return ESP_OK;
        }
        case MOTOR_DC_MODE_BRAKE: {
            if (value > 100) return ESP_ERR_INVALID_ARG;
            motor->mode = param;
            motor->value = value;
            BSP_ERR(motor_set_duty_type(motor, MCPWM_DUTY_MODE_0, MCPWM_DUTY_MODE_0));
            BSP_ERR(motor_set_frequency(motor, motor->frequency));
            BSP_ERR(motor_set_duty(motor, MCPWM_GEN_A, value));
            BSP_ERR(motor_set_duty(motor, MCPWM_GEN_B, value));
            return ESP_OK;
        }
        case MOTOR_DC_MODE_TONE: {
            if (value > 20000) return ESP_ERR_INVALID_ARG;
            motor->mode = param;
            if (value == 0) {
                BSP_ERR(motor_set_duty_type(motor, MCPWM_HAL_GENERATOR_MODE_FORCE_LOW, MCPWM_HAL_GENERATOR_MODE_FORCE_LOW));
            }
            else {
                BSP_ERR(motor_set_duty_type(motor, MCPWM_DUTY_MODE_0, MCPWM_DUTY_MODE_1));
                BSP_ERR(motor_set_frequency(motor, value));
                BSP_ERR(motor_set_duty(motor, MCPWM_GEN_A, 50));
                BSP_ERR(motor_set_duty(motor, MCPWM_GEN_B, 50));
            }
            return ESP_OK;
        }
        case MOTOR_DC_ENABLE: {
            if (value) {
                BSP_ERR(mcpwm_set_duty_type(motor->unit, motor->tim, MCPWM_GEN_A, motor->typeA));
                BSP_ERR(mcpwm_set_duty_type(motor->unit, motor->tim, MCPWM_GEN_B, motor->typeB));
            }
            else {
                BSP_ERR(motor_set_duty_type(motor, MCPWM_HAL_GENERATOR_MODE_FORCE_LOW, MCPWM_HAL_GENERATOR_MODE_FORCE_LOW));
            }
            motor->enable = value;
            return ESP_OK;
        }
        default: return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}
// Set motor parameter
esp_err_t bsp_motor_set(int8_t portID, uint32_t param, int32_t value) {
    if (param & 0x10) {
        // Process servo parameter
        if (portID < -1 || portID >= SERVO_CNT) return ESP_ERR_NOT_FOUND;
        if (portID == -1) {
            // Process all ports
            for (int i=0; i<SERVO_CNT; i++) {
                BSP_ERR(bsp_motor_set_servo(i, param, value));
            }
            return ESP_OK;
        }
        // Process single port
        return bsp_motor_set_servo(portID, param, value);
    }
    else {
        // Process dc parameter
        if (portID < -1 || portID >= DC_CNT) return ESP_ERR_NOT_FOUND;
        if (portID == -1) {
            // Process all ports
            uint8_t sync = 0;
            for (int i=0; i<DC_CNT; i++) {
                struct MotorDC_t *motor = &(motors.dc[i]);
                uint32_t frequency = motor->frequencySet;
                BSP_ERR(bsp_motor_set_dc(motor, param, value))
                if (frequency != motor->frequencySet) { sync = 1; }
            }
            if (sync) {
                taskDISABLE_INTERRUPTS();
                mcpwm_ll_timer_trigger_soft_sync(MCPWM_LL_GET_HW(0), 0);
                mcpwm_ll_timer_trigger_soft_sync(MCPWM_LL_GET_HW(0), 1);
                mcpwm_ll_timer_trigger_soft_sync(MCPWM_LL_GET_HW(0), 2);
                mcpwm_ll_timer_trigger_soft_sync(MCPWM_LL_GET_HW(1), 0);
                taskENABLE_INTERRUPTS();
            }
            return ESP_OK;
        }
        // Process single port
        struct MotorDC_t *motor = &(motors.dc[portID]);
        uint32_t frequency = motor->frequencySet;
        BSP_ERR(bsp_motor_set_dc(motor, param, value));
        if (frequency != motor->frequencySet) {
            mcpwm_ll_timer_trigger_soft_sync(MCPWM_LL_GET_HW(motor->unit), motor->tim);
        }
    }
    return ESP_OK;
}
// Get motor parameter
int32_t bsp_motor_get(uint8_t portID, uint32_t param) {
    if (portID >= (param&0x10?SERVO_CNT:DC_CNT)) return 0;
    switch (param) {
    case MOTOR_DC_FREQUENCY: { return motors.dc[portID].frequency; }
    case MOTOR_DC_DECAY: { return motors.dc[portID].decay; }
    case MOTOR_SERVO_PULSE: { return motors.servo_group[portID/2].servo[portID%2].pulse; }
    case MOTOR_SERVO_PERIOD: { return motors.servo_group[portID/2].period; }
    }
    return 0;
}

#endif
