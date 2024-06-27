/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include "esp_idf_version.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/mcpwm_prelude.h"
#include "soc/mcpwm_struct.h"
#include "hal/mcpwm_ll.h"
#include "hal/misc.h"

#include "bsp/roboboard_x3.h"
#include "bsp_drivers.h"

int bsp_servo_get_port_cnt();
#define DC_CNT 4
#define SERVO_CNT bsp_servo_get_port_cnt()

static adc_oneshot_unit_handle_t adc1_handle;
static adc_cali_handle_t adc1_cali_handle;

void bsp_adc1_init(adc_channel_t channel) {
    if (!adc1_handle) {
        adc_oneshot_unit_init_cfg_t init_config1 = {
            .unit_id = ADC_UNIT_1,
        };
        adc_oneshot_new_unit(&init_config1, &adc1_handle);
    }
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,
    };
    adc_oneshot_config_channel(adc1_handle, channel, &config);

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_handle) == ESP_OK)
        return;
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_cali_create_scheme_line_fitting(&cali_config, &adc1_cali_handle) == ESP_OK)
        return;
#endif
}

uint32_t bsp_adc1_get_raw(adc_channel_t channel) {
    int adc = 0;
    adc_oneshot_read(adc1_handle, channel, &adc);
    return adc;
}

uint32_t bsp_adc1_raw_to_voltage(uint32_t adc) {
    int voltage = 0;
    adc_cali_raw_to_voltage(adc1_cali_handle, adc, &voltage);
    return voltage;
}
// Motor status
static struct {
    struct MotorDC_t {
        int group_id;
        int timer_id;
        mcpwm_timer_handle_t tim;
        mcpwm_cmpr_handle_t cmpA;
        mcpwm_cmpr_handle_t cmpB;
        mcpwm_gen_handle_t genA;
        mcpwm_gen_handle_t genB;
        mcpwm_generator_action_t actionA;
        mcpwm_generator_action_t actionB;
        uint32_t frequencySet;
        uint32_t frequency;
        uint32_t period;
        uint32_t decay;
        uint8_t enable;
        uint8_t mode;
        int8_t forceLevelA;
        int8_t forceLevelB;
        int32_t value;
    } dc[4];
    struct MotorServoGroup_t {
        mcpwm_timer_handle_t tim;
        uint32_t period;
        struct MotorServo_t {
            mcpwm_cmpr_handle_t cmp;
            mcpwm_gen_handle_t gen;
            uint32_t pulse;
        } servo[2];
    } servo_group[2];
} motors;

/*******************************
 * DC control functions
 ******************************/

// Private DC ports initialization
esp_err_t bsp_motor_init() {
    // List of motor control pins
    const uint32_t pwm_pins_dc[][2] = {
        {BSP_IO_MOTORA_INA, BSP_IO_MOTORA_INB},
        {BSP_IO_MOTORB_INA, BSP_IO_MOTORB_INB},
        {BSP_IO_MOTORC_INA, BSP_IO_MOTORC_INB},
        {BSP_IO_MOTORD_INA, BSP_IO_MOTORD_INB},
    };
    const uint32_t pwm_pins_servo[][2] = {
        {BSP_IO_SERVOA_IN, BSP_IO_SERVOB_IN},
        {BSP_IO_SERVOC_IN, BSP_IO_SERVOD_IN},
    };
    // Prepare configuration
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .period_ticks = 1000000 / 20000,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .flags.update_period_on_empty = 0,
        .flags.update_period_on_sync = 0,
    };
    mcpwm_operator_config_t operator_config = { .group_id = 0 };
    mcpwm_comparator_config_t comparator_config = { .flags.update_cmp_on_tez = true };
    mcpwm_generator_config_t generator_config = {};
    // Initialize MCPWM for DC
    for (int motorID=0; motorID<4; motorID++) {
        mcpwm_oper_handle_t operator;
        struct MotorDC_t *motor = &(motors.dc[motorID]);
        motor->period = timer_config.period_ticks;
        motor->frequency = motor->frequencySet = 20000;
        motor->forceLevelA = motor->forceLevelB = -1;
        motor->decay = MOTOR_DECAY_SLOW;
        motor->enable = 1;
        if (motorID == 3) { timer_config.group_id = 1; operator_config.group_id = 1; }
        motor->group_id = timer_config.group_id;
        motor->timer_id = motorID%3;
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &(motor->tim)));
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator, motor->tim));
        ESP_ERROR_CHECK(mcpwm_new_comparator(operator, &comparator_config, &(motor->cmpA)));
        ESP_ERROR_CHECK(mcpwm_new_comparator(operator, &comparator_config, &(motor->cmpB)));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor->cmpA, 0));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(motor->cmpB, 0));
        generator_config.gen_gpio_num = pwm_pins_dc[motorID][0];
        ESP_ERROR_CHECK(mcpwm_new_generator(operator, &generator_config, &(motor->genA)));
        generator_config.gen_gpio_num = pwm_pins_dc[motorID][1];
        ESP_ERROR_CHECK(mcpwm_new_generator(operator, &generator_config, &(motor->genB)));
        ESP_ERROR_CHECK(mcpwm_timer_enable(motor->tim));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(motor->tim, MCPWM_TIMER_START_NO_STOP));
    }
    // Prepare configuration
    timer_config.group_id = 1;
    timer_config.resolution_hz = 1000000;
    timer_config.period_ticks = 20000;
    operator_config.group_id = 1;
    // Initialize MCPWM for Servo
    for (int groupID=0; groupID<2; groupID++) {
        struct MotorServoGroup_t *group = &(motors.servo_group[groupID]);
        group->period = timer_config.period_ticks;
        mcpwm_oper_handle_t operator;
        ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &(group->tim)));
        ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operator));
        ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator, group->tim));
        for (int servoID=0; servoID<2; servoID++) {
            ESP_ERROR_CHECK(mcpwm_new_comparator(operator, &comparator_config, &(group->servo[servoID].cmp)));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(group->servo[servoID].cmp, 0));
            generator_config.gen_gpio_num = pwm_pins_servo[groupID][servoID];
            ESP_ERROR_CHECK(mcpwm_new_generator(operator, &generator_config, &(group->servo[servoID].gen)));
            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(group->servo[servoID].gen,
                            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
            ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(group->servo[servoID].gen,
                            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, group->servo[servoID].cmp, MCPWM_GEN_ACTION_LOW)));
        }
        ESP_ERROR_CHECK(mcpwm_timer_enable(group->tim));
        ESP_ERROR_CHECK(mcpwm_timer_start_stop(group->tim, MCPWM_TIMER_START_NO_STOP));
    }
    // Enable synchronization
    mcpwm_ll_timer_enable_sync_input(MCPWM_LL_GET_HW(0), 0, true);
    mcpwm_ll_timer_enable_sync_input(MCPWM_LL_GET_HW(0), 1, true);
    mcpwm_ll_timer_enable_sync_input(MCPWM_LL_GET_HW(0), 2, true);
    mcpwm_ll_timer_enable_sync_input(MCPWM_LL_GET_HW(1), 0, true);
    return ESP_OK;
}
// Motor actions change
static esp_err_t motor_set_actions(struct MotorDC_t *motor, mcpwm_generator_action_t actionA, mcpwm_generator_action_t actionB) {
    if (motor->actionA != actionA) {
        motor->actionA = actionA;
        BSP_ERR(mcpwm_generator_set_action_on_timer_event(motor->genA,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, actionA)));
        BSP_ERR(mcpwm_generator_set_action_on_compare_event(motor->genA,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->cmpA, actionA==MCPWM_GEN_ACTION_LOW?MCPWM_GEN_ACTION_HIGH:MCPWM_GEN_ACTION_LOW)));
    }
    if (motor->actionB != actionB) {
        motor->actionB = actionB;
        BSP_ERR(mcpwm_generator_set_action_on_timer_event(motor->genB,
                MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, actionB)));
        BSP_ERR(mcpwm_generator_set_action_on_compare_event(motor->genB,
                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, motor->cmpB, actionB==MCPWM_GEN_ACTION_LOW?MCPWM_GEN_ACTION_HIGH:MCPWM_GEN_ACTION_LOW)));
    }
    return ESP_OK;
}
// Motor force levels change
static esp_err_t motor_set_force_level(struct MotorDC_t *motor, int levelA, int levelB) {
    if (motor->forceLevelA != levelA) {
        motor->forceLevelA = levelA;
        if (motor->enable) {
            mcpwm_generator_set_force_level(motor->genA, levelA, true);
        }
    }
    if (motor->forceLevelB != levelB) {
        motor->forceLevelB = levelB;
        if (motor->enable) {
            mcpwm_generator_set_force_level(motor->genB, levelB, true);
        }
    }
    return ESP_OK;
}
// Motor duty cycle update
static esp_err_t motor_set_duty(struct MotorDC_t *motor, int duty) {
    if (duty < 0) duty = -duty;
    int pulse = duty * motor->period / 100;
    BSP_ERR(mcpwm_comparator_set_compare_value(motor->cmpA, pulse));
    BSP_ERR(mcpwm_comparator_set_compare_value(motor->cmpB, pulse));
    return ESP_OK;
}
// Change timer period
static esp_err_t motor_set_period(struct MotorDC_t *motor, uint32_t period) {
    if (period > 0xFFFF) period = 0xFFFF;
    if (motor->period == period) return ESP_OK;
    motor->period = period;
    return mcpwm_timer_set_period(motor->tim, period);
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
    switch (param) {
        case MOTOR_SERVO_PULSE: {
            if (value > group->period) return ESP_ERR_INVALID_ARG;
            motor->pulse = value;
            return mcpwm_comparator_set_compare_value(motor->cmp, value);
        }
        case MOTOR_SERVO_PERIOD: {
            if (value < 1 || value > 0xFFFF) return ESP_ERR_INVALID_ARG;
            group->period = value;
            return mcpwm_timer_set_period(group->tim, value);
        }
        case MOTOR_SERVO_ENABLE: {
            return mcpwm_generator_set_force_level(motor->gen, value ? -1 : 0, true);
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
            int inactiveState = motor->decay == MOTOR_DECAY_FAST ? value = -value, 0 : 1;
            int forceAction = motor->decay == MOTOR_DECAY_FAST ? MCPWM_GEN_ACTION_HIGH : MCPWM_GEN_ACTION_LOW;
            BSP_ERR(motor_set_force_level(motor, (value<0)?-1:inactiveState, (value<0)?inactiveState:-1));
            BSP_ERR(motor_set_actions(motor, forceAction, forceAction));
            BSP_ERR(motor_set_frequency(motor, motor->frequency));
            BSP_ERR(motor_set_duty(motor, value));
            return ESP_OK;
        }
        case MOTOR_DC_MODE_BRAKE: {
            if (value > 100) return ESP_ERR_INVALID_ARG;
            motor->mode = param;
            motor->value = value;
            BSP_ERR(motor_set_actions(motor, MCPWM_GEN_ACTION_HIGH, MCPWM_GEN_ACTION_HIGH));
            BSP_ERR(motor_set_force_level(motor, -1, -1));
            BSP_ERR(motor_set_frequency(motor, motor->frequency));
            BSP_ERR(motor_set_duty(motor, value));
            return ESP_OK;
        }
        case MOTOR_DC_MODE_TONE: {
            if (value > 20000) return ESP_ERR_INVALID_ARG;
            motor->mode = param;
            motor_set_actions(motor, MCPWM_GEN_ACTION_LOW, MCPWM_GEN_ACTION_HIGH);
            if (value == 0) { BSP_ERR(motor_set_force_level(motor, 0, 0)); }
            else {
                BSP_ERR(motor_set_force_level(motor, -1, -1));
                BSP_ERR(motor_set_frequency(motor, value));
                BSP_ERR(motor_set_duty(motor, 50));
            }
            return ESP_OK;
        }
        case MOTOR_DC_ENABLE: {
            motor->enable = value;
            mcpwm_generator_set_force_level(motor->genA, value ? motor->forceLevelA : 0, true);
            mcpwm_generator_set_force_level(motor->genB, value ? motor->forceLevelB : 0, true);
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
                BSP_ERR(bsp_motor_set_dc(motor, param, value));
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
            mcpwm_ll_timer_trigger_soft_sync(MCPWM_LL_GET_HW(motor->group_id), motor->timer_id);
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
