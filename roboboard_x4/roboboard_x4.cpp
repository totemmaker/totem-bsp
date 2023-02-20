/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "esp_system.h"
#include "esp_timer.h"
#include "esp_event.h"

#include "bsp/roboboard_x4.h"
#include "bsp/roboboard_x4.hpp"

#define TOTEM_COLOR_1 0xff18c712
#define TOTEM_COLOR_2 0xdcffff00
#define TOTEM_COLOR_3 0xdcffff00
#define TOTEM_COLOR_4 0xff1254c7

#define LIMIT_H(value, max) if ((value) > (max)) { (value) = (max); }
#define LIMIT_LH(value, min, max)  if ((value) > (max)) { (value) = (max); } else if ((value) < (min)) { (value) = (min); }

static uint32_t IRAM_ATTR getUptime() {
    return (uint32_t) (esp_timer_get_time() / 1000ULL);
}
// Create instance
RoboBoardX4 X4;

/*******************************
          X4.button
*******************************/
enum {
    BtnEvtActive           = (1<<0),
    BtnEvtPress            = (1<<1),
    BtnEvtRelease          = (1<<2),
    BtnEvtDoubleClick      = (1<<3),
    BtnEvtLongPress        = (1<<4),
    BtnEvtPress_read       = (1<<5),
    BtnEvtRelease_read     = (1<<6),
    BtnEvtPressFor_read    = (1<<7),
    BtnEvtReleaseFor_read  = (1<<8),
    BtnEvtDoubleClick_read = (1<<9),
    BtnEvtLongPress_read   = (1<<10),
};
static struct ButtonState {
    void (*evtFunc)();
    uint32_t wasPressTime;
    uint32_t wasReleaseTime;
    uint32_t lastTimestamp;
    uint32_t event;
} Button_st;
static void Button_state_change(uint8_t state) {
    // Get new time
    uint32_t newTime = getUptime();
    uint32_t prevStateDuration = newTime-Button_st.lastTimestamp;
    Button_st.lastTimestamp = newTime;
    if (state) { // Press event
        // Set events
        Button_st.event |= (BtnEvtPress | BtnEvtActive);
        // Reset events
        Button_st.event &= ~(BtnEvtLongPress_read | BtnEvtPressFor_read);
        // Update time
        Button_st.wasPressTime = 0;
        Button_st.wasReleaseTime = prevStateDuration;
        // Detect double click
        if (Button_st.wasReleaseTime <= 250) {
            if (!(Button_st.event & BtnEvtDoubleClick_read)) {
                Button_st.event |= (BtnEvtDoubleClick | BtnEvtDoubleClick_read);
            }
        }
        else {
            Button_st.event &= ~(BtnEvtDoubleClick_read);
        }
    }
    else { // Release event
        // Set event
        Button_st.event |= (BtnEvtRelease);
        // Reset events
        Button_st.event &= ~(BtnEvtReleaseFor_read | BtnEvtActive);
        // Update time
        Button_st.wasReleaseTime = 0;
        Button_st.wasPressTime = prevStateDuration;
        // Detect long press
        if (prevStateDuration >= 500 && !(Button_st.event & BtnEvtLongPress_read)) {
            Button_st.event |= (BtnEvtLongPress);
        }
    }
    // Call application event
    if (Button_st.evtFunc) Button_st.evtFunc();
}

bool Feature::Button::isPressed() {
    return (Button_st.event & BtnEvtActive);
}
bool Feature::Button::isReleased() {
    return !this->isPressed();
}
bool Feature::Button::isPressedFor(uint32_t ms) {
    return this->isPressed() && (getUptime()-Button_st.lastTimestamp) > ms;
}
bool Feature::Button::isReleasedFor(uint32_t ms) {
    return this->isReleased() && (getUptime()-Button_st.lastTimestamp) > ms;
}
bool Feature::Button::isLongPress() {
    return this->isPressedFor(500);
}
bool Feature::Button::wasPressed() {
    bool ret = !!(Button_st.event & BtnEvtPress);
    if (ret) Button_st.event &= ~(BtnEvtPress);
    return ret;
}
bool Feature::Button::wasReleased() {
    bool ret = !!(Button_st.event & BtnEvtRelease);
    if (ret) Button_st.event &= ~(BtnEvtRelease);
    return ret;
}
bool Feature::Button::wasPressedFor(uint32_t ms) {
    if (Button_st.event & BtnEvtPressFor_read) return false;
    if (this->isPressedFor(ms)) {
        Button_st.event |= BtnEvtPressFor_read;
        return true;
    }
    if (Button_st.wasPressTime > ms) {
        Button_st.event |= BtnEvtPressFor_read;
        return true;
    }
    return false;
}
bool Feature::Button::wasReleasedFor(uint32_t ms) {
    if (Button_st.event & BtnEvtReleaseFor_read) return false;
    if (this->isReleasedFor(ms)) {
        Button_st.event |= BtnEvtReleaseFor_read;
        return true;
    }
    if (Button_st.wasReleaseTime > ms) {
        Button_st.event |= BtnEvtReleaseFor_read;
        return true;
    }
    return false;
}
bool Feature::Button::wasLongPress() {
    if (Button_st.event & BtnEvtLongPress_read) return false;
    if (this->isLongPress()) {
        Button_st.event &= ~(BtnEvtLongPress);
        Button_st.event |= BtnEvtLongPress_read;
        return true;
    }
    if (Button_st.event & BtnEvtLongPress) {
        Button_st.event &= ~(BtnEvtLongPress);
        Button_st.event |= BtnEvtLongPress_read;
        return true;
    }
    return false;
}
bool Feature::Button::wasDoubleClick() {
    bool ret = !!(Button_st.event & BtnEvtDoubleClick);
    if (ret) Button_st.event &= ~(BtnEvtDoubleClick);
    return ret;
}
bool Feature::Button::waitPress(uint32_t timeout) {
    uint32_t endTime = ~0;
    if (timeout)
        endTime = timeout + getUptime();
    while (endTime > getUptime()) {
        if (wasPressed()) return true;
        vTaskDelay(1);
    }
    return false;
}
uint8_t Feature::Button::getState() {
    return !gpio_get_level(BSP_IO_BUTTON);
}
uint32_t Feature::Button::lastChange() {
    return Button_st.lastTimestamp;
}
void Feature::Button::addEvent(void (*buttonEvt)()) {
    Button_st.evtFunc = buttonEvt;
}
/*******************************
          X4.led
*******************************/
static struct LedState {
    TimerHandle_t timerHandle;
    StaticTimer_t timerBuffer;
    uint32_t toggleCount;
    uint8_t state;
} Led_st;
// Called on timeout
static void Led_timer_callback(TimerHandle_t xTimer) {
    // Ignore if the pxTimer parameter is NULL
    if (!xTimer) return;
    // Toggle LED
    gpio_set_level(BSP_IO_LED, !gpio_get_level(BSP_IO_LED));
    // If the timer has expired some amount of times then stop it from running.
    if (--Led_st.toggleCount <= 0) {
        Led_st.toggleCount = 0;
        xTimerStop(xTimer, 0);
        gpio_set_level(BSP_IO_LED, Led_st.state);
    }
}
static void Led_start_blinker(uint32_t toggleCount, uint32_t rate) {
    // Validate parameters
    if (toggleCount == 0 || rate == 0) return;
    // Create / reload timer
    if (Led_st.timerHandle == nullptr)
        Led_st.timerHandle = xTimerCreateStatic("LED_blink", pdMS_TO_TICKS(rate), pdTRUE, 0, Led_timer_callback, &Led_st.timerBuffer);
    else
        xTimerChangePeriod(Led_st.timerHandle, pdMS_TO_TICKS(rate), 0);
    // Start timer
    if (xTimerStart(Led_st.timerHandle, 0) == pdTRUE)
        Led_st.toggleCount = toggleCount;
}
void Feature::Led::on() { this->set(1); }
void Feature::Led::off() { this->set(0); }
void Feature::Led::toggle() { this->set(!isOn()); }
void Feature::Led::set(uint8_t state) {
    Led_st.state = state;
    if (Led_st.toggleCount == 0) {
        gpio_set_level(BSP_IO_LED, state);
    }
}
int Feature::Led::isOn() {
    return (Led_st.toggleCount == 0) ? gpio_get_level(BSP_IO_LED) : 1;
}
void Feature::Led::blink() { this->blinkFor(500, 100); }
void Feature::Led::blinkTimes(uint32_t count, uint32_t blinkDuration) {
    if (Led_st.toggleCount == 0) {
        Led_start_blinker(count * 2, blinkDuration / 2);
    }
}
void Feature::Led::blinkFor(uint32_t durationMs, uint32_t blinkDuration) {
    if (blinkDuration == 0) return;
    if (Led_st.toggleCount == 0) {
        Led_start_blinker((durationMs / blinkDuration) * 2, blinkDuration / 2);
    }
}
/*******************************
          X4.imu
*******************************/
// Accelerometer. Measures acceleration
float Feature::IMU::Data::getX_G() {
    return this->accel.x;
}
float Feature::IMU::Data::getY_G() {
    return this->accel.y;
}
float Feature::IMU::Data::getZ_G() {
    return this->accel.z;
}
float Feature::IMU::Data::getX_mss() {
    return this->accel.x * 9.80665;
}
float Feature::IMU::Data::getY_mss() {
    return this->accel.y * 9.80665;
}
float Feature::IMU::Data::getZ_mss() {
    return this->accel.z * 9.80665;
}
// Gyroscope. Measures rotation speed
float Feature::IMU::Data::getX_dps() {
    return this->gyro.x;
}
float Feature::IMU::Data::getY_dps() {
    return this->gyro.y;
}
float Feature::IMU::Data::getZ_dps() {
    return this->gyro.z;
}
float Feature::IMU::Data::getX_rads() {
    return this->getX_dps() * 0.017453;
}
float Feature::IMU::Data::getY_rads() {
    return this->getY_dps() * 0.017453;
}
float Feature::IMU::Data::getZ_rads() {
    return this->getZ_dps() * 0.017453;
}
float Feature::IMU::Data::getX_rpm() {
    return this->getX_dps() / 6;
}
float Feature::IMU::Data::getY_rpm() {
    return this->getY_dps() / 6;
}
float Feature::IMU::Data::getZ_rpm() {
    return this->getZ_dps() / 6;
}

float Feature::IMU::Data::getTempC() {
    return this->temp;
}
float Feature::IMU::Data::getTempF() {
    return (this->temp * 9 / 5) + 32;
}

float Feature::IMU::Data::getOrientX() {
    return -atan2(this->accel.y , this->accel.z) * 180 / M_PI;
}
float Feature::IMU::Data::getOrientY() {
    return this->getRoll();
}

float Feature::IMU::Data::getRoll() {
    return -atan2(this->accel.x , this->accel.z) * 180 / M_PI;
}
float Feature::IMU::Data::getPitch() {
    return -atan2(-this->accel.y, sqrt((this->accel.x * this->accel.x) + (this->accel.z * this->accel.z))) * 180 / M_PI;
}

Feature::IMU::Data Feature::IMU::read() {
    Feature::IMU::Data data;
    bsp_imu_read((BspIMU_data_t*)&data);
    return data;
}

void Feature::IMU::setAccelRange(uint16_t range) {
    bsp_imu_set_accel_range(range);
}
void Feature::IMU::setGyroRange(uint16_t range) {
    bsp_imu_set_gyro_range(range);
}

const char* Feature::IMU::getName() {
    int revision = X4.getRevisionNum();
    switch (revision) {
        case 10: return "LSM6DS3";
        case 11: return "ICM-20689";
    }
    return "Unknown";
}
uint8_t Feature::IMU::getI2CAddr() {
    int revision = X4.getRevisionNum();
    switch (revision) {
        case 10: return 0x6A;
        case 11: return 0x68;
    }
    return 0;
}
/*******************************
          X4.dcABCD
*******************************/
void Feature::DCABCD::enable() {
    bsp_cmd_write(BSP_DC_CONFIG_ENABLE, BSP_PORT_ALL, 1);
}
void Feature::DCABCD::disable() {
    bsp_cmd_write(BSP_DC_CONFIG_ENABLE, BSP_PORT_ALL, 0);
}
void Feature::DCABCD::power(int8_t powerA, int8_t powerB, int8_t powerC, int8_t powerD) {
    LIMIT_LH(powerA, -100, 100);
    LIMIT_LH(powerB, -100, 100);
    LIMIT_LH(powerC, -100, 100);
    LIMIT_LH(powerD, -100, 100);
    bsp_cmd_write(BSP_DC_POWER, 0, powerA);
    bsp_cmd_write(BSP_DC_POWER, 1, powerB);
    bsp_cmd_write(BSP_DC_POWER, 2, powerC);
    bsp_cmd_write(BSP_DC_POWER, 3, powerD);
    // bsp_cmd_write(BSP_DC_POWER_ABCD, 0, (powerA&0xFF) << 24 | (powerB&0xFF) << 16 | (powerC&0xFF) << 8 | (powerD&0xFF));
}
void Feature::DCABCD::brake(int8_t powerA, int8_t powerB, int8_t powerC, int8_t powerD) {
    LIMIT_LH(powerA, 0, 100);
    LIMIT_LH(powerB, 0, 100);
    LIMIT_LH(powerC, 0, 100);
    LIMIT_LH(powerD, 0, 100);
    bsp_cmd_write(BSP_DC_BRAKE, 0, powerA);
    bsp_cmd_write(BSP_DC_BRAKE, 1, powerB);
    bsp_cmd_write(BSP_DC_BRAKE, 2, powerC);
    bsp_cmd_write(BSP_DC_BRAKE, 3, powerD);
    // bsp_cmd_write(BSP_DC_BRAKE_ABCD, 0, powerA << 24 | powerB << 16 | powerC << 8 | powerD);
}
void Feature::DCABCD::setInvert(bool invertA, bool invertB, bool invertC, bool invertD) {
    bsp_cmd_write(BSP_DC_CONFIG_INVERT, 0, invertA);
    bsp_cmd_write(BSP_DC_CONFIG_INVERT, 1, invertB);
    bsp_cmd_write(BSP_DC_CONFIG_INVERT, 2, invertC);
    bsp_cmd_write(BSP_DC_CONFIG_INVERT, 3, invertD);
}
void Feature::DCABCD::setAutobrake(int powerA, int powerB, int powerC, int powerD) {
    LIMIT_LH(powerA, 0, 100);
    LIMIT_LH(powerB, 0, 100);
    LIMIT_LH(powerC, 0, 100);
    LIMIT_LH(powerD, 0, 100);
    bsp_cmd_write(BSP_DC_CONFIG_BRAKE, 0, powerA);
    bsp_cmd_write(BSP_DC_CONFIG_BRAKE, 1, powerB);
    bsp_cmd_write(BSP_DC_CONFIG_BRAKE, 2, powerC);
    bsp_cmd_write(BSP_DC_CONFIG_BRAKE, 3, powerD);
}
void Feature::DCABCD::setAutobrake(bool stateA, bool stateB, bool stateC, bool stateD) {
    this->setAutobrake(stateA ? 100 : 0, stateB ? 100 : 0, stateC ? 100 : 0, stateD ? 100 : 0);
}
Feature::DCX& Feature::DCABCD::operator[](uint8_t port) {
    switch (port) {
        case 0: return X4.dcA;
        case 1: return X4.dcB;
        case 2: return X4.dcC;
        default: return X4.dcD;
    }
}
/*******************************
          X4.dcXX
*******************************/
void Feature::DCXX::tone(uint16_t frequency, uint16_t duration) {
    LIMIT_H(frequency, 20000);
    bsp_cmd_write(BSP_DC_TONE, port, (duration << 16) | frequency);
}
void Feature::DCXX::setModeIndividual() {
    // Not supported
}
void Feature::DCXX::setModeCombined() {
    // Not supported
}
void Feature::DCXX::setFrequency(uint32_t frequency) {
    LIMIT_LH(frequency, 1, 250000);
    bsp_cmd_write(BSP_DC_CONFIG_FREQUENCY, port, frequency);
}
/*******************************
          X4.dcX
*******************************/
void Feature::DCX::enable() {
    bsp_cmd_write(BSP_DC_CONFIG_ENABLE, port, 1);
}
void Feature::DCX::disable() {
    bsp_cmd_write(BSP_DC_CONFIG_ENABLE, port, 0);
}
void Feature::DCX::power(int8_t power) {
    LIMIT_LH(power, -100, 100);
    bsp_cmd_write(BSP_DC_POWER, port, power);
}
void Feature::DCX::brake(int8_t power) {
    LIMIT_LH(power, 0, 100);
    bsp_cmd_write(BSP_DC_BRAKE, port, power);
}
void Feature::DCX::coast() {
    this->brake(0);
}
void Feature::DCX::setInvert(bool state) {
    bsp_cmd_write(BSP_DC_CONFIG_INVERT, port, state);
}
void Feature::DCX::setAutobrake(int power) {
    LIMIT_LH(power, 0, 100);
    bsp_cmd_write(BSP_DC_CONFIG_BRAKE, port, power);
}
void Feature::DCX::setAutobrake(bool state) {
    this->setAutobrake(state ? 100 : 0);
}
/*******************************
          X4.servoABC
*******************************/
void Feature::SERVOABC::enable() {
    bsp_cmd_write(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_ALL, 1);
}
void Feature::SERVOABC::disable() {
    bsp_cmd_write(BSP_SERVO_CONFIG_ENABLE, BSP_PORT_ALL, 0);
}
void Feature::SERVOABC::setPeriod(uint32_t period) {
    LIMIT_LH(period, 1, 0xFFFF);
    bsp_cmd_write(BSP_SERVO_CONFIG_PERIOD, BSP_PORT_ALL, period);
}
void Feature::SERVOABC::pos(int8_t positionA, int8_t positionB, int8_t positionC) {
    X4.servoA.pos(positionA);
    X4.servoB.pos(positionB);
    X4.servoC.pos(positionC);
}
void Feature::SERVOABC::angle(uint8_t angleA, uint8_t angleB, uint8_t angleC) {
    X4.servoA.angle(angleA);
    X4.servoB.angle(angleB);
    X4.servoC.angle(angleC);
}
void Feature::SERVOABC::pulse(uint16_t pulseA, uint16_t pulseB, uint16_t pulseC) {
    X4.servoA.pulse(pulseA);
    X4.servoB.pulse(pulseB);
    X4.servoC.pulse(pulseC);
}
Feature::SERVOX& Feature::SERVOABC::operator[](uint8_t port) {
    switch (port) {
        case 0: return X4.servoA;
        case 1: return X4.servoB;
        default: return X4.servoC;
    }
}
/*******************************
          X4.servoX
*******************************/
void Feature::SERVOX::enable() {
    bsp_cmd_write(BSP_SERVO_CONFIG_ENABLE, port, 1);
}
void Feature::SERVOX::disable() {
    bsp_cmd_write(BSP_SERVO_CONFIG_ENABLE, port, 0);
}
void Feature::SERVOX::pos(int8_t position, uint16_t duration) {
    uint32_t range = bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, port);
    uint32_t pulseMin = range >> 16;
    uint32_t pulseMax = range & 0xFFFF;
    LIMIT_LH(position, -100, 100);
    pulse(((position+100) * (pulseMax-pulseMin) / 200) + pulseMin, duration);
}
void Feature::SERVOX::angle(uint8_t angle, uint16_t duration) {
    uint32_t range = bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, port);
    uint32_t pulseMin = range >> 16;
    uint32_t pulseMax = range & 0xFFFF;
    LIMIT_H(angle, 180);
    pulse((angle * (pulseMax-pulseMin) / 180) + pulseMin, duration);
}
void Feature::SERVOX::pulse(uint16_t pulse, uint16_t duration) {
    uint32_t range = bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, port);
    uint32_t pulseMin = range >> 16;
    uint32_t pulseMax = range & 0xFFFF;
    LIMIT_LH(pulse, pulseMin, pulseMax);
    bsp_cmd_write(BSP_SERVO_PULSE, port, (duration << 16) | pulse);
}
int8_t Feature::SERVOX::getPos() {
    uint32_t range = bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, port);
    uint32_t pulseMin = range >> 16;
    uint32_t pulseMax = range & 0xFFFF;
    return (((getPulse()-pulseMin)*200) / (pulseMax-pulseMin)) - 100;
}
uint8_t Feature::SERVOX::getAngle() {
    uint32_t range = bsp_cmd_read(BSP_SERVO_CONFIG_RANGE, port);
    uint32_t pulseMin = range >> 16;
    uint32_t pulseMax = range & 0xFFFF;
    return (((getPulse()-pulseMin)*180) / (pulseMax-pulseMin));
}
uint16_t Feature::SERVOX::getPulse() {
    return bsp_cmd_read(BSP_SERVO_PULSE, port);
}
void Feature::SERVOX::setInvert(bool state) {
    bsp_cmd_write(BSP_SERVO_CONFIG_INVERT, port, state);
}
void Feature::SERVOX::setSpeedRPM(float rpm) {
    bsp_cmd_write(BSP_SERVO_CONFIG_SPEED, port, rpm*60);
}
void Feature::SERVOX::setSpeedS60(float seconds) {
    if (seconds <= 0) this->setSpeedRPM(0);
    else this->setSpeedRPM(10 / seconds);
}
void Feature::SERVOX::setPulseMinMax(uint16_t min, uint16_t max) {
    uint32_t period = bsp_cmd_read(BSP_SERVO_CONFIG_PERIOD, 0);
    LIMIT_H(max, period);
    LIMIT_H(min, max);
    bsp_cmd_write(BSP_SERVO_CONFIG_RANGE, port, min << 16 | max);
}
/*******************************
          X4.rgbABCD
*******************************/
static struct {
    uint32_t color;
    uint32_t lastColor;
} RGBX_led[4];

void Feature::RGBABCD::enable() {
    bsp_cmd_write(BSP_RGB_CONFIG_ENABLE, BSP_PORT_ALL, 1);
}
void Feature::RGBABCD::disable() {
    bsp_cmd_write(BSP_RGB_CONFIG_ENABLE, BSP_PORT_ALL, 0);
}
void Feature::RGBABCD::reset() {
    this->colorTotem();
}
void Feature::RGBABCD::colorTotem() {
    X4.rgbA.colorHEX(TOTEM_COLOR_1); // Totem Green
    X4.rgbB.colorHEX(TOTEM_COLOR_2); // Totem Yellow
    X4.rgbC.colorHEX(TOTEM_COLOR_3); // Totem Yellow
    X4.rgbD.colorHEX(TOTEM_COLOR_4); // Totem Blue
}
void Feature::RGBABCD::colorARGB(uint8_t alpha, uint8_t r, uint8_t g, uint8_t b) {
    this->colorHEX(alpha << 24 | r << 16 | g << 8 | b);
}
void Feature::RGBABCD::colorRGB(uint8_t r, uint8_t g, uint8_t b) {
    this->colorHEX(0xFF << 24 | r << 16 | g << 8 | b);
}
void Feature::RGBABCD::colorAHEX(uint8_t alpha, uint32_t hex) {
    this->colorHEX(alpha << 24 | (hex & 0xFFFFFF));
}
void Feature::RGBABCD::colorHEX(uint32_t hex) {
    bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_ALL, hex);
}
void Feature::RGBABCD::on() {
    X4.rgbA.on();
    X4.rgbB.on();
    X4.rgbC.on();
    X4.rgbD.on();
}
void Feature::RGBABCD::off() {
    this->colorHEX(0);
}
void Feature::RGBABCD::set(uint8_t state) {
    state ? this->on() : this->off();
}
void Feature::RGBABCD::toggle() {
    X4.rgbA.toggle();
    X4.rgbB.toggle();
    X4.rgbC.toggle();
    X4.rgbD.toggle();
}
bool Feature::RGBABCD::isOn() {
    return (RGBX_led[0].color != 0)
    || (RGBX_led[1].color != 0)
    || (RGBX_led[2].color != 0)
    || (RGBX_led[3].color != 0);
}
void Feature::RGBABCD::fadeColorTotem() {
    X4.rgbA.fadeColorHEX(TOTEM_COLOR_1); // Totem Green
    X4.rgbB.fadeColorHEX(TOTEM_COLOR_2); // Totem Yellow
    X4.rgbC.fadeColorHEX(TOTEM_COLOR_3); // Totem Yellow
    X4.rgbD.fadeColorHEX(TOTEM_COLOR_4); // Totem Blue
}
void Feature::RGBABCD::fadeColorARGB(uint8_t alpha, uint8_t r, uint8_t g, uint8_t b) {
    this->fadeColorHEX(alpha << 24 | r << 16 | g << 8 | b);
}
void Feature::RGBABCD::fadeColorRGB(uint8_t r, uint8_t g, uint8_t b) {
    this->fadeColorHEX(0xFF << 24 | r << 16 | g << 8 | b);
}
void Feature::RGBABCD::fadeColorAHEX(uint8_t alpha, uint32_t hex) {
    this->fadeColorHEX(alpha << 24 | (hex & 0xFFFFFF));
}
void Feature::RGBABCD::fadeColorHEX(uint32_t hex) {
    bsp_cmd_write(BSP_RGB_FADE_COLOR, BSP_PORT_ALL, hex);
}
void Feature::RGBABCD::fadeStart(uint32_t duration) {
    bsp_cmd_write(BSP_RGB_FADE_START, BSP_PORT_ALL, duration);
}
Feature::RGBX& Feature::RGBABCD::operator[](uint8_t num) {
    switch (num) {
        case 0: return X4.rgbA;
        case 1: return X4.rgbB;
        case 2: return X4.rgbC;
        default: return X4.rgbD;
    }
}
/*******************************
          X4.rgbX
*******************************/
void Feature::RGBX::enable() {
    bsp_cmd_write(BSP_RGB_CONFIG_ENABLE, port, 1);
}
void Feature::RGBX::disable() {
    bsp_cmd_write(BSP_RGB_CONFIG_ENABLE, port, 0);
}
void Feature::RGBX::colorARGB(uint8_t alpha, uint8_t r, uint8_t g, uint8_t b) {
    this->colorHEX(alpha << 24 | r << 16 | g << 8 | b);
}
void Feature::RGBX::colorRGB(uint8_t r, uint8_t g, uint8_t b) {
    this->colorHEX(0xFF << 24 | r << 16 | g << 8 | b);
}
void Feature::RGBX::colorAHEX(uint8_t alpha, uint32_t hex) {
    this->colorHEX(alpha << 24 | (hex & 0xFFFFFF));
}
void Feature::RGBX::colorHEX(uint32_t hex) {
    bsp_cmd_write(BSP_RGB_COLOR, port, hex);
}
void Feature::RGBX::on() {
    this->colorHEX(RGBX_led[port].lastColor);
}
void Feature::RGBX::off() {
    this->colorHEX(0);
}
void Feature::RGBX::set(uint8_t state) {
    state ? this->on() : this->off();
}
void Feature::RGBX::toggle() {
    this->set(!this->isOn());
}
bool Feature::RGBX::isOn() {
    return RGBX_led[port].color != 0;
}
void Feature::RGBX::fadeColorARGB(uint8_t alpha, uint8_t r, uint8_t g, uint8_t b) {
    this->fadeColorHEX(alpha << 24 | r << 16 | g << 8 | b);
}
void Feature::RGBX::fadeColorRGB(uint8_t r, uint8_t g, uint8_t b) {
    this->fadeColorHEX(0xFF << 24 | r << 16 | g << 8 | b);
}
void Feature::RGBX::fadeColorAHEX(uint8_t alpha, uint32_t hex) {
    this->fadeColorHEX(alpha << 24 | (hex & 0xFFFFFF));
}
void Feature::RGBX::fadeColorHEX(uint32_t hex) {
    bsp_cmd_write(BSP_RGB_FADE_COLOR, port, hex);
}
void Feature::RGBX::fadeStart(uint32_t duration) {
    bsp_cmd_write(BSP_RGB_FADE_START, port, duration);
}
/*******************************
          X4.gpioX
*******************************/
void Feature::GPIOX::digitalWrite(uint8_t val) {
    bsp_gpio_digital_write(port, val);
}
int Feature::GPIOX::digitalRead() {
    return bsp_gpio_digital_read(port);
}
void Feature::GPIOX::analogWrite(uint8_t val) {
    bsp_gpio_analog_write(port, val);
}
int Feature::GPIOX::analogRead() {
    return bsp_gpio_analog_read(port);
}
void Feature::GPIOX::pullMode(uint8_t mode) {
    bsp_gpio_digital_input(port, mode);
}
// void addEvent(void (*gpioEvt)()) {}
/*******************************
          X4
*******************************/
RoboBoardX4::RoboBoardX4() : 
button(),
led(),
imu(),
dc(),
dcAB(0),
dcCD(2),
dcA(0), 
dcB(1), 
dcC(2), 
dcD(3),
servo(),
servoA(0),
servoB(1),
servoC(2),
rgb(),
rgbA(0),
rgbB(1),
rgbC(2),
rgbD(3),
gpioA(0),
gpioB(1),
gpioC(2),
gpioD(3)
{ }

ESP_EVENT_DEFINE_BASE(BSP_EVENT);
esp_event_loop_handle_t bsp_event_loop;

static void esp_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    uint32_t data = *((uint32_t*)event_data);
    if (event_id == BSP_BUTTON_STATE) Button_state_change(data);
}

static void bsp_cmd_change_callback(bsp_cmd_t cmd, uint8_t port, int32_t value, uint8_t isr) {
    if (cmd == BSP_RGB_COLOR) {
        RGBX_led[port].color = value;
        if (value) RGBX_led[port].lastColor = value;
    }
    else if (cmd == BSP_BUTTON_STATE) {
        BaseType_t task_unblocked = pdFALSE;
        esp_event_isr_post_to(bsp_event_loop, BSP_EVENT, BSP_BUTTON_STATE, &value, 4, &task_unblocked);
        portYIELD_FROM_ISR(task_unblocked);
    }
}

int RoboBoardX4::begin() {
    bsp_callback_register(bsp_cmd_change_callback);

    esp_event_loop_args_t loop_args = {
        .queue_size = 15,
        .task_name = "BSP_event",
        .task_priority = uxTaskPriorityGet(NULL),
        .task_stack_size = 10240,
        .task_core_id = tskNO_AFFINITY
    };
    esp_event_loop_create(&loop_args, &bsp_event_loop);
    esp_event_handler_register_with(bsp_event_loop, BSP_EVENT, ESP_EVENT_ANY_ID, esp_event_handler, NULL);

    return bsp_board_init();
}

float RoboBoardX4::getBatteryVoltage() {
    return ((float)bsp_cmd_read(BSP_BATTERY_VOLTAGE, 0))/1000;
}
int RoboBoardX4::getBatteryChargingTime() {
    return 0;
}
bool RoboBoardX4::isBatteryCharging() {
    return bsp_cmd_read(BSP_BATTERY_STATE, 0) == 1;
}
bool RoboBoardX4::isBatteryLow() {
    return bsp_cmd_read(BSP_BATTERY_VOLTAGE, 0) < 10500;
}
bool RoboBoardX4::isDC() {
    return bsp_cmd_read(BSP_DC_STATE, 0);
}
bool RoboBoardX4::isUSB() {
    return bsp_cmd_read(BSP_USB_STATE, 0);
}
void RoboBoardX4::restart() {
    esp_restart();
}
int RoboBoardX4::getSerial() {
    return bsp_cmd_read(BSP_BOARD_SERIAL, 0);
}
int RoboBoardX4::getRevisionNum() {
    return bsp_cmd_read(BSP_BOARD_REVISION, 0);
}
int RoboBoardX4::getDriverVersionNum() {
    return bsp_cmd_read(BSP_DRIVER_FIRMWARE, 0);
}
char* RoboBoardX4::getRevision() {
    int revision = getRevisionNum();
    static char str[4];
    str[0] = '0'+ (revision / 10);
    str[1] = '.';
    str[2] = '0'+ (revision % 10);
    str[3] = '\0';
    return str;
}
char* RoboBoardX4::getDriverVersion() {
    int version = getDriverVersionNum();
    static char str[5];
    str[0] = '0'+ (version / 100 % 10);
    str[1] = '.';
    str[2] = '0'+ (version / 10 % 10);
    str[3] = '0'+ (version % 10);
    str[4] = '\0';
    return str;
}
