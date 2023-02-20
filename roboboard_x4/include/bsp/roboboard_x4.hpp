/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_BSP_ROBOBOARD_X4_HPP
#define INCLUDE_BSP_ROBOBOARD_X4_HPP

namespace Feature {
/*******************************
          X4.config
*******************************/
class Config {
public:
    // Set Robot Name. Will be visible during App connection
    // Params:
    // `name`: name up to 30 characters
    void setRobotName(const char *name);
    // Get the Robot Name configured with setRobotName()
    // Returns: robot name string
    char* getRobotName();
    // Set the Robot Model name (description)
    // Returns: `name`: string of any length. Will be converted to 16-bit hash value
    void setRobotModel(const char *name);
    // Set the Robot Model name (description)
    // Params:
    // `hash`: 16-bit value of model description hash [0x0000:0xFFFF]
    void setRobotModel(uint16_t hash);
    // Get the Robot Model name (description)
    // Returns: 16-bit hash of model description name [0x0000:0xFFFF]
    uint16_t getRobotModel();
    // Set the Robot Color. Appearance color to easier identify robot. Will set provided color
    // Params:
    // `r`: Red [0:255]
    // `g`: Green [0:255]
    // `b`: Blue [0:255]
    void setRobotColor(uint8_t r, uint8_t g, uint8_t b);
    // Set the Robot Color. Appearance color to easier identify robot. Will set provided color
    // Params:
    // `hex`: 24-bit HEX color code [0x000000:0xFFFFFF]
    void setRobotColor(uint32_t hex);
    // Get the Robot Color 24-bit HEX color code
    // Returns: 24-bit HEX color code [0x000000:0xFFFFFF]
    uint32_t getRobotColor();
    // Invert DC port output polarity
    // Params:
    // `invertX`: invert port [true:false]
    void setDCInvert(bool invertA, bool invertB, bool invertC, bool invertD);
    // Get DC ports invert parameter
    // Returns: encoded 32-bit value A | B | C | D
    uint32_t getDCInvert();
    // Set DC automatic braking. Will brake motor when power is set to 0
    // Params:
    // `powerX` is port braking enabled [true:false]
    void setDCAutobrake(bool powerA, bool powerB, bool powerC, bool powerD);
    // Set DC automatic braking. Will brake motor when power is set to 0
    // Params:
    // `powerX`: amount of port braking power [0:100]%
    void setDCAutobrake(int powerA, int powerB, int powerC, int powerD);
    // Get DC ports automatic brake parameter
    // Returns: encoded 32-bit value A | B | C | D
    uint32_t getDCAutobrake();
    // Reset configuration to default
    void reset();
};
/*******************************
          X4.button
*******************************/
class Button {
public:
    /*
     * Check current button state. Returns true until condition is valid.
    */
    // Is pressed
    bool isPressed();
    // Is released
    bool isReleased();
    // Is pressed for a certain period of time (long press)
    // `ms`: time after button press (milliseconds)
    bool isPressedFor(uint32_t ms);
    // Is released for a certain period of time (cooldown)
    // `ms`: time after button release (milliseconds)
    bool isReleasedFor(uint32_t ms);
    // Is pressed for 500ms
    bool isLongPress();
    /*
     * Check current or previous button state. Returns true only once.
     * Convenient to use inside loop.
    */
    // Was pressed earlier
    bool wasPressed();
    // Was released earlier?
    bool wasReleased();
    // Was pressed for a certain period of time (long press)
    // `ms`: time after button press (milliseconds)
    bool wasPressedFor(uint32_t ms);
    // Was released for a certain period of time (cooldown)
    // `ms`: time after button release (milliseconds)
    bool wasReleasedFor(uint32_t ms);
    // Was pressed for 500ms
    bool wasLongPress(); // if is isLongPress or was 1 and > 500
    // Was double clicked
    bool wasDoubleClick();
    // Wait for button press
    bool waitPress(uint32_t timeout = 0);
    // Get time stamp of last button state change
    // Returns: 32bit milliseconds time (millis())
    uint32_t lastChange();
    // Get button state [0:1]
    uint8_t getState();
    // Register an event for button state change (press or release)
    // `buttonEvt`: button event function name
    void addEvent(void (*buttonEvt)());
};
/*******************************
          X4.led
*******************************/
class Led {
public:
    // Turn LED on
    void on();
    // Turn LED off
    void off();
    // Toggle LED on / off
    void toggle();
    // Set LED state [`0`:`1`]
    void set(uint8_t state);
    // Get LED state [`0`:`1`]
    int isOn();
    // Blink LED few times
    void blink();
    // Blink LED specified number of times.
    // `count`: number of times to blink.
    // `blinkDuration`: (optional) delay between blinks (milliseconds).
    void blinkTimes(uint32_t count, uint32_t blinkDuration = 200);
    // Blink LED for certain amount of time.
    // `durationMs`: total amount of time (milliseconds).
    // `blinkDuration`: (optional) delay between blinks (milliseconds).
    void blinkFor(uint32_t durationMs, uint32_t blinkDuration = 100);
};
/*******************************
          X4.imu (mems)
*******************************/
class IMU {
public:
    struct Data {
        // Accelerometer. Measures acceleration
        float getX_G(); // X G. Gravitational force
        float getY_G(); // Y G. Gravitational force
        float getZ_G(); // Z G. Gravitational force
        float getX_mss(); // X m/s^2. Acceleration in meters per second squared
        float getY_mss(); // Y m/s^2. Acceleration in meters per second squared
        float getZ_mss(); // Z m/s^2. Acceleration in meters per second squared
        // Gyroscope. Measures rotation speed
        float getX_dps(); // X dps. Degrees per second
        float getY_dps(); // Y dps. Degrees per second
        float getZ_dps(); // Z dps. Degrees per second
        float getX_rads(); // X rad/s. Radians per second
        float getY_rads(); // Y rad/s. Radians per second
        float getZ_rads(); // Z rad/s. Radians per second
        float getX_rpm(); // X RPM. Rounds per minute
        float getY_rpm(); // Y RPM. Rounds per minute
        float getZ_rpm(); // Z RPM. Rounds per minute
        // IMU internal temperature
        float getTemp() { return this->getTempC(); }
        float getTempC(); // Celsius
        float getTempF(); // Fahrenheit
        // Accelerometer based board orientation of X axis. [-180:180] degree
        float getOrientX();
        // Accelerometer based board orientation of Y axis. [-180:180] degree
        float getOrientY();
        // Accelerometer based roll estimation. [-180:180] degree
        float getRoll();
        // Accelerometer based pitch estimation. [-90:90] degree
        float getPitch();
    private:
        float temp;
        struct {
            float x, y, z;
        } accel, gyro;
    };
    // Read latest measurements
    Data read();
    // Set accelerometer maximum range of G force.
    // Allowed values: 2, 4, 8, 16. Default: 16 (G)
    void setAccelRange(uint16_t range);
    // Set gyroscope maximum range of angle speed.
    // Allowed values: 250, 500, 1000, 2000. Default 2000 (dps)
    void setGyroRange(uint16_t range);
    // Get name of IMU sensor
    const char* getName();
    // Get I2C address of IMU sensor
    uint8_t getI2CAddr();
};
/*******************************
          X4.dcX
*******************************/
class DCX {
protected:
    const uint32_t port;
public:
    DCX(uint32_t port) : port(port) { }
    // Enable DC ports
    void enable();
    // Disable DC ports
    void disable();
    // Spin motor at specified power and direction
    // Negative - backwards. `0` - stop
    // `power`: output power [`-100`:`100`]
    void power(int8_t power);
    // Brake motor at specified power
    // `power`: braking power [`0`:`100`]
    void brake(int8_t power = 100.0);
    // Power off motor and let it free spin
    void coast();
    // Invert motor spin direction
    // `state`: `false` - normal direction | `true` - swap direction
    void setInvert(bool state);
    // Set DC autobrake power (when power is set to 0)
    // `power`: autobrake power [`0`:`100`]. `0` - off (coast)
    void setAutobrake(int power);
    void setAutobrake(bool state);
};
/*******************************
          X4.dcXX
*******************************/
class DCXX {
protected:
    const uint32_t port;
public:
    DCXX(uint32_t port) : port(port) { }
    // Play specified frequency on DC port group
    // `frequency`: tone frequency [`0`:`20000`] Hz
    // `duration`: (optional) stop playing after time (milliseconds)
    void tone(uint16_t frequency, uint16_t duration = 0);
    // Switch individual mode
    void setModeIndividual();
    // Switch to combined mode
    void setModeCombined();
    // Set DC port group PWM frequency (default 50Hz)
    // `frequency`: PWM frequency [`1`:`250000`] Hz
    void setFrequency(uint32_t frequency);
    // Alias for setFrequency();
    void setFreq(uint32_t frequency) { this->setFrequency(frequency); }
};
/*******************************
          X4.dc
*******************************/
class DCABCD {
public:
    // Enable all DC ports
    void enable();
    // Disable all DC ports
    void disable();
    // Set output power of each motor port
    // Negative - backwards. `0` - stop
    // `powerX`: spin power of port [`-100`:`100`]
    void power(int8_t powerA, int8_t powerB, int8_t powerC, int8_t powerD);
    // Set braking power of each motor port
    // `powerX`: brake power of port [`0`:`100`]
    void brake(int8_t powerA, int8_t powerB, int8_t powerC, int8_t powerD);
    // Set spin direction inversion of each motor port
    // `invertX`: change spin direction of port `false` - no | `true` - yes
    void setInvert(bool invertA, bool invertB, bool invertC, bool invertD);
    // Set braking power of each motor port (when power is set to 0)
    // `powerX`: autobrake power of port [`0`:`100`]
    void setAutobrake(int powerA, int powerB, int powerC, int powerD);
    void setAutobrake(bool stateA, bool stateB, bool stateC, bool stateD);
    // Access DC port with numeric index. X4.dc[0] references X4.dcA
    DCX& operator[](uint8_t port);
};
/*******************************
          X4.servoX
*******************************/
class SERVOX {
protected:
    const uint32_t port;
public:
    SERVOX(uint32_t port) : port(port) { }
    // Enable Servo port
    void enable();
    // Disable Servo port
    void disable();
    // Turn motor to position [`-100`:`100`] in percentage. `0` - center
    void pos(int8_t position, uint16_t duration = 0);
    // Turn motor to angle [`0`:`180`] in degrees
    void angle(uint8_t angle, uint16_t duration = 0);
    // Turn motor to pulse [`0`:`period`] in microseconds (us). Typical: [500:2500]
    void pulse(uint16_t pulse, uint16_t duration = 0);
    // Get motor position in percentage [`-100`:`100`]
    int8_t getPos();
    // Get motor angle [`0`:`180`]
    uint8_t getAngle();
    // Get motor position in microseconds [`0`:`period`]
    uint16_t getPulse();

    // Invert Servo motor spin direction [`0`:`1`]
    void setInvert(bool state);
    // Set constant Servo motor speed RPM (Rounds-Per-Minute)
    void setSpeedRPM(float rpm); // RPM
    void setSpeedS60(float seconds); // seconds / 60 degree
    // Set Servo low & high pulse (microseconds) limits (default 500, 2500)
    void setPulseMinMax(uint16_t min, uint16_t max);
};
/*******************************
          X4.servo
*******************************/
class SERVOABC {
public:
    // Enable all Servo ports
    void enable();
    // Disable all Servo ports
    void disable();
    // Set Servo period [`1`:`65535`] us (microseconds) (can be higher)
    void setPeriod(uint32_t period);
    // Turn motor to position [`-100`:`100`] in percentage. `0` - center
    void pos(int8_t positionA, int8_t positionB, int8_t positionC);
    // Turn motor to angle [`0`:`180`] in degrees
    void angle(uint8_t angleA, uint8_t angleB, uint8_t angleC);
    // Turn motor to pulse [`0`:`period`] in microseconds (us). Typical: [500:2500]
    void pulse(uint16_t pulseA, uint16_t pulseB, uint16_t pulseC);
    // Access Servo port with numeric index. X4.servo[0] references X4.servoA
    SERVOX& operator[](uint8_t port);
};
/*******************************
          X4.rgbX
*******************************/
class RGBX {
protected:
    const uint32_t port;
public:
    RGBX(uint32_t port) : port(port) { }
    // Enable RGB LED
    void enable();
    // Disable RBB LED
    void disable();
    // Set LED with brightness, red, green, blue [`0`:`255`]
    void colorARGB(uint8_t alpha, uint8_t r, uint8_t g, uint8_t b);
    // Set LED with red, green, blue [`0`:`255`]
    void colorRGB(uint8_t r, uint8_t g, uint8_t b);
    // Set LED with brightness [`0`:`255`] and HEX color code [`0`:`0xFFFFFF`]
    void colorAHEX(uint8_t alpha, uint32_t hex);
    // Set LED with HEX color code [`0`:`0xFFFFFFFF`] (including brightness)
    void colorHEX(uint32_t hex);
    // Turn LED on
    void on();
    // Turn LED off
    void off();
    // Turn on or off
    void set(uint8_t state);
    // Toggle on / off
    void toggle();
    // Check if LED is on
    bool isOn();
    // Set LED fade with alpha, red, green, blue [`0`:`255`]
    void fadeColorARGB(uint8_t alpha, uint8_t r, uint8_t g, uint8_t b);
    // Set LED fade with red, green, blue [`0`:`255`]
    void fadeColorRGB(uint8_t r, uint8_t g, uint8_t b);
    // Set LED fade with brightness [`0`:`255`] and HEX color code [`0`:`0xFFFFFF`]
    void fadeColorAHEX(uint8_t alpha, uint32_t hex);
    // Set LED fade with HEX color code [`0`:`0xFFFFFFFF`] (including brightness)
    void fadeColorHEX(uint32_t hex);
    // Start LED port fading
    void fadeStart(uint32_t duration);
};
/*******************************
          X4.rgb
*******************************/
class RGBABCD  {
public:
    // Enable all RGB LEDs
    void enable();
    // Disable all RBB LEDs
    void disable();
    // Reset to default color
    void reset();
    // Light LED with Totem color
    void colorTotem();
    // Light LED with brightness, red, green, blue [`0`:`255`]
    void colorARGB(uint8_t alpha, uint8_t r, uint8_t g, uint8_t b);
    // Light LED with red, green, blue [`0`:`255`]
    void colorRGB(uint8_t r, uint8_t g, uint8_t b);
    // Light LED with brightness [`0`:`255`] and HEX color code [`0`:`0xFFFFFF`]
    void colorAHEX(uint8_t alpha, uint32_t hex);
    // Light LED with HEX color code [`0`:`0xFFFFFFFF`] (including brightness)
    void colorHEX(uint32_t hex);
    // Turn LED on
    void on();
    // Turn LED off
    void off();
    // Turn on or off
    void set(uint8_t state);
    // Toggle on / off
    void toggle();
    // Check if any of LED is on
    bool isOn();
    // Set LED fade with Totem color
    void fadeColorTotem();
    // Set LED fade with alpha, red, green, blue [`0`:`255`]
    void fadeColorARGB(uint8_t alpha, uint8_t r, uint8_t g, uint8_t b);
    // Set LED fade with red, green, blue [`0`:`255`]
    void fadeColorRGB(uint8_t r, uint8_t g, uint8_t b);
    // Set LED fade with brightness [`0`:`255`] and HEX color code [`0`:`0xFFFFFF`]
    void fadeColorAHEX(uint8_t alpha, uint32_t hex);
    // Set LED fade with HEX color code [`0`:`0xFFFFFFFF`] (including brightness)
    void fadeColorHEX(uint32_t hex);
    // Start all LED fading
    void fadeStart(uint32_t duration);
    // Access RGB LED with numeric index. X4.rgb[0] references X4.rgbA
    RGBX& operator[](uint8_t num);
};
/*******************************
          X4.qwiic
*******************************/
class Qwiic {
public:
    // Check if module with specified I2C address is connected
    bool isConnected(int address);
    // Scan for connected I2C modules
    // Returns number of devices found
    int scan(void (*foundEvt)(int addr));
    // Set I2C speed [100:400]kHz
    bool setSpeed(uint32_t frequency);
};
/*******************************
          X4.gpioX
*******************************/
class GPIOX { // v1.0 only
protected:
    const uint32_t port;
public:
    GPIOX(uint32_t port) : port(port) { }
    // Write pin digital state [`HIGH`:`LOW`]
    void digitalWrite(uint8_t val);
    // Read pin digital state [`HIGH`:`LOW`]
    int digitalRead();
    // Write PWM output [`0`:`20`]
    void analogWrite(uint8_t val);
    // Read analog input [`0`:`1023`]
    int analogRead();
    // Set pin pullup mode [`HIGH`:`LOW`]
    void pullMode(uint8_t mode);
    // Register GPIO event
    // void addEvent(void (*gpioEvt)());
};
} // namespace Feature

class RoboBoardX4 {
public:
    Feature::Config config;
    Feature::Button button;
    Feature::Led led;
    Feature::IMU imu;
    Feature::DCABCD dc;
    Feature::DCXX dcAB, dcCD;
    Feature::DCX dcA, dcB, dcC, dcD;
    Feature::SERVOABC servo;
    Feature::SERVOX servoA, servoB, servoC;
    Feature::RGBABCD rgb;
    Feature::RGBX rgbA, rgbB, rgbC, rgbD;
    Feature::Qwiic qwiic;
    Feature::GPIOX gpioA, gpioB, gpioC, gpioD; // v1.0 only
    
    RoboBoardX4();
    // Initialize board
    int begin();
    // Read battery voltage [8.40:12.60]
    float getBatteryVoltage();
    // Get battery charging time
    int getBatteryChargingTime();
    // Is battery charging
    bool isBatteryCharging();
    // Check if battery voltage is low (requires charging)
    bool isBatteryLow();
    // Is DC adapter connected
    bool isDC();
    // Is USB cable connected
    bool isUSB();
    // Restart processor
    void restart();

    // Get X4 serial number
    int getSerial();
    // Get X4 board revision number
    int getRevisionNum();
    // Get X4 driver firmware version number
    int getDriverVersionNum();
    // Get X4 board revision version string
    char* getRevision();
    // Get X4 driver firmware version string
    char* getDriverVersion();
};

extern RoboBoardX4 X4;

#endif /* INCLUDE_BSP_ROBOBOARD_X4_HPP */
