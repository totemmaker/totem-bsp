/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef PRIV_INCLUDE_PERIPH_REGMAP
#define PRIV_INCLUDE_PERIPH_REGMAP

#define PERIPH_CONTROL_CHANNEL 0xA0

typedef enum {

    // Driver group
    PERIPH_DRIVER = 0x000, // Set group to 0
    // Driver max channel
    PERIPH_DRIVER_MAX = PERIPH_DRIVER,
    // Driver control commands
    PERIPH_DRIVER_SET = PERIPH_DRIVER + PERIPH_CONTROL_CHANNEL, // 0xA0
    PERIPH_DRIVER_SET_INIT,      // Initialize all peripherals
    PERIPH_DRIVER_SET_SOFTRESET, // Hardware chip restart
    PERIPH_DRIVER_SET_ALIVE,     // Time update until next driver re-initialization
    // Driver custom control commands
    PERIPH_DRIVER_CTRL = PERIPH_DRIVER + 0xC0,
    PERIPH_DRIVER_CTRL_POWER_5V,  // Enable 5V power rail (if supported)
    // Driver result commands
    PERIPH_DRIVER_GET = PERIPH_DRIVER + 0xE0,
    PERIPH_DRIVER_GET_ID,        // Driver chip ID : 0x98
    PERIPH_DRIVER_GET_VERSION,   // Driver firmware version
    PERIPH_DRIVER_GET_STATUS,    // Driver status. Interrupt resets after read
    PERIPH_DRIVER_GET_BOARD,     // Board version

    // Servo group
    PERIPH_SERVO = 0x100, // Set group to 1
    // Servo channel commands mask
    PERIPH_SERVO_X = PERIPH_SERVO + 0x00,
    PERIPH_SERVO_X_POS,        // 0x101 [deprecated]
    PERIPH_SERVO_X_PULSE,      // 0x102
    PERIPH_SERVO_X_GET_PULSE,  // 0x103
    PERIPH_SERVO_X_ENABLE,     // 0x104
    PERIPH_SERVO_X_INVERT,     // 0x105
    PERIPH_SERVO_X_SPEED,      // 0x106
    PERIPH_SERVO_X_PULSE_MIN,  // 0x107
    PERIPH_SERVO_X_PULSE_MAX,  // 0x108
    // Servo channel A commands
    PERIPH_SERVO_A = PERIPH_SERVO + 0x10,
    PERIPH_SERVO_A_POS,        // Set servo channel A position
    PERIPH_SERVO_A_PULSE,      // Set servo channel A pulse (duration | pulse)
    PERIPH_SERVO_A_GET_PULSE,  // Get current servo A pulse
    PERIPH_SERVO_A_ENABLE,     // Set servo channel A enabled
    PERIPH_SERVO_A_INVERT,     // Set servo channel A inversion
    PERIPH_SERVO_A_SPEED,      // Set servo channel A speed RPH
    PERIPH_SERVO_A_PULSE_MIN,  // Set servo channel A min pulse us
    PERIPH_SERVO_A_PULSE_MAX,  // Set servo channel A max pulse us
    // Servo channel B commands
    PERIPH_SERVO_B = PERIPH_SERVO + 0x20,
    PERIPH_SERVO_B_POS,        // Set servo channel B position
    PERIPH_SERVO_B_PULSE,      // Set servo channel B pulse (duration | pulse)
    PERIPH_SERVO_B_GET_PULSE,  // Get current servo B pulse
    PERIPH_SERVO_B_ENABLE,     // Set servo channel B enabled
    PERIPH_SERVO_B_INVERT,     // Set servo channel B inversion
    PERIPH_SERVO_B_SPEED,      // Set servo channel B speed RPH
    PERIPH_SERVO_B_PULSE_MIN,  // Set servo channel B min pulse us
    PERIPH_SERVO_B_PULSE_MAX,  // Set servo channel B max pulse us
    // Servo channel C commands
    PERIPH_SERVO_C = PERIPH_SERVO + 0x30,
    PERIPH_SERVO_C_POS,        // Set servo channel C position
    PERIPH_SERVO_C_PULSE,      // Set servo channel C pulse (duration | pulse)
    PERIPH_SERVO_C_GET_PULSE,  // Get current servo C pulse
    PERIPH_SERVO_C_ENABLE,     // Set servo channel C enabled
    PERIPH_SERVO_C_INVERT,     // Set servo channel C inversion
    PERIPH_SERVO_C_SPEED,      // Set servo channel C speed RPH
    PERIPH_SERVO_C_PULSE_MIN,  // Set servo channel C min pulse us
    PERIPH_SERVO_C_PULSE_MAX,  // Set servo channel C max pulse us
    // Servo max channel
    PERIPH_SERVO_MAX = PERIPH_SERVO_C,
    // Servo control commands
    PERIPH_SERVO_SET = PERIPH_SERVO + PERIPH_CONTROL_CHANNEL, // 0xA0
    PERIPH_SERVO_SET_ABC_ENABLE, // Enable/Disable servo peripheral
    PERIPH_SERVO_SET_ABC_POS,    // [deprecated] Set same position for all channels
    PERIPH_SERVO_SET_ABC_PULSE,  // Set same pulse for all channels (duration | pulse)
    PERIPH_SERVO_SET_ABC_PERIOD, // Set channels period (max pulse)

    // DC group
    PERIPH_DC = 0x200, // Set group to 2
    // DC channel commands mask
    PERIPH_DC_X = PERIPH_DC + 0x00,
    PERIPH_DC_X_POWER,     // 0x201
    PERIPH_DC_X_BRAKE,     // 0x202
    PERIPH_DC_X_AUTOBRAKE, // 0x203
    PERIPH_DC_X_ENABLE,    // 0x204
    PERIPH_DC_X_INVERT,    // 0x205
    // DC channel A commands
    PERIPH_DC_A = PERIPH_DC + 0x10,
    PERIPH_DC_A_POWER,     // Set motor A spin power
    PERIPH_DC_A_BRAKE,     // Set motor A brake power
    PERIPH_DC_A_AUTOBRAKE, // Set motor A autobrake power
    PERIPH_DC_A_ENABLE,    // Set motor A enabled
    PERIPH_DC_A_INVERT,    // Set motor A inversion
    // DC channel B commands
    PERIPH_DC_B = PERIPH_DC + 0x20,
    PERIPH_DC_B_POWER,     // Set motor B spin power
    PERIPH_DC_B_BRAKE,     // Set motor B brake power
    PERIPH_DC_B_AUTOBRAKE, // Set motor B autobrake power
    PERIPH_DC_B_ENABLE,    // Set motor B enabled
    PERIPH_DC_B_INVERT,    // Set motor B inversion
    // DC channel C commands
    PERIPH_DC_C = PERIPH_DC + 0x30,
    PERIPH_DC_C_POWER,     // Set motor C spin power
    PERIPH_DC_C_BRAKE,     // Set motor C brake power
    PERIPH_DC_C_AUTOBRAKE, // Set motor C autobrake power
    PERIPH_DC_C_ENABLE,    // Set motor C enabled
    PERIPH_DC_C_INVERT,    // Set motor C inversion
    // DC channel D commands
    PERIPH_DC_D = PERIPH_DC + 0x40,
    PERIPH_DC_D_POWER,     // Set motor D spin power
    PERIPH_DC_D_BRAKE,     // Set motor D brake power
    PERIPH_DC_D_AUTOBRAKE, // Set motor D autobrake power
    PERIPH_DC_D_ENABLE,    // Set motor D enabled
    PERIPH_DC_D_INVERT,    // Set motor D inversion
    // DC max channel
    PERIPH_DC_MAX = PERIPH_DC_D,
    // DC control commands
    PERIPH_DC_SET = PERIPH_DC + PERIPH_CONTROL_CHANNEL, // 0xA0
    PERIPH_DC_SET_ABCD_ENABLE,    // Enable/Disable DC motor peripheral
    PERIPH_DC_SET_ABCD_INVERT,    // Set motors inversion
    PERIPH_DC_SET_ABCD_AUTOBRAKE, // Set motors autobrake enabled
    PERIPH_DC_SET_ABCD_POWER,     // Set motor ABCD spin power  | A | B | C | D |
    PERIPH_DC_SET_ABCD_BRAKE,     // Set motor ABCD brake power | A | B | C | D |
    PERIPH_DC_SET_AB_MODE,        // Set channels AB mode (0 - individual, 1 - combined)
    PERIPH_DC_SET_CD_MODE,        // Set channels CD mode (0 - individual, 1 - combined)
    PERIPH_DC_SET_AB_FREQUENCY,   // Set channels AB PWM frequency
    PERIPH_DC_SET_CD_FREQUENCY,   // Set channels CD PWM frequency
    PERIPH_DC_SET_AB_POWER_MAX,   // Set channels AB period
    PERIPH_DC_SET_CD_POWER_MAX,   // Set channels CD period
    PERIPH_DC_SET_AB_TONE,        // Play channels AB tone duration | freq
    PERIPH_DC_SET_CD_TONE,        // Play channels CD tone duration | freq

    // RGB group
    PERIPH_RGB = 0x300, // Set group to 3
    // RGB channel commands mask
    PERIPH_RGB_X = PERIPH_RGB + 0x00,
    PERIPH_RGB_X_SET,         // 0x301
    PERIPH_RGB_X_SET_FADE,    // 0x302
    PERIPH_RGB_X_START_FADE,  // 0x303
    // RGB channel A commands
    PERIPH_RGB_A = PERIPH_RGB + 0x10,
    PERIPH_RGB_A_SET,         // Set RGB led A color and brightness
    PERIPH_RGB_A_SET_FADE,    // Set RGB led A fade color and brightness
    PERIPH_RGB_A_START_FADE,  // Start RGB led A fade animation
    // RGB channel B commands
    PERIPH_RGB_B = PERIPH_RGB + 0x20,
    PERIPH_RGB_B_SET,         // Set RGB led B color and brightness
    PERIPH_RGB_B_SET_FADE,    // Set RGB led B fade color and brightness
    PERIPH_RGB_B_START_FADE,  // Start RGB led B fade animation
    // RGB channel C commands
    PERIPH_RGB_C = PERIPH_RGB + 0x30,
    PERIPH_RGB_C_SET,         // Set RGB led C color and brightness
    PERIPH_RGB_C_SET_FADE,    // Set RGB led C fade color and brightness
    PERIPH_RGB_C_START_FADE,  // Start RGB led C fade animation
    // RGB channel D commands
    PERIPH_RGB_D = PERIPH_RGB + 0x40,
    PERIPH_RGB_D_SET,         // Set RGB led D color and brightness
    PERIPH_RGB_D_SET_FADE,    // Set RGB led D fade color and brightness
    PERIPH_RGB_D_START_FADE,  // Start RGB led D fade animation
    // RGB max channel
    PERIPH_RGB_MAX = PERIPH_RGB_D,
    // RGB control commands
    PERIPH_RGB_SET = PERIPH_RGB + PERIPH_CONTROL_CHANNEL, // 0xA0
    PERIPH_RGB_SET_ABCD_ENABLE,  // Enable/Disable RGB led peripheral
    PERIPH_RGB_SET_ABCD_SET,     // Set all RGB leds color and brightness
    PERIPH_RGB_SET_ABCD_FADE,    // Set all RGB leds fade color and brightness
    PERIPH_RGB_SET_ABCD_START_FADE,  // Start all RGB fade animation

    // GPIO
    PERIPH_GPIO = 0x400, // Set group to 4
    // GPIO channel commands mask
    PERIPH_GPIO_X = PERIPH_GPIO + 0x00,
    PERIPH_GPIO_X_PULL,           // 0x401
    PERIPH_GPIO_X_DIGITAL_READ,   // 0x402
    PERIPH_GPIO_X_DIGITAL_WRITE,  // 0x403
    PERIPH_GPIO_X_DIGITAL_DETECT, // 0x404
    PERIPH_GPIO_X_ANALOG_READ,    // 0x405
    PERIPH_GPIO_X_ANALOG_WRITE,   // 0x406
    // GPIO channel A commands
    PERIPH_GPIO_A = PERIPH_GPIO + 0x10,
    PERIPH_GPIO_A_PULL,           // Configure as input and set pull mode. [HIGH(up):LOW(down)]
    PERIPH_GPIO_A_DIGITAL_READ,   // Read GPIO A pin state -> [LOW:HIGH]
    PERIPH_GPIO_A_DIGITAL_WRITE,  // Set GPIO A pin state -> [LOW:HIGH]
    PERIPH_GPIO_A_DIGITAL_DETECT, // Set GPIO A pin state detection
    PERIPH_GPIO_A_ANALOG_READ,    // Reads analog value of GPIO A pin -> [0:1023]
    PERIPH_GPIO_A_ANALOG_WRITE,   // Write analog value of GPIO A pin -> [0:20]
    // GPIO channel B commands
    PERIPH_GPIO_B = PERIPH_GPIO + 0x20,
    PERIPH_GPIO_B_PULL,           // Configure as input and set pull mode. [HIGH(up):LOW(down)]
    PERIPH_GPIO_B_DIGITAL_READ,   // Read GPIO B pin state -> [LOW:HIGH]
    PERIPH_GPIO_B_DIGITAL_WRITE,  // Set GPIO B pin state -> [LOW:HIGH]
    PERIPH_GPIO_B_DIGITAL_DETECT, // Set GPIO B pin state detection
    PERIPH_GPIO_B_ANALOG_READ,    // Reads analog value of GPIO B pin -> [0:1023]
    PERIPH_GPIO_B_ANALOG_WRITE,   // Write analog value of GPIO B pin -> [0:20]
    // GPIO channel C commands
    PERIPH_GPIO_C = PERIPH_GPIO + 0x30,
    PERIPH_GPIO_C_PULL,           // Configure as input and set pull mode. [HIGH(up):LOW(down)]
    PERIPH_GPIO_C_DIGITAL_READ,   // Read GPIO C pin state -> [LOW:HIGH]
    PERIPH_GPIO_C_DIGITAL_WRITE,  // Set GPIO C pin state -> [LOW:HIGH]
    PERIPH_GPIO_C_DIGITAL_DETECT, // Set GPIO C pin state detection
    PERIPH_GPIO_C_ANALOG_READ,    // Reads analog value of GPIO C pin -> [0:1023]
    PERIPH_GPIO_C_ANALOG_WRITE,   // Write analog value of GPIO C pin -> [0:20]
    // GPIO channel D commands
    PERIPH_GPIO_D = PERIPH_GPIO + 0x40,
    PERIPH_GPIO_D_PULL,           // Configure as input and set pull mode. [HIGH(up):LOW(down)]
    PERIPH_GPIO_D_DIGITAL_READ,   // Read GPIO D pin state -> [LOW:HIGH]
    PERIPH_GPIO_D_DIGITAL_WRITE,  // Set GPIO D pin state -> [LOW:HIGH]
    PERIPH_GPIO_D_DIGITAL_DETECT, // Set GPIO D pin state detection
    PERIPH_GPIO_D_ANALOG_READ,    // Reads analog value of GPIO D pin -> [0:1023]
    PERIPH_GPIO_D_ANALOG_WRITE,   // Write analog value of GPIO D pin -> [0:20]
    // GPIO max channel
    PERIPH_GPIO_MAX = PERIPH_GPIO_D,
} PeriphRegMap;

#define PeriphRegMap_group(command) (((command) >> 8) & 0xF)
#define PeriphRegMap_channel(command) (((command) >> 4) & 0xF)
#define PeriphRegMap_command(command) ((command) & 0xF)

typedef union {
    struct {
    uint32_t A_trig : 1;
    uint32_t A_high : 1;
    uint32_t B_trig : 1;
    uint32_t B_high : 1;
    uint32_t C_trig : 1;
    uint32_t C_high : 1;
    uint32_t D_trig : 1;
    uint32_t D_high : 1;
    uint32_t             : 0;
    } gpio;
    uint32_t value;
} PeriphRegMapStatus;

#endif /* PRIV_INCLUDE_PERIPH_REGMAP */
