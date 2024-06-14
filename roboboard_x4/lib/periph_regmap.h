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
    // [deprecated] - will be removed in future
    // [removed] - feature not available anymore

    // Driver group
    PERIPH_DRIVER = 0x000, // Set group to 0
    // Driver max channel
    PERIPH_DRIVER_MAX = PERIPH_DRIVER,
    // Driver control commands
    PERIPH_DRIVER_SET = PERIPH_DRIVER + PERIPH_CONTROL_CHANNEL, // 0xA0
    PERIPH_DRIVER_SET_INIT,      // Initialize all peripherals (DC duty | DC frequency)
    PERIPH_DRIVER_SET_REBOOT,    // Software chip restart
    PERIPH_DRIVER_SET_ALIVE,     // Time update until next driver re-initialization
    PERIPH_DRIVER_SET_BAUD,      // Set communication baudrate
    PERIPH_DRIVER_SET_COMM,      // Communication mode (0-half-duplex, 1-full-duplex)
    PERIPH_DRIVER_SET_STREAM,    // Stream changes of specified command
    // Board control commands
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
    PERIPH_SERVO_X_INVERT,     // 0x105 [removed]
    PERIPH_SERVO_X_SPEED,      // 0x106
    PERIPH_SERVO_X_PULSE_MIN,  // 0x107 [removed]
    PERIPH_SERVO_X_PULSE_MAX,  // 0x108 [removed]
    // Servo channel A commands
    PERIPH_SERVO_A = PERIPH_SERVO + 0x10,
    PERIPH_SERVO_A_POS,        // Set servo channel A position [deprecated]
    PERIPH_SERVO_A_PULSE,      // Set servo channel A pulse (duration | pulse)
    PERIPH_SERVO_A_GET_PULSE,  // Get current servo A pulse
    PERIPH_SERVO_A_ENABLE,     // Set servo channel A enabled
    PERIPH_SERVO_A_INVERT,     // [removed]
    PERIPH_SERVO_A_SPEED,      // Set servo channel A speed PPP
    PERIPH_SERVO_A_PULSE_MIN,  // [removed]
    PERIPH_SERVO_A_PULSE_MAX,  // [removed]
    // Servo channel B commands
    PERIPH_SERVO_B = PERIPH_SERVO + 0x20,
    PERIPH_SERVO_B_POS,        // Set servo channel B position [deprecated]
    PERIPH_SERVO_B_PULSE,      // Set servo channel B pulse (duration | pulse)
    PERIPH_SERVO_B_GET_PULSE,  // Get current servo B pulse
    PERIPH_SERVO_B_ENABLE,     // Set servo channel B enabled
    PERIPH_SERVO_B_INVERT,     // [removed]
    PERIPH_SERVO_B_SPEED,      // Set servo channel B speed PPP
    PERIPH_SERVO_B_PULSE_MIN,  // [removed]
    PERIPH_SERVO_B_PULSE_MAX,  // [removed]
    // Servo channel C commands
    PERIPH_SERVO_C = PERIPH_SERVO + 0x30,
    PERIPH_SERVO_C_POS,        // Set servo channel C position [deprecated]
    PERIPH_SERVO_C_PULSE,      // Set servo channel C pulse (duration | pulse)
    PERIPH_SERVO_C_GET_PULSE,  // Get current servo C pulse
    PERIPH_SERVO_C_ENABLE,     // Set servo channel C enabled
    PERIPH_SERVO_C_INVERT,     // [removed]
    PERIPH_SERVO_C_SPEED,      // Set servo channel C speed PPP
    PERIPH_SERVO_C_PULSE_MIN,  // [removed]
    PERIPH_SERVO_C_PULSE_MAX,  // [removed]
    // Servo max channel
    PERIPH_SERVO_MAX = PERIPH_SERVO_C,
    // Servo control commands
    PERIPH_SERVO_SET = PERIPH_SERVO + PERIPH_CONTROL_CHANNEL, // 0xA0
    PERIPH_SERVO_SET_ABC_ENABLE, // Enable/Disable servo peripheral | A | B | C |
    PERIPH_SERVO_SET_ABC_POS,    // Set same position for all channels [deprecated]
    PERIPH_SERVO_SET_ABC_PULSE,  // Set same pulse for all channels (duration | pulse)
    PERIPH_SERVO_SET_ABC_PERIOD, // Set channels period
    PERIPH_SERVO_SET_ABC_SPEED,  // Set channels constant motion speed (Pulses Per Period)

    // DC group
    PERIPH_DC = 0x200, // Set group to 2
    // DC channel commands mask
    PERIPH_DC_X = PERIPH_DC + 0x00,
    PERIPH_DC_X_POWER,     // 0x201
    PERIPH_DC_X_BRAKE,     // 0x202
    PERIPH_DC_X_AUTOBRAKE, // 0x203 [deprecated]
    PERIPH_DC_X_ENABLE,    // 0x204
    PERIPH_DC_X_INVERT,    // 0x205 [deprecated]
    PERIPH_DC_X_DECAY,     // 0x206
    // DC channel A commands
    PERIPH_DC_A = PERIPH_DC + 0x10,
    PERIPH_DC_A_POWER,     // Set motor A spin power
    PERIPH_DC_A_BRAKE,     // Set motor A brake power
    PERIPH_DC_A_AUTOBRAKE, // Set motor A autobrake power [deprecated]
    PERIPH_DC_A_ENABLE,    // Set motor A enabled
    PERIPH_DC_A_INVERT,    // Set motor A inversion [deprecated]
    PERIPH_DC_A_DECAY,     // Set motor A decay mode (0-slow, 1-fast)
    // DC channel B commands
    PERIPH_DC_B = PERIPH_DC + 0x20,
    PERIPH_DC_B_POWER,     // Set motor B spin power
    PERIPH_DC_B_BRAKE,     // Set motor B brake power
    PERIPH_DC_B_AUTOBRAKE, // Set motor B autobrake power [deprecated]
    PERIPH_DC_B_ENABLE,    // Set motor B enabled
    PERIPH_DC_B_INVERT,    // Set motor B inversion [deprecated]
    PERIPH_DC_B_DECAY,     // Set motor B decay mode (0-slow, 1-fast)
    // DC channel C commands
    PERIPH_DC_C = PERIPH_DC + 0x30,
    PERIPH_DC_C_POWER,     // Set motor C spin power
    PERIPH_DC_C_BRAKE,     // Set motor C brake power
    PERIPH_DC_C_AUTOBRAKE, // Set motor C autobrake power [deprecated]
    PERIPH_DC_C_ENABLE,    // Set motor C enabled
    PERIPH_DC_C_INVERT,    // Set motor C inversion [deprecated]
    PERIPH_DC_C_DECAY,     // Set motor C decay mode (0-slow, 1-fast)
    // DC channel D commands
    PERIPH_DC_D = PERIPH_DC + 0x40,
    PERIPH_DC_D_POWER,     // Set motor D spin power
    PERIPH_DC_D_BRAKE,     // Set motor D brake power
    PERIPH_DC_D_AUTOBRAKE, // Set motor D autobrake power [deprecated]
    PERIPH_DC_D_ENABLE,    // Set motor D enabled
    PERIPH_DC_D_INVERT,    // Set motor D inversion [deprecated]
    PERIPH_DC_D_DECAY,     // Set motor D decay mode (0-slow, 1-fast)
    // DC max channel
    PERIPH_DC_MAX = PERIPH_DC_D,
    // DC control commands
    PERIPH_DC_SET = PERIPH_DC + PERIPH_CONTROL_CHANNEL, // 0xA0
    PERIPH_DC_SET_ABCD_ENABLE,    // Enable/Disable DC motor peripheral | A | B | C | D |
    PERIPH_DC_SET_ABCD_INVERT,    // Set motors inversion | A | B | C | D | [deprecated]
    PERIPH_DC_SET_ABCD_AUTOBRAKE, // Set motors autobrake enabled | A | B | C | D | [deprecated]
    PERIPH_DC_SET_ABCD_POWER,     // Set motor ABCD spin power  | A | B | C | D |
    PERIPH_DC_SET_ABCD_BRAKE,     // Set motor ABCD brake power | A | B | C | D |
    // Motor ports AB and CD modes:
    // 0 - ports A, B, C, D are controller individually
    // 1 - ports AB or CD are tied together (writing one affects another)
    PERIPH_DC_SET_AB_MODE,        // (0-motor, 1-motor_group)
    PERIPH_DC_SET_CD_MODE,        // (0-motor, 1-motor_group)
    PERIPH_DC_SET_AB_FREQUENCY,   // Set channels AB PWM frequency
    PERIPH_DC_SET_CD_FREQUENCY,   // Set channels CD PWM frequency
    PERIPH_DC_SET_ABCD_FREQUENCY, // Set channels ABCD PWM frequency
    PERIPH_DC_SET_RESERVED,       // reserved
    PERIPH_DC_SET_AB_TONE,        // Play channels AB tone (duration | freq)
    PERIPH_DC_SET_CD_TONE,        // Play channels CD tone (duration | freq)
    PERIPH_DC_SET_ABCD_TONE,      // Play channels ABCD tone (duration | freq)
    PERIPH_DC_SET_RESERVED2,      // reserved
    PERIPH_DC_SET_ABCD_DECAY,     // Set channels ABCD decay | A | B | C | D | (decay(0-s,1-f))

    // RGB group
    PERIPH_RGB = 0x300, // Set group to 3
    // RGB channel commands mask
    PERIPH_RGB_X = PERIPH_RGB + 0x00,
    PERIPH_RGB_X_SET,         // 0x301
    PERIPH_RGB_X_SET_FADE,    // 0x302
    PERIPH_RGB_X_START_FADE,  // 0x303
    PERIPH_RGB_X_ENABLE,      // 0x304
    // RGB channel A commands
    PERIPH_RGB_A = PERIPH_RGB + 0x10,
    PERIPH_RGB_A_SET,         // Set RGB led A color and brightness
    PERIPH_RGB_A_SET_FADE,    // Set RGB led A fade color and brightness
    PERIPH_RGB_A_START_FADE,  // Start RGB led A fade animation
    PERIPH_RGB_A_ENABLE,      // Enable/Disable light A
    // RGB channel B commands
    PERIPH_RGB_B = PERIPH_RGB + 0x20,
    PERIPH_RGB_B_SET,         // Set RGB led B color and brightness
    PERIPH_RGB_B_SET_FADE,    // Set RGB led B fade color and brightness
    PERIPH_RGB_B_START_FADE,  // Start RGB led B fade animation
    PERIPH_RGB_B_ENABLE,      // Enable/Disable light B
    // RGB channel C commands
    PERIPH_RGB_C = PERIPH_RGB + 0x30,
    PERIPH_RGB_C_SET,         // Set RGB led C color and brightness
    PERIPH_RGB_C_SET_FADE,    // Set RGB led C fade color and brightness
    PERIPH_RGB_C_START_FADE,  // Start RGB led C fade animation
    PERIPH_RGB_C_ENABLE,      // Enable/Disable light C
    // RGB channel D commands
    PERIPH_RGB_D = PERIPH_RGB + 0x40,
    PERIPH_RGB_D_SET,         // Set RGB led D color and brightness
    PERIPH_RGB_D_SET_FADE,    // Set RGB led D fade color and brightness
    PERIPH_RGB_D_START_FADE,  // Start RGB led D fade animation
    PERIPH_RGB_D_ENABLE,      // Enable/Disable light D
    // RGB max channel
    PERIPH_RGB_MAX = PERIPH_RGB_D,
    // RGB control commands
    PERIPH_RGB_SET = PERIPH_RGB + PERIPH_CONTROL_CHANNEL, // 0xA0
    PERIPH_RGB_SET_ABCD_ENABLE,  // Enable/Disable RGB led peripheral | A | B | C | D |
    PERIPH_RGB_SET_ABCD_SET,     // Set all RGB leds color and brightness
    PERIPH_RGB_SET_ABCD_FADE,    // Set all RGB leds fade color and brightness
    PERIPH_RGB_SET_ABCD_START_FADE,  // Start all RGB fade animation

    // GPIO (RoboBoard X4 v1.0 only)
    PERIPH_GPIO = 0x400, // Set group to 4
    // GPIO channel commands mask
    PERIPH_GPIO_X = PERIPH_GPIO + 0x00,
    PERIPH_GPIO_X_MODE,           // 0x401
    PERIPH_GPIO_X_DIGITAL_READ,   // 0x402
    PERIPH_GPIO_X_DIGITAL_WRITE,  // 0x403
    PERIPH_GPIO_X_DIGITAL_DETECT, // 0x404 [removed]
    PERIPH_GPIO_X_ANALOG_READ,    // 0x405
    PERIPH_GPIO_X_ANALOG_WRITE,   // 0x406
    // GPIO channel A commands
    PERIPH_GPIO_A = PERIPH_GPIO + 0x10,
    PERIPH_GPIO_A_MODE,           // Configure pin mode. 0-pd,1-pu,2-float,3-out,4-analog
    PERIPH_GPIO_A_DIGITAL_READ,   // Read GPIO A pin state -> [LOW:HIGH]
    PERIPH_GPIO_A_DIGITAL_WRITE,  // Set GPIO A pin state -> [LOW:HIGH]
    PERIPH_GPIO_A_DIGITAL_DETECT, // [removed]
    PERIPH_GPIO_A_ANALOG_READ,    // Reads analog value of GPIO A pin -> [0:1023]
    PERIPH_GPIO_A_ANALOG_WRITE,   // Write analog value of GPIO A pin -> [0:20]
    // GPIO channel B commands
    PERIPH_GPIO_B = PERIPH_GPIO + 0x20,
    PERIPH_GPIO_B_MODE,           // Configure pin mode. 0-pd,1-pu,2-float,3-out,4-analog
    PERIPH_GPIO_B_DIGITAL_READ,   // Read GPIO B pin state -> [LOW:HIGH]
    PERIPH_GPIO_B_DIGITAL_WRITE,  // Set GPIO B pin state -> [LOW:HIGH]
    PERIPH_GPIO_B_DIGITAL_DETECT, // [removed]
    PERIPH_GPIO_B_ANALOG_READ,    // Reads analog value of GPIO B pin -> [0:1023]
    PERIPH_GPIO_B_ANALOG_WRITE,   // Write analog value of GPIO B pin -> [0:20]
    // GPIO channel C commands
    PERIPH_GPIO_C = PERIPH_GPIO + 0x30,
    PERIPH_GPIO_C_MODE,           // Configure pin mode. 0-pd,1-pu,2-float,3-out,4-analog
    PERIPH_GPIO_C_DIGITAL_READ,   // Read GPIO C pin state -> [LOW:HIGH]
    PERIPH_GPIO_C_DIGITAL_WRITE,  // Set GPIO C pin state -> [LOW:HIGH]
    PERIPH_GPIO_C_DIGITAL_DETECT, // [removed]
    PERIPH_GPIO_C_ANALOG_READ,    // Reads analog value of GPIO C pin -> [0:1023]
    PERIPH_GPIO_C_ANALOG_WRITE,   // Write analog value of GPIO C pin -> [0:20]
    // GPIO channel D commands
    PERIPH_GPIO_D = PERIPH_GPIO + 0x40,
    PERIPH_GPIO_D_MODE,           // Configure pin mode. 0-pd,1-pu,2-float,3-out,4-analog
    PERIPH_GPIO_D_DIGITAL_READ,   // Read GPIO D pin state -> [LOW:HIGH]
    PERIPH_GPIO_D_DIGITAL_WRITE,  // Set GPIO D pin state -> [LOW:HIGH]
    PERIPH_GPIO_D_DIGITAL_DETECT, // [removed]
    PERIPH_GPIO_D_ANALOG_READ,    // Reads analog value of GPIO D pin -> [0:1023]
    PERIPH_GPIO_D_ANALOG_WRITE,   // Write analog value of GPIO D pin -> [0:20]
    // GPIO max channel
    PERIPH_GPIO_MAX = PERIPH_GPIO_D,
} PeriphRegMap;

#define PeriphRegMap_group(command) (((command) >> 8) & 0xF)
#define PeriphRegMap_channel(command) (((command) >> 4) & 0xF)
#define PeriphRegMap_command(command) ((command) & 0xF)

#endif /* PRIV_INCLUDE_PERIPH_REGMAP */
