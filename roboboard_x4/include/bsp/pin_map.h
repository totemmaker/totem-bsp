/* 
 * Copyright 2023 Totem Technology, UAB
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */
#ifndef INCLUDE_BSP_PIN_MAP
#define INCLUDE_BSP_PIN_MAP

/**************************************************************************************************
 * Totem RoboBoard X4 pinout
 **************************************************************************************************/

/* I2C */
#define BSP_IO_I2C_SDA           (GPIO_NUM_21)
#define BSP_IO_I2C_SCL           (GPIO_NUM_22)

/* CAN */
#define BSP_IO_CAN_TX            (GPIO_NUM_17)
#define BSP_IO_CAN_RX            (GPIO_NUM_34)
#define BSP_IO_CAN_EN            (GPIO_NUM_5)  // v1.1 only

/* GPIO */
#define BSP_IO_GPIOA             (GPIO_NUM_14) // v1.1 only
#define BSP_IO_GPIOB             (GPIO_NUM_23) // v1.1 only
#define BSP_IO_GPIOC             (GPIO_NUM_25) // v1.1 only
#define BSP_IO_GPIOD             (GPIO_NUM_26) // v1.1 only

/* Driver */
#define BSP_IO_DRIVER_DFU        (GPIO_NUM_4)
#define BSP_IO_DRIVER_RESET      (GPIO_NUM_15)
#define BSP_IO_DRIVER_UART_RX    (GPIO_NUM_16)
#define BSP_IO_DRIVER_UART_TX    (GPIO_NUM_27)

/* Others */
#define BSP_IO_LED               (GPIO_NUM_13)
#define BSP_IO_BUTTON            (GPIO_NUM_18)
#define BSP_IO_DC_DETECT         (GPIO_NUM_19)
#define BSP_IO_USB_DETECT        (GPIO_NUM_39)
#define BSP_IO_BATTERY_CHARGE    (GPIO_NUM_33)
#define BSP_IO_BATTERY_VOLTAGE   (GPIO_NUM_36)
#define BSP_IO_ACCEL_INT         (GPIO_NUM_35) // v1.1 only
#define BSP_IO_ACCEL_INT_v10     (GPIO_NUM_14) // v1.0 only

#endif /* INCLUDE_BSP_PIN_MAP */
