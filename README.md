# Totem Board Support Package

Dependencies and low-level C drivers, required for RoboBoard programming using ESP-IDF framework.

Check setup guide at: [https://docs.totemmaker.net/setup/](https://docs.totemmaker.net/setup/)

## Supported boards
| Board name | SoC | Revision | Features | Photo |
| --- | --- | :-: | --- | --- |
| [RoboBoard X3](https://docs.totemmaker.net/roboboard-x3/) | ESP32 | v.3.0 | 4 Motor, 2 Servo<br>3 IO, RGB, IMU, Qwiic<br> 3.7V battery | <img src="https://docs.totemmaker.net/assets/images/photo/roboboard-x3-v3.0-card.jpg" width="150"> |
| [RoboBoard X4](https://docs.totemmaker.net/roboboard-x4/) | ESP32 | v.1.0<br>v.1.1 | 4 Motor, 3 Servo<br>4 IO, RGB, IMU, Qwiic, CAN<br> 11.1V battery | <a href="https://totemmaker.net/product/roboboard-x4-power-adapter-battery/"><img src="https://docs.totemmaker.net/assets/images/photo/roboboard-x4-v1.1-photo.jpg" width="150"></a> |

### Supported ESP-IDF versions

* ESP-IDF v4.4
* ESP-IDF v5.0
* ESP-IDF v5.1
* ESP-IDF v5.2

## How to use

Install Espressif toolchain: [Getting started](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)

1. Create new ESP-IDF project or use command:  
`idf.py create-project roboboard_project`
1. Inside project create manifest file `main/idf_component.yml` or use command:  
`idf.py create-manifest`  
1. Add `roboboard_x3` or `roboboard_x4` as dependency:  
```yml
## IDF Component Manager Manifest File
dependencies:
  # Include roboboard_x3 or roboboard_x4
  roboboard_x4:
    path: roboboard_x4
    git: https://github.com/totemmaker/totem-bsp.git
```
4. Include header into project `roboboard_project.c` file:  
```c
#include "bsp/totem-bsp.h"

void app_main(void)
{
    // Initialize board
    bsp_board_init();
    // Spin motor A at 100% power
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_A, 100);
    // Spin servo A to 500us pulse
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_A, 500);
    // RoboBoard X4 LED on
    // bsp_cmd_write(BSP_LED_STATE, 0, 1);
    // bsp_cmd_write(BSP_RGB_COLOR, BSP_PORT_ALL, 0xFF00FF00);
}
```
5. Build project:  
`idf.py build`

## Documentation

Information for function usage and board features:

* RoboBoard X3
  * [roboboard_x3.h](https://github.com/totemmaker/totem-bsp/blob/master/roboboard_x3/include/bsp/roboboard_x3.h)
  * [imu.h](https://github.com/totemmaker/totem-bsp/blob/master/roboboard_x3/include/bsp/imu.h)
  * [rgb.h](https://github.com/totemmaker/totem-bsp/blob/master/roboboard_x3/include/bsp/rgb.h)
* RoboBoard X4
  * [roboboard_x4.h](https://github.com/totemmaker/totem-bsp/blob/master/roboboard_x4/include/bsp/roboboard_x4.h)
  * [imu.h](https://github.com/totemmaker/totem-bsp/blob/master/roboboard_x4/include/bsp/imu.h)
  * [can.h](https://github.com/totemmaker/totem-bsp/blob/master/roboboard_x4/include/bsp/can.h)

Example code located in [examples](https://github.com/totemmaker/totem-bsp/tree/master/examples).

## Contacts

To report issue, ask for feature or general discussion:

* Create [Github issue](https://github.com/totemmaker/totem-bsp/issues/new)
* Discuss in our [forum.totemmaker.net](https://forum.totemmaker.net)
* Contact our support [https://totemmaker.net/contacts/](https://totemmaker.net/contacts/)