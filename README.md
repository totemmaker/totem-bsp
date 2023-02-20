# Totem Board Support Package

Components for using Totem boards with ESP-IDF framework.  
Contains C and C++ interface to control specific board features.  
Allows to develop a standalone project or integrate into existing applications.

## Supported boards
| Board name | SoC | Features | Photo |
| --- | --- | --- | --- |
| [RoboBoard X4](roboboard_x4) | ESP32 | Motor, Servo, GPIO, RGB, MEMS, TBUS, Qwiic, 12V | <a href="https://totemmaker.net/product/roboboard-x4-power-adapter-battery/"><img src="https://docs.totemmaker.net/assets/images/photo/roboboard-x4-v1.1-photo.jpg" width="150"></a> |

### Supported IDF versions

* v4.4
* v5.0

## How to use

Read [Getting started](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html) to setup Espressif toolchain.

This package is intended for use with a [IDF Component Manager](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-component-manager.html), a tool to automatically download specified dependencies. It works by adding configuration file with a list of components required. Build system will check if this file is present and attempts to download listed components.  
Manifest file `idf_component.yml` can be created using command:  
`idf.py create-manifest`  

To add RoboBoard X4 component into ESP-IDF project:  
Include `roboboard_x4` into `dependencies`  
```yml
## IDF Component Manager Manifest File
dependencies:
  ## Required IDF version
  idf:
    version: ">=4.4.0"
  # # Put list of dependencies here
  # # For components maintained by Espressif:
  # component: "~1.0.0"
  # # For 3rd party components:
  totemmaker/bsp:
    path: roboboard_x4
    git: https://github.com/totemmaker/totem-bsp
  # username/component: ">=1.0.0,<2.0.0"
  # username2/component2:
  #   version: "~1.0.0"
  #   # For transient dependencies `public` flag can be set.
  #   # `public` flag doesn't have an effect dependencies of the `main` component.
  #   # All dependencies of `main` are public by default.
  #   public: true
```
_Or simply create `main/idf_component.yml` file with content above._

Include header into project `main.c` file:  
```C
#include "bsp/roboboard_x4.h" // Board control functions

void app_main(void)
{
    bsp_board_init(); // Must be called to initialize board
}
```

## Step-by-step guide

* Create project directory  
`idf.py create-project test_project`  
* Change directory  
`cd test_project`
* Create manifest file  
`idf.py create-manifest`  
* Add dependency to `main/idf_component.yml`  
```yml
dependencies:
  totemmaker/bsp:
    path: roboboard_x4
    git: https://github.com/totemmaker/totem-bsp
```   
* Include header to `main/test_project.c` and call `bsp_board_init()`  
```C
#include "bsp/roboboard_x4.h"
```
* Build project  
`idf.py build`

## Documentation

Information for function usage and board features:

* For C read [roboboard_x4.h](https://github.com/totemmaker/totem-bsp/blob/master/roboboard_x4/include/bsp/roboboard_x4.h) header file
* For C++ read [roboboard_x4.hpp](https://github.com/totemmaker/totem-bsp/blob/master/roboboard_x4/include/bsp/roboboard_x4.hpp) header file or website [https://docs.totemmaker.net/roboboard-x4/](https://docs.totemmaker.net/roboboard-x4/)

## Run C example project

Use low level functions `bsp_cmd_write` and `bsp_cmd_read` to control board features.

* Clone totem-bsp repository  
`git clone https://github.com/totemmaker/totem-bsp/`  
* Change directory into example project  
`cd totem-bsp/examples/roboboard_x4_led_blink`  
* Build and flash project
`idf.py build flash`  

_Minimum example code:_
```cpp
#include "bsp/roboboard_x4.h"
void app_main(void)
{
    // Initialize board components
    bsp_board_init();
    // Turn LED on
    bsp_cmd_write(BSP_LED_STATE, 0, 1);
    // Write servo port A to output 500us pulse
    bsp_cmd_write(BSP_SERVO_PULSE, BSP_PORT_A, 500);
}
```

## Run C++ example project

Use high level C++ API to control components. Contains extensive and user friendly interface.  
Low level functions can be used concurrently if `roboboard_x4.h` header is included.

* Clone totem-bsp repository  
`git clone https://github.com/totemmaker/totem-bsp/`  
* Change directory into example project  
`cd totem-bsp/examples/roboboard_x4_led_blink_cpp`  
* Build and flash project
`idf.py build flash`  

_Minimum example code:_
```cpp
#include "bsp/roboboard_x4.hpp"
extern "C" void app_main(void)
{
    // Initialize board components
    X4.begin();
    // Turn LED on
    X4.led.on();
    // Write servo port A to output 500us pulse
    X4.servoA.pulse(500);
}
```

## Using both C and C++

Example project: `cd totem-bsp/examples/roboboard_x4_led_blink_mix`
```cpp
#include "bsp/roboboard_x4.h"
#include "bsp/roboboard_x4.hpp"
extern "C" void app_main(void)
{
    // Initialize board components
    X4.begin();
    // Turn LED on
    X4.led.on();
    // Write servo port A to output 500us pulse
    X4.servoA.pulse(500);
    // Write DC port A to 80% power
    bsp_cmd_write(BSP_DC_POWER, BSP_PORT_A, 80);
}
```

## Structure

* `examples` - Examples of using package with ESP-IDF
* `roboboard_x4` - RoboBoard X4 BSP package
* `test` - Unit tests
* `LICENSE` - License
* `README.md` - This file


## Contacts

To report issue, ask for feature or general discussion:

* Create [Github issue](https://github.com/totemmaker/totem-bsp/issues/new)
* Discuss in our [forum.totemmaker.net](https://forum.totemmaker.net)
* Contact our support [https://totemmaker.net/contacts/](https://totemmaker.net/contacts/)