# Source files
set(srcs "roboboard_x4.c" "roboboard_x4.cpp")
# BSP components
list(APPEND srcs "components/bsp_adc_idf${IDF_VERSION_MAJOR}.c")
list(APPEND srcs "components/icm20689.c")
list(APPEND srcs "components/lsm6ds3.c")
list(APPEND srcs "components/periph_driver.c")
list(APPEND srcs "components/tbus.c")
# IDF components
set(required_components "driver")
list(APPEND required_components "nvs_flash")

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.0")
    list(APPEND required_components "esp_adc")
    list(APPEND required_components "esp_event")
    list(APPEND required_components "esp_timer")
else()
    list(APPEND required_components "esp_adc_cal")
endif()

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "include"
                    REQUIRES ${required_components})
