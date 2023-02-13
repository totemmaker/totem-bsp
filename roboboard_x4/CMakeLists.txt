# Source files
set(srcs "roboboard_x4.c" "roboboard_x4.cpp")
# BSP drivers
list(APPEND srcs "src/bsp_adc_idf${IDF_VERSION_MAJOR}.c")
list(APPEND srcs "src/periph_driver.c")
# IDF components
set(required_components "driver")

if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_GREATER_EQUAL "5.0")
    list(APPEND required_components "esp_adc")
    list(APPEND required_components "esp_event")
    list(APPEND required_components "esp_timer")
else()
    list(APPEND required_components "esp_adc_cal")
endif()

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "include"
                    PRIV_INCLUDE_DIRS "priv_include"
                    REQUIRES ${required_components})
