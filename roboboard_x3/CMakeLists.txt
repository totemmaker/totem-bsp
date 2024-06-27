# Source files
set(srcs "roboboard_x3.c")
list(APPEND srcs "roboboard_x3_imu.c")
list(APPEND srcs "roboboard_x3_rgb.c")
list(APPEND srcs "roboboard_x3_idf4.c")
list(APPEND srcs "roboboard_x3_idf5.c")
# BSP libs
list(APPEND srcs "lib/qmi8658.c")
# IDF components
set(components "driver")
# IDF version is less than IDF5.0
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.0")
    list(APPEND components "esp_adc_cal")
else()
    list(APPEND components "esp_adc")
    list(APPEND components "esp_event")
    list(APPEND components "esp_timer")
endif()
list(APPEND components "led_strip")
# Register component
idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES ${components}
)
