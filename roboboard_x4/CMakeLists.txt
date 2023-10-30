# Source files
set(srcs "roboboard_x4.c")
list(APPEND srcs "roboboard_x4_can.c")
list(APPEND srcs "roboboard_x4_imu.c")
# BSP libs
list(APPEND srcs "lib/icm20689.c")
list(APPEND srcs "lib/lsm6ds3.c")
list(APPEND srcs "lib/periph_driver.c")
# IDF components
set(components "driver")
# IDF version is less than IDF5.0
if("${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}" VERSION_LESS "5.0")
    list(APPEND srcs "roboboard_x4_idf4.c")
    list(APPEND components "esp_adc_cal")
else()
    list(APPEND srcs "roboboard_x4_idf5.c")
    list(APPEND components "esp_adc")
    list(APPEND components "esp_event")
    list(APPEND components "esp_timer")
endif()
# Register component
idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "include"
    REQUIRES ${components}
)
