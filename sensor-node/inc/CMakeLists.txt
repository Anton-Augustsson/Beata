add_library(
    inc
    "${CMAKE_CURRENT_SOURCE_DIR}/bosch/bme680.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/bosch/bme680.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion/amn1.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/motion/amn1.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/sound/dfr0034.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/sound/dfr0034.c"
)

target_include_directories(inc PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

if(BUILD_TESTS)
    target_compile_definitions(inc PUBLIC -D BUILD_TESTS)
elseif(BUILD_FIND_PARAMETERS)
    target_compile_definitions(inc PUBLIC -D BUILD_TESTS)
else()
    target_link_libraries(inc pico_stdlib hardware_i2c hardware_adc)
endif(BUILD_TESTS)
