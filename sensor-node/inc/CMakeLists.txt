add_library(
    inc
    "${CMAKE_CURRENT_SOURCE_DIR}/bosch/bme680.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/bosch/bme680.c"
)

target_include_directories(inc PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

if(BUILD_TESTS)
    target_compile_definitions(inc PUBLIC -D BUILD_TESTS)
    target_link_libraries(inc)
else()
    target_link_libraries(inc pico_stdlib hardware_i2c)
endif(BUILD_TESTS)