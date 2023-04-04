cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(sensor-node)

set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(PICO_BOARD pico)

add_library(bme680_lib inc/bosch/bme680_reg.h inc/bosch/bme680.h inc/bosch/bme680.c)
target_link_libraries(
    bme680_lib
    pico_stdlib
    hardware_i2c
)

add_executable(
    sensor-node main.c
)

pico_add_extra_outputs(sensor-node)
target_link_libraries(
    sensor-node
    bme680_lib
    pico_stdlib
)
