cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(sensor-node)

set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(PICO_BOARD pico)


add_executable(
    sensor-node main.c
)

pico_add_extra_outputs(sensor-node)
target_link_libraries(
    sensor-node
    pico_stdlib
)
