cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(lab1)

set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(PICO_BOARD pico)

add_executable(
    main.c
)

# pico_add_extra_outputs(lab1)
target_link_libraries(
    pico_stdlib
    hardware_pwm
)
