cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(beata)

set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(PICO_BOARD pico)

set(path_bosch inc/bosch)
add_library(LibBme680 path_bosch/bme680.cpp path_bosch/bme680.hpp)

add_executable(
    beata main.c
)

pico_add_extra_outputs(beata)
target_link_libraries(
    beata
    pico_stdlib
    LibBme680
)
