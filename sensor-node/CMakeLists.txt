cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(sensor-node)

set(CMAKE_C_STANDARD 11)
pico_sdk_init()

set(PICO_BOARD pico)


####### Bosch bme680 sensor ########

add_library(bme680_lib inc/bosch/bme680_reg.h inc/bosch/bme680.h inc/bosch/bme680.c)
target_link_libraries(
    bme680_lib
    pico_stdlib
    hardware_i2c
)


######## Main executable ########

add_executable(
    sensor-node main.c
)

pico_add_extra_outputs(sensor-node)
target_link_libraries(
    sensor-node
    bme680_lib
    pico_stdlib
)


######## Tests ########

add_library(bme680_test_lib tests/unit-test-suit-bme680.h tests/unit-test-suit-bme680.c)
target_link_libraries(
    bme680_test_lib
    bme680_lib
)

add_executable(
    tests tests/cunit-bme680.c
)

target_link_libraries(
    tests
    bme680_test_lib
)
