cmake_minimum_required(VERSION 3.13)

function(pico_add_extra_outputs)
  # Do nothing - this is a dummy function
endfunction()

set(SENSOR_NODE_DIR ../sensor-node)
set(SENSOR_NODE_TESTS_DIR "${SENSOR_NODE_DIR}/tests")
set(SENSOR_NODE_INC_DIR "${SENSOR_NODE_DIR}/inc")

set(DRIVER_DIR ../beata-driver)
set(DRIVER_TESTS_DIR "${SENSOR_NODE_DIR}/tests")
set(DRIVER_SRC_DIR "${SENSOR_NODE_DIR}/drivers/sensor/beata")


OPTION(BUILD_TESTS "Build the test binary" OFF)
if(BUILD_TESTS)
    project(system-tests)
    set(CMAKE_C_STANDARD 11)

    add_executable(SystemTests system-tests.c)

    add_library(
        mocks_sensor_node
        "${SENSOR_NODE_TESTS_DIR}/mocks.h"
        "${SENSOR_NODE_TESTS_DIR}/mocks.c"
    )

    add_library(
        inc
        "${SENSOR_NODE_INC_DIR}/bosch/bme680.h"
        "${SENSOR_NODE_INC_DIR}/bosch/bme680.c"
        "${SENSOR_NODE_INC_DIR}/motion/amn1.h"
        "${SENSOR_NODE_INC_DIR}/motion/amn1.c"
        "${SENSOR_NODE_INC_DIR}/sound/dfr0034.h"
        "${SENSOR_NODE_INC_DIR}/sound/dfr0034.c"
    )

    add_library(
        mocks_driver
        "${DRIVER_TESTS_DIR}/mocks.h"
        "${DRIVER_TESTS_DIR}/mocks.c"
    )

    #add_library(
    #    driver
    #    "${DRIVER_SRC_DIR}/beata.h"
    #    "${DRIVER_SRC_DIR}/beata.c"
    #)

    add_library(
        mocks_base_station
        ../base-station/tests/mocks.h
        ../base-station/tests/mocks.c
    )

    target_compile_definitions(inc PUBLIC -D BUILD_TESTS)
    #target_compile_definitions(driver PUBLIC -D BUILD_TESTS)

    target_link_libraries(SystemTests PUBLIC 
        inc 
        mocks_sensor_node
        mocks_driver
        #driver   TODO: write mock functions
        mocks_base_station
    )

    target_compile_options(SystemTests PRIVATE -g) 

endif()