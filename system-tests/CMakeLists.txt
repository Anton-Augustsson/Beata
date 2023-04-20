cmake_minimum_required(VERSION 3.13)

function(pico_add_extra_outputs)
  # Do nothing - this is a dummy function
endfunction()


OPTION(BUILD_TESTS "Build the test binary" OFF)
if(BUILD_TESTS)
    project(system-tests)
    set(CMAKE_C_STANDARD 11)

    add_executable(SystemTests system-tests.c)

    add_library(
        mocks
        ../sensor-node/tests/mocks.h
        ../sensor-node/tests/mocks.c
    )

    #include_directories(../sensor-node/inc)
    add_library(
        inc
        ../sensor-node/inc/bosch/bme680.h
        ../sensor-node/inc/bosch/bme680.c
        ../sensor-node/inc/motion/amn1.h
        ../sensor-node/inc/motion/amn1.c
        ../sensor-node/inc/sound/dfr0034.h
        ../sensor-node/inc/sound/dfr0034.c
    )

    target_compile_definitions(inc PUBLIC -D BUILD_TESTS)

    target_link_libraries(SystemTests PUBLIC inc mocks)
    target_compile_options(SystemTests PRIVATE -g) 

endif()