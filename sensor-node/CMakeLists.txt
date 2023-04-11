cmake_minimum_required(VERSION 3.13)
set(BOARD rpi_pico)

function(pico_add_extra_outputs)
  # Do nothing - this is a dummy function
endfunction()


OPTION(BUILD_TESTS "Build the test binary" OFF)
if(BUILD_TESTS)
  project(sensor-node)
  set(CMAKE_C_STANDARD 11)

  add_subdirectory(inc)
  add_subdirectory(tests)

else()
  include(pico_sdk_import.cmake)

  project(sensor-node)  # project has to be after include
  set(CMAKE_C_STANDARD 11)

  pico_sdk_init()
  set(PICO_BOARD pico)

  add_subdirectory(inc)
  add_subdirectory(src)
endif()