cmake_minimum_required(VERSION 3.13)
set(BOARD rpi_pico)

function(pico_add_extra_outputs)
  # Do nothing - this is a dummy function
endfunction()


OPTION(BUILD_TESTS "Build the test binary" OFF)
OPTION(BUILD_FIND_PARAMETERS "Build the test binary to find parameters" OFF)
if(BUILD_TESTS)
	project(CMake_GCov C)

	# Set the compiler options
	set(CMAKE_C_STANDARD 11)
	set(CMAKE_C_FLAGS "-g -O0 -Wall -fprofile-arcs -ftest-coverage")
	set(CMAKE_C_OUTPUT_EXTENSION_REPLACE ON)
	
	# Create OBJECT_DIR variable
	set(OBJECT_DIR ${CMAKE_BINARY_DIR}/CMakeFiles/RunAdder.dir)
	message("-- Object files will be output to: ${OBJECT_DIR}")
	set(COVERAGE_DIR inc/bosch)
	
	# Set the sources
	set(SOURCES
		tests/unit-tests.c
		inc/bosch/bme680.c
		#inc/motion/amn1.c
    	#inc/sound/dfr0034.c
    )

	# Create the executable
	add_executable(RunAdder ${SOURCES})
	
	add_library(
		inc
    	"inc/motion/amn1.h"
    	"inc/motion/amn1.c"
    	"inc/sound/dfr0034.h"
    	"inc/sound/dfr0034.c"
	)

	add_library(
		mocks
    	tests/mocks.h
    	tests/mocks.c
	)

	target_link_libraries(RunAdder PUBLIC inc mocks)
	
	target_compile_definitions(inc PUBLIC -D BUILD_TESTS)
	target_compile_definitions(RunAdder PUBLIC -D BUILD_TESTS)

	# Create the gcov target. Run coverage tests with 'make gcov'
	add_custom_target(gcov
	    COMMAND mkdir -p coverage
	    COMMAND ${CMAKE_MAKE_PROGRAM} test
	    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	    )
	add_custom_command(TARGET gcov
	    COMMAND echo "=================== GCOV ===================="
	    COMMAND gcov -b ${CMAKE_SOURCE_DIR}/${COVERAGE_DIR}/*.c -o ${OBJECT_DIR}/${COVERAGE_DIR}
	        | grep -A 5 "bme680.c" > CoverageSummary.tmp
	    COMMAND cat CoverageSummary.tmp
	    COMMAND echo "-- Coverage files have been output to ${CMAKE_BINARY_DIR}/coverage"
	    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/coverage  # Need separate command for this line
	)
	add_dependencies(gcov RunAdder)
	# Make sure to clean up the coverage folder
	set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES coverage)
	
	# Create the gcov-clean target. This cleans the build as well as generated 
	# .gcda and .gcno files.
	add_custom_target(scrub
	    COMMAND ${CMAKE_MAKE_PROGRAM} clean
	    COMMAND rm -f ${OBJECT_DIR}/*.gcno
	    COMMAND rm -f ${OBJECT_DIR}/*.gcda
	    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	    )
	
	# Testing
	enable_testing()
	
	add_test(output_test ${CMAKE_CURRENT_BINARY_DIR}/RunAdder)


elseif(BUILD_FIND_PARAMETERS)
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

