cmake_minimum_required(VERSION 3.13)
set(BOARD rpi_pico)

function(pico_add_extra_outputs)
  # Do nothing - this is a dummy function
endfunction()


OPTION(BUILD_TESTS "Build the test binary" OFF)
OPTION(BUILD_COVERAGE "Build the test binary to get coverage" OFF)
OPTION(BUILD_FIND_PARAMETERS "Build the test binary to find parameters" OFF)
if(BUILD_TESTS)
	project(sensor-node)
	set(CMAKE_C_STANDARD 11)

	add_subdirectory(inc)
	add_subdirectory(tests)

elseif(BUILD_COVERAGE)
	project(CMake_GCov C)

	# Set the compiler options
	set(CMAKE_C_STANDARD 11)
	set(CMAKE_C_FLAGS "-g -O0 -Wall -fprofile-arcs -ftest-coverage")
	set(CMAKE_C_OUTPUT_EXTENSION_REPLACE ON)
	
	# Create OBJECT_DIR variable
	set(OBJECT_DIR_BME680 ${CMAKE_BINARY_DIR}/CMakeFiles/RunCoverageBme680.dir)
	set(OBJECT_DIR_AMN1 ${CMAKE_BINARY_DIR}/CMakeFiles/RunCoverageAmn1.dir)
	set(OBJECT_DIR_DFR0034 ${CMAKE_BINARY_DIR}/CMakeFiles/RunCoverageDfr0034.dir)
	message("-- Object files will be output to: ${OBJECT_DIR_BME680}")
	message("                               and ${OBJECT_DIR_AMN1}")
	message("                               and ${OBJECT_DIR_DFR0034}")

	set(COVERAGE_DIR_BME680 inc/bosch)
	set(COVERAGE_DIR_AMN1 inc/motion)
	set(COVERAGE_DIR_DFR0034 inc/sound)
	
	set(SOURCES
		tests/unit-tests.c
		inc/bosch/bme680.c
    )
	add_executable(RunCoverageBme680 ${SOURCES})
	
	set(SOURCES
		tests/unit-tests.c
		inc/motion/amn1.c
    )
	add_executable(RunCoverageAmn1 ${SOURCES})

	set(SOURCES
		tests/unit-tests.c
    	inc/sound/dfr0034.c
    )
	add_executable(RunCoverageDfr0034 ${SOURCES})

	add_library(
		inc_bme680
		inc/bosch/bme680.h
		inc/bosch/bme680.c
	)

	add_library(
		inc_amn1
    	inc/motion/amn1.h
    	inc/motion/amn1.c
	)

	add_library(
		inc_dfr0034
    	inc/sound/dfr0034.h
    	inc/sound/dfr0034.c
	)

	add_library(
		mocks
    	tests/mocks.h
    	tests/mocks.c
	)

	target_link_libraries(RunCoverageBme680 PUBLIC inc_amn1 inc_dfr0034 mocks)
	target_link_libraries(RunCoverageAmn1 PUBLIC inc_bme680 inc_dfr0034 mocks)
	target_link_libraries(RunCoverageDfr0034 PUBLIC inc_bme680 inc_amn1 mocks)
	
	target_compile_definitions(inc_bme680 PUBLIC -D BUILD_TESTS)
	target_compile_definitions(inc_amn1 PUBLIC -D BUILD_TESTS)
	target_compile_definitions(inc_dfr0034 PUBLIC -D BUILD_TESTS)

	target_compile_definitions(RunCoverageBme680 PUBLIC -D BUILD_TESTS)
	target_compile_definitions(RunCoverageAmn1 PUBLIC -D BUILD_TESTS)
	target_compile_definitions(RunCoverageDfr0034 PUBLIC -D BUILD_TESTS)

	# Create the gcov target. Run coverage tests with 'make gcov'
	add_custom_target(gcov
	    COMMAND mkdir -p coverage
	    COMMAND ${CMAKE_MAKE_PROGRAM} test
	    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	    )
	add_custom_command(TARGET gcov
	    COMMAND echo "=================== GCOV ===================="
	    COMMAND echo "             ===== bme680 =====              "
	    COMMAND gcov -b ${CMAKE_SOURCE_DIR}/${COVERAGE_DIR_BME680}/*.c -o ${OBJECT_DIR_BME680}/${COVERAGE_DIR_BME680}
	        | grep -A 5 "bme680.c" > CoverageSummaryBme680.tmp
	    COMMAND cat CoverageSummaryBme680.tmp
	    COMMAND echo "-- Coverage files have been output to ${CMAKE_BINARY_DIR}/coverage"
	    COMMAND echo "              ===== amn1 =====               "
	    COMMAND gcov -b ${CMAKE_SOURCE_DIR}/${COVERAGE_DIR_AMN1}/*.c -o ${OBJECT_DIR_AMN1}/${COVERAGE_DIR_AMN1}
		| grep -A 5 "amn1.c" > CoverageSummaryAmn1.tmp
		COMMAND cat CoverageSummaryAmn1.tmp
	    COMMAND echo "             ===== dfr0034 =====             "
	    COMMAND gcov -b ${CMAKE_SOURCE_DIR}/${COVERAGE_DIR_DFR0034}/*.c -o ${OBJECT_DIR_DFR0034}/${COVERAGE_DIR_DFR0034}
		| grep -A 5 "dfr0034.c" > CoverageSummaryDfr0034.tmp
		COMMAND cat CoverageSummaryDfr0034.tmp
	    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/coverage  # Need separate command for this line
	)
	add_dependencies(gcov RunCoverageBme680)
	add_dependencies(gcov RunCoverageAmn1)
	add_dependencies(gcov RunCoverageDfr0034)

	# Make sure to clean up the coverage folder
	set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES coverage)
	
	# Create the gcov-clean target. This cleans the build as well as generated 
	# .gcda and .gcno files.
	add_custom_target(scrub
	    COMMAND ${CMAKE_MAKE_PROGRAM} clean
	    COMMAND rm -f ${OBJECT_DIR_BME680}/*.gcno
		COMMAND rm -f ${OBJECT_DIR_BME680}/*.gcda
	    COMMAND rm -f ${OBJECT_DIR_AMN1}/*.gcno
	    COMMAND rm -f ${OBJECT_DIR_AMN1}/*.gcda
	    COMMAND rm -f ${OBJECT_DIR_DFR0034}/*.gcno
	    COMMAND rm -f ${OBJECT_DIR_DFR0034}/*.gcda
	    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	    )
	
	# Testing
	enable_testing()
	
	add_test(output_bme680_test ${CMAKE_CURRENT_BINARY_DIR}/RunCoverageBme680)
	add_test(output_amn1_test ${CMAKE_CURRENT_BINARY_DIR}/RunCoverageAmn1)
	add_test(output_dfr0034_test ${CMAKE_CURRENT_BINARY_DIR}/RunCoverageDfr0034)


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

