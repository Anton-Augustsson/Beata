add_executable(UnitTests unit-tests.c)

add_library(
    mocks
    ../tests/mocks.h
    ../tests/mocks.c
)

target_link_libraries(UnitTests PUBLIC inc mocks)
target_compile_options(UnitTests PRIVATE -g)

if(BUILD_FIND_PARAMETERS)
    target_compile_definitions(UnitTests PUBLIC -D BUILD_FIND_PARAMETERS)
endif(BUILD_FIND_PARAMETERS)

