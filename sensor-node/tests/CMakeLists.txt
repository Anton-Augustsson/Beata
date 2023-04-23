add_executable(UnitTests unit-tests.c)

add_library(
    mocks
    ../tests/mocks.h
    ../tests/mocks.c
)

target_link_libraries(UnitTests PUBLIC inc mocks)
target_compile_options(UnitTests PRIVATE -g)