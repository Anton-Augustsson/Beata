add_executable(UnitTests unit-tests-bme680.c)

add_library(
    mocks
    ../tests/mocks.h
    ../tests/mocks.c
)

target_link_libraries(UnitTests PUBLIC inc mocks)