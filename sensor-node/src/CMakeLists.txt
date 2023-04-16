add_executable(SensorNode main.c)

pico_enable_stdio_uart(SensorNode 1)
pico_add_extra_outputs(SensorNode)
target_link_libraries(SensorNode PUBLIC inc pico_stdlib)