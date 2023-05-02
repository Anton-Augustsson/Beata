add_executable(SensorNode main.c)

pico_enable_stdio_uart(SensorNode 1)
target_link_libraries(SensorNode PUBLIC inc pico_i2c_slave hardware_i2c pico_stdlib)
pico_add_extra_outputs(SensorNode)