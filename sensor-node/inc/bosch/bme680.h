#include <stdio.h>

uint8_t bme680_init();
int32_t bme680_read_temp();
uint16_t bme680_read_hm();
uint16_t bme680_read_gas();