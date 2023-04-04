#include <stdio.h>
#include "pico/stdlib.h"
#include "bme680_reg.h"

void bme680_init();
int16_t bme680_read_temp();
uint16_t bme680_read_hum();
uint16_t bme680_read_gas();