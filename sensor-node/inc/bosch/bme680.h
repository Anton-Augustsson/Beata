#ifndef _BME680_H_
#define _BME680_H_

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "bme680_reg.h"

#ifdef BUILD_TESTS
#include "../../tests/mocks.h"
#endif

#ifndef BUILD_TESTS
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#endif

uint8_t bme680_init();
int32_t bme680_read_temp();
uint16_t bme680_read_hum();
uint16_t bme680_read_press();

#endif /* _BME680_H_ */