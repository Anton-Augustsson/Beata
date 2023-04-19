#ifndef _BME680_H_
#define _BME680_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "bme680_reg.h"
#include "../common.h"

#ifdef BUILD_TESTS
#include "../../tests/mocks.h"
#endif

#ifndef BUILD_TESTS
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#endif

typedef struct bme680_rslt {
    int32_t data;
    error_t error;
} bme680_rslt_t;

error_t bme680_init();
bme680_rslt_t bme680_read_temp();
bme680_rslt_t bme680_read_hum();
bme680_rslt_t bme680_read_press();

#endif /* _BME680_H_ */
