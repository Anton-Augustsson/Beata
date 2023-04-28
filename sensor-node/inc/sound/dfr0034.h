#ifndef _DFR0034_H_
#define _DFR0034_H_

#include <stdio.h>
#include "../common.h"

#ifdef BUILD_TESTS
#include "../../tests/mocks.h"
#else
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#endif

typedef struct dfr0034_rslt
{
    uint16_t data;
    error_t error;
} dfr0034_rslt_t;

error_t dfr0034_init();
dfr0034_rslt_t dfr0034_read_sound();

#endif /* _DFR0034_H_ */
