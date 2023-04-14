#ifndef _AMN1_H_
#define _AMN1_H_

#include <stdio.h>
#include <stdbool.h>
#include "../common.h"

#ifdef BUILD_TESTS
#include "../../tests/mocks.h"
#else
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#endif

typedef struct amn1_rslt {
    bool data;
    error_t error;
} amn1_rslt_t;

error_t amn1_init();
amn1_rslt_t amn1_read_motion();

#endif /* _AMN1_H_ */
