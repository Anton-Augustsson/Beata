#ifndef _BME680_H_
#define _BME680_H_

#include <stdio.h>

typedef struct bme680_rslt {
    int32_t data;
    uint8_t error; // 1: no error
                   // 0: error
} bme680_rslt_t;

uint8_t bme680_init();
bme680_rslt_t bme680_read_temp();
bme680_rslt_t bme680_read_hum();
bme680_rslt_t bme680_read_press();

#endif /* _BME680_H_ */