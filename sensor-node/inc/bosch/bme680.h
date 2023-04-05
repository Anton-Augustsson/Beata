#ifndef _BME680_H_
#define _BME680_H_

#include <stdio.h>

uint8_t bme680_init();
int32_t bme680_read_temp();
uint16_t bme680_read_hum();
uint16_t bme680_read_gas();

#endif /* _BME680_H_ */