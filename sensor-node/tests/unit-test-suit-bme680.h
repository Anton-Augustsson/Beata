#ifndef _BME680_SUITE_H_
#define _BME680_SUITE_H_

#include "../inc/bosch/bme680.h"


/* 
 * Suite initialization function
 *
 * Returns zero on success, non-zero otherwise.
 */
int init_suite_fileA(void);

/* 
 * Suite cleanup function
 *
 * Returns zero on success, non-zero otherwise.
 */
int clean_suite_fileA(void);

/* 
 * Read the temperature from bme680
 */
int test_read_temp(void);

/* 
 * Read the humidity from bme680
 */
int test_read_humidity(void);

/* 
 * Read the gas from bme680
 */
int test_read_gas(void);

#endif  /* _BME680_SUITE_H_ */