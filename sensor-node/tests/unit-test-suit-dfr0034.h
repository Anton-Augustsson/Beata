#ifndef _DFR0034_SUITE_H_
#define _DFR0034_SUITE_H_

#include "../inc/sound/dfr0034.h"

/* 
 * Suite initialization function
 *
 * Returns zero on success, non-zero otherwise.
 */
int init_suite_dfr0034(void) 
{
  printf("Init test suite dfr0034\n");
  //if (bme680_init() != SUCCESS) {
  //      printf("SENSORNODE_ERROR: could not connect to BME680.");
  //}

  return 0;
}


/* 
 * Suite cleanup function
 *
 * Returns zero on success, non-zero otherwise.
 */
int clean_suite_dfr0034(void) 
{
  printf("Clean test suite dfr0034\n");

  return 0;
}

/* 
 * TODO
 */
int test_read_sound(void) 
{
  //printf("-Test of reading temperature value of bme680\n");
  //bme680_rslt_t temp_celsius = bme680_read_temp();
  //if (temp_celsius.error == ERROR)
  //{
  //    printf("---BME680_ERROR: Could not fetch temperature.");
  //    return 0;
  //}

  //if (temp_celsius.data != 2654)
  //{
  //  printf("---BME680_ERROR: Wrong temp");
  //  return 0;
  //}

  //printf("---BME680_SUCCESS: Temperature (C) is %d.%02d\n",
  //        temp_celsius.data / TEMP_RESOLUTION,
  //        temp_celsius.data % TEMP_RESOLUTION);
  return 1;
}

#endif  /* _DFR0034_SUITE_H_ */