#ifndef _BME680_SUITE_H_
#define _BME680_SUITE_H_

#include "../inc/bosch/bme680.h"

#define TEMP_RESOLUTION 100
#define HUM_RESOLUTION 100
#define PRESS_RESOLUTION 1000
#define I2C_FREQUENCY 400000
#define SENSOR_QUERY_PERIOD_MS 3000
#define INIT_RETRY_DELAY_MS (SENSOR_QUERY_PERIOD_MS / 6)


/* 
 * Suite initialization function
 *
 * Returns zero on success, non-zero otherwise.
 */
int init_suite_bme680(void) 
{
  printf("Init test suit bme680\n");
  if (bme680_init() != SUCCESS) {
        printf("SENSORNODE_ERROR: could not connect to BME680.");
  }


  return 0;
}


/* 
 * Suite cleanup function
 *
 * Returns zero on success, non-zero otherwise.
 */
int clean_suite_bme680(void) 
{
  printf("Clean test suit bme680\n");

  return 0;
}

/* 
 * Read the temperature from bme680
 */
int test_read_temp(void) 
{
  printf("-Test of reading temperature value of bme680\n");
  bme680_rslt_t temp_celsius = bme680_read_temp();
  if (temp_celsius.error == ERROR)
  {
      printf("---BME680_ERROR: Could not fetch temperature.");
      return 0;
  }

  printf("---BME680_SUCCESS: the temp is %d.\n", temp_celsius.data);
  return 1;
}

/* 
 * Read the humidity from bme680
 */
int test_read_humidity(void) 
{
  printf("-Test of reading humidity value of bme680\n");

  bme680_rslt_t humidity = bme680_read_hum();
  if (humidity.error == ERROR)
  {
    printf("---BME680_ERROR: Could not fetch humidity.");
    return 0;
  }

  printf("---BME680_SUCCESS: the bumidity is %d.\n", humidity.data);
  return 1;
}

/* 
 * Read the gas from bme680
 */
int test_read_gas(void) 
{
  printf("-Test of reading gas value of bme680\n");

  bme680_rslt_t press = bme680_read_press();
  if (press.error == ERROR)
  {
    printf("---BME680_ERROR: Could not fetch pressure.");
    return 0;
  }

  printf("---BME680_SUCCESS: the press is %d.\n", press.data);
  return 1;
}


#endif  /* _BME680_SUITE_H_ */