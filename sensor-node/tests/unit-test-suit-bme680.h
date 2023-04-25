#ifndef _BME680_SUITE_H_
#define _BME680_SUITE_H_

#include "../inc/bosch/bme680.h"
#include "mocks.h"

#define TEMP_RESOLUTION 100
#define HUM_RESOLUTION 100
#define PRESS_RESOLUTION 1000


/*
 * Helper function to test read temperature
 */
int assert_read_temp(int correct_value)
{
  bme680_rslt_t temp_celsius = bme680_read_temp();
  if (temp_celsius.error == ERROR)
  {
      printf("---BME680_ERROR: Could not fetch temperature.\n");
      return 0;
  }

  if (temp_celsius.data != correct_value)
  {
    printf("---BME680_ERROR: Wrong temp (expected %d but got %d)\n", correct_value, temp_celsius.data);
    return 0;
  }

  printf("---BME680_SUCCESS: Temperature (C) is %d.%02d\n",
          temp_celsius.data / TEMP_RESOLUTION,
          temp_celsius.data % TEMP_RESOLUTION);
  return 1;
}

/*
 * Helper function to test read humidity
 */
int assert_read_humidity(int correct_value) 
{
  bme680_rslt_t humidity = bme680_read_hum();
  if (humidity.error == ERROR)
  {
    printf("---BME680_ERROR: Could not fetch humidity.\n");
    return 0;
  }

  // FIXME: wrong humidity, change when it is correct
  if (humidity.data != correct_value)
  {
    printf("---BME680_ERROR: Wrong humidity (expected %d but got %d)\n", correct_value, humidity.data);
    return 0;
  }

  printf("---BME680_SUCCESS: Humidity (%%) is %d.%02d\n", 
          humidity.data / HUM_RESOLUTION,
          humidity.data % HUM_RESOLUTION);
  return 1;
}

/*
 * Helper function to test read pressure
 */
int assert_read_pressure(int correct_value) 
{
  bme680_rslt_t press = bme680_read_press();
  if (press.error == ERROR)
  {
    printf("---BME680_ERROR: Could not fetch pressure.\n");
    return 0;
  }

  if (press.data != correct_value)
  {
    printf("---BME680_ERROR: Wrong press (expected %d but got %d)\n", correct_value, press.data);
    return 0;
  }

  printf("---BME680_SUCCESS: Pressure (hPa) is %d.%02d\n", 
          press.data / PRESS_RESOLUTION,
          press.data % PRESS_RESOLUTION);
  return 1;
}


/* 
 * Suite initialization function
 *
 * Returns zero on success, non-zero otherwise.
 */
int init_suite_bme680(void) 
{
  printf("Init test suite bme680\n");
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
  printf("Clean test suite bme680\n");

  return 0;
}

/* 
 * Read the temperature from bme680
 */
int test_read_temp(void) 
{
  printf("-Test of reading temperature value of bme680\n");
  set_reg_mode(Hot);
  return assert_read_temp(2654);
}

/* 
 * Read the humidity from bme680
 */
int test_read_humidity(void) 
{
  printf("-Test of reading humidity value of bme680\n");
  set_reg_mode(Humid);
  return assert_read_humidity(-224735);
}

/* 
 * Read the gas from bme680
 */
int test_read_press(void) 
{
  printf("-Test of reading pressure value of bme680\n");
  set_reg_mode(HighPress);
  return assert_read_pressure(94625);
}


#endif  /* _BME680_SUITE_H_ */