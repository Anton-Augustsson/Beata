#include "../inc/bosch/bme680.h"


int init_suite_bme680(void) 
{
  printf("Init test suit bme680\n");

  return 0;
}


int clean_suite_bme680(void) 
{
  printf("Clean test suit bme680\n");

  return 0;
}


int test_read_temp(void) 
{
  printf("test of reading temperature value of bme680\n");

  // TODO: create real test cases
  if ( 1 == 1 ) { return 1; }
  else { return 0; }
}


int test_read_humidity(void) 
{
  printf("test of reading humidity value of bme680\n");

  // TODO: create real test cases
  if ( 1 == 1 ) { return 1; }
  else { return 0; }
}

int test_read_gas(void) 
{
  printf("test of reading gas value of bme680\n");

  // TODO: create real test cases
  if ( 1 == 1 ) { return 1; }
  else { return 0; }
}