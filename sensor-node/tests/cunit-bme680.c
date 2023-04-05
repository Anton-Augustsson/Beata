/*
 *  Tests for bosch bme680 gas sensor
 *
 */

#include "../inc/bosch/bme680.h"
#include "unit-test-suit-bme680.h"


int main() 
{

  int succedded = 0;
  int tests = 0;

  init_suite_bme680();
  
  succedded += test_read_temp();  tests++;
  succedded += test_read_humidity();  tests++;
  succedded += test_read_gas();  tests++;

  printf("Succedded: %d/%d\n", succedded, tests);

  clean_suite_bme680();
}