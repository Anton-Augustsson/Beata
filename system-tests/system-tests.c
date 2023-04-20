#include <stdio.h>
#include "../sensor-node/tests/unit-test-suit-bme680.h"

int main() 
{

  int succeeded = 0;
  int tests = 0;

  init_suite_bme680();
  
  succeeded += test_read_temp();  tests++;
  succeeded += test_read_humidity();  tests++;
  succeeded += test_read_press();  tests++;

  printf("Succeeded: %d/%d\n", succeeded, tests);

  clean_suite_bme680();
}