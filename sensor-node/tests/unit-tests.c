/*
 *  Tests for bosch bme680 gas sensor
 *
 */

#include "unit-test-suit-bme680.h"
#include "unit-test-suit-amn1.h"
#include "unit-test-suit-dfr0034.h"


int main() {
    int succeeded = 0;
    int tests = 0;
    init_suite_bme680();
    init_suite_amn1();
    init_suite_dfr0034();
    succeeded += test_read_temp();
    tests++;
    succeeded += test_read_humidity();
    tests++;
    succeeded += test_read_press();
    tests++;
    succeeded += test_read_sound();
    tests++;
    succeeded += test_read_motion();
    tests++;
    printf("Succeeded: %d/%d\n", succeeded, tests);
    clean_suite_bme680();
    clean_suite_amn1();
    clean_suite_dfr0034();
}
