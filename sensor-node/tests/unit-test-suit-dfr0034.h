#ifndef _DFR0034_SUITE_H_
#define _DFR0034_SUITE_H_

#include "../inc/sound/dfr0034.h"

/*
 * Suite initialization function
 *
 * Returns zero on success, non-zero otherwise.
 */
int init_suite_dfr0034(void) {
    printf("Init test suite dfr0034\n");

    if (dfr0034_init() != SUCCESS) {
        printf("SENSORNODE_ERROR: could not connect to BME680.\n");
    }

    return 0;
}


/*
 * Suite cleanup function
 *
 * Returns zero on success, non-zero otherwise.
 */
int clean_suite_dfr0034(void) {
    printf("Clean test suite dfr0034\n");
    return 0;
}

int assert_read_sound(int correct_value) {
    dfr0034_rslt_t sound = dfr0034_read_sound();

    if (sound.error == ERROR) {
        printf("---DFE0034_ERROR: Could not fetch sound level.\n");
        return 0;
    }

    if (sound.data != correct_value) {
        printf("---DFE0034_ERROR: Wrong sound level (expected %d but got %d)\n", correct_value, sound.data);
        return 0;
    }

    printf("---DFE0034_SUCCESS: Sound level: %d\n", sound.data);
    return 1;
}


int test_read_sound(void) {
    printf("-Test for reading sound level of dfr0034\n");
    /* Reading sound with normal sound levels */
    set_adc_mode(NormalAdc);
    assert_read_sound(100);
    /* Reading sound when it is high sound level */
    set_adc_mode(Loud);
    assert_read_sound(500);
    /* Reading sound when it is high sound level */
    set_adc_mode(Quiet);
    assert_read_sound(0);
    /* Reading sound when it is high sound level */
    set_adc_mode(InvalidAdc);
    assert_read_sound(65535); // FIXME: make sure the value was received is unsucessful
}

#endif  /* _DFR0034_SUITE_H_ */