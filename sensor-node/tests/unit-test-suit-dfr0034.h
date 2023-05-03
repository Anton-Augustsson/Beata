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
        printf("SENSORNODE_ERROR: could not connect to BME680.");
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

/*
 * TODO
 */
int test_read_sound(void) {
    printf("-Test for reading sound level of dfr0034\n");
    dfr0034_rslt_t sound = dfr0034_read_sound();

    if (sound.error == ERROR) {
        printf("---DFE0034_ERROR: Could not fetch sound level.");
        return 0;
    }

    if (sound.data != 100) {
        printf("---DFE0034_ERROR: Wrong sound level (expected 100 but got %d)", sound.data);
        return 0;
    }

    printf("---DFE0034_SUCCESS: Sound level: %d\n", sound.data);
    return 1;
}

#endif  /* _DFR0034_SUITE_H_ */
