#ifndef _AMN1_SUITE_H_
#define _AMN1_SUITE_H_

#include "../inc/motion/amn1.h"

/*
 * Suite initialization function
 *
 * Returns zero on success, non-zero otherwise.
 */
int init_suite_amn1(void)
{
    printf("Init test suite amn1\n");
    if (amn1_init() != SUCCESS) {
        printf("SENSORNODE_ERROR: could not connect to AMN1.");
    }

    return 0;
}


/*
 * Suite cleanup function
 *
 * Returns zero on success, non-zero otherwise.
 */
int clean_suite_amn1(void)
{
    printf("Clean test suite amn1\n");

    return 0;
}

/*
 * TODO
 */
int test_read_motion(void)
{
    printf("-Test of reading motion value of amn1\n");
    amn1_rslt_t motion = amn1_read_motion();
    if (motion.error == ERROR)
    {
        printf("---AMN1_ERROR: Could not check for motion.\n");
        return 0;
    }

    if (motion.data != 1)
    {
        printf("---AMN1_ERROR: Wrong motion (expected 1 but got %d).\n", motion.data);
        return 0;
    }

    printf("---AMN1_SUCCESS: Has motion (bool): %d\n", motion.data);
    return 1;
}

#endif  /* _AMN1_SUITE_H_ */
