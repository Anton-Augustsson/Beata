#ifndef _COMMON_H_
#define _COMMON_H_

enum sensor_error_t {
    CONFIG_READ_ERROR,
    CONFIG_WRITE_ERROR,
    READ_ERROR,
    WRITE_ERROR,

    /* Success statuses below */
    CONFIG_SUCCESS,
    READ_SUCCESS,
    WRITE_SUCCESS
};
typedef enum sensor_error_t error_t;

#endif /* _COMMON_H_ */