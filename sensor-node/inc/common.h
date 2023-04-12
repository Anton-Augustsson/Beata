#ifndef _COMMON_H_
#define _COMMON_H_

#define ADC_CONVERSION_FACTOR ((const float)3.3f / (1 << 12))

enum sensor_error_t {
    SUCCESS,
    ERROR,
    NOT_READY,
};
typedef enum sensor_error_t error_t;

#endif /* _COMMON_H_ */
