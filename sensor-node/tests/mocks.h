#ifndef _MOCKS_H_
#define _MOCKS_H_


#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#define PICO_ERROR_GENERIC  -1
#define PICO_DEFAULT_I2C_SDA_PIN  1
#define PICO_DEFAULT_I2C_SCL_PIN 1
#define GPIO_FUNC_I2C 1

static int i2c_default;

int i2c_write_blocking(int i2c, int addr, uint8_t *buf, size_t len, bool option);

int i2c_read_blocking(int i2c, int addr, uint8_t *buf, size_t len, bool option);


#endif /* _MOCKS_H_ */