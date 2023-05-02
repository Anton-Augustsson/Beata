#ifndef _MOCKS_H_
#define _MOCKS_H_


#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

#define PICO_ERROR_GENERIC  -1
#define PICO_DEFAULT_I2C_SDA_PIN  1
#define PICO_DEFAULT_I2C_SCL_PIN 1
#define GPIO_FUNC_I2C 1

static int i2c_default;
static const int i2c0 = 0;
static const int i2c1 = 1;

int i2c_write_blocking(int i2c, int addr, uint8_t *reg, size_t len, bool option);
int i2c_read_blocking(int i2c, int addr, uint8_t *buf, size_t len, bool option);

typedef unsigned int alarm_id_t;

int adc_gpio_init(int gpio_pin);
int add_alarm_in_ms(int stability_time_ms, int64_t (*callback)(alarm_id_t, void*), uint8_t *null_value, bool option);
int adc_select_input(int adc_channel);
uint16_t adc_read();

// TODO: for can't read and can't write
enum reg_mode_t{Hot = 0, Cold = 1, Humid = 2, Dry = 3, HighPress = 4, 
                LowPress = 5, NormalReg = 6, InvalidReg = 7};
enum adc_mode_t{Loud = 0, Quiet = 1, Motion = 2, NoMotion = 3, 
                NormalAdc = 4, InvalidAdc = 5};

void set_reg_mode(enum reg_mode_t new_mode);
void set_adc_mode(enum adc_mode_t new_mode);
void set_reg_press(int value1, int value2, int value3, int mode);

int i2c_init(int i2c, int i2c_baudrate);
int gpio_set_function(int i2c_sda_pin, int gpio_func_i2c);
int gpio_pull_up(int i2c);

#endif /* _MOCKS_H_ */
