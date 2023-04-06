#include <stdbool.h>

#define PICO_ERROR_GENERIC  -1
#define PICO_DEFAULT_I2C_SDA_PIN  1
#define PICO_DEFAULT_I2C_SCL_PIN 1
#define GPIO_FUNC_I2C 1

static int i2c_default;

int i2c_write_blocking(int i2c, int addr, uint8_t *buf, size_t len, bool option)
{
    // Don't need to do anything
    return 1;
}

int i2c_read_blocking(int i2c, int addr, uint8_t *buf, size_t len, bool option)
{
    // Need to write dummy value to buf
    // Should probably be able to change what value we will use like have 
    // A global varible that can be changed by the test function
    for (int i = 0; i < len; i++) 
    {
        buf[i] = 1;
    }

    return 1;
}

int i2c_init(int i2c, int pin)
{
    return 1;
}

int gpio_set_function(int i2c_pin, int i2c_func)
{
    return 1;
}

int gpio_pull_up(int i2c_pin)
{
    return 1;
}

int bi_2pins_with_func(int i2c_pin_sda, int i2c_pin_scl, int i2c_func)
{
    return 1;
}

int bi_decl(int return_from_bi_func)
{
    return 1;
}