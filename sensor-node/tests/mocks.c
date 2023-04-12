#include "mocks.h"

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