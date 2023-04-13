#include "mocks.h"
#include "../inc/bosch/bme680_reg.h"

int reg_read;

int i2c_write_blocking(int i2c, int addr, uint8_t *reg, size_t len, bool option)
{
    reg_read = *reg;
    return 1;
}

int i2c_read_blocking(int i2c, int addr, uint8_t *buf, size_t len, bool option)
{
    switch(reg_read){    
        case BME680_TEMP_PAR_T1_LSB:    
            buf[0] = 243;
            break;
        case BME680_TEMP_PAR_T1_MSB:    
            buf[0] = 101;
            break;
        case BME680_TEMP_PAR_T2_LSB:    
            buf[0] = 85;
            break;
        case BME680_TEMP_PAR_T2_MSB:    
            buf[0] = 103;
            break;
        case BME680_TEMP_PAR_T3: 
            buf[0] = 3;
            break;
        case BME680_TEMP_ADC_MSB: 
            if (len != 3)
                break;
            
            buf[0] = 122;
            buf[1] = 125;
            buf[2] = 144;
            break;
        default:
            for (int i = 0; i < len; i++)
            {
               buf[i] = 1;
            }
            break;
    }


    // Need to write dummy value to buf
    // Should probably be able to change what value we will use like have 
    // A global varible that can be changed by the test function
    return 1;
}