#include "mocks.h"
#include "../inc/bosch/bme680_reg.h"

int reg_read;

int i2c_write_blocking(int i2c, int addr, uint8_t *reg, size_t len, bool option)
{
    if (*reg == BME680_TEMP_PAR_T1_LSB || 
        *reg == BME680_TEMP_PAR_T1_MSB || 
        *reg == BME680_TEMP_PAR_T2_LSB || 
        *reg == BME680_TEMP_PAR_T2_MSB || 
        *reg == BME680_TEMP_PAR_T3)
    {
        reg_read = *reg;
        return 1;
    }
    else 
    {
        // TODO: can add accrual write hear
        reg_read = *reg;
        return 1;
    }
}

int i2c_read_blocking(int i2c, int addr, uint8_t *buf, size_t len, bool option)
{
    switch(reg_read){    
        case BME680_TEMP_PAR_T1_LSB:    
            buf[0] = 243;
        case BME680_TEMP_PAR_T1_MSB:    
            buf[0] = 101;
        case BME680_TEMP_PAR_T2_LSB:    
            buf[0] = 85;
        case BME680_TEMP_PAR_T2_MSB:    
            buf[0] = 103;
        case BME680_TEMP_PAR_T3: 
            buf[0] = 3;
        case BME680_TEMP_ADC_MSB: 
            buf[0] = 122;
            buf[1] = 125;
            buf[2] = 144;
    }


    // Need to write dummy value to buf
    // Should probably be able to change what value we will use like have 
    // A global varible that can be changed by the test function
    return 1;
}