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
        case BME680_HUM_PAR_H1_LSB || BME680_HUM_PAR_H2_LSB:
            buf[0] = 55;
            break;
        case BME680_HUM_PAR_H1_MSB:
            buf[0] = 67;
            break;
        case BME680_HUM_PAR_H2_MSB:
            buf[0] = 59;
            break;
        case BME680_HUM_PAR_H3:
            buf[0] = 0;
            break;
        case BME680_HUM_PAR_H4:
            buf[0] = 45;
            break;
        case BME680_HUM_PAR_H5:
            buf[0] = 20;
            break;
        case BME680_HUM_PAR_H6:
            buf[0] = 120;
            break;
        case BME680_HUM_PAR_H7:
            buf[0] = 156;
            break;
        case BME680_HUM_ADC_MSB:
            if (len != 2)
                break;
            buf[0] = 92;
            buf[1] = 213;
            break;
        case BME680_PRESS_PAR_P1_LSB:
            buf[0] = 244;
            break;
        case BME680_PRESS_PAR_P1_MSB:
            buf[0] = 143;
            break;
        case BME680_PRESS_PAR_P2_LSB:
            buf[0] = 217;
            break;
        case BME680_PRESS_PAR_P2_MSB:
            buf[0] = 215;
            break;
        case BME680_PRESS_PAR_P3:
            buf[0] = 88;
            break;
        case BME680_PRESS_PAR_P4_LSB:
            buf[0] = 6;
            break;
        case BME680_PRESS_PAR_P4_MSB:
            buf[0] = 43;
            break;
        case BME680_PRESS_PAR_P5_LSB:
            buf[0] = 215;
            break;
        case BME680_PRESS_PAR_P5_MSB:
            buf[0] = 254;
            break;
        case BME680_PRESS_PAR_P6:
            buf[0] = 30;
            break;
        case BME680_PRESS_PAR_P7:
            buf[0] = 48;
            break;
        case BME680_PRESS_PAR_P8_LSB:
            buf[0] = 132;
            break;
        case BME680_PRESS_PAR_P8_MSB:
            buf[0] = 242;
            break;
        case BME680_PRESS_PAR_P9_LSB:
            buf[0] = 37;
            break;
        case BME680_PRESS_PAR_P9_MSB:
            buf[0] = 247;
            break;
        case BME680_PRESS_PAR_P10:
            buf[0] = 30;
            break;
        case BME680_PRESS_ADC_MSB:
            if (len != 3)
                break;
            buf[0] = 65;
            buf[1] = 154;
            buf[2] = 224;
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