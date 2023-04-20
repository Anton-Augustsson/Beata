#include "mocks.h"
#include "../inc/bosch/bme680_reg.h"
#include "../inc/common.h"

int reg_read = -1;
int channel = -1;

int set_buf_3(uint8_t *buf, size_t len, uint8_t value1, uint8_t value2, uint8_t value3)
{
    if (len == 3) 
    {
        buf[0] = value1;
        buf[1] = value2;
        buf[2] = value3;
        reg_read = -1;
        return 1;
    }

    return -1;
}
int set_buf_2(uint8_t *buf, size_t len, uint8_t value1, uint8_t value2)
{
    if (len == 2) 
    {
        buf[0] = value1;
        buf[1] = value2;
        reg_read = -1;
        return 1;
    }

    return -1;
}
int set_buf_1(uint8_t *buf, size_t len, uint8_t value1)
{
    if (len == 1) 
    {
        buf[0] = value1;
        reg_read = -1;
        return 1;
    }

    return -1;
}

int i2c_write_blocking(int i2c, int addr, uint8_t *reg, size_t len, bool option)
{
    reg_read = *reg;
    return 1;
}

int i2c_read_blocking(int i2c, int addr, uint8_t *buf, size_t len, bool option)
{
    switch(reg_read){    
        case BME680_TEMP_PAR_T1_LSB:    
            return set_buf_1(buf, len, 243);
        case BME680_TEMP_PAR_T1_MSB:    
            return set_buf_1(buf, len, 101);
        case BME680_TEMP_PAR_T2_LSB:    
            return set_buf_1(buf, len, 85);
        case BME680_TEMP_PAR_T2_MSB:    
            return set_buf_1(buf, len, 103);
        case BME680_TEMP_PAR_T3: 
            return set_buf_1(buf, len, 3);
        case BME680_TEMP_ADC_MSB: 
            return set_buf_3(buf, len, 122, 125, 144);
        case BME680_HUM_PAR_H1_LSB: // same as BME680_HUM_PAR_H2_LSB
            return set_buf_1(buf, len, 55);
        case BME680_HUM_PAR_H1_MSB:
            return set_buf_1(buf, len, 67);
        case BME680_HUM_PAR_H2_MSB:
            return set_buf_1(buf, len, 59);
        case BME680_HUM_PAR_H3:
            return set_buf_1(buf, len, 0);
        case BME680_HUM_PAR_H4:
            return set_buf_1(buf, len, 45);
        case BME680_HUM_PAR_H5:
            return set_buf_1(buf, len, 20);
        case BME680_HUM_PAR_H6:
            return set_buf_1(buf, len, 120);
        case BME680_HUM_PAR_H7:
            return set_buf_1(buf, len, 156);
        case BME680_HUM_ADC_MSB:
            return set_buf_2(buf, len, 92, 213);
        case BME680_PRESS_PAR_P1_LSB:
            return set_buf_1(buf, len, 244);
        case BME680_PRESS_PAR_P1_MSB:
            return set_buf_1(buf, len, 143);
        case BME680_PRESS_PAR_P2_LSB:
            return set_buf_1(buf, len, 217);
        case BME680_PRESS_PAR_P2_MSB:
            return set_buf_1(buf, len, 215);
        case BME680_PRESS_PAR_P3:
            return set_buf_1(buf, len, 88);
        case BME680_PRESS_PAR_P4_LSB:
            return set_buf_1(buf, len, 6);
        case BME680_PRESS_PAR_P4_MSB:
            return set_buf_1(buf, len, 43);
        case BME680_PRESS_PAR_P5_LSB:
            return set_buf_1(buf, len, 215);
        case BME680_PRESS_PAR_P5_MSB:
            return set_buf_1(buf, len, 254);
        case BME680_PRESS_PAR_P6:
            return set_buf_1(buf, len, 30);
        case BME680_PRESS_PAR_P7:
            return set_buf_1(buf, len, 48);
        case BME680_PRESS_PAR_P8_LSB:
            return set_buf_1(buf, len, 132);
        case BME680_PRESS_PAR_P8_MSB:
            return set_buf_1(buf, len, 242);
        case BME680_PRESS_PAR_P9_LSB:
            return set_buf_1(buf, len, 37);
        case BME680_PRESS_PAR_P9_MSB:
            return set_buf_1(buf, len, 247);
        case BME680_PRESS_PAR_P10:
            return set_buf_1(buf, len, 30);
        case BME680_PRESS_ADC_MSB:
            return set_buf_3(buf, len, 65, 154, 224);
        case BME680_ID:
            return set_buf_1(buf, len, 1);
        default: // will go to default if reg_read == -1 
            printf("ERROR: Reading a unknown register (%d).\n", reg_read);
            return -1;
    }
}


int adc_gpio_init(int gpio_pin)
{
    return 1;
}

int add_alarm_in_ms(int stability_time_ms, int64_t (*callback)(alarm_id_t, void*), uint8_t *null_value, bool option)
{
    callback(1, NULL);
    return 1;
}

int adc_select_input(int adc_channel)
{
    switch (adc_channel)
    {
    case AMN1_ADC_CHANNEL:
        channel = AMN1_ADC_CHANNEL;
        return 1;
    
    case DFR0034_ADC_CHANNEL:
        channel = DFR0034_ADC_CHANNEL;
        return 1;

    default:
        printf("ERROR: unknown adc channel (%d).\n", adc_channel);
        return -1;
    }
}

uint16_t adc_read()
{
    switch (channel)
    {
    case AMN1_ADC_CHANNEL:
        channel = -1;
        return 4/ADC_CONVERSION_FACTOR;
    
    case DFR0034_ADC_CHANNEL:
        channel = -1;
        return 100;

    default:
        printf("ERROR: reading a unknown adc channel (%d).\n", channel);
        channel = -1;
        return 0;
    }
}