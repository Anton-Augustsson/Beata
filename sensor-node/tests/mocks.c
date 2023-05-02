#include "mocks.h"
#include "../inc/bosch/bme680_reg.h"
#include "../inc/common.h"

int reg_read = -1;
int channel = -1;
static enum reg_mode_t reg_mode = NormalReg;

struct reg_value1 {
    int reg;
    uint8_t value[7][1];
};

struct reg_value2 {
    int reg;
    uint8_t value[7][2];
};

struct reg_value3 {
    int reg;
    uint8_t value[7][3];
};

struct values {
    uint8_t *value;
    int success;
};


#define REGS_VALUES1_SIZE 30
#define REGS_VALUES2_SIZE 1
#define REGS_VALUES3_SIZE 2
#define ERROR_VALUE1 {{0}}
#define ERROR_VALUE2 {{0, 0}}
#define ERROR_VALUE3 {{0, 0, 0}}


// TODO: fix const
struct reg_value1 regs_values1[REGS_VALUES1_SIZE] = {
    (struct reg_value1){BME680_TEMP_PAR_T1_LSB, {{243}, {243}, {243}, {243}, {243}, {243}, {243}}},
    (struct reg_value1){BME680_TEMP_PAR_T1_MSB, {{101}, {101}, {101}, {101}, {101}, {101}, {101}}},
    (struct reg_value1){BME680_TEMP_PAR_T2_LSB, {{85}, {85}, {85}, {85}, {85}, {85}, {85}}},
    (struct reg_value1){BME680_TEMP_PAR_T2_MSB, {{103}, {103}, {103}, {103}, {103}, {103}, {103}}},
    (struct reg_value1){BME680_TEMP_PAR_T3, {{3}, {3}, {3}, {3}, {3}, {3}, {3}}},
    (struct reg_value1){BME680_HUM_PAR_H1_LSB, {{55}, {55}, {55}, {55}, {55}, {55}, {55}}},
    (struct reg_value1){BME680_HUM_PAR_H1_MSB, {{67}, {67}, {67}, {67}, {67}, {67}, {67}}},
    (struct reg_value1){BME680_HUM_PAR_H2_MSB, {{59}, {59}, {59}, {59}, {59}, {59}, {59}}},
    (struct reg_value1){BME680_HUM_PAR_H3, {{0}, {0}, {0}, {0}, {0}, {0}, {0}}},
    (struct reg_value1){BME680_HUM_PAR_H4, {{45}, {45}, {45}, {45}, {45}, {45}, {45}}},
    (struct reg_value1){BME680_HUM_PAR_H5, {{20}, {20}, {20}, {20}, {20}, {20}, {20}}},
    (struct reg_value1){BME680_HUM_PAR_H6, {{120}, {120}, {120}, {120}, {120}, {120}, {120}}},
    (struct reg_value1){BME680_HUM_PAR_H7, {{156}, {156}, {156}, {156}, {156}, {156}, {156}}},
    (struct reg_value1){BME680_PRESS_PAR_P1_LSB, {{244}, {244}, {244}, {244}, {244}, {244}, {244}}},
    (struct reg_value1){BME680_PRESS_PAR_P1_MSB, {{143}, {143}, {143}, {143}, {143}, {143}, {143}}},
    (struct reg_value1){BME680_PRESS_PAR_P2_LSB, {{217}, {217}, {217}, {217}, {217}, {217}, {217}}},
    (struct reg_value1){BME680_PRESS_PAR_P2_MSB, {{215}, {215}, {215}, {215}, {215}, {215}, {215}}},
    (struct reg_value1){BME680_PRESS_PAR_P3, {{88}, {88}, {88}, {88}, {88}, {88}, {88}}},
    (struct reg_value1){BME680_PRESS_PAR_P4_LSB, {{6}, {6}, {6}, {6}, {6}, {6}, {6}}},
    (struct reg_value1){BME680_PRESS_PAR_P4_MSB, {{43}, {43}, {43}, {43}, {43}, {43}, {43}}},
    (struct reg_value1){BME680_PRESS_PAR_P5_LSB, {{215}, {215}, {215}, {215}, {215}, {215}, {215}}},
    (struct reg_value1){BME680_PRESS_PAR_P5_MSB, {{254}, {254}, {254}, {254}, {254}, {254}, {254}}},
    (struct reg_value1){BME680_PRESS_PAR_P6, {{30}, {30}, {30}, {30}, {30}, {30}, {30}}},
    (struct reg_value1){BME680_PRESS_PAR_P7, {{48}, {48}, {48}, {48}, {48}, {48}, {48}}},
    (struct reg_value1){BME680_PRESS_PAR_P8_LSB, {{132}, {132}, {132}, {132}, {132}, {132}, {132}}},
    (struct reg_value1){BME680_PRESS_PAR_P8_MSB, {{242}, {242}, {242}, {242}, {242}, {242}, {242}}},
    (struct reg_value1){BME680_PRESS_PAR_P9_LSB, {{37}, {37}, {37}, {37}, {37}, {37}, {37}}},
    (struct reg_value1){BME680_PRESS_PAR_P9_MSB, {{247}, {247}, {247}, {247}, {247}, {247}, {247}}},
    (struct reg_value1){BME680_PRESS_PAR_P10, {{30}, {30}, {30}, {30}, {30}, {30}, {30}}},
    (struct reg_value1){BME680_ID, {{0}, {0}, {0}, {0}, {0}, {0}, {0}}}

};

struct reg_value2 regs_values2[REGS_VALUES2_SIZE] = {
    (struct reg_value2){BME680_HUM_ADC_MSB, {{92, 213}, {92, 213}, {92, 213}, {92, 213}, {92, 213}, {92, 213}, {92, 213}}}
};

struct reg_value3 regs_values3[REGS_VALUES3_SIZE] = {
    (struct reg_value3){BME680_TEMP_ADC_MSB, {{122, 125, 144}, {122, 125, 144}, {122, 125, 144}, {122, 125, 144}, {122, 125, 144}, {122, 125, 144}, {122, 125, 144}}},
    (struct reg_value3){BME680_PRESS_ADC_MSB, {{65, 154, 224}, {65, 154, 224}, {65, 154, 224}, {65, 154, 224}, {65, 154, 224}, {65, 154, 224}, {65, 154, 224}}}
};

struct values get_value1(int reg) {
    struct values res;
    for (int i = 0; i < REGS_VALUES1_SIZE; i++) {
        if (regs_values1[i].reg == reg) {
            res.value = regs_values1[i].value[reg_mode];
            res.success = 1;
            return res;
        }
    }
    res.value = regs_values1[0].value[reg_mode]; //ERROR_VALUE3;
    res.success = -1;
    return res;
}

struct values get_value2(int reg) {
    struct values res;
    for (int i = 0; i < REGS_VALUES2_SIZE; i++) {
        if (regs_values2[i].reg == reg) {
            res.value = regs_values2[i].value[reg_mode];
            res.success = 1;
            return res;
        }
    }
    res.value = regs_values2[0].value[reg_mode]; //ERROR_VALUE3;
    res.success = -1;
    return res;
}

struct values get_value3(int reg) {
    struct values res;
    for (int i = 0; i < REGS_VALUES3_SIZE; i++) {
        if (regs_values3[i].reg == reg) {
            res.value = regs_values3[i].value[reg_mode];
            res.success = 1;
            return res;
        }
    }
    res.value = regs_values3[0].value[reg_mode]; //ERROR_VALUE3;
    res.success = -1;
    return res;
}

int set_buf(uint8_t *buf, size_t len, struct values value){
    if (value.success != 1) {
        return -1;
    }

    for (int i = 0; i < len; i++)
    {
        buf[i] = value.value[i];
    }

    return 1;
}


int i2c_write_blocking(int i2c, int addr, uint8_t *reg, size_t len, bool option)
{
    reg_read = *reg;
    return 1;
}

int i2c_read_blocking(int i2c, int addr, uint8_t *buf, size_t len, bool option)
{
    switch(len){
        case 1:
            return set_buf(buf, len, get_value1(reg_read));
        case 2:
            return set_buf(buf, len, get_value2(reg_read));
        case 3:
            return set_buf(buf, len, get_value3(reg_read));
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

int i2c_init(int i2c, int i2c_baudrate)
{
    return 1;
}

int gpio_set_function(int i2c_sda_pin, int gpio_func_i2c)
{
    return 1;
}

int gpio_pull_up(int i2c)
{
    return 1;
}
