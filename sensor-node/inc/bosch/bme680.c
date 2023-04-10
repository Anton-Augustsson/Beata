#include <string.h>
#include "bme680.h"
#include "bme680_reg.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"

#define BME680_FORCED_MODE          (0xFF & 0x01) // (0 << 1) | (1 << 0)
#define BME680_IIR_FILTER1          (0xFF & 0x04) // (0 << 4) | (0 << 3) | (1 << 2)
#define BME680_HUM_SETTINGS         (0xFF & 0x01) // (1 << 0)
#define BME680_TEMP_PRESS_SETTINGS  (0xFF & 0x34) // (0 << 7) | (0 << 6) | (1 << 5) | (1 << 4) | (0 << 3) | (1 << 2)
#define BME680_READ_VAL             (BME680_TEMP_PRESS_SETTINGS | BME680_FORCED_MODE)
#define BME680_WRITE_LEN            2

struct bme680_data {
    uint8_t id, config;
    uint8_t temp_calib_params[5];
    uint8_t press_calib_params[16];
    uint8_t hum_calib_params[9];
    int32_t t_fine, current_temp, current_hum, current_press;
};

/* Global variables. */
static struct bme680_data sensor_data;

static error_t
bme680_write(uint8_t reg, uint8_t val) 
{
    uint8_t buf[2] = {reg, val};
    /* A '2' is chosen here because you are always sending an array with 2
     * elements. One element as the address of where to write, and one with the
     * value. */
    return i2c_write_blocking(i2c_default, BME680_ADDR, buf, BME680_WRITE_LEN,
                              false) == PICO_ERROR_GENERIC
               ? ERROR
               : SUCCESS;
}

static error_t
bme680_read(uint8_t reg, uint8_t *buf, size_t len) 
{
    if (i2c_write_blocking(i2c_default, BME680_ADDR, &reg, 1, true) == PICO_ERROR_GENERIC)
       return ERROR;

    if (i2c_read_blocking(i2c_default, BME680_ADDR, buf, len, false) == PICO_ERROR_GENERIC)
       return ERROR;

    return SUCCESS;
}

static void
prepare_calibration_params() 
{
    // Temperature
    bme680_read(BME680_TEMP_PAR_T1_LSB, &sensor_data.temp_calib_params[0], 1);
    bme680_read(BME680_TEMP_PAR_T1_MSB, &sensor_data.temp_calib_params[1], 1);
    bme680_read(BME680_TEMP_PAR_T2_LSB, &sensor_data.temp_calib_params[2], 1);
    bme680_read(BME680_TEMP_PAR_T2_MSB, &sensor_data.temp_calib_params[3], 1);
    bme680_read(BME680_TEMP_PAR_T3, &sensor_data.temp_calib_params[4], 1);

    // Humidity
    bme680_read(BME680_HUM_PAR_H1_LSB, &sensor_data.hum_calib_params[0], 1);
    bme680_read(BME680_HUM_PAR_H1_MSB, &sensor_data.hum_calib_params[1], 1);
    bme680_read(BME680_HUM_PAR_H2_LSB, &sensor_data.hum_calib_params[2], 1);
    bme680_read(BME680_HUM_PAR_H2_MSB, &sensor_data.hum_calib_params[3], 1);
    bme680_read(BME680_HUM_PAR_H3, &sensor_data.hum_calib_params[4], 1);
    bme680_read(BME680_HUM_PAR_H4, &sensor_data.hum_calib_params[5], 1);
    bme680_read(BME680_HUM_PAR_H5, &sensor_data.hum_calib_params[6], 1);
    bme680_read(BME680_HUM_PAR_H6, &sensor_data.hum_calib_params[7], 1);
    bme680_read(BME680_HUM_PAR_H7, &sensor_data.hum_calib_params[8], 1);

    // Pressure
    bme680_read(BME680_PRESS_PAR_P1_LSB, &sensor_data.press_calib_params[0], 1);
    bme680_read(BME680_PRESS_PAR_P1_MSB, &sensor_data.press_calib_params[1], 1);
    bme680_read(BME680_PRESS_PAR_P2_LSB, &sensor_data.press_calib_params[2], 1);
    bme680_read(BME680_PRESS_PAR_P2_MSB, &sensor_data.press_calib_params[3], 1);
    bme680_read(BME680_PRESS_PAR_P3, &sensor_data.press_calib_params[4], 1);
    bme680_read(BME680_PRESS_PAR_P4_LSB, &sensor_data.press_calib_params[5], 1);
    bme680_read(BME680_PRESS_PAR_P4_MSB, &sensor_data.press_calib_params[6], 1);
    bme680_read(BME680_PRESS_PAR_P5_LSB, &sensor_data.press_calib_params[7], 1);
    bme680_read(BME680_PRESS_PAR_P5_MSB, &sensor_data.press_calib_params[8], 1);
    bme680_read(BME680_PRESS_PAR_P6, &sensor_data.press_calib_params[9], 1);
    bme680_read(BME680_PRESS_PAR_P7, &sensor_data.press_calib_params[10], 1);
    bme680_read(BME680_PRESS_PAR_P8_LSB, &sensor_data.press_calib_params[11], 1);
    bme680_read(BME680_PRESS_PAR_P8_MSB, &sensor_data.press_calib_params[12], 1);
    bme680_read(BME680_PRESS_PAR_P9_LSB, &sensor_data.press_calib_params[13], 1);
    bme680_read(BME680_PRESS_PAR_P9_MSB, &sensor_data.press_calib_params[14], 1);
    bme680_read(BME680_PRESS_PAR_P10, &sensor_data.press_calib_params[15], 1);
}

/*
 * Function: to_pascal
 * ----------------------------
 *  convert to pascal (pressure) for the pressure
 *  data values given from the sensor.
 *
 *  raw_data[3]: ADC value(s) that has been read the sensor
 *
 *  returns: the pressure (in pascal) given the input raw_data[16]
 */
static int32_t
to_pascal(uint8_t raw_data[3]) 
{
    /* Prepare data */
    /* Concat MSB, LSB (8-bit), and XLSB (4-bit) values into a single 20-bit
       value, which corresponds to the current raw reading of the sensor. */
    uint16_t press_adc = (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4);

    uint16_t par_p1     = (sensor_data.press_calib_params[1] << 8) |
                          (sensor_data.press_calib_params[0] << 0);
    uint16_t par_p2     = (sensor_data.press_calib_params[3] << 8) |
                          (sensor_data.press_calib_params[2] << 0);
    uint16_t par_p4     = (sensor_data.press_calib_params[6] << 8) |
                          (sensor_data.press_calib_params[5] << 0);
    uint16_t par_p5     = (sensor_data.press_calib_params[8] << 8) |
                          (sensor_data.press_calib_params[7] << 0);
    uint16_t par_p8     = (sensor_data.press_calib_params[12] << 8) |
                          (sensor_data.press_calib_params[11] << 0);
    uint16_t par_p9     = (sensor_data.press_calib_params[14] << 8) |
                          (sensor_data.press_calib_params[13] << 0);
    uint8_t par_p3      = sensor_data.press_calib_params[4];
    uint8_t par_p6      = sensor_data.press_calib_params[9];
    uint8_t par_p7      = sensor_data.press_calib_params[10];
    uint8_t par_p10     = sensor_data.press_calib_params[15];

    /* Perform conversion */
    int32_t var1 = (sensor_data.t_fine >> 1) - 64000;
    int32_t var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)par_p4 << 16);
    var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
            ((int32_t)par_p3 << 5)) >> 3) + (((int32_t)par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1) * (int32_t)par_p1) >> 15;

    uint32_t press_comp = 1048576 - press_adc;
    press_comp = (uint32_t)((press_comp - (var2 >> 12)) * ((uint32_t)3125));

    if (press_comp >= (1 << 30))
        press_comp = ((press_comp / (uint32_t)var1) << 1); 
    else
        press_comp = ((press_comp << 1) / (uint32_t)var1);

    var1 = ((int32_t)par_p9 * (int32_t)(((press_comp >> 3) *
            (press_comp >> 3)) >> 13)) >> 12;
    var2 = ((int32_t)(press_comp >> 2) * (int32_t)par_p8) >> 13;
    int32_t var3 = ((int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) *  
            (int32_t)(press_comp >> 8) * (int32_t)par_p10) >> 17;  

    press_comp = (int32_t)(press_comp) +
            ((var1 + var2 + var3 + ((int32_t)par_p7 << 7)) >> 4);

    return press_comp;
}

/*
 * Function: to_percent
 * ----------------------------
 *   convert to percentage for the humidity data
 *   values given from the sensor
 *
 *   data[2]: ADC value(s) that has been read the sensor
 *
 *   returns: the percentage given the input raw_data[2].
 */
static int32_t
to_percent(uint8_t raw_data[2]) 
{ 
    /* Prepare data */
    /* Concat MSB, LSB (16-bit) values into a single 16-bit value, which
     * corresponds to the raw reading of the humidity from the sensor */
    uint16_t hum_adc = (raw_data[0] << 8) | (raw_data[1] << 0);

    uint16_t par_h1 =   (sensor_data.hum_calib_params[1] << 8) | 
                        ((sensor_data.hum_calib_params[0] & 0x0F) << 0);
    uint16_t par_h2 =   (sensor_data.hum_calib_params[3] << 8) |
                        ((sensor_data.hum_calib_params[2] & 0xF0) << 0);
    uint8_t par_h3  = sensor_data.hum_calib_params[4];
    uint8_t par_h4  = sensor_data.hum_calib_params[5];
    uint8_t par_h5  = sensor_data.hum_calib_params[6];
    uint8_t par_h6  = sensor_data.hum_calib_params[7];
    uint8_t par_h7  = sensor_data.hum_calib_params[8];

    /* Perform conversion */
    int32_t temp_scaled = (int32_t)sensor_data.current_temp;
    int32_t var1 = (int32_t)hum_adc - (int32_t)((int32_t)par_h1 << 4) -
        (((temp_scaled * (int32_t)par_h3) / ((int32_t)100)) >> 1);
    int32_t var2 = ((int32_t)par_h2 * (((temp_scaled *
        (int32_t)par_h4) / ((int32_t)100)) +
        (((temp_scaled * ((temp_scaled * (int32_t)par_h5) /
        ((int32_t)100))) >> 6) / ((int32_t)100)) + ((int32_t)(1 << 14)))) >> 10;

    int32_t var3 = var1 * var2;
    int32_t var4 = (((int32_t)par_h6 << 7) + 
        ((temp_scaled * (int32_t)par_h7) / ((int32_t)100))) >> 4;

    int32_t var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    int32_t var6 = (var4 * var5) >> 1;

    return (((var3 + var6) >> 10) * ((int32_t)1000)) >> 12;
}

/*
 * Function: to_celsius
 * ----------------------------
 *   convert to celsius for the temperature data
 *   values given from the sensor
 *
 *   raw_data[3]: ADC value(s) that has been read the sensor
 *
 *   returns: the temperature given the input raw_data[2] in celcius.
 */
static int32_t
to_celsius(uint8_t raw_data[3]) 
{
    /* Prepare data */
    /* Concat MSB, LSB (8-bit), and XLSB (4-bit) values into a single 20-bit
     * value, which corresponds to the current raw reading of the sensor. */
    uint32_t temp_adc = (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4);

    uint16_t par_t1 = (sensor_data.temp_calib_params[1] << 8) |
                      (sensor_data.temp_calib_params[0] << 0);
    uint16_t par_t2 = (sensor_data.temp_calib_params[3] << 8) |
                      (sensor_data.temp_calib_params[2] << 0);
    uint8_t par_t3  = sensor_data.temp_calib_params[4];

    /* Perform conversion */
    int32_t var1 = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
    int32_t var2 = (var1 * (int32_t)par_t2) >> 11;
    int32_t var3 =
        ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)par_t3 << 4)) >> 14;
    sensor_data.t_fine = var2 + var3;

    return ((sensor_data.t_fine * 5) + 128) >> 8;
}

error_t
bme680_init()
{
    /* Ensure that the sensor is indeed connected and available for
     * communication. */
    if (bme680_read(BME680_ID, &sensor_data.id, 1) == ERROR) {
        printf("BME680_CONFIG_ERROR: Could not receive bme680 sensor ID");
        return ERROR;
    }

    /* Setup config paramets for sensor */
    if(bme680_write(BME680_CONFIG, BME680_IIR_FILTER1) == ERROR) {
        printf("BME680_CONFIG_ERROR: Could not write config to bme680\n");
        return ERROR;
    }

    prepare_calibration_params();

    /* Write configuration for the three types of functionalities */
    /* Temperature & pressure is set at the same register, i.e., 0x74 */
    /* Humidity:    1x oversampling */
    /* Temperature: 2x oversampling */
    /* Pressure:    16x oversampling */
    if(bme680_write(BME680_CTRL_HUM, BME680_HUM_SETTINGS) == ERROR) {
        printf("BME680_CONFIG_ERROR: Could not write humidity settings to bme680\n");
        return ERROR;
    }
    if (bme680_write(BME680_CTRL_MEAS, BME680_TEMP_PRESS_SETTINGS) == ERROR) {
        printf("BME680_CONFIG_ERROR: Could not write temperature/pressure settings to bme680\n");
        return ERROR;
    }

    /* Intitial run to populate struct */
    bme680_read_temp();
    bme680_read_hum();
    bme680_read_press();

    return SUCCESS;
}

bme680_rslt_t
bme680_read_temp()
{
    /* initiate an ADC conversion */
    if (bme680_write(BME680_CTRL_MEAS, BME680_READ_VAL) == ERROR)
        return (bme680_rslt_t){0, ERROR};

    uint8_t buf[3];
    /* Read the 20-bit value of the temperature from three regs (MSB, LSB, and
     * XLSB), in one go, we know that MSB (0x22) -> XLSB (0x24), so we read 3
     * bytes */
    if (bme680_read(BME680_TEMP_ADC_MSB, buf, 3) == ERROR)
        return (bme680_rslt_t){0, ERROR};

    sensor_data.current_temp = to_celsius(buf);
    return (bme680_rslt_t){sensor_data.current_temp, SUCCESS};
}

bme680_rslt_t
bme680_read_hum()
{ 
    uint8_t buf[2];
    if (bme680_read(BME680_HUM_ADC_MSB, buf, 2) == ERROR)
        return (bme680_rslt_t){0, ERROR};

    sensor_data.current_hum = to_percent(buf);
    return (bme680_rslt_t){sensor_data.current_hum, SUCCESS};
}

bme680_rslt_t
bme680_read_press() 
{ 
    uint8_t buf[3];
    if (bme680_read(BME680_PRESS_ADC_MSB, buf, 3) == ERROR)
        return (bme680_rslt_t){0, ERROR};

    sensor_data.current_press = to_pascal(buf);
    return (bme680_rslt_t){sensor_data.current_press, SUCCESS}; 
}