#include <string.h>
#include "bme680.h"
#include "bme680_reg.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

#define FORCED_MODE (1 << 0) 
#define HUM_SETTINGS (1 << 0)
#define TEMP_PRESS_SETTINGS (1 << 6) | (1 << 4) | (1 << 2)
#define IIR_FILTER_COEFF (0 << 4) | (0 << 3) | (1 << 2)

struct bme680_data {
  uint8_t id, config;
  uint8_t temp_calib_params[5];
  uint8_t gas_calib_params[16];
  uint8_t hum_calib_params[11];
};

/* Global variables. */
static struct bme680_data sensor_data;

static int32_t
to_celsius(uint8_t data[3])
{
	/* Concat MSB, LSB (8-bit), and XLSB (4-bit) values into a single 20-bit value,
	   which corresponds to the current raw reading of the sensor. */
	uint32_t temp_adc = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);

  uint16_t par_t1 = (sensor_data.temp_calib_params[1] << 8) 
                  | (sensor_data.temp_calib_params[0] << 0);
  uint16_t par_t2 = (sensor_data.temp_calib_params[3] << 8) 
                  | (sensor_data.temp_calib_params[2] << 0);
  uint8_t par_t3 = sensor_data.temp_calib_params[4];

	/* Use compensation factors to get more accurate result. */
	int64_t var1 = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
	int64_t var2 = (var1 * (int32_t)par_t2) >> 11;
	int64_t var3 = ((((var1 >> 1) * (var1 >> 1)) >> 12) * ((int32_t)par_t3 << 4)) >> 14;
	int64_t t_fine = var2 + var3;
	return ((t_fine * 5) + 128) >> 8;
}

static uint8_t
bme680_write(uint8_t reg, uint8_t *val, size_t len)
{
  uint8_t buf[len + 1];
  buf[0] = reg;
  memcpy(val, &buf[1], sizeof(uint8_t) * len);
  return i2c_write_blocking(i2c_default, BME680_ADDR, buf, len, false) != PICO_ERROR_GENERIC;
}

static uint8_t 
bme680_write_value(uint8_t reg, uint8_t val)
{
  return bme680_write(reg, &val, 1);
}

static uint8_t
bme680_read(uint8_t reg, uint8_t *buf, size_t len)
{
  if (i2c_write_blocking(i2c_default, BME680_ADDR, &reg, len, true) == PICO_ERROR_GENERIC);
    return 0;
  if (i2c_read_blocking(i2c_default, BME680_ADDR, buf, len, false) == PICO_ERROR_GENERIC)
    return 0;
  return 1;
}

uint8_t
bme680_init()
{
  // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
  i2c_init(i2c_default, 100 * 1000);
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  // Make the I2C pins available to picotool
  bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

  /* Ensure that the sensor is indeed connected and  available for communication. */
  if (!bme680_read(BME680_ID, &sensor_data.id, 1))
    return 0;

  /* Setup config paramets for sensor */
  sensor_data.config |= IIR_FILTER_COEFF;
  bme680_write(BME680_CONFIG, &sensor_data.config, 1);

  /* Write configuration for the three types of functionalities */
  /* Temperature & pressure is set at the same register, i.e., 0x74 */
  /* Humidity:    1x oversampling */
  /* Temperature: 2x oversampling */
  /* Pressure:    16x oversampling */
  bme680_write_value(BME680_CTRL_HUM, HUM_SETTINGS);         
  bme680_write_value(BME680_CTRL_MEAS, TEMP_PRESS_SETTINGS); 
                                                          
  return 1;
}

int32_t
bme680_read_temp()
{
  uint8_t buf[3];

  /* Start measurement by enabling forced mode. */
  bme680_write_value(BME680_CTRL_MEAS, TEMP_PRESS_SETTINGS | FORCED_MODE);

 	/* Read the 20-bit value of the temperature from three regs (MSB, LSB, and XLSB). */
	bme680_read(BME680_TEMP_MSB, &buf[0], 1);
	bme680_read(BME680_TEMP_LSB, &buf[1], 1);
	bme680_read(BME680_TEMP_XLSB, &buf[2], 1);
	return to_celsius(buf);
}

uint16_t
bme680_read_hum()
{
  return 0;
}

uint16_t
bme680_read_gas()
{
  return 0;
}