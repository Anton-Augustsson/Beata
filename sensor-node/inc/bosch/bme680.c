#include "bme680.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

struct bme680_data {
  uint8_t conf;
  uint8_t bme680_id;
  uint8_t temp_calib_params[5];
};

static uint8_t 
bme680_write(uint8_t reg, uint8_t *val, size_t len)
{
  uint8_t buf[len + 1];
  buf[0] = reg;
  memcpy(val, buf[1], sizeof(uint8_t) * len);
  return i2c_write_blocking(i2c_default, BME680_ADDR, buf, len, false) != PICO_ERROR_GENERIC;
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
  struct bme680_data;

  // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on a Pico)
  i2c_init(i2c_default, 100 * 1000)
  gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
  gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
  // Make the I2C pins available to picotool
  bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

  /* Ensure that the sensor is indeed connected and  available for communication. */
  if (!bme680_read(BME680_ID, &bme680_data->id, 1))
    return 0;

  // set default parameters
  // configure the oversampling settings for:
    // - temperature
    // - pressure
    // - humidity
  // This is done by setting control registers (BME680_CONFIG)
  // Set humidity to 1x (i.e write 0b001 osrs_h<2:0>)
  // Set temp to 2x (i.e write 0b010 osrs_t<2:0>)
  // Set pressure to to 16x (i.e write 0b101 to osrs_p<2:0>)
  // NOTE, set osrs_h first, then followed by osrs_t and osrs_p in one command
  // prepare BME680_CONFIG below:

  // 0b00000010 temp only for BME680_CONF
  // uint8_t data[2] = {};
  // data[0] = BME680_CONFIG; // Register address
  // data[1] = ; // Data to write
  // i2c_write_blocking(i2c_default, 0x50, data, 2, false);
  uint8_t value = (0 << 4) | (0 << 3) | (1 << 2);

  bme680_write(BME680_CONFIG, ...);
  bme680_write(BME680_CTRL_HUM, ...);
  i2c_write_blocking(i2c_default, BME680_ADDR, (uint8_t[]){BME680_CONFIG, 0b00000000}, 2, true); // true to keep master control of bus
  i2c_write_blocking(i2c_default, BME680_ADDR, (uint8_t[]){BME680_CTRL_HUM, 0b01000000}, 2, true); // true to keep master control of bus
  
  return 1;
}

int16_t
bme680_read_temp()
{
  uint8_t buffer[6];
  
  // Start reading acceleration registers from register 0x3B for 6 bytes
  uint8_t val = 0x3B;
  i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
  i2c_read_blocking(i2c_default, addr, buffer, 6, false);
   return 0; 
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
