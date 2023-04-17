#ifndef __ZEPHYR_DRIVERS_SENSOR_BEATA__
#define __ZEPHYR_DRIVERS_SENSOR_BEATA__

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>

#define BEATA_BUS_I2C DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)

/* We only support I2C. */
typedef struct i2c_dt_spec beata_bus;

typedef int (*beata_bus_check_fn)(const beata_bus *bus);
typedef int (*beata_reg_read_fn)(const struct device *dev, uint8_t start, uint8_t *buf, int size);
typedef int (*beata_reg_write_fn)(const struct device *dev, uint8_t start, uint8_t val);

int beata_bus_check_i2c(const beata_bus *bus);
int beata_reg_read_i2c(const struct device *dev, uint8_t start, uint8_t *buf, int size);
int beata_reg_write_i2c(const struct device *dev, uint8_t reg, uint8_t val);

struct beata_bus_io {
	beata_bus_check_fn check;
	beata_reg_read_fn read;
	beata_reg_write_fn write;
};

const struct beata680_bus_io beata_bus_io_i2c = {
	.check = beata_bus_check_i2c,
	.read = beata_reg_read_i2c,
	.write = beata_reg_write_i2c,
};

struct beata_config {
	beata_bus bus;
	const struct beata_bus_io *bus_io;
};

/* TODO: Add register defines for our protocol */

struct beata_data
{
    beata_bus bus;
    
    int32_t humidity;
    int32_t press;
    int32_t temp_celsius;
    bool has_motion;
    uint16_t sound_level;
}

#endif /* __ZEPHYR_DRIVERS_SENSOR_BEATA__ */