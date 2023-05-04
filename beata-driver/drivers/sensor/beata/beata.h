#ifndef __ZEPHYR_DRIVERS_SENSOR_BEATA__
#define __ZEPHYR_DRIVERS_SENSOR_BEATA__

#define DT_DRV_COMPAT zephyr_beata

#ifdef BUILD_TESTS
#include "../../../tests/mocks.h"
#else
#include <zephyr/irq.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>
#endif

LOG_MODULE_REGISTER(BEATA, CONFIG_SENSOR_LOG_LEVEL);

/* Data registers */
#define SENSOR_NODE_ADDR    0x17
#define SENSOR_NODE_TEMP    0x00
#define SENSOR_NODE_HUM     0x04
#define SENSOR_NODE_PRESS   0x08
#define SENSOR_NODE_SOUND   0x0C
#define SENSOR_NODE_MOTION  0x0E

/* Config registers */
#define SENSOR_SAMPLING_FREQUENCY 0x0F

/* 1 register for disabling respective sensor.
 * bit 0:   CLIMATE
 * bit 1:   SOUND
 * bit 2:   MOTION
 * bit >2:  discarded */
#define SENSOR_DISABLED_SENSORS   0x11

struct beata_config
{
    struct i2c_dt_spec i2c;
};

struct beata_data
{
    int32_t temp_celsius;
    int32_t humidity;
    int32_t press;
    uint16_t sound_level;
    uint8_t has_motion;
};

#endif /* __ZEPHYR_DRIVERS_SENSOR_BEATA__ */
