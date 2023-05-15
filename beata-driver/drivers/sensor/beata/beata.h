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
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>
#endif

/* sensor resolutions */
#define TEMP_RESOLUTION 100
#define HUM_RESOLUTION 1000
#define PRESS_RESOLUTION 1000

/* Data registers */
#define SENSOR_NODE_ADDR 0x17
#define REG_TEMP         0x00
#define REG_HUM          0x04
#define REG_PRESS        0x08
#define REG_SOUND        0x0C
#define REG_MOTION       0x0E

/* Config registers */
#define REG_SAMPLING_FREQUENCY 0x0F

/* 1 register for disabling respective sensor.
 * bit 0:   CLIMATE
 * bit 1:   SOUND
 * bit 2:   MOTION
 * bit >2:  discarded */
#define REG_DISABLED_SENSORS 0x11

/* Interrupt (trigger) enable registers */
#define REG_INT_STATUS    0x12 // Which interrupt was triggered
#define REG_INT_THRESHOLD 0x13
#define REG_INT_MOTION    0x14

/* Trigger target value registers */
#define REG_INT_TEMP_LOW    0x15
#define REG_INT_TEMP_HIGH   0x19
#define REG_INT_HUM_LOW     0x1d
#define REG_INT_HUM_HIGH    0x21
#define REG_INT_PRESS_LOW   0x25
#define REG_INT_PRESS_HIGH  0x29
#define REG_INT_SOUND_LOW   0x2d
#define REG_INT_SOUND_HIGH  0x31

#define REG_ID 0xFF

#define SENSOR_NODE_INT_STATUS_TEMP     0
#define SENSOR_NODE_INT_STATUS_HUM      1
#define SENSOR_NODE_INT_STATUS_PRESS    2
#define SENSOR_NODE_INT_STATUS_SOUND    3
#define SENSOR_NODE_INT_STATUS_MOTION   4

struct beata_config {
    struct i2c_dt_spec i2c;
#ifdef CONFIG_BEATA_TRIGGER
    struct gpio_dt_spec int_gpio;
#endif
};

struct beata_data {
    int32_t temp_celsius;
    int32_t humidity;
    int32_t press;
    uint16_t sound_level;
    uint8_t has_motion;

#ifdef CONFIG_BEATA_TRIGGER
    struct k_work work;
	const struct device *dev;
	struct gpio_callback gpio_cb;

    const struct sensor_trigger *temp_trig;
    const struct sensor_trigger *hum_trig;
    const struct sensor_trigger *press_trig;
    const struct sensor_trigger *sound_trig;
    const struct sensor_trigger *motion_trig;

    sensor_trigger_handler_t temp_handler;
    sensor_trigger_handler_t hum_handler;
    sensor_trigger_handler_t press_handler;
    sensor_trigger_handler_t sound_handler;
    sensor_trigger_handler_t motion_handler;
#endif
};

int beata_trigger_init(const struct device *dev);
int beata_trigger_set(const struct device *dev,
    const struct sensor_trigger *trig,
	sensor_trigger_handler_t handler);

#endif /* __ZEPHYR_DRIVERS_SENSOR_BEATA__ */
