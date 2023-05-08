#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "beata.h"

LOG_MODULE_DECLARE(BEATA, CONFIG_SENSOR_LOG_LEVEL);

extern struct beata_data beata_data;

static int beata_trigger_init(const struct device *dev) {
    const struct beata_config *config = dev->config;
    struct beata_data *data = dev->data;
    // int ret;

    // if ((ret = i2c_burst_write_dt(&config->i2c, REG_MOTION, &has_motion, 1)) < 0) {
    //     return ret;
    // }
    gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_BOTH);
}

static int beata_setup_motion(const struct device *dev,
	const struct sensor_trigger *trig,
	sensor_trigger_handler_t handler) {

}

static int beata_setup_threshold(const struct device *dev,
	const struct sensor_trigger *trig,
	sensor_trigger_handler_t handler) {

}

static int beata_trigger_set(const struct device *dev,
	const struct sensor_trigger *trig,
	sensor_trigger_handler_t handler)
{
	const struct beata_config *config = dev->config;

	if (!config->int_gpio.port) {
		return -ENOTSUP;
	}

	if (trig->type == SENSOR_TRIG_MOTION) {
        return beata_setup_motion(dev, trig, handler);
	} else if (trig->type == SENSOR_TRIG_THRESHOLD) {
        /* Used for everything except motion */
        return beata_setup_threshold(dev, trig, handler);
	}

	return -ENOTSUP;
}