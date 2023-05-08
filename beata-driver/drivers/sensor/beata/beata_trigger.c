#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>

#include "beata.h"

static void beata_gpio_callback(const struct device *port,
				 struct gpio_callback *cb,
				 uint32_t pin)
{
	struct beata_data *data = CONTAINER_OF(cb, struct beata_data, gpio_cb);
    ARG_UNUSED(port);
	ARG_UNUSED(pin);
    k_work_submit(&data->work);
}

static int beata_handle_int(const struct device *dev) {
    int ret;
	uint8_t status;
    const struct beata_data *data = dev->data;
    const struct beata_config *config = dev->config;

	if ((ret = i2c_burst_read_dt(&config->i2c, REG_INT_STATUS, &status, 1)) < 0) {
		return ret;
	}

    // TODO: Check if bit is set for each sensor
    // if (status & (1 << ))
    return 0;
}

static void beata_work_cb(struct k_work *work) {
	struct beata_data *beata = CONTAINER_OF(work, struct beata_data, work);
	beata_handle_int(beata->dev);
}

static int beata_setup_motion(const struct device *dev,
	const struct sensor_trigger *trig,
	sensor_trigger_handler_t handler)
{
    int ret;
	struct beata_data *data = dev->data;
	struct beata_config *config = dev->config;
    uint8_t enable_int = 1;

    /* Enable interrupts for motion on the sensor node */
    if ((ret = i2c_burst_write_dt(&config->i2c, REG_INT_MOTION, &enable_int, 1))) {
		return ret;
	}

	data->motion_handler = handler;
	data->motion_trig = trig;
	return 0;
}

static int beata_setup_threshold(const struct device *dev,
	const struct sensor_trigger *trig,
	sensor_trigger_handler_t handler)
{
    int ret;
	struct beata_data *data = dev->data;
	struct beata_config *config = dev->config;
    uint8_t enable_int = 1;

    /* Enable interrupts for threshold values on the sensor node.
       This will cause the sensor node to check the threshold values
       and trigger interrupts if they are set. To check which sensor
       (temp, hum, press, sound, etc) that triggered the interrupt,
       we read the REG_INT_STATUS register. */
    if ((ret = i2c_burst_write_dt(&config->i2c, REG_INT_THRESHOLD, &enable_int, 1))) {
		return ret;
	}

	data->threshold_handler = handler;
	data->threshold_trig = trig;
	return 0;
}


int beata_trigger_set(const struct device *dev,
	const struct sensor_trigger *trig,
	sensor_trigger_handler_t handler)
{
	struct beata_config *config = dev->config;

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

int beata_trigger_init(const struct device *dev) {
    int ret;
    struct beata_config *config = dev->config;
    struct beata_data *data = dev->data;

	if (!device_is_ready(config->int_gpio.port)) {
		printk("Sensor node interrupt GPIO device not ready");
		return -ENODEV;
	}

    data->work.handler = beata_work_cb;

    if ((ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INT_EDGE_BOTH)) < 0) {
        return ret;
    }

    gpio_init_callback(&data->gpio_cb, beata_gpio_callback, BIT(config->int_gpio.pin));

    if ((ret = gpio_add_callback(config->int_gpio.port, &data->gpio_cb)) < 0) {
        return ret;
    }

    return gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_BOTH);
}