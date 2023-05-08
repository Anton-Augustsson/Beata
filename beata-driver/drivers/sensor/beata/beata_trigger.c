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

static void beata_work_cb(struct k_work *work) {
	uint8_t status;
	const struct beata_data *data = CONTAINER_OF(work, struct beata_data, work);
    const struct beata_config *config = data->dev->config;

	if (i2c_burst_read_dt(&config->i2c, REG_INT_STATUS, &status, 1) < 0) {
		return;
	}

    printk("status: %d\n", status);
    /* Check which sensor triggered the interrupt.
        Multiple interrupts can theoretically be triggered
        at the same time, which means that we check each sensor
        in separate ifs. */
    if (data->temp_handler) {
        if (status & (1 << SENSOR_NODE_INT_STATUS_TEMP)) {
            data->temp_handler(data->dev, data->temp_trig);
        }
    }

    if (data->hum_handler) {
        if (status & (1 << SENSOR_NODE_INT_STATUS_HUM)) {
            data->hum_handler(data->dev, data->hum_trig);
        }
    }

    if (data->press_handler) {
        if (status & (1 << SENSOR_NODE_INT_STATUS_PRESS)) {
            data->press_handler(data->dev, data->press_trig);
        }
    }

    if (data->sound_handler) {
        if (status & (1 << SENSOR_NODE_INT_STATUS_SOUND)) {
            data->sound_handler(data->dev, data->sound_trig);
        }
    }

    if (data->motion_handler) {
        if (status & (1 << SENSOR_NODE_INT_STATUS_MOTION)) {
            data->motion_handler(data->dev, data->motion_trig);
        }
    }
}

static int beata_setup_motion(const struct device *dev,
	const struct sensor_trigger *trig,
	sensor_trigger_handler_t handler)
{
    int ret;
    uint8_t enable_int = 1;
	struct beata_data *data = dev->data;
	const struct beata_config *config = dev->config;

    /* Enable interrupts for motion on the sensor node */
    if ((ret = i2c_burst_write_dt(&config->i2c, REG_INT_MOTION, &enable_int, 1))) {
		return ret;
	}

    if (trig->chan != SENSOR_CHAN_IR) {
        printk("Failed to add trigger for motion, expected channel SENSOR_CHAN_IR");
        return -ENOTSUP;
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
	const struct beata_config *config = dev->config;
    uint8_t enable_int = 1;

    /* Enable interrupts for threshold values on the sensor node.
       This will cause the sensor node to check the threshold values
       and trigger interrupts if they are set. To check which sensor
       (temp, hum, press, sound, etc) that triggered the interrupt,
       we read the REG_INT_STATUS register. */
    if ((ret = i2c_burst_write_dt(&config->i2c, REG_INT_THRESHOLD, &enable_int, 1))) {
		return ret;
	}

    /* Set correct trigger based on channel. */
    if (trig->chan == SENSOR_CHAN_AMBIENT_TEMP) {
        data->temp_trig = trig;
	    data->temp_handler = handler;
    } else if (trig->chan == SENSOR_CHAN_HUMIDITY) {
        data->hum_trig = trig;
	    data->hum_handler = handler;
    } else if (trig->chan == SENSOR_CHAN_PRESS) {
        data->press_trig = trig;
	    data->press_handler = handler;
    } else if (trig->chan == SENSOR_CHAN_PROX) {
        data->sound_trig = trig;
	    data->sound_handler = handler;
    } else {
        printk("Failed to add trigger for threshold, channel not supported");
        return -ENOTSUP;
    }

	return 0;
}

int beata_trigger_set(const struct device *dev,
	const struct sensor_trigger *trig,
	sensor_trigger_handler_t handler) {
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

int beata_trigger_init(const struct device *dev) {
    int ret;
    const struct beata_config *config = dev->config;
    struct beata_data *data = dev->data;

	if (!device_is_ready(config->int_gpio.port)) {
		printk("DRIVER: Interrupt GPIO device is not ready.");
		return -ENODEV;
	}

    data->dev = dev;
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