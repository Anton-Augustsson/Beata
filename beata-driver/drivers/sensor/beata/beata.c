#include "beata.h"
#include <string.h>

static int beata_init(const struct device *dev) {
    printk("\n\nInitialising Beata.\n");
 	const struct beata_config *config = dev->config;
    // TODO: Check that connected device is a sensor node.
#ifdef CONFIG_BEATA_TRIGGER
	if (config->int_gpio.port) {
		beata_trigger_init(dev);
	}
#endif
    return 0;
}

static int beata_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    int ret;
    struct beata_data *data = dev->data;
    const struct beata_config *config = dev->config;
    uint8_t temperature[4], humidity[4], press[4], sound_level[2], has_motion;

    if ((ret = i2c_burst_read_dt(&config->i2c, REG_TEMP, temperature, 4)) < 0) {
        return ret;
    }

    if ((ret = i2c_burst_read_dt(&config->i2c, REG_HUM, humidity, 4)) < 0) {
        return ret;
    }

    if ((ret = i2c_burst_read_dt(&config->i2c, REG_PRESS, press, 4)) < 0) {
        return ret;
    }

    if ((ret = i2c_burst_read_dt(&config->i2c, REG_SOUND, sound_level, 2)) < 0) {
        return ret;
    }

    if ((ret = i2c_burst_read_dt(&config->i2c, REG_MOTION, &has_motion, 1)) < 0) {
        return ret;
    }

    memcpy(&data->temp_celsius, temperature, sizeof(data->temp_celsius));
    memcpy(&data->humidity, humidity, sizeof(data->humidity));
    memcpy(&data->press, press, sizeof(data->press));
    memcpy(&data->sound_level, sound_level, sizeof(data->sound_level));
    memcpy(&data->has_motion, &has_motion, sizeof(data->has_motion));
    return 0;
}

static int beata_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val) {
    struct beata_data *data = dev->data;

    if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
        val->val1 = data->temp_celsius / 100;
        val->val2 = data->temp_celsius % 100;
    } else if (chan == SENSOR_CHAN_HUMIDITY) {
        val->val1 = data->humidity / 1000;
        val->val2 = data->humidity % 1000;
    } else if (chan == SENSOR_CHAN_PRESS) {
        val->val1 = data->press / 1000;
        val->val2 = data->press % 1000;
    } else if (chan == SENSOR_CHAN_IR) {
        val->val1 = data->has_motion;
        val->val2 = 0;
    } else if (chan == SENSOR_CHAN_PROX) {
        val->val1 = data->sound_level;
        val->val2 = data->sound_level;
    } else {
        printk("Sensor node does not support this channel.");
        return -ENOTSUP;
    }

    return 0;
}

static int beata_attr_set(const struct device *dev, enum sensor_channel chan,
                          enum sensor_attribute attr, const struct sensor_value *val) {
    int ret;
    const struct beata_config *config = dev->config;

    if (chan != SENSOR_CHAN_ALL) {
        printk("attr_set() not supported on this channel.");
        return -ENOTSUP;
    }

    if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
        if (val->val1 <= 0) {
            printk("Invalid sampling frequency for sensor node, must be > 0.");
            return -EINVAL;
        }

        if ((ret = i2c_burst_write_dt(&config->i2c, REG_SAMPLING_FREQUENCY,
                                      (uint8_t *)(&val->val1), 2)) < 0) {
            return ret;
        }
    } else if (attr == SENSOR_ATTR_FEATURE_MASK) {
        if ((ret = i2c_burst_write_dt(&config->i2c, REG_DISABLED_SENSORS,
                                      (uint8_t *)(&val->val1), 1)) < 0) {
            return ret;
        }
    } else {
        printk("attr_set() does not support this attribute.");
        return -ENOTSUP;
    }

    return 0;
}

static int beata_attr_get(const struct device *dev, enum sensor_channel chan,
                          enum sensor_attribute attr, struct sensor_value *val) {
    const struct beata_config *config = dev->config;
    int ret;
    uint8_t buf[2];

    if (attr == SENSOR_ATTR_SAMPLING_FREQUENCY) {
        if ((ret = i2c_burst_read_dt(&config->i2c, REG_SAMPLING_FREQUENCY, buf, 2)) < 0) {
            return ret;
        }

        memcpy(&val->val1, buf, 2);
    } else if (attr == SENSOR_ATTR_FEATURE_MASK) {
        if ((ret = i2c_burst_read_dt(&config->i2c, REG_DISABLED_SENSORS, buf, 1)) < 0) {
            return ret;
        }

        memcpy(&val->val1, buf, 1);
    } else {
        printk("Sensor node does not support this attribute.");
        return -ENOTSUP;
    }

    val->val2 = 0;
    return 0;
}

static const struct sensor_driver_api beata_api = {
    .sample_fetch   = beata_sample_fetch,
    .channel_get    = beata_channel_get,
    .attr_set       = beata_attr_set,
    .attr_get       = beata_attr_get,
#ifdef CONFIG_BEATA_TRIGGER
    .trigger_set    = beata_trigger_set,
#endif
};

#define BEATA_INIT(inst)                                     \
    static struct beata_data beata_data_##inst;              \
    static const struct beata_config beata_config_##inst = { \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                   \
        IF_ENABLED(CONFIG_BEATA_TRIGGER,				\
            (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }),))	\
    };                                                       \
    DEVICE_DT_INST_DEFINE(inst, beata_init, NULL,            \
            &beata_data_##inst,                              \
            &beata_config_##inst,                            \
            POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,        \
            &beata_api);

DT_INST_FOREACH_STATUS_OKAY(BEATA_INIT)