#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/sensor.h>

#include "beata.h"

LOG_MODULE_REGISTER(BEATA, CONFIG_SENSOR_LOG_LEVEL);

static int
beata_init(const struct device *dev)
{
    LOG_DBG("Initialising Beata.\n");
    return 1; // TODO:
}

static int
beata_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    return 1; // TODO:
}

static int
beata_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    strcut beata_data *data = dev->data;

    switch (chan)
    {
        case SENSOR_CHAN_AMBIENT_TEMP:
            //TODO:
            break;
        case SENSOR_CHAN_HUMIDITY:
            //TODO:
            break;
        case SENSOR_CHAN_PRESS:
            //TODO:
            break;
        case SENSOR_CHAN_IR:
            //TODO:
            break;
        case SENSOR_CHAN_SOUND:
            //TODO:
            break;
        case SENSOR_CHAN_ALL:
            // TODO:
            break;
        default:
            return -EINVAL;
    }

}


static int
beata_attr_set(const struct device *dev, enum sensor_channel chan,
    enum sensor_attribute attr, const struct sensor_value *val)
{
    return 1; // TODO:
}

static const struct sensor_driver_api beata_api = {
	.sample_fetch   = beata_sample_fetch,
	.channel_get    = beata_channel_get,
	.attr_set       = beata_attr_set,
};

DEVICE_DT_INST_DEFINE(0, beata_init, NULL, &beata_data, &beata_config,
                      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &beata_api);