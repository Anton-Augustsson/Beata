#include "beata.h"

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
    /* struct beata_data *data = dev->data; */

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
	case SENSOR_CHAN_PROX:
	    //TODO:
	    break;
	default:
	    return -EINVAL;
    }

    return -EINVAL;
}

static int
beata_attr_set(const struct device *dev, enum sensor_channel chan,
    enum sensor_attribute attr, const struct sensor_value *val)
{
    return 1; // TODO:
}

static const struct sensor_driver_api beata_api = {
	.sample_fetch   = &beata_sample_fetch,
	.channel_get    = &beata_channel_get,
	.attr_set       = &beata_attr_set,
};

#define BEATA_INIT(inst)						\
	static struct beata_data beata_data_##inst;			\
									\
	static const struct beata_config beata_config_##inst = {	\
		.i2c = I2C_DT_SPEC_INST_GET(inst),			\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst, beata_init, NULL,			\
			      &beata_data_##inst,			\
			      &beata_config_##inst, POST_KERNEL,	\
			      CONFIG_SENSOR_INIT_PRIORITY, &beata_api);

DT_INST_FOREACH_STATUS_OKAY(BEATA_INIT)
