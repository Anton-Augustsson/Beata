#include "beata.h"

static int
beata_init(const struct device *dev)
{
    printk("\n\nInitialising Beata.\n");
    return 1; // TODO:
}

static int
beata_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct beata_data *data = dev->data;
	const struct beata_config *config = dev->config;
	i2c_burst_read_dt(
		&config->i2c, SENSOR_NODE_HUM, &data->humidity, 4);
	i2c_burst_read_dt(
		&config->i2c, SENSOR_NODE_TEMP, &data->temp_celsius, 4);
	i2c_burst_read_dt(
		&config->i2c, SENSOR_NODE_PRESS, &data->press, 4);
	i2c_burst_read_dt(
		&config->i2c, SENSOR_NODE_SOUND, &data->sound_level, 2);
	i2c_burst_read_dt(
		&config->i2c, SENSOR_NODE_MOTION, &data->has_motion, 1);
}

static int
beata_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	printk("AHAHAHAHA!!");
    struct beata_data *data = dev->data;

    switch (chan)
    {
	case SENSOR_CHAN_AMBIENT_TEMP:
		val->val1 = data->temp_celsius / 100;
		val->val2 = data->temp_celsius % 100;
	case SENSOR_CHAN_HUMIDITY:
		val->val1 = data->humidity / 1000;
		val->val2 = data->humidity % 1000;
	case SENSOR_CHAN_PRESS:
		val->val1 = data->press / 1000;
		val->val2 = data->press % 1000;
	case SENSOR_CHAN_IR:
		val->val1 = data->has_motion;
		val->val2 = 0;
	case SENSOR_CHAN_PROX:
		val->val1 = data->sound_level / 100;
		val->val2 = data->sound_level % 100;
	default:
	    return -EINVAL;
    }

    return 0;
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
};

#define BEATA_INIT(inst)						\
	static struct beata_data beata_data_##inst;	\
												\
	static const struct beata_config beata_config_##inst = { \
		.i2c = I2C_DT_SPEC_INST_GET(inst),					 \
	};													     \
															 \
	DEVICE_DT_INST_DEFINE(inst, beata_init, NULL,			 \
			      &beata_data_##inst,			             \
			      &beata_config_##inst, POST_KERNEL,	     \
			      CONFIG_SENSOR_INIT_PRIORITY, &beata_api);

DT_INST_FOREACH_STATUS_OKAY(BEATA_INIT)
