#include "beata.h"

static int
beata_init(const struct device *dev)
{
    printk("\n\nInitialising Beata.\n");
    return 0;
}

static int
beata_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	const struct beata_config *config = dev->config;
	uint8_t temperature[4], humidity[4], press[4], sound_level[2], has_motion;

	if (i2c_burst_read_dt(
		&config->i2c, SENSOR_NODE_TEMP, temperature, 4))
		return EIO;

	if (i2c_burst_read_dt(
		&config->i2c, SENSOR_NODE_HUM, humidity, 4))
		return EIO;

	if (i2c_burst_read_dt(
		&config->i2c, SENSOR_NODE_PRESS, press, 4))
		return EIO;

	if (i2c_burst_read_dt(
		&config->i2c, SENSOR_NODE_SOUND, sound_level, 2))
		return EIO;

	if (i2c_burst_read_dt(
		&config->i2c, SENSOR_NODE_MOTION, &has_motion, 1))
		return EIO;

	((struct beata_data *)dev->data)->temp_celsius 	= temperature[3] << 24 |
													  temperature[2] << 16 |
													  temperature[1] << 8 |
													  temperature[0];
	((struct beata_data *)dev->data)->humidity 		= humidity[3] << 24 |
													  humidity[2] << 16 |
													  humidity[1] << 8 |
													  humidity[0];
	((struct beata_data *)dev->data)->press 		= press[3] << 24 |
													  press[2] << 16 |
													  press[1] << 8 |
													  press[0];
	((struct beata_data *)dev->data)->sound_level 	= sound_level[1] << 8 |
													  sound_level[0];
	((struct beata_data *)dev->data)->has_motion 	= has_motion; // already uint8_t

	return 0;
}

static int
beata_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct beata_data *data = dev->data;

	if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
			val->val1 = data->temp_celsius / 100;
			val->val2 = data->temp_celsius % 100;
	} else if (SENSOR_CHAN_HUMIDITY) {
			val->val1 = data->humidity / 1000;
			val->val2 = data->humidity % 1000;
	} else if (SENSOR_CHAN_PRESS) {
			val->val1 = data->press / 1000;
			val->val2 = data->press % 1000;
	} else if (SENSOR_CHAN_IR) {
			val->val1 = data->has_motion;
			val->val2 = 0;
	} else if (SENSOR_CHAN_PROX) {
			val->val1 = data->sound_level / 100;
			val->val2 = data->sound_level % 100;
	} else
		return EINVAL;

    return 0;
}

static const struct sensor_driver_api beata_api = {
	.sample_fetch   = beata_sample_fetch,
	.channel_get    = beata_channel_get,
};

#define BEATA_INIT(inst)						\
	static struct beata_data beata_data_##inst;	\
												\
	static const struct beata_config beata_config_##inst = { \
		.i2c = I2C_DT_SPEC_INST_GET(inst),					 \
	};													     \
	DEVICE_DT_INST_DEFINE(inst, beata_init, NULL,			 \
			      &beata_data_##inst,			             \
			      &beata_config_##inst,						\
				  POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, \
				  &beata_api);

DT_INST_FOREACH_STATUS_OKAY(BEATA_INIT)
