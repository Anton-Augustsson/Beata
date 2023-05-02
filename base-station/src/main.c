#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#define SLEEP_TIME 3000

int
main()
{
    const struct device *dev = DEVICE_DT_GET_ANY(zephyr_beata);
    struct sensor_value temperature, humidity, pressure, motion, sound;

    while (!device_is_ready(dev))
    {
	printk("sensor: device not ready.\n");
	k_msleep(SLEEP_TIME);
    }

    printk("Starting base station...");

    for (;;)
    {
	if (sensor_sample_fetch(dev) == EIO)
	    printk("ERROR: Could not fetch samples from sensor node.");

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature) == EINVAL)
	    printk("ERROR: Could not fetch 'TEMP' from sensor node.");

	if (sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity) == EINVAL)
	    printk("ERROR: Could not fetch 'HUMIDITY' from sensor node.");

	if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure) == EINVAL)
	    printk("ERROR: Could not fetch 'PRESSURE' from sensor node.");

	if (sensor_channel_get(dev, SENSOR_CHAN_PROX, &sound) == EINVAL)
	    printk("ERROR: Could not fetch 'SOUND' from sensor node.");

	if (sensor_channel_get(dev, SENSOR_CHAN_IR, &motion) == EINVAL)
	    printk("ERROR: Could not fetch 'MOTION' from sensor node.");

	printk("============ SENSOR READINGS ==============\n");
	printk("Humidity (%%): %d.%02d\n", humidity.val1, humidity.val2);
	printk("Pressure (hPa): %d.%02d\n", pressure.val1, pressure.val2);
	printk("Temperature (C): %d.%02d\n", temperature.val1, temperature.val2);
	printk("Has motion (bool): %d\n", motion.val1);
	printk("Sound level (dB): %d\n", sound.val1);
	printk("===========================================\n\n");

	k_msleep(SLEEP_TIME);
    }

    return 0;
}
