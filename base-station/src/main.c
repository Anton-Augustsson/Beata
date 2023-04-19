#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>

#include <sensor/beata/beata.h>

#define SLEEP_TIME 3000

// TODO:
// void
// print_sensor_value(struct sensor_readings sensor)
// {
//     printf("============ SENSOR READINGS ==============\n");
//     printf("Humidity (%%): %d.%02d\n", sensor.humidity.data / HUM_RESOLUTION,
// 	   (sensor.humidity.data % HUM_RESOLUTION) * 1000);
//     printf("Pressure (hPa): %d.%02d\n", sensor.press.data / PRESS_RESOLUTION,
// 	   sensor.press.data % PRESS_RESOLUTION);
//     printf("Temperature (C): %d.%02d\n",
// 	   sensor.temp_celsius.data / TEMP_RESOLUTION,
// 	   sensor.temp_celsius.data % TEMP_RESOLUTION);
//     printf("Has motion (bool): %d\n", sensor.has_motion.data);
//     printf("Sound level: %d\n", sensor.sound_level.data);
//     printf("===========================================\n\n");
// }


int
main()
{
    const struct device *const dev = DEVICE_DT_GET_ONE(beata);
    struct sensor_value temp, humidity, pressure, motion, sound;

    if (!device_is_ready(dev))
    {
		printk("sensor: device not ready.\n");
		return 0;
	}

	for(;;)
    {
		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);

        printk("Value: %d.%d\n", temp.val1, temp.val2);
        k_msleep(SLEEP_TIME);
	}
    return 0;
}