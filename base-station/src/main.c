#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#define SLEEP_TIME 3000

int main() {
    const struct device *dev = DEVICE_DT_GET_ANY(zephyr_beata);
    struct sensor_value temperature, humidity, pressure, motion, sound;

    while (!device_is_ready(dev)) {
        printk("sensor: device not ready.\n");
        k_msleep(SLEEP_TIME);
    }

	printk("starting...");

    for (;;) {
        sensor_sample_fetch(dev);
        sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
        sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);
        sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure);
        /* sensor_channel_get(dev, SENSOR_CHAN_PROX, &sound); */
        sensor_channel_get(dev, SENSOR_CHAN_IR, &motion);

        printf("============ SENSOR READINGS ==============\n");
        printf("Humidity (%%): %d.%02d\n", humidity.val1, humidity.val2);
        printf("Pressure (hPa): %d.%02d\n", pressure.val1, pressure.val2);
        printf("Temperature (C): %d.%02d\n", temperature.val1,
               temperature.val2);
        printf("Has motion (bool): %d\n", motion.val1);
        /* printf("Sound level (dB): %d\n", sound.val1); */
        printf("===========================================\n\n");

        k_msleep(SLEEP_TIME);
    }

    return 0;
}
