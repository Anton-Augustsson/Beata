#include <stdio.h>

#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/binary_info.h"

#include "../inc/bosch/bme680.h"
#include "../inc/motion/amn1.h"
#include "../inc/sound/dfr0034.h"
#include "../inc/common.h"

#define TEMP_RESOLUTION 100
#define HUM_RESOLUTION 100
#define PRESS_RESOLUTION 1000
#define I2C_FREQUENCY 400000
#define SENSOR_QUERY_PERIOD_MS 3000
#define INIT_RETRY_DELAY_MS (SENSOR_QUERY_PERIOD_MS / 6)

// sensor readings
struct sensor_readings {
    bme680_rslt_t humidity;
    bme680_rslt_t press;
    bme680_rslt_t temp_celsius;
    amn1_rslt_t has_motion;
    dfr0034_rslt_t sound_level;
};

void
print_sensor_value(struct sensor_readings sensor)
{
    printf("============ SENSOR READINGS ==============\n");
    printf("Humidity (%%): %d.%02d\n", sensor.humidity.data / HUM_RESOLUTION,
	   sensor.humidity.data % HUM_RESOLUTION);
    printf("Pressure (hPa): %d.%02d\n", sensor.press.data / PRESS_RESOLUTION,
	   sensor.press.data % PRESS_RESOLUTION);
    printf("Temperature (C): %d.%02d\n",
	   sensor.temp_celsius.data / TEMP_RESOLUTION,
	   sensor.temp_celsius.data % TEMP_RESOLUTION);
    printf("Has motion (bool): %d\n", sensor.has_motion.data);
    printf("Sound level: %d\n", sensor.sound_level.data);
    printf("===========================================\n\n");
}

void
init_serial_connections()
{
    /* This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on
     * a Pico)
     * I2C is "open drain", pull ups to keep signal high when no data is being
     * sent */
    i2c_init(i2c_default, I2C_FREQUENCY);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
}

int
main()
{
    stdio_init_all();
    adc_init();

    init_serial_connections();

    struct sensor_readings sensor = {0, 0, 0};
    printf("Starting sensor-node and attempting connections\n");

    while (bme680_init() != SUCCESS) {
	printf(
	    "SENSORNODE_ERROR: could not connect to BME680. Trying again in %d "
	    "ms.\n",
	    INIT_RETRY_DELAY_MS);
	sleep_ms(INIT_RETRY_DELAY_MS);
    }

    while (amn1_init() != SUCCESS) {
	printf(
	    "SENSORNODE_ERROR: could not setup AMN1. Trying again in %d "
	    "ms.\n",
	    INIT_RETRY_DELAY_MS);
	sleep_ms(INIT_RETRY_DELAY_MS);
    }

    while (dfr0034_init() != SUCCESS) {
	printf(
	    "SENSORNODE_ERROR: could not setup DFR0034. Trying again in %d "
	    "ms.\n",
	    INIT_RETRY_DELAY_MS);
	sleep_ms(INIT_RETRY_DELAY_MS);
    }

    printf("Starting readings...\n");

    for (;;) {
	sensor.temp_celsius = bme680_read_temp();
	if (sensor.temp_celsius.error == ERROR)
	    printf("BME680_ERROR: Could not fetch temperature.");

	sensor.humidity = bme680_read_hum();
	if (sensor.humidity.error == ERROR)
	    printf("BME680_ERROR: Could not fetch humidity.");

	sensor.press = bme680_read_press();
	if (sensor.press.error == ERROR)
	    printf("BME680_ERROR: Could not fetch pressure.");

	sensor.has_motion = amn1_read_motion();
	if (sensor.has_motion.error == ERROR)
	    printf("AMN1_ERROR: Could not check for motion.");

	sensor.sound_level = dfr0034_read_sound();
	if (sensor.sound_level.error == ERROR)
	    printf("DFR0034_ERROR: Could not check sound level.");

	print_sensor_value(sensor);
	sleep_ms(SENSOR_QUERY_PERIOD_MS);
    }

    return 0;
}
