#include <stdio.h>

#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/binary_info.h"
#include "pico/i2c_slave.h"

#include "../inc/bosch/bme680.h"
#include "../inc/motion/amn1.h"
#include "../inc/sound/dfr0034.h"
#include "../inc/common.h"

#define I2C_BAUDRATE 400000
#define I2C_SLAVE_ADDRESS 0x17
#define I2C_SLAVE_SDA_PIN 18
#define I2C_SLAVE_SCL_PIN 19

#define SENSOR_QUERY_PERIOD_MS 3000
#define INIT_RETRY_DELAY_MS (SENSOR_QUERY_PERIOD_MS / 6)

struct sensor_readings {
    bme680_rslt_t humidity;
    bme680_rslt_t press;
    bme680_rslt_t temp_celsius;
    amn1_rslt_t has_motion;
    dfr0034_rslt_t sound_level;
};

void
i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event)
{
    /* Called from ISR, so keep it concise. */
    switch (event) {
        case I2C_SLAVE_RECEIVE:
            break;
        case I2C_SLAVE_REQUEST:
            break;
        case I2C_SLAVE_FINISH:
            break;
        default:
            break;
    }
}

/* Initializes the I2C communication to the BEATA base station.
   All sensor nodes will act as slaves, and the base station as master. */
void
base_station_i2c_init()
{
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c1, I2C_BAUDRATE);
    i2c_slave_init(i2c1, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

void
sensors_init()
{
    while (bme680_init() != SUCCESS) {
	    printf("ERROR: could not connect to BME680, retrying...\n");
	    sleep_ms(INIT_RETRY_DELAY_MS);
    }

    while (amn1_init() != SUCCESS) {
	    printf("ERROR: could not connect to AMN1, retrying...\n");
	    sleep_ms(INIT_RETRY_DELAY_MS);
    }

    while (dfr0034_init() != SUCCESS) {
	    printf("ERROR: could not connect to DFR0034, retrying...\n");
	    sleep_ms(INIT_RETRY_DELAY_MS);
    }
}

int
main()
{
    struct sensor_readings sensor;

    stdio_init_all();
    adc_init();
    sensors_init();
    base_station_i2c_init();

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

        sleep_ms(SENSOR_QUERY_PERIOD_MS);
    }

    return 0;
}
