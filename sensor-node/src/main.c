#include <stdio.h>

#include "hardware/i2c.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/binary_info.h"

#include "../inc/bosch/bme680.h"
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
};

void
print_sensor_value(struct sensor_readings sensor)
{
    printf("=============== BME680 =================\n");
    printf("Humidity (%%): %d.%02d\n", sensor.humidity.data / HUM_RESOLUTION,
           sensor.humidity.data % HUM_RESOLUTION);
    printf("Pressure (hPa): %d.%02d\n", sensor.press.data / PRESS_RESOLUTION,
           sensor.press.data % PRESS_RESOLUTION);
    printf("Temperature (C): %d.%02d\n",
           sensor.temp_celsius.data / TEMP_RESOLUTION,
           sensor.temp_celsius.data % TEMP_RESOLUTION);
    printf("========================================\n");
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

        print_sensor_value(sensor);
        sleep_ms(SENSOR_QUERY_PERIOD_MS);
    }

    return 0;
}
