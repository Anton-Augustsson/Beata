#include <stdio.h>

#include "hardware/i2c.h"
#include "inc/bosch/bme680.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/binary_info.h"

#define DELAY_MS 3000
#define TEMP_RESOLUTION 100
#define I2C_FREQUENCY 400000

// sensor readings
struct sensor_reading {
    bme680_rslt_t humidity;
    bme680_rslt_t press;
    bme680_rslt_t temp_celcius;
    // uint8_t noise;
    // uint8_t motion;
};

void 
print_sensor_value(struct sensor_reading sensor) 
{
    printf("=============== BME680 =================\n");
    printf("Humidity (%%): %d.%02d\n", sensor.humidity.data / 100, sensor.humidity.data % 100);
    printf("Pressure (Pa): %d.%02d\n", sensor.press.data / 1000, sensor.press.data % 1000);
    printf("Temperature (C): %d.%02d\n", sensor.temp_celcius.data / TEMP_RESOLUTION,
           sensor.temp_celcius.data % TEMP_RESOLUTION);
    printf("========================================\n");
    // sensor.temp_celcius); printf("Sensor 2: %d dB noise\n", sensor.noise);
    // printf("Sensor 3: %d (cm) motion\n\n", sensor.noise);
}

void 
init_serial_connections() 
{
    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on
    // a Pico)
    // I2C is "open drain", pull ups to keep signal high when no data is being sent
    i2c_init(i2c_default, 100 * 1000);
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
    struct sensor_reading sensor = {0, 0};
    printf("Starting sensor-node and attempting connections\n");

    /* Configure BME680 */
    while (!bme680_init()) {
        printf("SENSORNODE_ERROR: could not connect to bme680. Trying again.\n");
        sleep_ms(DELAY_MS / 6);
    }

    printf("Starting readings...\n");

    for (;;) {
        /* BME680 Readings */
        sensor.temp_celcius = bme680_read_temp();
        if (!sensor.temp_celcius.error) printf("BME680_ERROR: Could not fetch temperature.");

        sensor.humidity = bme680_read_hum();
        if (!sensor.humidity.error) printf("BME680_ERROR: Could not fetch humidity.");

        sensor.press = bme680_read_press();
        if (!sensor.press.error) printf("BME680_ERROR: Could not fetch pressure.");

        print_sensor_value(sensor);
        sleep_ms(DELAY_MS);
    }

    return 0;
}