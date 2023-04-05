#include <stdio.h>
#include "pico/time.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "inc/bosch/bme680.h"

// sensor readings
struct sensor_reading
{
    uint8_t humidity;
    uint8_t co2;
    int16_t temp_celcius;
    uint8_t noise;
    uint8_t motion;
};

void print_sensor_value(struct sensor_reading sensor)
{
    printf("Sensor 1: %dunit humidity ", sensor.humidity);
    printf("%dunit co2, %dC temperature\n", sensor.co2, sensor.temp_celcius);
    printf("Sensor 2: %ddB noise\n", sensor.noise);
    printf("Sensor 3: %dunit motion\n", sensor.noise);
}

void read_sensor(struct sensor_reading sensor)
{
    while (1) {
        sensor.temp_celcius = bme680_read_temp();
        print_sensor_value(sensor);
        sleep_ms(500);
    }
}

int main()
{
    stdio_init_all();
    struct sensor_reading sensor = {0, 0, 0, 0, 0};

    bme680_init();
    read_sensor(sensor);

    return 0;
}
