#include <stdio.h>
#include "pico/time.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "inc/bosch/bme680.h"

// sensor readings
struct sensor_readings 
{
    uint8_t humidity;
    uint8_t co2;
    int16_t temp_celsius;
    uint8_t noise;
    uint8_t motion;
};

void print_sensor_value(struct sensor_readings sr)
{
    printf("Sensor 1: %dunit humidity", sr.humidity);
    printf("%dunit co2, %dC temperature\n", sr.co2, sr.temp_celsius);
    printf("Sensor 2: %ddB noise\n", sr.noise);
    printf("Sensor 3: %dunit motion\n", sr.noise);
}

void read_sensor(struct sensor_reading sr)
{
    while (1) {
        sr.temp_celcius = bme680_read_temp();
        print_sensor_value(sr);
        sleep_ms(500);
    }
}

int main()
{
    struct sensor_readings sr = {0, 0, 0, 0, 0};
    
    bme680_init();
    read_sensor(sr);

    return 0;
}
