#include <stdio.h>

#include "hardware/i2c.h"
#include "inc/bosch/bme680.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/binary_info.h"

#define DELAY_MS 3000
#define I2C_FREQUENCY 400000

// sensor readings
struct sensor_reading {
    uint8_t humidity;
    // uint8_t press;
    int16_t temp_celcius;
    // uint8_t noise;
    // uint8_t motion;
};

void 
print_sensor_value(struct sensor_reading sensor) 
{
    // printf("Sensor 1: %d %% humidity ", sensor.humidity);
    // printf(", %d kPc, %d C temperature\n", sensor.press,
    // sensor.temp_celcius); printf("Sensor 2: %d dB noise\n", sensor.noise);
    // printf("Sensor 3: %d (cm) motion\n\n", sensor.noise);
    printf("%d C temperature\n", sensor.temp_celcius);
}

void 
init_serial_connections() 
{
    // This example will use I2C0 on the default SDA and SCL pins (GP4, GP5 on
    // a Pico)
    // I2C is "open drain", pull ups to keep signal high when no data is being sent
    i2c_init(i2c_default, I2C_FREQUENCY);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN,
                               PICO_DEFAULT_I2C_SCL_PIN,
                               GPIO_FUNC_I2C));
}

int
main() 
{
    stdio_init_all();
    init_serial_connections();
    struct sensor_reading sensor = {0, 0};

    /* Configure BME680 */
    printf("Starting sensor-node and attempting connections\n");
    while (!bme680_init()) {
        printf("ERROR: could not connect to sensor. Trying again.\n");
        sleep_ms(DELAY_MS / 6);
    }

    printf("SUCCESS: Connected to sensor.\n");
    printf("Starting readings...\n");

    for (;;) {
        sensor.temp_celcius = bme680_read_temp();
        print_sensor_value(sensor);
        sleep_ms(DELAY_MS);
    }

    return 0;
}
