#include <stdio.h>

#include "../inc/bosch/bme680.h"
#include "../inc/common.h"
#include "../inc/motion/amn1.h"
#include "../inc/sound/dfr0034.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/i2c_slave.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/util/queue.h"

#define I2C_BAUDRATE 400000
#define I2C_SLAVE_ADDRESS 0x17
#define I2C_SLAVE_SDA_PIN 16
#define I2C_SLAVE_SCL_PIN 17

/* Memory registers to read/write */
#define SENSOR_NODE_TEMP 0x00
#define SENSOR_NODE_HUM 0x04
#define SENSOR_NODE_PRESS 0x08
#define SENSOR_NODE_SOUND 0x0C
#define SENSOR_NODE_MOTION 0x0E
#define SENSOR_NODE_CONFIG 0x0F

#define SENSOR_QUERY_PERIOD_MS 3000
#define INIT_RETRY_DELAY_MS (SENSOR_QUERY_PERIOD_MS / 6)

/* Implement a memory. The master first writes the memory address, followed by
   the data.*/
static struct
{
    uint8_t mem[256];
    uint8_t mem_address;
    uint8_t mem_address_written;
} context;

void
i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event)
{
    /* Called from ISR, so keep it concise. */
    switch (event)
    {
	case I2C_SLAVE_RECEIVE:
	    if (!context.mem_address_written) {
		// writes always start with the memory address
		context.mem_address = i2c_read_byte_raw(i2c);
		context.mem_address_written = 1;
	    } else {
		// save into memory
		context.mem[context.mem_address] = i2c_read_byte_raw(i2c);
		context.mem_address++;
	    }
	    break;
	case I2C_SLAVE_REQUEST:
	    // load from memory
	    i2c_write_byte_raw(i2c, context.mem[context.mem_address]);
	    context.mem_address++;
	    break;
	case I2C_SLAVE_FINISH:
	    context.mem_address_written = 0;
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

    i2c_init(i2c0, I2C_BAUDRATE);
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

void
read_all_sensor_values()
{
    bme680_rslt_t temp_celsius = bme680_read_temp();
    if (temp_celsius.error == ERROR)
	printf("BME680_ERROR: Could not fetch temperature.");

    bme680_rslt_t humidity = bme680_read_hum();
    if (humidity.error == ERROR)
	printf("BME680_ERROR: Could not fetch humidity.");

    bme680_rslt_t press = bme680_read_press();
    if (press.error == ERROR) printf("BME680_ERROR: Could not fetch pressure.");

    dfr0034_rslt_t sound_level = dfr0034_read_sound();
    if (sound_level.error == ERROR)
	printf("DFR0034_ERROR: Could not check sound level.");

    amn1_rslt_t has_motion = amn1_read_motion();
    if (has_motion.error == ERROR)
	printf("AMN1_ERROR: Could not check for motion.");

    memcpy(&context.mem[SENSOR_NODE_TEMP], &temp_celsius.data, sizeof(int32_t));
    memcpy(&context.mem[SENSOR_NODE_HUM], &humidity.data, sizeof(int32_t));
    memcpy(&context.mem[SENSOR_NODE_PRESS], &press.data, sizeof(int32_t));
    memcpy(&context.mem[SENSOR_NODE_SOUND], &has_motion.data, sizeof(uint16_t));
    memcpy(&context.mem[SENSOR_NODE_MOTION], &has_motion.data, sizeof(uint8_t));
}

void
sensors_init()
{
    while (bme680_init() != SUCCESS)
    {
	printf("ERROR: could not connect to BME680, retrying...\n");
	sleep_ms(INIT_RETRY_DELAY_MS);
    }

    while (amn1_init() != SUCCESS)
    {
	printf("ERROR: could not connect to AMN1, retrying...\n");
	sleep_ms(INIT_RETRY_DELAY_MS);
    }

    while (dfr0034_init() != SUCCESS)
    {
	printf("ERROR: could not connect to DFR0034, retrying...\n");
	sleep_ms(INIT_RETRY_DELAY_MS);
    }

    read_all_sensor_values();
}

int
main()
{
    stdio_init_all();
    adc_init();
    sensors_init();
    base_station_i2c_init();

    for (;;)
    {
	printf("Starting conversion...\n");
	read_all_sensor_values();
	sleep_ms(SENSOR_QUERY_PERIOD_MS);
    }

    return 0;
}
