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
#define INTERRUPT_PIN 18

/* Memory registers to read/write */
#define REG_TEMP 0x00
#define REG_HUM 0x04
#define REG_PRESS 0x08
#define REG_SOUND 0x0C
#define REG_MOTION 0x0E
#define REG_SAMPLING_FREQUENCY 0x0F
#define REG_DISABLED_SENSORS 0x11

/* Interrupt (trigger) enable registers */
#define REG_INT_STATUS    0x12 // Which interrupt was triggered
#define REG_INT_THRESHOLD 0x13
#define REG_INT_MOTION    0x14

/* Trigger target value registers */
#define REG_INT_TEMP_LOW    0x15
#define REG_INT_TEMP_HIGH   0x19
#define REG_INT_HUM_LOW     0x1d
#define REG_INT_HUM_HIGH    0x21
#define REG_INT_PRESS_LOW   0x25
#define REG_INT_PRESS_HIGH  0x29
#define REG_INT_SOUND_LOW   0x2d
#define REG_INT_SOUND_HIGH  0x31

/* Sensor node id */
#define REG_ID  0xFF

#define SENSOR_NODE_INT_STATUS_TEMP     0
#define SENSOR_NODE_INT_STATUS_HUM      1
#define SENSOR_NODE_INT_STATUS_PRESS    2
#define SENSOR_NODE_INT_STATUS_SOUND    3
#define SENSOR_NODE_INT_STATUS_MOTION   4

#define DISABLED_MASK_CLIMATE (1 << 0)
#define DISABLED_MASK_SOUND (1 << 1)
#define DISABLED_MASK_MOTION (1 << 2)

#define INIT_RETRY_DELAY_MS 1000

static uint8_t motion_prev = 0;
static int32_t no_threshold = INT32_MIN;
static uint8_t prev_int_status = 0;

static struct {
    uint8_t mem[256];
    uint8_t mem_address;
    uint8_t mem_address_written;
} context;

void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    /* Called from ISR, so keep it concise. */
    switch (event) {
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
void base_station_i2c_init() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_init(INTERRUPT_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_dir(INTERRUPT_PIN, GPIO_OUT);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);
    i2c_init(i2c0, I2C_BAUDRATE);
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

uint8_t check_threshold(uint8_t reg_low, uint8_t reg_high, uint8_t status_bit, int32_t value) {
    int32_t t_low, t_high;
    memcpy(&t_low, &context.mem[reg_low], sizeof(int32_t));
    memcpy(&t_high, &context.mem[reg_high], sizeof(int32_t));
    if ((t_low != no_threshold && t_low > value) || (t_high != no_threshold && t_high < value)) {
        return 1 << status_bit;
    }
    return 0;
}

void read_all_sensor_values() {
    uint8_t int_status = 0;

    // Clear memory to ensure that disabled sensors always produce zeros
    memset(&context.mem[REG_TEMP], 0, sizeof(int32_t));
    memset(&context.mem[REG_HUM], 0, sizeof(int32_t));
    memset(&context.mem[REG_PRESS], 0, sizeof(int32_t));
    memset(&context.mem[REG_MOTION], 0, sizeof(uint8_t));
    memset(&context.mem[REG_SOUND], 0, sizeof(uint16_t));

    if (!(context.mem[REG_DISABLED_SENSORS] & DISABLED_MASK_CLIMATE)) {
        bme680_rslt_t temp_celsius = bme680_read_temp();

        if (temp_celsius.error == ERROR) {
            printf("BME680_ERROR: Could not fetch temperature.");
        }

        bme680_rslt_t humidity = bme680_read_hum();

        if (humidity.error == ERROR) {
            printf("BME680_ERROR: Could not fetch humidity.");
        }

        bme680_rslt_t press = bme680_read_press();

        if (press.error == ERROR) {
            printf("BME680_ERROR: Could not fetch pressure.");
        }

        if (context.mem[REG_INT_THRESHOLD]) {
            int_status |= check_threshold(
                REG_INT_TEMP_LOW,
                REG_INT_TEMP_HIGH,
                SENSOR_NODE_INT_STATUS_TEMP,
                temp_celsius.data
            );

            int_status |= check_threshold(
                REG_INT_HUM_LOW,
                REG_INT_HUM_HIGH,
                SENSOR_NODE_INT_STATUS_HUM,
                humidity.data
            );

            int_status |= check_threshold(
                REG_INT_PRESS_LOW,
                REG_INT_PRESS_HIGH,
                SENSOR_NODE_INT_STATUS_PRESS,
                press.data
            );
        }

        memcpy(&context.mem[REG_TEMP], &temp_celsius.data, sizeof(int32_t));
        memcpy(&context.mem[REG_HUM], &humidity.data, sizeof(int32_t));
        memcpy(&context.mem[REG_PRESS], &press.data, sizeof(int32_t));
    }

    if (!(context.mem[REG_DISABLED_SENSORS] & DISABLED_MASK_SOUND)) {
        dfr0034_rslt_t sound_level = dfr0034_read_sound();

        if (sound_level.error == ERROR) {
            printf("DFR0034_ERROR: Could not check sound level.");
        }

        if (context.mem[REG_INT_THRESHOLD]) {
            int_status |= check_threshold(
                REG_INT_SOUND_LOW,
                REG_INT_SOUND_HIGH,
                SENSOR_NODE_INT_STATUS_SOUND,
                (int32_t)sound_level.data
            );
        }

        memcpy(&context.mem[REG_SOUND], &sound_level.data, sizeof(uint16_t));
    }

    if (!(context.mem[REG_DISABLED_SENSORS] & DISABLED_MASK_MOTION)) {
        amn1_rslt_t has_motion = amn1_read_motion();

        if (has_motion.error == ERROR) {
            printf("AMN1_ERROR: Could not check for motion.");
        }

        /* Check if interrupt for motion is enabled, and trigger it
           whenever it changes. */
        if (context.mem[REG_INT_MOTION]) {
            if (motion_prev != has_motion.data) {
                int_status |= 1 << SENSOR_NODE_INT_STATUS_MOTION;
            }

            motion_prev = has_motion.data;
        }

        memcpy(&context.mem[REG_MOTION], &has_motion.data, sizeof(uint8_t));
    }
    /* Update interrupt status based on current readings.
       If no interrupts will be triggered during this iteration,
       it will be set to 0. */
    memcpy(&context.mem[REG_INT_STATUS], &int_status, sizeof(int_status));

    /* trigger alarm */
    if (int_status != 0 && prev_int_status != int_status)
        gpio_xor_mask(1 << INTERRUPT_PIN);

    prev_int_status = int_status;
}

void sensors_init() {
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

    read_all_sensor_values();
}

int main() {
    stdio_init_all();
    adc_init();
    sensors_init();
    base_station_i2c_init();

    uint16_t sampling_time = 0;
    printf("Starting sensor node...\n");
    memset(&context.mem[REG_ID], 17, 1);

    // Reset all interrupt thresholds to a known value.
    // This way we can differentiate between a set and unset value.
    memcpy(&context.mem[REG_INT_TEMP_LOW], &no_threshold, sizeof(no_threshold));
    memcpy(&context.mem[REG_INT_TEMP_HIGH], &no_threshold, sizeof(no_threshold));
    memcpy(&context.mem[REG_INT_HUM_LOW], &no_threshold, sizeof(no_threshold));
    memcpy(&context.mem[REG_INT_HUM_HIGH], &no_threshold, sizeof(no_threshold));
    memcpy(&context.mem[REG_INT_PRESS_LOW], &no_threshold, sizeof(no_threshold));
    memcpy(&context.mem[REG_INT_PRESS_HIGH], &no_threshold, sizeof(no_threshold));
    memcpy(&context.mem[REG_INT_SOUND_LOW], &no_threshold, sizeof(no_threshold));
    memcpy(&context.mem[REG_INT_SOUND_HIGH], &no_threshold, sizeof(no_threshold));

    for (;;) {
        // Reset interrupt status
        memset(&context.mem[REG_INT_STATUS], 0, 1);

        read_all_sensor_values();

        // TODO: This operation is not safe
        memcpy(&sampling_time, &context.mem[REG_SAMPLING_FREQUENCY], sizeof(uint16_t));
        sleep_ms(sampling_time);
    }

    return 0;
}
