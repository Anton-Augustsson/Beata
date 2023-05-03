#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#define STACK_SIZE 500
#define ERROR_MARGIN 200
#define BUTTON_DEBOUNCE_DELAY 50
#define BUTTON_HANDLE_DELAY 1

#define SLEEP_TIME 1000
#define DISABLE_CLIMATE (1 << 0)
#define DISABLE_SOUND   (1 << 1)
#define DISABLE_MOTION  (1 << 2)

/* Synchronization parameters */
K_MUTEX_DEFINE(config_lock);
K_SEM_DEFINE(climate_btn_sem, 0, 1);
K_SEM_DEFINE(sound_btn_sem, 0, 1);
K_SEM_DEFINE(motion_btn_sem, 0, 1);

// Global variables used
volatile uint8_t update_conf = 0;
static unsigned long button_time = 0;
static struct sensor_value config;
static struct sensor_value sampling_frequency = {1000, 0};

static struct gpio_dt_spec climate_led = GPIO_DT_SPEC_GET(DT_ALIAS(climateled), gpios);
static struct gpio_dt_spec sound_led = GPIO_DT_SPEC_GET(DT_ALIAS(soundled), gpios);
static struct gpio_dt_spec motion_led = GPIO_DT_SPEC_GET(DT_ALIAS(motionled), gpios);
static struct gpio_dt_spec climate_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_dt_spec sound_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static struct gpio_dt_spec motion_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);

static struct gpio_callback climate_btn_cb_data, sound_btn_cb_data, motion_btn_cb_data;

void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if ((k_uptime_get_32() - button_time) > BUTTON_DEBOUNCE_DELAY) {
        button_time = k_uptime_get_32();

        // TODO: make sure pins actually checks for the GPIO pinout
        if (pins & BIT(climate_button.pin)) {
            k_sem_give(&climate_btn_sem);
        } else if (pins & BIT(sound_button.pin)) {
            k_sem_give(&sound_btn_sem);
        } else if (pins & BIT(motion_button.pin)) {
            k_sem_give(&motion_btn_sem);
        }
    }
}

void button_task() {
    // Initialization of the ISR, button(s), and led(s)
    gpio_pin_configure_dt(&climate_button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&climate_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&climate_btn_cb_data, button_isr, BIT(climate_button.pin));
    gpio_add_callback(climate_button.port, &climate_btn_cb_data);
    gpio_pin_configure_dt(&sound_button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&sound_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&sound_btn_cb_data, button_isr, BIT(sound_button.pin));
    gpio_add_callback(sound_button.port, &sound_btn_cb_data);
    gpio_pin_configure_dt(&motion_button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&motion_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&motion_btn_cb_data, button_isr, BIT(motion_button.pin));
    gpio_add_callback(motion_button.port, &motion_btn_cb_data);
    gpio_pin_configure_dt(&climate_led, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&climate_led, 1);
    gpio_pin_configure_dt(&sound_led, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&sound_led, 1);
    gpio_pin_configure_dt(&motion_led, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&motion_led, 1);

    for (;;) {
        if (k_sem_take(&climate_btn_sem, K_NO_WAIT) == 0) {
            k_mutex_lock(&config_lock, K_FOREVER);
            gpio_pin_toggle_dt(&climate_led);
            config.val1 ^= DISABLE_CLIMATE;
            update_conf = 1;
            k_mutex_unlock(&config_lock);
        } else if (k_sem_take(&sound_btn_sem, K_NO_WAIT) == 0) {
            k_mutex_lock(&config_lock, K_FOREVER);
            gpio_pin_toggle_dt(&sound_led);
            config.val1 ^= DISABLE_SOUND;
            update_conf = 1;
            k_mutex_unlock(&config_lock);
        } else if (k_sem_take(&motion_btn_sem, K_NO_WAIT) == 0) {
            k_mutex_lock(&config_lock, K_FOREVER);
            gpio_pin_toggle_dt(&motion_led);
            config.val1 ^= DISABLE_MOTION;
            update_conf = 1;
            k_mutex_unlock(&config_lock);
        }

        k_msleep(BUTTON_HANDLE_DELAY);
    }
}

void main_task() {
    const struct device *dev = DEVICE_DT_GET_ANY(zephyr_beata);
    struct sensor_value temperature, humidity, pressure, motion, sound;

    while (!device_is_ready(dev)) {
        printk("sensor: device not ready.\n");
        k_msleep(SLEEP_TIME);
    }

    printk("Starting base station...\n");
    printk("Configuring sensor node(s)...\n");
    sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_frequency);

    for (;;) {
        if (update_conf) {
            printk("Writing new configurations...\n");
            k_mutex_lock(&config_lock, K_FOREVER);
            sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_FEATURE_MASK, &config);
            update_conf = 0;
            k_mutex_unlock(&config_lock);
        }

        if (sensor_sample_fetch(dev) < 0) {
            printk("ERROR: Could not fetch samples from sensor node.\n");
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temperature) < 0) {
            printk("ERROR: Could not fetch 'TEMP' from sensor node.\n");
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity) < 0) {
            printk("ERROR: Could not fetch 'HUMIDITY' from sensor node.\n");
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_PRESS, &pressure) < 0) {
            printk("ERROR: Could not fetch 'PRESSURE' from sensor node.\n");
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_PROX, &sound) < 0) {
            printk("ERROR: Could not fetch 'SOUND' from sensor node.\n");
        }

        if (sensor_channel_get(dev, SENSOR_CHAN_IR, &motion) < 0) {
            printk("ERROR: Could not fetch 'MOTION' from sensor node.\n");
        }

        printk("============ SENSOR READINGS ==============\n");
        printk("Humidity (%%): %d.%02d\n", humidity.val1, humidity.val2);
        printk("Pressure (hPa): %d.%02d\n", pressure.val1, pressure.val2);
        printk("Temperature (C): %d.%02d\n", temperature.val1, temperature.val2);
        printk("Has motion (bool): %d\n", motion.val1);
        printk("Sound level: %d\n", sound.val1);
        printk("===========================================\n\n");
        k_msleep(SLEEP_TIME);
    }
}

K_THREAD_DEFINE(button_thread, STACK_SIZE, button_task, NULL, NULL, NULL, 1, 0, 0);
K_THREAD_DEFINE(main_thread, STACK_SIZE, main_task, NULL, NULL, NULL, 1, 0, 0);