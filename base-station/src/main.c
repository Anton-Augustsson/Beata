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
#define NUM_OF_SHOW_LEDS 5

#define SLEEP_TIME 1000
#define DISABLE_CLIMATE (1 << 0)
#define DISABLE_SOUND   (1 << 1)
#define DISABLE_MOTION  (1 << 2)

/* Trigger on interupts */
#define UCEL_PER_CEL 1000000
#define UCEL_PER_MCEL 1000
#define TEMP_INITIAL_CEL 25
#define TEMP_WINDOW_HALF_UCEL 500000

/* Synchronization parameters */
K_MUTEX_DEFINE(config_lock);
K_SEM_DEFINE(mode_btn_sem, 0, 1);
K_SEM_DEFINE(select_btn_sem, 0, 1);
K_SEM_DEFINE(config_btn_sem, 0, 1);

// Global variables used
volatile uint8_t update_conf = 0;
static unsigned long button_time = 0;
static struct sensor_trigger temp_trig;
static struct sensor_trigger hum_trig;
static struct sensor_trigger press_trig;
static struct sensor_trigger sound_trig;
static struct sensor_trigger motion_trig;
static struct sensor_value config;
static struct sensor_value temperature, humidity, pressure, motion, sound;
static struct sensor_value sampling_frequency = {50};
static struct sensor_value threshold_temp_upper = {30};
static struct sensor_value threshold_temp_lower = {20};
static struct sensor_value threshold_hum_upper = {60};
static struct sensor_value threshold_hum_lower = {5};
static struct sensor_value threshold_press_upper = {130};
static struct sensor_value threshold_press_lower = {100};
static struct sensor_value threshold_sound_upper = {1000};
static struct sensor_value threshold_sound_lower = {5};

static struct gpio_dt_spec led_show0 = GPIO_DT_SPEC_GET(DT_ALIAS(ledshow0), gpios);
static struct gpio_dt_spec led_show1 = GPIO_DT_SPEC_GET(DT_ALIAS(ledshow1), gpios);
static struct gpio_dt_spec led_show2 = GPIO_DT_SPEC_GET(DT_ALIAS(ledshow2), gpios);
static struct gpio_dt_spec led_show3 = GPIO_DT_SPEC_GET(DT_ALIAS(ledshow3), gpios);
static struct gpio_dt_spec led_show4 = GPIO_DT_SPEC_GET(DT_ALIAS(ledshow4), gpios);
static struct gpio_dt_spec led_selected0 = GPIO_DT_SPEC_GET(DT_ALIAS(ledselect0), gpios);
static struct gpio_dt_spec led_selected1 = GPIO_DT_SPEC_GET(DT_ALIAS(ledselect1), gpios);
static struct gpio_dt_spec led_selected2 = GPIO_DT_SPEC_GET(DT_ALIAS(ledselect2), gpios);
static struct gpio_dt_spec led_selected_mode = GPIO_DT_SPEC_GET(DT_ALIAS(ledselect3), gpios);

static struct gpio_dt_spec mode_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_dt_spec select_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);
static struct gpio_dt_spec config_button = GPIO_DT_SPEC_GET(DT_ALIAS(sw2), gpios);

static struct gpio_callback mode_btn_cb_data, select_btn_cb_data, config_btn_cb_data;


/* Function pointer primitive */
typedef void (*state_func_t)(void);

typedef struct _state_t {
    uint8_t id;
    state_func_t Enter;
    state_func_t Do;
    state_func_t Exit;
    uint32_t delay_ms;
} state_t;

typedef enum _event_t {
    b1_evt = 0, // mode btn
    b2_evt = 1, // select btn
    b3_evt = 2, // config btn
    no_evt = 3
} event_t;

typedef enum _visual_select_t {
    motion_select,
    climate_temp_select,
    climate_hum_select,
    climate_press_select,
    sound_select,
    config_select
} visual_select_t;

typedef enum _toggle_config_select_t {
    toggle_motion_select,
    toggle_climate_select,
    toggle_sound_select,
} toggle_config_select_t;

typedef enum _base_station_mode_t {
    visual_mode,
    config_mode
} base_station_mode_t;

typedef enum _prev_config_state_t {
    motion_state,
    climate_state,
    sound_state,
    configure_state
} prev_config_state_t;

// No global variables
prev_config_state_t set_get_prev_config_state(prev_config_state_t prev_config_state_in) {
    static prev_config_state_t prev_config_state = motion_state;

    if (prev_config_state_in != configure_state) {
        prev_config_state = prev_config_state_in;
    }

    return prev_config_state;
}


/* The next three methods are for convenience, you might want to use them. */
event_t get_event(void) {
    if (k_sem_take(&mode_btn_sem, K_NO_WAIT) == 0) {
        printk("Get_event: mode_btn b1_evt\n");
        return b1_evt;
    } else if (k_sem_take(&select_btn_sem, K_NO_WAIT) == 0) {
        printk("Get_event: select_btn b2_evt\n");
        return b2_evt;
    } else if (k_sem_take(&config_btn_sem, K_NO_WAIT) == 0) {
        printk("Get_event: config_btn b3_evt\n");
        return b3_evt;
    } else {
        return no_evt;
    }
}

//////////////////////////////////////////////////////////////////////////////
/* BEGIN: LEDs should only be modified within this section to proven't bugs */
//////////////////////////////////////////////////////////////////////////////

/* Initialize all LEDs */
void leds_init() {
    gpio_pin_configure_dt(&led_show0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&led_show0, 1);
    gpio_pin_configure_dt(&led_show1, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&led_show1, 1);
    gpio_pin_configure_dt(&led_show2, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&led_show2, 1);
    gpio_pin_configure_dt(&led_show3, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&led_show3, 1);
    gpio_pin_configure_dt(&led_show4, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&led_show4, 1);
    gpio_pin_configure_dt(&led_selected0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&led_selected0, 1);
    gpio_pin_configure_dt(&led_selected1, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&led_selected1, 1);
    gpio_pin_configure_dt(&led_selected2, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&led_selected2, 1);
    gpio_pin_configure_dt(&led_selected_mode, GPIO_OUTPUT_ACTIVE);
    gpio_pin_set_dt(&led_selected_mode, 1);
}

/* Turn off all show LEDs */
void leds_show_off() {
    gpio_pin_set_dt(&led_show0, 0);
    gpio_pin_set_dt(&led_show1, 0);
    gpio_pin_set_dt(&led_show2, 0);
    gpio_pin_set_dt(&led_show3, 0);
    gpio_pin_set_dt(&led_show4, 0);
}

/*  Turn off all selected LEDs except mode.
    Does not shut down select_mode since this function is used to clear 
    which visual or configuration has been selected, not mode.
*/
void leds_select_off() {
    gpio_pin_set_dt(&led_selected0, 0);
    gpio_pin_set_dt(&led_selected1, 0);
    gpio_pin_set_dt(&led_selected2, 0);
}

/* Turn off all LEDs.
   Call every time a state is entered, do, and exiting to prevent bugs.
*/
void leds_off() {
    leds_show_off();
    leds_select_off();
    gpio_pin_set_dt(&led_selected_mode, 0);
}

/* Turn on selected mode LED if visual_mode and off if config_mode 
    Call every time you in the do function for all states.
*/
void led_selected_mode_on(base_station_mode_t mode) {
    if (mode == visual_mode) {
        gpio_pin_set_dt(&led_selected_mode, 0);
    } else if (mode == config_mode) {
        gpio_pin_set_dt(&led_selected_mode, 1);
    }
}

/* Turn on show LEDs (0, 1, 2) that correspond to each config option 
    Turns off other show LEDs.
*/
void leds_show_config() {
    leds_show_off();

    if ((config.val1 & DISABLE_CLIMATE) == 0) {
        gpio_pin_set_dt(&led_show0, 1);
    }

    if ((config.val1 & DISABLE_SOUND) == 0) {
        gpio_pin_set_dt(&led_show1, 1);
    }

    if ((config.val1 & DISABLE_MOTION) == 0) {
        gpio_pin_set_dt(&led_show2, 1);
    }
}

/* Turn on show LEDs that correspond to each visual_select.
    Turns off all other show LEDs that is not used to display 
    the visual_select.
*/
void leds_show_visual_select(visual_select_t visual_select) {
    leds_select_off();

    if (visual_select == motion_select) {
        gpio_pin_set_dt(&led_selected0, 1);
    }
    else if (visual_select == climate_temp_select) {
        gpio_pin_set_dt(&led_selected1, 1);
    }
    else if (visual_select == climate_hum_select) {
        gpio_pin_set_dt(&led_selected0, 1);
        gpio_pin_set_dt(&led_selected1, 1);
    }
    else if (visual_select == climate_press_select) {
        gpio_pin_set_dt(&led_selected1, 1);
        gpio_pin_set_dt(&led_selected2, 1);
    }
    else if (visual_select == sound_select) {
        gpio_pin_set_dt(&led_selected2, 1);
    }
}

/* Turn on show LEDs that correspond to each toggle_config_select.
    Turns off all other show LEDs that is not used to display 
    the toggle_config_select.
*/
void leds_show_toggle_config_select(toggle_config_select_t toggle_config_select) {
    leds_select_off();

    if (toggle_config_select == toggle_motion_select) {
        gpio_pin_set_dt(&led_selected0, 1);
    }
    else if (toggle_config_select == toggle_climate_select) {
        gpio_pin_set_dt(&led_selected1, 1);
    }
    else if (toggle_config_select == toggle_sound_select) {
        gpio_pin_set_dt(&led_selected2, 1);
    }
}

void leds_show_level(uint16_t value, uint16_t min_level, uint16_t max_level) {
    leds_show_off();
    int num_of_led_to_turn_on = ((value - min_level) * NUM_OF_SHOW_LEDS) / (max_level - min_level);
    
    gpio_pin_set_dt(&led_show0, 1); // One LED is always on
    for (int i = 2; i <= num_of_led_to_turn_on; i++) {
        if (i == 2) { gpio_pin_set_dt(&led_show1, 1); }
        else if (i == 3) { gpio_pin_set_dt(&led_show2, 1); }
        else if (i == 4) { gpio_pin_set_dt(&led_show3, 1); }
        else if (i == 5) { gpio_pin_set_dt(&led_show4, 1); }
    }
}

////////////////////////////////////////////////////////////////////////////
/* END: LEDs should only be modified within this section to proven't bugs */
////////////////////////////////////////////////////////////////////////////


/* Show motion */
void do_state_v0(void) {
    leds_off();
    led_selected_mode_on(visual_mode);
    leds_show_visual_select(motion_select); 
    // TODO: have proper limits and values
    leds_show_level((uint16_t)motion.val1, 0, 100);
}

void enter_state_v0(void) {
    printk("Entering Show motion\n");
    leds_off();
}

void exit_state_v0(void) {
    printk("Exiting Show motion\n");
    leds_off();
}

const state_t state_v0 = {
    0,
    enter_state_v0,
    do_state_v0,
    exit_state_v0,
    300
};

/* Show climate temp */
void do_state_v1(void) {
    leds_off();
    led_selected_mode_on(visual_mode);
    leds_show_visual_select(climate_temp_select); 
    // TODO: have proper limits and values
    leds_show_level((uint16_t)temperature.val1, 0, 100);
}

void enter_state_v1(void) {
    printk("Entering Show climate temp\n");
    leds_off();
}

void exit_state_v1(void) {
    printk("Exiting Show climate temp\n");
    leds_off();
}

const state_t state_v1 = {
    1,
    enter_state_v1,
    do_state_v1,
    exit_state_v1,
    300
};


/* Show climate humidity */
void do_state_v2(void) {
    leds_off();
    led_selected_mode_on(visual_mode);
    leds_show_visual_select(climate_hum_select);  
    // TODO: have proper limits and values
    leds_show_level((uint16_t)humidity.val1, 0, 100);
}

void enter_state_v2(void) {
    printk("Entering Show climate humidity\n");
    leds_off();
}

void exit_state_v2(void) {
    printk("Exiting Show climate humidity\n");
    leds_off();
}

const state_t state_v2 = {
    2,
    enter_state_v2,
    do_state_v2,
    exit_state_v2,
    300
};


/* Show climate pressure */
void do_state_v3(void) {
    leds_off();
    led_selected_mode_on(visual_mode);
    leds_show_visual_select(climate_press_select); 
    // TODO: have proper limits and values
    leds_show_level((uint16_t)pressure.val1, 0, 100);
}

void enter_state_v3(void) { 
    printk("Entering Show climate pressure\n");
    leds_off(); 
}

void exit_state_v3(void) { 
    printk("Exiting Show climate pressure\n");
    leds_off(); 
}

const state_t state_v3 = {
    3,
    enter_state_v3,
    do_state_v3,
    exit_state_v3,
    300
};


/* Show sound */
void do_state_v4(void) {
    leds_off();
    led_selected_mode_on(visual_mode);
    leds_show_visual_select(sound_select);
    // TODO: have proper limits and values
    leds_show_level((uint16_t)sound.val1, 0, 100);
}

void enter_state_v4(void) {
    printk("Entering Show sound\n");
    leds_off();
}
void exit_state_v4(void) {
    printk("Exiting Show sound\n");
    leds_off();
}

const state_t state_v4 = {
    4,
    enter_state_v4,
    do_state_v4,
    exit_state_v4,
    300
};


/* Show config */
void do_state_v5(void) {
    leds_off();
    led_selected_mode_on(visual_mode);
    leds_show_visual_select(config_select); 
    leds_show_config();
}

void enter_state_v5(void) {
    printk("Entering Show config\n");
    leds_off();
}

void exit_state_v5(void) {
    printk("Exiting Show config\n");
    leds_off();
}

const state_t state_v5 = {
    5,
    enter_state_v5,
    do_state_v5,
    exit_state_v5,
    300
};


/* Toggle motion */
void do_state_c0(void) {
    leds_off();
    led_selected_mode_on(config_mode);
    leds_show_toggle_config_select(toggle_motion_select);
    leds_show_config();
}

void enter_state_c0(void) {
    printk("Entering Toggle motion\n");
    leds_off();
}

void exit_state_c0(void) {
    printk("Exiting Toggle motion\n");
    leds_off();
    set_get_prev_config_state(motion_state);
}

const state_t state_c0 = {
    6,
    enter_state_c0,
    do_state_c0,
    exit_state_c0,
    300
};


/* Toggle climate */
void do_state_c1(void) {
    leds_off();
    led_selected_mode_on(config_mode);
    leds_show_toggle_config_select(toggle_climate_select);
    leds_show_config();
}

void enter_state_c1(void) {
    printk("Entering Toggle climate\n");
    leds_off();
}

void exit_state_c1(void) {
    printk("Exiting Toggle climate\n");
    leds_off();
    set_get_prev_config_state(sound_state);
}

const state_t state_c1 = {
    7,
    enter_state_c1,
    do_state_c1,
    exit_state_c1,
    300
};


/* Toggle sound */
void do_state_c2(void) {
    leds_off();
    led_selected_mode_on(config_mode);
    leds_show_toggle_config_select(toggle_sound_select);
    leds_show_config();
}

void enter_state_c2(void) {
    printk("Entering Toggle sound\n");
    leds_off();
}

void exit_state_c2(void) {
    printk("Exiting Toggle sound\n");
    leds_off();
    set_get_prev_config_state(sound_state);
}


const state_t state_c2 = {
    8,
    enter_state_c2,
    do_state_c2,
    exit_state_c2,
    300
};

/* Change the value of the config */
void do_state_cc(void) {
    leds_off();
    led_selected_mode_on(config_mode);
    prev_config_state_t prev_config_state = set_get_prev_config_state(configure_state);

    if (prev_config_state == sound_state) {
        k_mutex_lock(&config_lock, K_FOREVER);
        config.val1 ^= DISABLE_CLIMATE;
        update_conf = 1;
        k_mutex_unlock(&config_lock);
    } else if (prev_config_state == climate_state) {
        k_mutex_lock(&config_lock, K_FOREVER);
        config.val1 ^= DISABLE_SOUND;
        update_conf = 1;
        k_mutex_unlock(&config_lock);
    } else if (prev_config_state == motion_state) {
        k_mutex_lock(&config_lock, K_FOREVER);
        config.val1 ^= DISABLE_MOTION;
        update_conf = 1;
        k_mutex_unlock(&config_lock);
    }

    leds_show_config();
}

void enter_state_cc(void) {
    printk("Entering change config\n");
    leds_off();
}

void exit_state_cc(void) {
    printk("Exiting change config\n");
    leds_off();
}

const state_t state_cc = {
    9,
    enter_state_cc,
    do_state_cc,
    exit_state_cc,
    300
};


const state_t state_table[10][4] = {
    /*  STATE  B1          B2          B3         NO-EVT */
    {/* S0 */  state_c0,   state_v1,   state_v0,  state_v0}, // show motion
    {/* S1 */  state_c0,   state_v2,   state_v1,  state_v1}, // show temp
    {/* S2 */  state_c0,   state_v3,   state_v2,  state_v2}, // show humidity
    {/* S3 */  state_c0,   state_v4,   state_v3,  state_v3}, // show pressure
    {/* S4 */  state_c0,   state_v5,   state_v4,  state_v4}, // show sound
    {/* S5 */  state_c0,   state_v0,   state_v5,  state_v5}, // show config
    {/* S6 */  state_v0,   state_c1,   state_cc,  state_c0}, // toggle motion
    {/* S7 */  state_v0,   state_c2,   state_cc,  state_c1}, // toggle climate
    {/* S8 */  state_v0,   state_c0,   state_cc,  state_c2}, // toggle sound
    {/* S9 */  state_v0,   state_c0,   state_cc,  state_cc}  // change config
};



/* Handle triggers */
static void temp_trigger_handler(const struct device *dev, const struct sensor_trigger *trig) {
    printk("TEMPERATURE THRESHOLD NOW BAMS\n");
}
static void hum_trigger_handler(const struct device *dev, const struct sensor_trigger *trig) {
    printk("HUMIDITY THRESHOLD NOW BAMS\n");
}

static void press_trigger_handler(const struct device *dev, const struct sensor_trigger *trig) {
    printk("PRESS THRESHOLD NOW BAMS\n");
}

static void motion_trigger_handler(const struct device *dev, const struct sensor_trigger *trig) {
    printk("MOTION NOW BAMS\n");
}

static void sound_trigger_handler(const struct device *dev, const struct sensor_trigger *trig) {
    printk("SOUND THRESHOLD NOW BAMS\n");
}

void button_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    if ((k_uptime_get_32() - button_time) > BUTTON_DEBOUNCE_DELAY) {
        button_time = k_uptime_get_32();

        if (pins & BIT(mode_button.pin)) {
            k_sem_give(&mode_btn_sem);
        } else if (pins & BIT(select_button.pin)) {
            k_sem_give(&select_btn_sem);
        } else if (pins & BIT(config_button.pin)) {
            k_sem_give(&config_btn_sem);
        }
    }
}

void interface_init() {
    /* Initialise buttons */
    gpio_pin_configure_dt(&mode_button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&mode_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&mode_btn_cb_data, button_isr, BIT(mode_button.pin));
    gpio_add_callback(mode_button.port, &mode_btn_cb_data);
    gpio_pin_configure_dt(&select_button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&select_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&select_btn_cb_data, button_isr, BIT(select_button.pin));
    gpio_add_callback(select_button.port, &select_btn_cb_data);
    gpio_pin_configure_dt(&config_button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&config_button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&config_btn_cb_data, button_isr, BIT(config_button.pin));
    gpio_add_callback(config_button.port, &config_btn_cb_data);
    /* Initialise led */
    leds_init();
}

void user_interface_task() {
    printk("Started\n");
    interface_init();

    // TODO: remove
    config.val1 = 0;
    config.val2 = 0;
    temperature.val1 = 0;
    temperature.val2 = 0;
    humidity.val1 = 10;
    humidity.val2 = 0;
    pressure.val1 = 40;
    pressure.val2 = 0;
    motion.val1 = 60;
    motion.val2 = 0;
    sound.val1 = 90;
    sound.val2 = 0;

    state_t current_state = state_v0;
    event_t evt = no_evt;

    for (;;) {
        printk("New iteration\n");
        current_state.Enter();
        evt = get_event();

        while(current_state.id == state_table[current_state.id][evt].id) {
            current_state.Do();
            k_msleep(current_state.delay_ms);
            evt = get_event();
        }

        current_state.Exit();
        current_state = state_table[current_state.id][evt];
    }
}

void main_task() {
    const struct device *dev = DEVICE_DT_GET_ANY(zephyr_beata);
    //struct sensor_value temperature, humidity, pressure, motion, sound;
    printk("*******************************************\n");

    while (!device_is_ready(dev)) {
        printk("Sensor: device not ready.\n");
        k_msleep(SLEEP_TIME);
    }

    printk("Starting base station...\n");
    printk("Configuring sensor node(s)...\n");
    printk("Setting sampling frequency...\n");
    sensor_attr_set(dev, SENSOR_CHAN_ALL, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_frequency);
#ifdef CONFIG_BEATA_TRIGGER
    int ret;
    printk("Enabling trigger for temperature...\n");
    temp_trig.type = SENSOR_TRIG_THRESHOLD;
    temp_trig.chan = SENSOR_CHAN_AMBIENT_TEMP;
    ret = sensor_trigger_set(dev, &temp_trig, temp_trigger_handler);

    if (ret < 0) {
        printk("Temperature trigger set failed: %d\n", ret);
    }

    printk("Enabling trigger for humidity...\n");
    hum_trig.type = SENSOR_TRIG_THRESHOLD;
    hum_trig.chan = SENSOR_CHAN_HUMIDITY;
    ret = sensor_trigger_set(dev, &hum_trig, hum_trigger_handler);

    if (ret < 0) {
        printk("Humidity trigger set failed: %d\n", ret);
    }

    printk("Enabling trigger for pressure...\n");
    press_trig.type = SENSOR_TRIG_THRESHOLD;
    press_trig.chan = SENSOR_CHAN_PRESS;
    ret = sensor_trigger_set(dev, &press_trig, press_trigger_handler);

    if (ret < 0) {
        printk("Pressure trigger set failed: %d\n", ret);
    }

    printk("Enabling trigger for sound level...\n");
    sound_trig.type = SENSOR_TRIG_THRESHOLD;
    sound_trig.chan = SENSOR_CHAN_PROX;
    ret = sensor_trigger_set(dev, &sound_trig, sound_trigger_handler);

    if (ret < 0) {
        printk("Sound trigger set failed: %d\n", ret);
    }

    printk("Enabling trigger for motion...\n");
    motion_trig.type = SENSOR_TRIG_MOTION;
    motion_trig.chan = SENSOR_CHAN_IR;
    ret = sensor_trigger_set(dev, &motion_trig, motion_trigger_handler);

    if (ret < 0) {
        printk("Motion trigger set failed: %d\n", ret);
    }

    printk("Setting trigger thresholds...\n");
    sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_UPPER_THRESH, &threshold_temp_upper);
    sensor_attr_set(dev, SENSOR_CHAN_AMBIENT_TEMP, SENSOR_ATTR_LOWER_THRESH, &threshold_temp_lower);
    sensor_attr_set(dev, SENSOR_CHAN_HUMIDITY, SENSOR_ATTR_UPPER_THRESH, &threshold_hum_upper);
    sensor_attr_set(dev, SENSOR_CHAN_HUMIDITY, SENSOR_ATTR_LOWER_THRESH, &threshold_hum_lower);
    sensor_attr_set(dev, SENSOR_CHAN_PRESS, SENSOR_ATTR_UPPER_THRESH, &threshold_press_upper);
    sensor_attr_set(dev, SENSOR_CHAN_PRESS, SENSOR_ATTR_LOWER_THRESH, &threshold_press_lower);
    sensor_attr_set(dev, SENSOR_CHAN_PROX, SENSOR_ATTR_UPPER_THRESH, &threshold_sound_upper);
    sensor_attr_set(dev, SENSOR_CHAN_PROX, SENSOR_ATTR_LOWER_THRESH, &threshold_sound_lower);
#endif
    printk("*******************************************\n\n");
    k_msleep(3000);

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

//K_THREAD_DEFINE(button_thread, STACK_SIZE, button_task, NULL, NULL, NULL, 1, 0, 0);
K_THREAD_DEFINE(button_thread, STACK_SIZE, user_interface_task, NULL, NULL, NULL, 1, 0, 0);
//K_THREAD_DEFINE(main_thread, STACK_SIZE, main_task, NULL, NULL, NULL, 1, 0, 0);
