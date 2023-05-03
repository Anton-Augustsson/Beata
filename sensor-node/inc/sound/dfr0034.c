#include "dfr0034.h"

#define DFR0034_GPIO_PIN 27
#define DFR0034_STABILITY_TIME_MS 30 * 1000

static volatile bool circuit_stable = false;

static int64_t circuit_stable_callback(alarm_id_t id, void *user_data) {
    printf("DFR0034: Circuit is now stable\n");
    circuit_stable = true;
    /* Do not re-run alarm. */
    return 0;
}

error_t dfr0034_init() {
    adc_gpio_init(DFR0034_GPIO_PIN);
    add_alarm_in_ms(DFR0034_STABILITY_TIME_MS, circuit_stable_callback, NULL, false);
    return SUCCESS;
}

dfr0034_rslt_t dfr0034_read_sound() {
    if (!circuit_stable)
        return (dfr0034_rslt_t) {
        .error = NOT_READY
    };

    adc_select_input(DFR0034_ADC_CHANNEL);

    uint16_t value = adc_read();

    /* TODO: Convert to dB */
    return (dfr0034_rslt_t) {
        .error = SUCCESS,
        .data = value
    };
}
