#include "amn1.h"

/* TODO: Set correct pin. */
#define AMN1_GPIO_PIN 26

/* GPIO [26..29] maps to ADC channel [0..3], respectively */
#define AMN1_ADC_CHANNEL 0

#define AMN1_STABILITY_TIME_MS 30 * 1000
#define AMN1_MOTION_THRESHOLD_V 3

static volatile bool circuit_stable = false;

static int64_t
circuit_stable_callback(alarm_id_t id, void *user_data)
{
	printf("AMN1: Circuit is now stable\n");
	circuit_stable = true;
	/* Do not re-run alarm. */
	return 0;
}

error_t
amn1_init()
{
	adc_gpio_init(AMN1_GPIO_PIN);
	add_alarm_in_ms(AMN1_STABILITY_TIME_MS, circuit_stable_callback, NULL, false);
}

amn1_rslt_t
amn1_has_motion()
{
	/* If the previous ADC value is 0, we know that we have not read any data,
	   since the sensor will never go down that low after the circuit is stable. */
	if (!circuit_stable || prev_value == 0)
		return (amn1_rslt_t){.error = NOT_READY};

	/* TODO: Use the ADC FIFO queue? */
	adc_select_input(AMN1_ADC_CHANNEL);
	uint16_t value = adc_read();

	return (amn1_rslt_t){
		.error = SUCCESS,
		.data = (AMN1_MOTION_THRESHOLD < (value * ADC_CONVERSION_FACTOR))
	};
}
