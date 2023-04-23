#include "amn1.h"

#define AMN1_GPIO_PIN 26
#define AMN1_STABILITY_TIME_MS 30 * 1000
#define AMN1_MOTION_THRESHOLD_V 3

static volatile uint8_t circuit_stable = 0;

static int64_t
circuit_stable_callback(alarm_id_t id, void *user_data)
{
	printf("AMN1: Circuit is now stable\n");
	circuit_stable = 1;
	/* Do not re-run alarm. */
	return 0;
}

error_t
amn1_init()
{
	adc_gpio_init(AMN1_GPIO_PIN);
	add_alarm_in_ms(AMN1_STABILITY_TIME_MS, circuit_stable_callback, NULL, false);
	return SUCCESS;
}

amn1_rslt_t
amn1_read_motion()
{
	if (!circuit_stable)
		return (amn1_rslt_t){.error = NOT_READY};

	adc_select_input(AMN1_ADC_CHANNEL);
	uint16_t value = adc_read();

	return (amn1_rslt_t){
		.error = SUCCESS,
		.data = (AMN1_MOTION_THRESHOLD_V < (value * ADC_CONVERSION_FACTOR))
	};
}
