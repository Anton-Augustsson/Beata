#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "hardware/sync.h"
#include <stdio.h>
#include "pico/time.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"

#define ADC_NUM1 0
#define ADC_PIN1 (26 + ADC_NUM1)
#define ADC_VREF1 3.3
#define ADC_RANGE1 (1 << 12)
#define ADC_CONVERT1 (ADC_VREF1 / (ADC_RANGE1 - 1))

#define ADC_NUM2 1
#define ADC_PIN2 (26 + ADC_NUM2)
#define ADC_VREF2 3.3
#define ADC_RANGE2 (1 << 12)
#define ADC_CONVERT2 (ADC_VREF2 / (ADC_RANGE2 - 1))

#define ADC_NUM3 2
#define ADC_PIN3 (26 + ADC_NUM3)
#define ADC_VREF3 3.3
#define ADC_RANGE3 (1 << 12)
#define ADC_CONVERT3 (ADC_VREF3 / (ADC_RANGE3 - 1))


// sensor readings
struct sr{
    uin8_t humidity;
    uin8_t co2;
    uin8_t temp_celsius;
    uin8_t noise;
    uin8_t motion;
} ;


void print_sensor_value(){
    printf("Sensor 1: %dunit humidity", sr.humidity);
    printf("%dunit co2, %dC temperature\n", sr.co2, sr.temp_celsius);
    printf("Sensor 2: %ddB noise\n", sr.noise);
    printf("Sensor 3: %dunit motion\n", sr.noise);
}


void read_sensor(){
    uint adc_raw;
    while (1) {
        adc_raw = adc_read(); // raw voltage from ADC
        sleep_ms(10));
    }
}


int main() 
{
    stdio_init_all();
    printf("Beep boop, listening...\n");

    bi_decl(bi_program_description("Analog microphone example for Raspberry Pi Pico")); // for picotool
    bi_decl(bi_1pin_with_name(ADC_PIN1, "ADC input pin"));

    adc_init();
    adc_gpio_init(ADC_PIN1);
    adc_select_input(ADC_NUM1);

    read_sensor();    

    return 0;
}