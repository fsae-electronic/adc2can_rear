#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdint.h>
#include <stdbool.h>


typedef enum
{
    ADC0 = 0,
    ADC1,
    ADC2,
    ADC3,
    ADC4,
    ADC5,
    ADC6,
    ADC7,
    ADC_NUM_CHANNELS
} adc_id_t;

typedef struct
{
    uint16_t adc_value;
    uint32_t adc_id;
} adc_data_t;

extern adc_data_t adc_data[ADC_NUM_CHANNELS];

void init_adc(void);
void process_adc_data(void);
uint16_t read_adc_value(adc_id_t adc_id);


#endif /* SENSORS_H_ */

