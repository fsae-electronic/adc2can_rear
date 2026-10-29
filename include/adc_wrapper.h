#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdint.h>
#include <stdbool.h>


// RTI counter block 0 clock (RTICLK 20 MHz / (CPUC0 + 1), CPUC0 = 1)
#define ADC_RTI_CLOCK_HZ 10000000U
// ADC event group trigger rate (RTI compare 0). Every CURRENT_ADC_AVERAGE conversions are
// averaged into one current sample (sensors.h), like the LPC55S06 hardware averaging.
#define ADC_SAMPLE_HZ 16000U
// Event group sample window: (EVSAMP + 2) ADCLK cycles
#define ADC_EV_SAMPLE_CYCLES 10U // 12 cycles at 10 MHz: 8 channels convert in ~20 us, within the 62.5 us period

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
void adc_update_from_dma(void);
uint16_t read_adc_value(adc_id_t adc_id);


#endif /* SENSORS_H_ */

