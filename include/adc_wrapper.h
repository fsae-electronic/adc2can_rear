#ifndef SENSORS_H_
#define SENSORS_H_

#include <stdint.h>
#include <stdbool.h>


// RTI counter block 0 clock (RTICLK 20 MHz / (CPUC0 + 1))
#define ADC_RTI_CLOCK_HZ 10000000U
// ADC event group trigger rate (RTI compare 0). Phase RMS needs fs >> output frequency
// (emDrive 150_300/120: 0 to 1.2 kHz).
#define ADC_SAMPLE_HZ 20000U
// Event group sample window: (EVSAMP + 2) ADCLK cycles
#define ADC_EV_SAMPLE_CYCLES 10U

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

