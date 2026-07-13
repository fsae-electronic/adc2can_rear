#ifndef FREQ_MEASURE_H_
#define FREQ_MEASURE_H_

#include <stdbool.h>

#include "ecap.h"


typedef enum
{
    FREQ_MEASURE_CH1 = 0,
    FREQ_MEASURE_CH2,
    FREQ_MEASURE_COUNT
} freq_measure_id_t;

typedef struct
{
    ecapBASE_t *ecap;
    float period_us;
    uint32_t cap_ticks;
    bool new_value;
} freq_measure_t;

void init_freq_measure(void);
uint32_t get_ticks_measure(freq_measure_id_t measure);
float get_period_us_measure(freq_measure_id_t measure);


extern freq_measure_t freq_ch1;
extern freq_measure_t freq_ch2;


#endif /* FREQ_MEASURE_H_ */
