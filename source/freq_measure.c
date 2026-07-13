#include "freq_measure.h"

#include "ecap.h"
#include "sys_common.h"
#include "sys_core.h"
#include "system.h"

#define FREQ_MEASURE_SIGNAL_TIMEOUT_US 300000.0f

freq_measure_t freq_ch1;
freq_measure_t freq_ch2;


void init_freq_measure(void)
{
    // Initialize ECAP modules for wheel speed measurement
    freq_ch1.ecap = ecapREG1;  // ECAP1 for front left wheel
    freq_ch2.ecap = ecapREG2; // ECAP2 for front right wheel

    freq_ch1.cap_ticks = 0;
    freq_ch1.new_value = false;

    freq_ch2.cap_ticks = 0;
    freq_ch2.new_value = false;



    ecapInit();

    // Configure ECAP1 for front left wheel
    ecapSetCaptureEvent1(ecapREG1, FALLING_EDGE, RESET_ENABLE);
    ecapSetCaptureMode(ecapREG1, CONTINUOUS, CAPTURE_EVENT1);
    ecapStartCounter(ecapREG1);
    ecapEnableCapture(ecapREG1);
    ecapEnableInterrupt(ecapREG1, ecapInt_CEVT1);

    // Configure ECAP2 for front right wheel
    ecapSetCaptureEvent1(ecapREG2, FALLING_EDGE, RESET_ENABLE);
    ecapSetCaptureMode(ecapREG2, CONTINUOUS, CAPTURE_EVENT1);
    ecapStartCounter(ecapREG2);
    ecapEnableCapture(ecapREG2);
    ecapEnableInterrupt(ecapREG2, ecapInt_CEVT1);
}

uint32_t get_ticks_measure(freq_measure_id_t measure)
{
    if (measure == FREQ_MEASURE_CH1)
    {
        freq_ch1.new_value = false;
        return freq_ch1.cap_ticks;
    }
    else if (measure == FREQ_MEASURE_CH2)
    {
        freq_ch2.new_value = false;
        return freq_ch2.cap_ticks;
    }
    else
    {
        return 0; // Invalid wheel ID
    }
}

float get_period_us_measure(freq_measure_id_t measure)
{
    if (measure == FREQ_MEASURE_CH1)
    {
        freq_ch1.new_value = false;

        if (freq_ch1.cap_ticks == 0U)
        {
            return 0.0f;
        }

        if (((float)freq_ch1.ecap->TSCTR / VCLK4_FREQ) > FREQ_MEASURE_SIGNAL_TIMEOUT_US)
        {
            return 0.0f;
        }

        return freq_ch1.period_us;
    }
    else if (measure == FREQ_MEASURE_CH2)
    {
        freq_ch2.new_value = false;

        if (freq_ch2.cap_ticks == 0U)
        {
            return 0.0f;
        }

        if (((float)freq_ch2.ecap->TSCTR / VCLK4_FREQ) > FREQ_MEASURE_SIGNAL_TIMEOUT_US)
        {
            return 0.0f;
        }

        return freq_ch2.period_us;
    }
    else
    {
        return 0.0f; // Invalid wheel ID
    }
}

void ecapNotification(ecapBASE_t *ecap, uint16 flags)
{
    (void)flags;

    if (ecap == freq_ch1.ecap)
    {
        freq_ch1.cap_ticks = ecapGetCAP1(ecap);
        freq_ch1.period_us = ((float)freq_ch1.cap_ticks / VCLK4_FREQ);
        freq_ch1.new_value = true;
    }
    else if (ecap == freq_ch2.ecap)
    {
        freq_ch2.cap_ticks = ecapGetCAP1(ecap);
        freq_ch2.period_us = ((float)freq_ch2.cap_ticks / VCLK4_FREQ);
        freq_ch2.new_value = true;
    }
}



