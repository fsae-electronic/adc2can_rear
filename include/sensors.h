#include "adc_wrapper.h"
#include "freq_measure.h"

#define VREF_C

#define VREF_CH ADC1
#define AC_DRV1_CH ADC2
#define DC_DRV1_CH ADC6
#define AC_DRV2_CH ADC5
#define DC_DRV2_CH ADC0
#define REAR_BRAKE_CH ADC7



// Definimos las constantes del sensor
#define ADC_RESOLUTION 4095.0f // Resolución del ADC (12 bits)
#define SENSITIVITY (0.625f / 200.0f) // Sensibilidad (en voltios por amperio)
#define ADC_VREF 3.3f        // VRef del ADC
#define ADC_GAIN 1.5f   // Inversa de divisor resistivo
#define COUNTS_TO_AMPS ((ADC_VREF * ADC_GAIN) / (ADC_RESOLUTION * SENSITIVITY))

// Current sensing: same measurement as fsae_current_sense (LPC55S06 board).
// The LPC read vo and vref of each sensor in its while(1) (~4 kHz per sensor, 32x hardware
// averaging), kept the last SAMPLES currents and, every 10 ms (CAN send), computed RMS/max/min
// of the buffer. AC sends the RMS, DC sends the last sample.
// Here: ADC at ADC_SAMPLE_HZ, CURRENT_ADC_AVERAGE conversions averaged per sample.
#define CURRENT_ADC_AVERAGE 4U                                     // 16 kHz / 4 = 4 kHz per sample
#define CURRENT_SAMPLE_HZ (ADC_SAMPLE_HZ / CURRENT_ADC_AVERAGE)
#define SAMPLES 300U                                               // Cantidad de muestras para el RMS (75 ms at 4 kHz)

// Data conversion and processing frequency (convert_data() rate)
#define DATA_PROCESS_HZ 150

// Implausibility detection (reglamento T.4.3.3/4, BSE)
// Rear brake sensor raw range on 12-bit ADC over 5 V.
// Fixed limits, independent of the 0 PSI calibration (open circuit reads ~0).
#define BRAKE_RAW_MIN 328U            // 0.4 V -> 0.4 / 5.0 * 4095
#define BRAKE_RAW_MAX 3767U           // 4.6 V -> 4.6 / 5.0 * 4095
// Fault must persist more than this time to be confirmed
#define IMPLAUSIBILITY_TIME_MS 100U
#define IMPLAUSIBILITY_CYCLES ((IMPLAUSIBILITY_TIME_MS * DATA_PROCESS_HZ + 999U) / 1000U)

#define IMPLAUSIBILITY_MSG_BOX canMESSAGE_BOX4 // TX: 0x401


typedef struct
{
    bool calibration_state;
    bool vo_higher;
    bool vref_higher;

    adc_id_t current_port_vo;

    uint16_t calibration_value;          // |vo - vref| at zero current, ADC counts
    uint16_t calibration_current_value;  // Signed (vo - vref) offset as stored in EEPROM

    uint16_t adc_value_vo;               // Averaged ADC counts of the last sample
    uint16_t adc_value_vref;
    uint32_t adc_sum_vo;                 // CURRENT_ADC_AVERAGE accumulator (ISR)

    float vo;
    float vref;

    float current_buffer[SAMPLES];       // Ring of the last SAMPLES currents [A]
    uint16_t buffer_index;

    float current;                       // Last sample [A]
    float rms_current;
    float max_current;
    float min_current;

    uint16_t current_value_rms;          // Sent on CAN [A]: AC channels RMS, DC channels |last sample|
} current_data_t;

typedef struct
{
    uint16_t rear_brake_value;
    uint16_t rear_brake_raw_value;
    uint16_t calibration_brake_value;

    uint16_t implausibility_counter; // Consecutive cycles with raw value out of range
    bool implausibility_latched;     // Fault confirmed and reported, waiting for it to clear
    bool implausibility_pending;     // 0x401 frame still to be sent
} brake_data_t;


typedef struct
{
    current_data_t ac_drv1_data;
    current_data_t dc_drv1_data;
    current_data_t ac_drv2_data;
    current_data_t dc_drv2_data;
    brake_data_t rear_brake_data;
} sensors_data_t;

enum calibration_cmd_id_t
{
    CAL_CMD_NONE = 0,
    CAL_CMD_TPS_0 = 1,
    CAL_CMD_TPS_100 = 2,
    CAL_CMD_LEFT_STEER = 3,
    CAL_CMD_CENTER_STEER = 4,
    CAL_CMD_RIGHT_STEER = 5,
    CAL_CMD_CURRENT_SENSORS = 6
};



extern sensors_data_t sensors_data;

extern uint8_t calibration_cmd_id;




void init_sensors(void);
void run_sensors(void);
void send_data_to_serial(void);
void send_data_to_can(void);
void convert_data(void);
void sample_currents(void);
void process_calibration_command(void);
