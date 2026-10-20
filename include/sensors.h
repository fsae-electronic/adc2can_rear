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

// Current window: sampled at ADC_SAMPLE_HZ, one value (RMS for AC, mean for DC) per window.
// 1000 samples at 20 kHz = 50 ms, at least 2 electrical periods above 40 Hz.
#define CURRENT_WINDOW_SAMPLES 1000U

// Data conversion and processing frequency (convert_data() rate)
#define DATA_PROCESS_HZ 150

// Implausibility detection (reglamento T.4.3.3/4, BSE)
// Rear brake sensor raw range on 12-bit ADC over 5 V.
// Low limit is relative to the 0 PSI calibration value (open circuit reads ~0).
#define BRAKE_RAW_MIN_CAL_PERCENT 25U // Fail below 10 % of calibration_brake_value
#define BRAKE_RAW_MAX 3686U           // 4.5 V -> 4.5 / 5.0 * 4095
// Fault must persist more than this time to be confirmed
#define IMPLAUSIBILITY_TIME_MS 100U
#define IMPLAUSIBILITY_CYCLES ((IMPLAUSIBILITY_TIME_MS * DATA_PROCESS_HZ + 999U) / 1000U)

#define IMPLAUSIBILITY_MSG_BOX canMESSAGE_BOX4 // TX: 0x401


typedef struct
{
    uint16_t current_vo_raw_value;
    uint16_t current_vref_raw_value;
    uint16_t calibration_current_value;

    uint16_t current_value_rms;  // AC channels: RMS [A]. DC channels: mean [A]

    // Window accumulators of (vo - vref) in ADC counts, written by sample_currents() (ISR)
    int32_t sum;
    uint64_t sum_sq;
    // Last complete window, read by convert_data() while current_window_ready is set
    int32_t ready_sum;
    uint64_t ready_sum_sq;
    int16_t window_mean_raw;     // Mean of (vo - vref) over the last window, used for calibration
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
