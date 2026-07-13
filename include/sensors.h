#include "adc_wrapper.h"
#include "freq_measure.h"

#define VREF_C

#define AC_DRV1_CH ADC1
#define DC_DRV1_CH ADC2
#define AC_DRV2_CH ADC3
#define DC_DRV2_CH ADC4
#define REAR_BRAKE_CH ADC5


// Definimos las constantes del sensor
#define ADC_RESOLUTION 4095.0f // Resolución del ADC (12 bits)
#define SENSITIVITY (0.625f / 200.0f) // Sensibilidad (en voltios por amperio)
#define ADC_VREF 3.3f        // VRef del ADC
#define ADC_GAIN 1.5f   // Inversa de divisor resistivo
#define SAMPLES 100          // Cantidad de muestras para el RMS


typedef struct
{
    uint16_t current_vo_raw_value;
    
    uint16_t calibration_current_value;
    uint16_t current_value_rms;
    float current_buffer[SAMPLES];
    uint16_t buffer_index;
} current_data_t;

typedef struct
{
    uint16_t rear_brake_value;
    uint16_t rear_brake_raw_value;
    uint16_t calibration_brake_value;
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
void process_calibration_command(void);
