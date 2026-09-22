#include "sensors.h"

#include <math.h>

#include "ti_fee.h"
#include "sci.h"
#include "can.h"

sensors_data_t sensors_data;

uint8_t calibration_cmd_id = CAL_CMD_NONE;

uint16_t current_vref_raw_value;

// Calibration values to save on EPROM
struct EEPROM_data
{
    union
    {
        uint8_t raw[12];
        struct
        {
            uint16_t magic;

            uint16_t ac_drv1_calibration_value;
            uint16_t dc_drv1_calibration_value;
            uint16_t ac_drv2_calibration_value;
            uint16_t dc_drv2_calibration_value;
            uint16_t calibration_brake_value;
        } values;
    };
};

static const uint16_t EEPROM_MAGIC = 0xAAAA;

static uint16_t convert_voltage2current(current_data_t *current_data)
{
    float sum_squares = 0.0f;
    uint16_t i;
    uint16_t index;

    // Raw ADC diff (vo - vref) with calibration offset removed, so current reads 0 when calibrated.
    int16_t raw_diff = (int16_t)current_data->current_vo_raw_value - (int16_t)current_vref_raw_value
        - (int16_t)current_data->calibration_current_value;

    float current = (((float)raw_diff / ADC_RESOLUTION) * ADC_VREF * ADC_GAIN) / SENSITIVITY;

    index = (uint16_t)(current_data->buffer_index % SAMPLES);
    current_data->current_buffer[index] = current;
    current_data->buffer_index = (uint16_t)((index + 1U) % SAMPLES);

    for (i = 0U; i < SAMPLES; i++)
    {
        sum_squares += current_data->current_buffer[i] * current_data->current_buffer[i];
    }

    current = sqrtf(sum_squares / (float)SAMPLES);

    current_data->current_value_rms = (uint16_t)(current + 0.5f);

    return current_data->current_value_rms;
}

/********************************
 * Static Functions
 *******************************/

bool load_e_from_eeprom()
{
    struct EEPROM_data e;

    while(TI_Fee_GetStatus(0) != IDLE)
    {
        TI_Fee_MainFunction();
    }
    TI_Fee_ReadSync(1, 0, e.raw, sizeof(struct EEPROM_data));

    if (e.values.magic != EEPROM_MAGIC)
    {
        return false;
    }

    sensors_data.ac_drv1_data.calibration_current_value = e.values.ac_drv1_calibration_value;
    sensors_data.dc_drv1_data.calibration_current_value = e.values.dc_drv1_calibration_value;
    sensors_data.ac_drv2_data.calibration_current_value = e.values.ac_drv2_calibration_value;
    sensors_data.dc_drv2_data.calibration_current_value = e.values.dc_drv2_calibration_value;
    sensors_data.rear_brake_data.calibration_brake_value = e.values.calibration_brake_value;

    return true;
}

void save_e_to_eeprom(void)
{
    while(TI_Fee_GetStatus(0) != IDLE)
    {
        TI_Fee_MainFunction();
    }
    struct EEPROM_data e;
    e.values.magic = EEPROM_MAGIC;

    e.values.ac_drv1_calibration_value = sensors_data.ac_drv1_data.calibration_current_value;
    e.values.dc_drv1_calibration_value = sensors_data.dc_drv1_data.calibration_current_value;
    e.values.ac_drv2_calibration_value = sensors_data.ac_drv2_data.calibration_current_value;
    e.values.dc_drv2_calibration_value = sensors_data.dc_drv2_data.calibration_current_value;
    e.values.calibration_brake_value = sensors_data.rear_brake_data.calibration_brake_value;

    TI_Fee_WriteAsync(1, e.raw);
    while(TI_Fee_GetStatus(0) != IDLE)
    {
        TI_Fee_MainFunction();
    }
}

void init_sensors(void)
{

    TI_Fee_Init();
    while (TI_Fee_GetStatus(0) != IDLE)
    {
        TI_Fee_MainFunction();
    }

    sensors_data.ac_drv1_data.calibration_current_value = 0;
    sensors_data.dc_drv1_data.calibration_current_value = 0;
    sensors_data.ac_drv2_data.calibration_current_value = 0;
    sensors_data.dc_drv2_data.calibration_current_value = 0;
    sensors_data.rear_brake_data.calibration_brake_value = 0;

    sensors_data.ac_drv1_data.current_vo_raw_value = 0;
    sensors_data.dc_drv1_data.current_vo_raw_value = 0;
    sensors_data.ac_drv2_data.current_vo_raw_value = 0;
    sensors_data.dc_drv2_data.current_vo_raw_value = 0;

    sensors_data.ac_drv1_data.buffer_index = 0;
    sensors_data.dc_drv1_data.buffer_index = 0;
    sensors_data.ac_drv2_data.buffer_index = 0;
    sensors_data.dc_drv2_data.buffer_index = 0;

    sensors_data.ac_drv1_data.current_value_rms = 0;
    sensors_data.dc_drv1_data.current_value_rms = 0;
    sensors_data.ac_drv2_data.current_value_rms = 0;
    sensors_data.dc_drv2_data.current_value_rms = 0;

    sensors_data.rear_brake_data.rear_brake_value = 0;

    // Load calibration values from EEPROM
    load_e_from_eeprom();

    init_adc();
    init_freq_measure();
}

void send_data_to_serial(void)
{
    uint16_t start = 0xAA55;
    sciSend(sciREG, 2, (uint8_t *)&start);
    sciSend(sciREG, 2, (uint8_t *)&sensors_data.ac_drv1_data.current_value_rms);
    sciSend(sciREG, 2, (uint8_t *)&sensors_data.dc_drv1_data.current_value_rms);
    sciSend(sciREG, 2, (uint8_t *)&sensors_data.ac_drv2_data.current_value_rms);
    sciSend(sciREG, 2, (uint8_t *)&sensors_data.dc_drv2_data.current_value_rms);
    sciSend(sciREG, 2, (uint8_t *)&sensors_data.rear_brake_data.rear_brake_value);
}

void send_data_to_can(void)
{
    uint8_t current_data[8];
    current_data[0] = (uint8_t)(sensors_data.ac_drv1_data.current_value_rms & 0xFF);
    current_data[1] = (uint8_t)((sensors_data.ac_drv1_data.current_value_rms >> 8) & 0xFF);
    current_data[2] = (uint8_t)(sensors_data.ac_drv2_data.current_value_rms & 0xFF);
    current_data[3] = (uint8_t)((sensors_data.ac_drv2_data.current_value_rms >> 8) & 0xFF);
    current_data[4] = (uint8_t)(sensors_data.dc_drv1_data.current_value_rms & 0xFF);
    current_data[5] = (uint8_t)((sensors_data.dc_drv1_data.current_value_rms >> 8) & 0xFF);
    current_data[6] = (uint8_t)(sensors_data.dc_drv2_data.current_value_rms & 0xFF);
    current_data[7] = (uint8_t)((sensors_data.dc_drv2_data.current_value_rms >> 8) & 0xFF);
    canTransmit(canREG1, canMESSAGE_BOX2, current_data);

    uint8_t rear_data[8];
    rear_data[0] = (uint8_t)(sensors_data.rear_brake_data.rear_brake_value & 0xFF);
    rear_data[1] = (uint8_t)((sensors_data.rear_brake_data.rear_brake_value >> 8) & 0xFF);
    rear_data[2] = 0U;
    rear_data[3] = 0U;
    rear_data[4] = 0U;
    rear_data[5] = 0U;
    rear_data[6] = 0U;
    rear_data[7] = 0U;
    canTransmit(canREG1, canMESSAGE_BOX3, rear_data);
}

/* EMA low-pass filter: alpha = 1/2^ADC_FILTER_SHIFT. Higher shift = smoother but slower response */
#define ADC_FILTER_SHIFT 3u

static uint16_t adc_filtered_value[ADC_NUM_CHANNELS];
static bool adc_filter_initialized[ADC_NUM_CHANNELS];

static uint16_t filter_adc_channel(adc_id_t ch)
{
    uint16_t raw = adc_data[ch].adc_value;

    if (!adc_filter_initialized[ch])
    {
        adc_filtered_value[ch] = raw;
        adc_filter_initialized[ch] = true;
    }
    else
    {
        adc_filtered_value[ch] = (uint16_t)(adc_filtered_value[ch] +
            (((int32_t)raw - (int32_t)adc_filtered_value[ch]) >> ADC_FILTER_SHIFT));
    }

    return adc_filtered_value[ch];
}

void convert_data(void)
{
    // ADC raw value (filtered)
    current_vref_raw_value = filter_adc_channel(VREF_CH);
    sensors_data.ac_drv1_data.current_vo_raw_value = filter_adc_channel(AC_DRV1_CH);
    sensors_data.dc_drv1_data.current_vo_raw_value = filter_adc_channel(DC_DRV1_CH);
    sensors_data.ac_drv2_data.current_vo_raw_value = filter_adc_channel(AC_DRV2_CH);
    sensors_data.dc_drv2_data.current_vo_raw_value = filter_adc_channel(DC_DRV2_CH);
    sensors_data.rear_brake_data.rear_brake_raw_value = filter_adc_channel(REAR_BRAKE_CH);


    // Current Conversion
    sensors_data.ac_drv1_data.current_value_rms = convert_voltage2current(&sensors_data.ac_drv1_data);
    sensors_data.dc_drv1_data.current_value_rms = convert_voltage2current(&sensors_data.dc_drv1_data);
    sensors_data.ac_drv2_data.current_value_rms = convert_voltage2current(&sensors_data.ac_drv2_data);
    sensors_data.dc_drv2_data.current_value_rms = convert_voltage2current(&sensors_data.dc_drv2_data);

    // Brake Conversion
    // P [PSI] = 400*(V - 0.5V)
    // V = raw_value/4095 * 5.0
    if (sensors_data.rear_brake_data.rear_brake_raw_value < sensors_data.rear_brake_data.calibration_brake_value)
        sensors_data.rear_brake_data.rear_brake_value = 0;
    else
        sensors_data.rear_brake_data.rear_brake_value =
            (uint16_t)(400.0f * ((((float)sensors_data.rear_brake_data.rear_brake_raw_value / 4095.0f) * 5.0f) - 0.5f));
}

void process_calibration_command(void)
{
    if (calibration_cmd_id == CAL_CMD_NONE)
    {
        return;
    }

    switch (calibration_cmd_id)
    {
    case CAL_CMD_TPS_0:
        sensors_data.rear_brake_data.calibration_brake_value = sensors_data.rear_brake_data.rear_brake_raw_value;
        break;
    case CAL_CMD_CURRENT_SENSORS:
        // Store vo - vref (raw ADC counts) so it can be subtracted to zero out the current reading.
        sensors_data.ac_drv1_data.calibration_current_value =
            (uint16_t)((int16_t)sensors_data.ac_drv1_data.current_vo_raw_value - (int16_t)current_vref_raw_value);
        sensors_data.dc_drv1_data.calibration_current_value =
            (uint16_t)((int16_t)sensors_data.dc_drv1_data.current_vo_raw_value - (int16_t)current_vref_raw_value);
        sensors_data.ac_drv2_data.calibration_current_value =
            (uint16_t)((int16_t)sensors_data.ac_drv2_data.current_vo_raw_value - (int16_t)current_vref_raw_value);
        sensors_data.dc_drv2_data.calibration_current_value =
            (uint16_t)((int16_t)sensors_data.dc_drv2_data.current_vo_raw_value - (int16_t)current_vref_raw_value);
        break;
    default:
        return; // Ignore unknown commands
        break;
    }

    save_e_to_eeprom();
    calibration_cmd_id = CAL_CMD_NONE;
}
