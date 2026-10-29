#include "sensors.h"

#include <math.h>
#include <stdlib.h>

#include "ti_fee.h"
#include "sci.h"
#include "can.h"
#include "rti.h"

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

static uint16_t current_average_count;
static uint32_t adc_sum_vref;

static void init_current_sense(current_data_t *sense, adc_id_t current_port_vo)
{
    uint16_t i;

    sense->calibration_state = false;
    sense->vo_higher = false;
    sense->vref_higher = false;

    sense->current_port_vo = current_port_vo;

    sense->calibration_value = 0;
    sense->calibration_current_value = 0;
    sense->adc_value_vo = 0;
    sense->adc_value_vref = 0;
    sense->adc_sum_vo = 0U;

    sense->vo = 0.0f;
    sense->vref = 0.0f;
    sense->current = 0.0f;
    sense->rms_current = 0.0f;
    sense->max_current = 0.0f;
    sense->min_current = 0.0f;
    sense->current_value_rms = 0;
    sense->buffer_index = 0U;

    for (i = 0U; i < SAMPLES; i++)
    {
        sense->current_buffer[i] = 0.0f;
    }
}

// Signed EEPROM offset -> LPC calibration (|offset| + which side was higher).
static void apply_calibration_offset(current_data_t *sense)
{
    int16_t offset = (int16_t)sense->calibration_current_value;

    sense->calibration_value = (uint16_t)((offset >= 0) ? offset : -offset);
    sense->vo_higher = (offset > 0);
    sense->vref_higher = (offset < 0);
    sense->calibration_state = (offset != 0);
}

static void calibrate_current_sense(current_data_t *sense)
{
    sense->calibration_value = (uint16_t)abs((int32_t)sense->adc_value_vo - (int32_t)sense->adc_value_vref);

    if (sense->adc_value_vo > sense->adc_value_vref)
    {
        sense->vo_higher = true;
        sense->vref_higher = false;
    }
    else
    {
        sense->vo_higher = false;
        sense->vref_higher = true;
    }

    sense->calibration_state = true;
    sense->calibration_current_value = (uint16_t)((int16_t)sense->adc_value_vo - (int16_t)sense->adc_value_vref);
}

static void convert_voltage2current(current_data_t *sense)
{
    uint16_t adc_value_vref = sense->adc_value_vref;

    if (sense->vo_higher)
    {
        adc_value_vref += sense->calibration_value;
    }
    else if (sense->vref_higher)
    {
        adc_value_vref -= sense->calibration_value;
    }

    sense->vo = (ADC_VREF * (((float)sense->adc_value_vo) / ADC_RESOLUTION));
    sense->vref = (ADC_VREF * (((float)adc_value_vref) / ADC_RESOLUTION));

    // ADC_GAIN: this board has a divider in front of the ADC, the LPC board did not.
    sense->current = ((sense->vo - sense->vref) * ADC_GAIN) / SENSITIVITY;

    // Save current value in current_buffer (ring instead of the LPC shift: same RMS/max/min)
    sense->current_buffer[sense->buffer_index] = sense->current;
    sense->buffer_index = (uint16_t)((sense->buffer_index + 1U) % SAMPLES);
}

static void current_sense(current_data_t *sense, uint16_t adc_value_vref)
{
    sense->adc_value_vo = (uint16_t)((sense->adc_sum_vo + (CURRENT_ADC_AVERAGE / 2U)) / CURRENT_ADC_AVERAGE);
    sense->adc_value_vref = adc_value_vref;
    sense->adc_sum_vo = 0U;

    convert_voltage2current(sense);
}

static void calculate_current_values(current_data_t *sense)
{
    uint16_t i;

    //Calulate the RMS VALUE
    sense->rms_current = 0.0f;
    for (i = 0U; i < SAMPLES; i++)
    {
        sense->rms_current += (sense->current_buffer[i] * sense->current_buffer[i]);
    }
    sense->rms_current /= (float)SAMPLES;
    sense->rms_current = sqrtf(sense->rms_current);

    //Calulate the MAX VALUE
    sense->max_current = 0.0f;
    for (i = 0U; i < SAMPLES; i++)
    {
        if (sense->current_buffer[i] > sense->max_current)
        {
            sense->max_current = sense->current_buffer[i];
        }
    }

    //Calulate the MIN VALUE
    sense->min_current = 10000.0f;
    for (i = 0U; i < SAMPLES; i++)
    {
        if (sense->current_buffer[i] < sense->min_current)
        {
            sense->min_current = sense->current_buffer[i];
        }
    }
}

// Called from the RTI compare 0 interrupt at ADC_SAMPLE_HZ: accumulates CURRENT_ADC_AVERAGE
// conversions, then takes one sample of every sensor (the LPC while(1) body).
void sample_currents(void)
{
    adc_update_from_dma();

    adc_sum_vref += adc_data[VREF_CH].adc_value;
    sensors_data.ac_drv1_data.adc_sum_vo += adc_data[sensors_data.ac_drv1_data.current_port_vo].adc_value;
    sensors_data.ac_drv2_data.adc_sum_vo += adc_data[sensors_data.ac_drv2_data.current_port_vo].adc_value;
    sensors_data.dc_drv1_data.adc_sum_vo += adc_data[sensors_data.dc_drv1_data.current_port_vo].adc_value;
    sensors_data.dc_drv2_data.adc_sum_vo += adc_data[sensors_data.dc_drv2_data.current_port_vo].adc_value;

    current_average_count++;
    if (current_average_count < CURRENT_ADC_AVERAGE)
    {
        return;
    }

    uint16_t adc_value_vref = (uint16_t)((adc_sum_vref + (CURRENT_ADC_AVERAGE / 2U)) / CURRENT_ADC_AVERAGE);
    current_vref_raw_value = adc_value_vref;
    adc_sum_vref = 0U;
    current_average_count = 0U;

    current_sense(&sensors_data.ac_drv1_data, adc_value_vref);
    current_sense(&sensors_data.ac_drv2_data, adc_value_vref);
    current_sense(&sensors_data.dc_drv1_data, adc_value_vref);
    current_sense(&sensors_data.dc_drv2_data, adc_value_vref);
}

// Like the LPC send_current_can(): RMS/max/min of the buffers right before sending.
// AC: RMS. DC: last sample, as a magnitude (CAN frame is unsigned, DC_DRV2 is on the negative cable).
static void update_current_values(void)
{
    calculate_current_values(&sensors_data.ac_drv1_data);
    calculate_current_values(&sensors_data.ac_drv2_data);
    calculate_current_values(&sensors_data.dc_drv1_data);
    calculate_current_values(&sensors_data.dc_drv2_data);

    sensors_data.ac_drv1_data.current_value_rms = (uint16_t)(sensors_data.ac_drv1_data.rms_current + 0.5f);
    sensors_data.ac_drv2_data.current_value_rms = (uint16_t)(sensors_data.ac_drv2_data.rms_current + 0.5f);
    sensors_data.dc_drv1_data.current_value_rms = (uint16_t)(fabsf(sensors_data.dc_drv1_data.current) + 0.5f);
    sensors_data.dc_drv2_data.current_value_rms = (uint16_t)(fabsf(sensors_data.dc_drv2_data.current) + 0.5f);
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
        dwdReset(); // flash erase/write can outlast the watchdog (sys_main.c)
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
        dwdReset(); // flash erase/write can outlast the watchdog (sys_main.c)
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
        dwdReset(); // flash erase/write can outlast the watchdog (sys_main.c)
    }
}

void init_sensors(void)
{

    TI_Fee_Init();
    while (TI_Fee_GetStatus(0) != IDLE)
    {
        TI_Fee_MainFunction();
        dwdReset(); // flash erase/write can outlast the watchdog (sys_main.c)
    }

    sensors_data.ac_drv1_data.calibration_current_value = 0;
    sensors_data.dc_drv1_data.calibration_current_value = 0;
    sensors_data.ac_drv2_data.calibration_current_value = 0;
    sensors_data.dc_drv2_data.calibration_current_value = 0;
    sensors_data.rear_brake_data.calibration_brake_value = 0;

    init_current_sense(&sensors_data.ac_drv1_data, AC_DRV1_CH);
    init_current_sense(&sensors_data.ac_drv2_data, AC_DRV2_CH);
    init_current_sense(&sensors_data.dc_drv1_data, DC_DRV1_CH);
    init_current_sense(&sensors_data.dc_drv2_data, DC_DRV2_CH);
    current_average_count = 0U;
    adc_sum_vref = 0U;

    sensors_data.rear_brake_data.rear_brake_value = 0;

    sensors_data.rear_brake_data.implausibility_counter = 0;
    sensors_data.rear_brake_data.implausibility_latched = false;
    sensors_data.rear_brake_data.implausibility_pending = false;

    // Load calibration values from EEPROM
    load_e_from_eeprom();
    apply_calibration_offset(&sensors_data.ac_drv1_data);
    apply_calibration_offset(&sensors_data.ac_drv2_data);
    apply_calibration_offset(&sensors_data.dc_drv1_data);
    apply_calibration_offset(&sensors_data.dc_drv2_data);

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
    update_current_values();

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

// BSE failure (open/short circuit): raw value below 0.4 V (BRAKE_RAW_MIN)
// or above 4.6 V (BRAKE_RAW_MAX) for more than IMPLAUSIBILITY_TIME_MS. Must be checked
// on the raw value, the PSI value is clamped to 0.
// Reported right away on the confirmation, then every IMPLAUSIBILITY_REPEAT_CYCLES
// while the fault lasts (send_implausibility_to_can).
static void check_brake_implausibility(brake_data_t *brake_data)
{
    uint16_t raw = brake_data->rear_brake_raw_value;

    if (raw >= BRAKE_RAW_MIN && raw <= BRAKE_RAW_MAX)
    {
        brake_data->implausibility_counter = 0;
        brake_data->implausibility_latched = false;
        return;
    }

    if (brake_data->implausibility_counter <= IMPLAUSIBILITY_CYCLES)
    {
        brake_data->implausibility_counter++;
    }

    if (brake_data->implausibility_counter > IMPLAUSIBILITY_CYCLES && !brake_data->implausibility_latched)
    {
        brake_data->implausibility_latched = true;
        brake_data->implausibility_pending = true;
    }
}

// 0x401: b0 = TPS implausibility (always 0 on rear), b1 = brake implausibility.
// Retried every cycle until the mailbox accepts it, repeated while the fault lasts.
static uint16_t implausibility_repeat_counter = 0U;

static void send_implausibility_to_can(void)
{
    if (!sensors_data.rear_brake_data.implausibility_latched)
    {
        sensors_data.rear_brake_data.implausibility_pending = false;
        implausibility_repeat_counter = 0U;
        return;
    }

    if (!sensors_data.rear_brake_data.implausibility_pending)
    {
        implausibility_repeat_counter++;
        if (implausibility_repeat_counter < IMPLAUSIBILITY_REPEAT_CYCLES)
        {
            return;
        }
        sensors_data.rear_brake_data.implausibility_pending = true;
    }

    uint8_t implausibility_data[8] = {0U};
    implausibility_data[0] = 0U;
    implausibility_data[1] = 1U;

    if (canTransmit(canREG1, IMPLAUSIBILITY_MSG_BOX, implausibility_data) != 0U)
    {
        sensors_data.rear_brake_data.implausibility_pending = false;
        implausibility_repeat_counter = 0U;
    }
}

void convert_data(void)
{
    // ADC raw value (filtered). Current channels are sampled in sample_currents() and
    // converted in send_data_to_can().
    sensors_data.rear_brake_data.rear_brake_raw_value = filter_adc_channel(REAR_BRAKE_CH);

    // Brake Conversion
	// P [PSI] = 400*(V - V0), where V0 is the voltage measured at 0 PSI (calibration_brake_value)
	// V = raw_value/4095 * 5.0
    if(sensors_data.rear_brake_data.rear_brake_raw_value < sensors_data.rear_brake_data.calibration_brake_value)
        sensors_data.rear_brake_data.rear_brake_value = 0;
    else
        sensors_data.rear_brake_data.rear_brake_value =
            (uint16_t)(400.0f * (((float)sensors_data.rear_brake_data.rear_brake_raw_value - (float)sensors_data.rear_brake_data.calibration_brake_value) / 4095.0f * 5.0f));

    check_brake_implausibility(&sensors_data.rear_brake_data);
    send_implausibility_to_can();
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
        // Same as the LPC button: offset vo - vref of the last sample. Must be sent with zero
        // current (inverters disabled).
        calibrate_current_sense(&sensors_data.ac_drv1_data);
        calibrate_current_sense(&sensors_data.dc_drv1_data);
        calibrate_current_sense(&sensors_data.ac_drv2_data);
        calibrate_current_sense(&sensors_data.dc_drv2_data);
        break;
    default:
        return; // Ignore unknown commands
        break;
    }

    save_e_to_eeprom();
    calibration_cmd_id = CAL_CMD_NONE;
}
