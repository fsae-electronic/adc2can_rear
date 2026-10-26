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

static uint16_t current_block_count;
static uint8_t current_block_index;
static volatile bool current_window_ready = false;

static void accumulate_current_sample(current_data_t *current_data, adc_id_t ch, int32_t vref)
{
    // No filtering here: the phase current must reach the RMS with its full waveform.
    current_data->current_vo_raw_value = adc_data[ch].adc_value;

    int32_t raw_diff = (int32_t)current_data->current_vo_raw_value - vref;

    current_data->sum += raw_diff;
    current_data->sum_sq += (uint64_t)((int64_t)raw_diff * raw_diff);
}

// Closes the current block into the ring and publishes the sum of the last CURRENT_WINDOW_BLOCKS.
static void close_current_block(current_data_t *current_data)
{
    uint8_t i;
    int32_t window_sum = 0;
    uint64_t window_sum_sq = 0U;

    current_data->block_sum[current_block_index] = current_data->sum;
    current_data->block_sum_sq[current_block_index] = current_data->sum_sq;

    current_data->sum = 0;
    current_data->sum_sq = 0U;

    // If the previous window was not consumed yet, this one is dropped.
    if (current_window_ready)
    {
        return;
    }

    for (i = 0U; i < CURRENT_WINDOW_BLOCKS; i++)
    {
        window_sum += current_data->block_sum[i];
        window_sum_sq += current_data->block_sum_sq[i];
    }

    current_data->ready_sum = window_sum;
    current_data->ready_sum_sq = window_sum_sq;
}

// Called from the RTI compare 0 interrupt at ADC_SAMPLE_HZ.
void sample_currents(void)
{
    adc_update_from_dma();

    // vref is DC, its filtered value (updated in convert_data) is used to reduce noise.
    int32_t vref = (int32_t)current_vref_raw_value;

    accumulate_current_sample(&sensors_data.ac_drv1_data, AC_DRV1_CH, vref);
    accumulate_current_sample(&sensors_data.dc_drv1_data, DC_DRV1_CH, vref);
    accumulate_current_sample(&sensors_data.ac_drv2_data, AC_DRV2_CH, vref);
    accumulate_current_sample(&sensors_data.dc_drv2_data, DC_DRV2_CH, vref);

    current_block_count++;
    if (current_block_count >= CURRENT_BLOCK_SAMPLES)
    {
        close_current_block(&sensors_data.ac_drv1_data);
        close_current_block(&sensors_data.dc_drv1_data);
        close_current_block(&sensors_data.ac_drv2_data);
        close_current_block(&sensors_data.dc_drv2_data);

        current_window_ready = true;
        current_block_count = 0U;
        current_block_index = (uint8_t)((current_block_index + 1U) % CURRENT_WINDOW_BLOCKS);
    }
}

// True RMS over the window with the calibration offset removed:
// sum((d - cal)^2) = sum(d^2) - 2 * cal * sum(d) + N * cal^2
static uint16_t window_rms_current(current_data_t *current_data)
{
    int64_t cal = (int16_t)current_data->calibration_current_value;
    int64_t n = (int64_t)CURRENT_WINDOW_SAMPLES;

    int64_t sum_sq = (int64_t)current_data->ready_sum_sq - 2 * cal * (int64_t)current_data->ready_sum + n * cal * cal;
    if (sum_sq < 0)
    {
        sum_sq = 0;
    }

    float current = sqrtf((float)sum_sq / (float)n) * COUNTS_TO_AMPS;

    return (uint16_t)(current + 0.5f);
}

// Mean over the window with the calibration offset removed, as a magnitude: the CAN frame is
// unsigned and a sensor on the negative cable (DC_DRV2) reads the current with the opposite sign.
static uint16_t window_mean_current(current_data_t *current_data)
{
    float mean = ((float)current_data->ready_sum / (float)CURRENT_WINDOW_SAMPLES)
        - (float)(int16_t)current_data->calibration_current_value;

    float current = fabsf(mean) * COUNTS_TO_AMPS;

    return (uint16_t)(current + 0.5f);
}

static int16_t window_mean_raw(current_data_t *current_data)
{
    float mean = (float)current_data->ready_sum / (float)CURRENT_WINDOW_SAMPLES;

    return (int16_t)((mean >= 0.0f) ? (mean + 0.5f) : (mean - 0.5f));
}

static void update_current_values(void)
{
    if (!current_window_ready)
    {
        return;
    }

    sensors_data.ac_drv1_data.current_value_rms = window_rms_current(&sensors_data.ac_drv1_data);
    sensors_data.ac_drv2_data.current_value_rms = window_rms_current(&sensors_data.ac_drv2_data);
    sensors_data.dc_drv1_data.current_value_rms = window_mean_current(&sensors_data.dc_drv1_data);
    sensors_data.dc_drv2_data.current_value_rms = window_mean_current(&sensors_data.dc_drv2_data);

    sensors_data.ac_drv1_data.window_mean_raw = window_mean_raw(&sensors_data.ac_drv1_data);
    sensors_data.dc_drv1_data.window_mean_raw = window_mean_raw(&sensors_data.dc_drv1_data);
    sensors_data.ac_drv2_data.window_mean_raw = window_mean_raw(&sensors_data.ac_drv2_data);
    sensors_data.dc_drv2_data.window_mean_raw = window_mean_raw(&sensors_data.dc_drv2_data);

    current_window_ready = false;
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

    sensors_data.ac_drv1_data.sum = 0;
    sensors_data.dc_drv1_data.sum = 0;
    sensors_data.ac_drv2_data.sum = 0;
    sensors_data.dc_drv2_data.sum = 0;

    sensors_data.ac_drv1_data.sum_sq = 0U;
    sensors_data.dc_drv1_data.sum_sq = 0U;
    sensors_data.ac_drv2_data.sum_sq = 0U;
    sensors_data.dc_drv2_data.sum_sq = 0U;

    sensors_data.ac_drv1_data.window_mean_raw = 0;
    sensors_data.dc_drv1_data.window_mean_raw = 0;
    sensors_data.ac_drv2_data.window_mean_raw = 0;
    sensors_data.dc_drv2_data.window_mean_raw = 0;

    for (uint8_t i = 0U; i < CURRENT_WINDOW_BLOCKS; i++)
    {
        sensors_data.ac_drv1_data.block_sum[i] = 0;
        sensors_data.dc_drv1_data.block_sum[i] = 0;
        sensors_data.ac_drv2_data.block_sum[i] = 0;
        sensors_data.dc_drv2_data.block_sum[i] = 0;

        sensors_data.ac_drv1_data.block_sum_sq[i] = 0U;
        sensors_data.dc_drv1_data.block_sum_sq[i] = 0U;
        sensors_data.ac_drv2_data.block_sum_sq[i] = 0U;
        sensors_data.dc_drv2_data.block_sum_sq[i] = 0U;
    }

    current_block_count = 0U;
    current_block_index = 0U;

    sensors_data.ac_drv1_data.current_value_rms = 0;
    sensors_data.dc_drv1_data.current_value_rms = 0;
    sensors_data.ac_drv2_data.current_value_rms = 0;
    sensors_data.dc_drv2_data.current_value_rms = 0;

    sensors_data.rear_brake_data.rear_brake_value = 0;

    sensors_data.rear_brake_data.implausibility_counter = 0;
    sensors_data.rear_brake_data.implausibility_latched = false;
    sensors_data.rear_brake_data.implausibility_pending = false;

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

// BSE failure (open/short circuit): raw value below 0.4 V (BRAKE_RAW_MIN)
// or above 4.6 V (BRAKE_RAW_MAX) for more than IMPLAUSIBILITY_TIME_MS. Must be checked
// on the raw value, the PSI value is clamped to 0.
// Edge triggered: reported once, re-armed only after the condition clears.
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
// Retried every cycle until the mailbox accepts it.
static void send_implausibility_to_can(void)
{
    if (!sensors_data.rear_brake_data.implausibility_pending)
    {
        return;
    }

    uint8_t implausibility_data[8] = {0U};
    implausibility_data[0] = 0U;
    implausibility_data[1] = 1U;

    if (canTransmit(canREG1, IMPLAUSIBILITY_MSG_BOX, implausibility_data) != 0U)
    {
        sensors_data.rear_brake_data.implausibility_pending = false;
    }
}

void convert_data(void)
{
    // ADC raw value (filtered). Current channels are not filtered, they are sampled in sample_currents().
    current_vref_raw_value = filter_adc_channel(VREF_CH);
    sensors_data.rear_brake_data.rear_brake_raw_value = filter_adc_channel(REAR_BRAKE_CH);


    // Current Conversion (AC: RMS, DC: mean, over the last complete window)
    update_current_values();

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
        // Store the window mean of vo - vref (raw ADC counts) so it can be subtracted to zero out the current reading.
        // Must be sent with zero current (inverters disabled).
        sensors_data.ac_drv1_data.calibration_current_value = (uint16_t)sensors_data.ac_drv1_data.window_mean_raw;
        sensors_data.dc_drv1_data.calibration_current_value = (uint16_t)sensors_data.dc_drv1_data.window_mean_raw;
        sensors_data.ac_drv2_data.calibration_current_value = (uint16_t)sensors_data.ac_drv2_data.window_mean_raw;
        sensors_data.dc_drv2_data.calibration_current_value = (uint16_t)sensors_data.dc_drv2_data.window_mean_raw;
        break;
    default:
        return; // Ignore unknown commands
        break;
    }

    save_e_to_eeprom();
    calibration_cmd_id = CAL_CMD_NONE;
}
