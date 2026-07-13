#include "adc_wrapper.h"
#include "adc.h"
#include "rti.h"
#include "sys_dma.h"

#define ADC_MAX_VALUE 4095U

adc_data_t adc_data[ADC_NUM_CHANNELS];
uint32_t adc_dma_buffer[ADC_NUM_CHANNELS];
bool conversion_complete = false;

g_dmaCTRL g_dmaCTRLPKT;

void dmaConfig_ADC_Event(void)
{
    dmaEnable();

    /* LINEA 15 = ADC1 EVENT GROUP */
    dmaReqAssign(DMA_CH0, 7U);

    /* CONFIGURACION PARA LEER 1 POR 1 */
    /* SADD: Apuntamos al BUF0 del Grupo 0 (Event Group) */
    g_dmaCTRLPKT.SADD = (uint32)(&adcREG1->GxBUF[0].BUF0);
    g_dmaCTRLPKT.DADD = (uint32)(&adc_dma_buffer[0]);
    g_dmaCTRLPKT.CHCTRL = 0;

    /* Importante: 8 Frames de 1 Elemento cada uno */
    g_dmaCTRLPKT.FRCNT = ADC_NUM_CHANNELS;
    g_dmaCTRLPKT.ELCNT = 1U;

    g_dmaCTRLPKT.ELSOFFSET = 0;
    g_dmaCTRLPKT.ELDOFFSET = 0;
    g_dmaCTRLPKT.FRSOFFSET = 0;
    g_dmaCTRLPKT.FRDOFFSET = 0;
    g_dmaCTRLPKT.PORTASGN = 4U;

    g_dmaCTRLPKT.RDSIZE = ACCESS_32_BIT;
    g_dmaCTRLPKT.WRSIZE = ACCESS_32_BIT;

    /* FRAME_TRANSFER: El DMA espera un pulso del ADC por cada dato */
    g_dmaCTRLPKT.TTYPE = FRAME_TRANSFER;
    g_dmaCTRLPKT.ADDMODERD = ADDR_FIXED; /* La FIFO no cambia de direcci�n */
    g_dmaCTRLPKT.ADDMODEWR = ADDR_INC1;  /* El array en RAM s� incrementa */
    g_dmaCTRLPKT.AUTOINIT = AUTOINIT_ON;

    dmaSetCtrlPacket(DMA_CH0, g_dmaCTRLPKT);
    dmaEnableInterrupt(DMA_CH0, FTC);
    dmaSetChEnable(DMA_CH0, DMA_HW);
}

void adcConfig_Event_DMA(void)
{
    /* Habilitar DMA para el Event Group (EVDMACR - offset 0x4C) */
    adcREG1->EVDMACR = 0x00000001U;

    /* Resetear FIFO del Event Group por si hay basura */
    adcREG1->GxFIFORESETCR[0U] = 1U;
}

void init_adc(void)
{
    rtiInit();
    adcInit();

    adcCalibration(adcREG1);
    dmaConfig_ADC_Event();
    adcConfig_Event_DMA();

    // RTI Compare 0 --> Hardware trigger to ADC1 Group Event
    // CLOCK RTI = 10MHz --> 10.000.000 ticks --> 1 seg
    rtiSetPeriod(rtiCOMPARE0, 50000);
    rtiEnableNotification(rtiNOTIFICATION_COMPARE0); /* Enable notification for RTI compare 0 */
    rtiStartCounter(rtiCOUNTER_BLOCK0);

    adcStartConversion(adcREG1, adcGROUP0);

    for (int i = 0; i < ADC_NUM_CHANNELS; i++)
    {
        adc_data[i].adc_value = (uint16_t)(0U);
    }
}

void process_adc_data(void)
{
    if (conversion_complete)
    {
        for (int i = 0; i < ADC_NUM_CHANNELS; i++)
        {
            adc_data[i].adc_value = (uint16_t)(adc_dma_buffer[i] & 0xFFFU); // Extraer el valor de ADC (12 bits)
            adc_data[i].adc_id = (adc_dma_buffer[i] >> 16U) & 0x1FU;        // Extraer el ID del canal/pin
        }
        conversion_complete = false; // Resetear la bandera
    }
}

uint16_t read_adc_value(adc_id_t adc_id)
{
    if (adc_id < ADC_NUM_CHANNELS)
    {
        return adc_data[adc_id].adc_value;
    }
    else
    {
        return 0; // Valor por defecto si el ID es inválido
    }
}

void dmaGroupANotification(dmaInterrupt_t inttype, uint32 channel)
{
    if (inttype == FTC && channel == DMA_CH0)
    {
        conversion_complete = true; // Indicar que la conversión está completa
        process_adc_data();
    }
}
