/** @file sys_main.c 
*   @brief Application main file
*   @date 11-Dec-2018
*   @version 04.07.01
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2018 Texas Instruments Incorporated - www.ti.com 
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
/* USER CODE END */

/* Include Files */

#include "sys_common.h"

/* USER CODE BEGIN (1) */
#include "sys_core.h"
#include "system.h"

#include "sci.h"
#include "rti.h"
#include "can.h"

#include "sensors.h"

#define CLOCK_RTI_HZ 10000000 

// Data conversion and processing frequency (DATA_PROCESS_HZ in sensors.h)
#define DATA_PROCESS_TICKS (CLOCK_RTI_HZ / DATA_PROCESS_HZ)

// CAN and SCI data sending frequency
#define DATA_SEND_HZ 100
#define DATA_SEND_TICKS (CLOCK_RTI_HZ / DATA_SEND_HZ)

// Set by the RTI interrupts, run in the main loop so the ADC sampling interrupt
// (compare 0, ADC_SAMPLE_HZ) is never blocked by SCI/CAN/EEPROM work.
static volatile bool process_pending = false;
static volatile bool send_pending = false;
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */
/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    _enable_interrupt_();

    sciInit();
    sciSetBaudrate(sciREG, 115200);

    canInit();

    init_sensors();

    // Delay to allow sensors to stabilize
    for(volatile int i = 0; i < 10000000; i++); // Simple delay loop


    rtiSetPeriod(rtiCOMPARE2, DATA_PROCESS_TICKS);
    rtiEnableNotification(rtiNOTIFICATION_COMPARE2);

    rtiSetPeriod(rtiCOMPARE3, DATA_SEND_TICKS);
    rtiEnableNotification(rtiNOTIFICATION_COMPARE3);
    


    rtiStartCounter(rtiCOUNTER_BLOCK1);
    while(1)
    {
        if (process_pending)
        {
            process_pending = false;
            convert_data();
            process_calibration_command();
        }

        if (send_pending)
        {
            send_pending = false;
            send_data_to_serial();
            send_data_to_can();
        }
    }
/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
void rtiNotification(uint32 notification)
{
    if (notification == rtiNOTIFICATION_COMPARE0)
    {
        sample_currents();
    }
    else if (notification == rtiNOTIFICATION_COMPARE2)
    {
        process_pending = true;
    }
    else if (notification == rtiNOTIFICATION_COMPARE3)
    {
        send_pending = true;
    }
}

void canMessageNotification(canBASE_t *node, uint32 messageBox)
{
    if(node == canREG1)
    {
        switch(messageBox)
        {
            case canMESSAGE_BOX1: // RX: 0x600 -> Calibration command DLC = 1
                canGetData(canREG1, canMESSAGE_BOX1, &calibration_cmd_id);
                break;
            default:
                // Handle other message boxes if needed
                break;
        }
    }
}
/* USER CODE END */
