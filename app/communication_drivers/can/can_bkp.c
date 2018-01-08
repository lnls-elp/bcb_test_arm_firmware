/******************************************************************************
 * Copyright (C) 2017 by LNLS - Brazilian Synchrotron Light Laboratory
 *
 * Redistribution, modification or use of this software in source or binary
 * forms is permitted as long as the files maintain this copyright. LNLS and
 * the Brazilian Center for Research in Energy and Materials (CNPEM) are not
 * liable for any misuse of this material.
 *
 *****************************************************************************/

/**
 * @file can_bkp.c
 * @brief Backplane CAN module.
 *
 * Module to process data in CAN BUS for backplane.
 *
 * @author joao.rosa
 *
 * @date 21/01/2016
 *
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "inc/hw_nvic.h"
#include "inc/hw_sysctl.h"

#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

//#include "communication_drivers/shared_memory/structs.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/system_task/system_task.h"
#include "board_drivers/hardware_def.h"
#include "can_bkp.h"


#define LOAD_CURRENT_ALARM      0x00000001
#define LOAD_VOLTAGE_ALARM      0x00000002
#define OUT_VOLTAGE_ALARM       0x00000004
#define IN_VOLTAGE_ALARM        0x00000008
#define ARM1_CURRENT_ALARM      0x00000010
#define ARM2_CURRENT_ALARM      0x00000020
#define IN_CURRENT_ALARM        0x00000040
#define OUT1_CURRENT_ALARM      0x00000080
#define OUT2_CURRENT_ALARM      0x00000100
#define OUT1_VOLTAGE_ALARM      0x00000200
#define OUT2_VOLTAGE_ALARM      0x00000400
#define LEAKAGE_CURRENT_ALARM   0x00000800
#define IGBT1_TEMP_ALARM        0x00001000
#define IGBT2_TEMP_ALARM        0x00002000
#define L1_TEMP_ALARM           0x00004000
#define L2_TEMP_ALARM           0x00008000
#define HEATSINK_TEMP_ALARM     0x00010000
#define WATER_TEMP_ALARM        0x00020000
#define RECTFIER1_TEMP_ALARM    0x00040000
#define RECTFIER2_TEMP_ALARM    0x00080000
#define HUMIDITY_ALARM          0x00100000

volatile uint32_t PSModuleAlarms = 0;

//*****************************************************************************
//
// A counter that keeps track of the number of times the RX interrupt has
// occurred, which should match the number of messages that were received.
//
//*****************************************************************************
volatile uint32_t g_ui32MsgCount = 0;

//*****************************************************************************
//
// A flag for the interrupt handler to indicate that a message was received.
//
//*****************************************************************************
volatile bool g_bRXFlag = 0;

//*****************************************************************************
//
// A flag to indicate that some reception error occurred.
//
//*****************************************************************************
volatile bool g_bErrFlag = 0;

//Rx
tCANMsgObject sCANMessage;
uint8_t pui8MsgData[8];

//Tx
tCANMsgObject sCANMessageTx;
uint8_t pui8MsgDataTx[8];


//*****************************************************************************
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been transmitted.
//*****************************************************************************
void can_int_handler(void)
{
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INT0ID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    //
    // Check if the cause is message object 1.
    //
    else if(ui32Status == 1)
    {
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 1);

        //
        // Set flag to indicate received message is pending for this message
        // object.
        //
        g_bRXFlag = 1;

        // Indicate new message that needs to be processed
        TaskSetNew(PROCESS_CAN_MESSAGE);

        //
        // Since a message was received, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }
}

void can_check(void)
{
    uint8_t i;

    //
    // If the flag for message object 1 is set, that means that the RX
    // interrupt occurred and there is a message ready to be read from
    // this CAN message object.
    //
    if(g_bRXFlag)
    {
        sCANMessage.pucMsgData = pui8MsgData;
        CANMessageGet(CAN0_BASE, 1, &sCANMessage, 0);

        //  Echo CAN message
        for(i = 0; i < 8; i++)
        {
            pui8MsgDataTx[i] = pui8MsgData[i];
        }

        sCANMessageTx.pucMsgData = pui8MsgDataTx;
        CANMessageSet(CAN0_BASE, 2, &sCANMessageTx, MSG_OBJ_TYPE_TX);

        g_bRXFlag = 0;
    }
}

void init_can_bkp(void)
{
    // Initialize the CAN controller
    CANInit(CAN0_BASE);

    // Setup CAN to be clocked off the M3/Master subsystem clock
    CANClkSourceSelect(CAN0_BASE, CAN_CLK_M3);

    // Configure the controller for 1 Mbit operation.
    CANBitRateSet(CAN0_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 1000000);

    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.  If you want to use dynamic
    // allocation of the vector table, then you must also call CANIntRegister()
    // here.
    // CANIntRegister(CAN0_BASE, CANIntHandler); // if using dynamic vectors
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    // Register interrupt handler in RAM vector table
    IntRegister(INT_CAN0INT0, can_int_handler);

    // Enable the CAN interrupt on the processor (NVIC).
    IntEnable(INT_CAN0INT0);
    //IntMasterEnable();

    // Enable test mode and select external loopback
    //HWREG(CAN0_BASE + CAN_O_CTL) |= CAN_CTL_TEST;
    //HWREG(CAN0_BASE + CAN_O_TEST) = CAN_TEST_EXL;

    // Enable the CAN for operation.
    CANEnable(CAN0_BASE);

    //
    // Initialize a message object to receive CAN messages with ID 0x010.
    // The expected ID must be set along with the mask to indicate that all
    // bits in the ID must match.
    //
    sCANMessage.ulMsgID = 0x010;
    sCANMessage.ulMsgIDMask = 0x7FF;
    sCANMessage.ulFlags = (MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER | MSG_OBJ_FIFO);
    sCANMessage.ulMsgLen = 8;
    sCANMessage.pucMsgData = pui8MsgData;

    //
    // Now load the message object into the CAN peripheral message object 1.
    // Once loaded the CAN will receive any messages with this CAN ID into
    // this message object, and an interrupt will occur.
    //
    CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_RX);

    //
    // Initialize message object 1 to be able to send CAN message 1.  This
    // message object is not shared so it only needs to be initialized one
    // time, and can be used for repeatedly sending the same message ID.
    //
    sCANMessageTx.ulMsgID = 0x200;
    sCANMessageTx.ulMsgIDMask = 0;
    sCANMessageTx.ulFlags = (MSG_OBJ_TX_INT_ENABLE | MSG_OBJ_FIFO);
    sCANMessageTx.ulMsgLen = 8;
    sCANMessageTx.pucMsgData = pui8MsgDataTx;
}
