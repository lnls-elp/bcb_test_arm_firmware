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
 * @file i2c_offboard_isolated.c
 * @brief I2C offboard module.
 *
 * @author joao.rosa
 *
 * @date 15/07/2015
 *
 */

#include <stdint.h>

#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_ints.h"
#include "inc/hw_sysctl.h"

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"

#include "hardware_def.h"

#include "i2c_offboard_isolated.h"

#define I2C_READ true
#define I2C_WRITE false
#define I2C_MESSAGE_SIZE 2

#define I2CWhileMasterBusy while (I2CMasterBusy(I2C_OFFBOARD_ISO_MASTER_BASE)) {}

//*****************************************************************************
//
// Global variable to hold the I2C data that has been received.
//
//*****************************************************************************
volatile uint32_t g_ui32DataRx;
volatile uint32_t g_ui32Status;

#if UDC_SELECT
void isr_i2c_slave_offboard_isolated(void)
{
    if (I2CSlaveIntStatus(I2C_OFFBOARD_ISO_SLAVE_BASE, I2C_SLAVE_INT_DATA)) {

        g_ui32Status = I2CSlaveStatus(I2C_OFFBOARD_ISO_SLAVE_BASE);

        if (g_ui32Status & I2C_SLAVE_ACT_RREQ) {
            g_ui32DataRx = I2CSlaveDataGet(I2C_OFFBOARD_ISO_SLAVE_BASE);
        }

        if (g_ui32Status & I2C_SLAVE_ACT_TREQ) {
            I2CSlaveDataPut(I2C_OFFBOARD_ISO_SLAVE_BASE, 0xbb);
        }
    }

    I2CSlaveIntClear(I2C_OFFBOARD_ISO_SLAVE_BASE);
    I2CSlaveIntClearEx(I2C_OFFBOARD_ISO_SLAVE_BASE, I2C_SLAVE_INT_DATA);

}


void init_i2c_slave_offboard_isolated(void)
{
    //
    // Set the slave address to SLAVE_ADDRESS
    //
    I2CSlaveInit(I2C_OFFBOARD_ISO_SLAVE_BASE, SLAVE_ADDRESS);

    //
    // Register ISR for I2C slave handler
    //
    I2CIntRegister(I2C_OFFBOARD_ISO_SLAVE_BASE, isr_i2c_slave_offboard_isolated);

    //
    // Configure and turn on the I2C1 slave interrupt.  The I2CSlaveIntEnableEx()
    // gives you the ability to only enable specific interrupts.  For this case
    // we are only interrupting when the slave device receives data.
    //
    I2CSlaveIntEnableEx(I2C_OFFBOARD_ISO_SLAVE_BASE, I2C_SLAVE_INT_DATA);

    //
    // Enable the I2C1 slave module.
    //
    I2CSlaveEnable(I2C_OFFBOARD_ISO_SLAVE_BASE);

    //
    // Enable the I2C1 interrupt on the processor (NVIC).
    //
    IntEnable(INT_I2C1);
}
#endif

void read_i2c_offboard_isolated(uint8_t SLAVE_ADDR, uint8_t TYPE_REGISTER_ADDR, uint8_t MESSAGE_SIZE, uint8_t *data)
{

    I2CMasterSlaveAddrSet(I2C_OFFBOARD_ISO_MASTER_BASE, SLAVE_ADDR, I2C_WRITE);

    // Dummy Write to set the future read address
    I2CMasterDataPut(I2C_OFFBOARD_ISO_MASTER_BASE, *data); // address zero in the EEPROM memory
    I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    I2CWhileMasterBusy

    if(TYPE_REGISTER_ADDR == DOUBLE_ADDRESS)
    {
        data++; // Increase the pointer
        I2CMasterDataPut(I2C_OFFBOARD_ISO_MASTER_BASE, *data); // Send second byte address
        I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        I2CWhileMasterBusy
        data--; // Decrease the pointer
    }

    // Start reading the address
    I2CMasterSlaveAddrSet(I2C_OFFBOARD_ISO_MASTER_BASE, SLAVE_ADDR, I2C_READ);

    int i;
    for (i = 0; i < MESSAGE_SIZE; i++)
    {

        if(i == 0) I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        else if(i == (MESSAGE_SIZE - 1)) I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        else I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

        I2CWhileMasterBusy
        *data = I2CMasterDataGet(I2C_OFFBOARD_ISO_MASTER_BASE);
        data++;

    }

}

void write_i2c_offboard_isolated(uint8_t SLAVE_ADDR, uint8_t MESSAGE_SIZE, uint8_t *data)
{

    I2CMasterSlaveAddrSet(I2C_OFFBOARD_ISO_MASTER_BASE, SLAVE_ADDR, I2C_WRITE);

    int i;
    for (i = 0; i < MESSAGE_SIZE; i++) { // +2 for the address byte
        I2CMasterDataPut(I2C_OFFBOARD_ISO_MASTER_BASE, *data);

        if(i == 0) I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        else if(i == (MESSAGE_SIZE - 1)) I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        else I2CMasterControl(I2C_OFFBOARD_ISO_MASTER_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
        data++;

        I2CWhileMasterBusy
    }
}

uint8_t get_i2c_message()
{
    uint8_t data[2];
    //data[0] = 0x00;
    //read_i2c_offboard_isolated(0b1001110, SINGLE_ADDRESS, 2, data);
    data[0] = SLAVE_READ_REG_ADD;
    read_i2c_offboard_isolated(SLAVE_ADDRESS, SINGLE_ADDRESS, 2, data);
    return data[0];

}

void init_i2c_offboard_isolated(void)
{
    // I2C0 configuration (EEPROM memory, IO expander e Temperature sensor.)
    // Data rate is set to 400kbps
    I2CMasterInitExpClk(I2C_OFFBOARD_ISO_MASTER_BASE, SysCtlClockGet(
                               SYSTEM_CLOCK_SPEED), true);

    //I2C enable
    I2CMasterEnable(I2C_OFFBOARD_ISO_MASTER_BASE);

}

