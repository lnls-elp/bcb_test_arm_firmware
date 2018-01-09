/*
 * rs485.c
 *
 *  Created on: 29/05/2015
 *      Author: joao.rosa
 */


#include "inc/hw_sysctl.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"
#include "driverlib/debug.h"
#include "driverlib/ram.h"

#include "usb_to_serial.h"

#include "../board_drivers/hardware_def.h"

#include "../rs485/rs485.h"

#include "../i2c_onboard/eeprom.h"

#include "../i2c_onboard/exio.h"

#include "../system_task/system_task.h"

#include <stdint.h>
#include <stdarg.h>
#include <string.h>
#include "../bsmp/bsmp_lib.h"

// Put the code in to the RAM memory
#pragma CODE_SECTION(isr_usb2serial, "ramfuncs");
#pragma CODE_SECTION(usb2serial_process_data, "ramfuncs");

//*****************************************************************************

#pragma DATA_SECTION(recv_buffer, "SERIALBUFFER")
#pragma DATA_SECTION(send_buffer, "SERIALBUFFER")

//*****************************************************************************

#define SERIAL_HEADER           1   // Destination
#define SERIAL_CSUM             1

static uint8_t SERIAL_ADDRESS = 1;   // My Address
static uint8_t BCAST_ADDRESS  = 255; // Broadcast Adress
#define SERIAL_MASTER_ADDRESS   0   // Master Address

#define SERIAL_BUF_SIZE         (SERIAL_HEADER+3+3+16834+SERIAL_CSUM)


//*****************************************************************************
struct serial_buffer
{
#if 0
    uint8_t data[SERIAL_BUF_SIZE];
    uint16_t index;*/
#endif

    uint8_t data[SERIAL_BUF_SIZE];
    uint16_t index;
    uint8_t csum;
};

static struct serial_buffer recv_buffer = {.index = 0};
static struct serial_buffer send_buffer = {.index = 0};

static struct bsmp_raw_packet recv_packet =
                             { .data = recv_buffer.data + 1 };
static struct bsmp_raw_packet send_packet =
                             { .data = send_buffer.data + 1 };

//*****************************************************************************

void isr_usb2serial(void)
{
	uint32_t lChar;
	uint16_t sCarga;
	uint8_t ucChar;
	uint32_t ulStatus;

	uint8_t time_out = 0;

	// Get the interrrupt status.
	ulStatus = UARTIntStatus(FT230_UART_BASE, true);

	// Clear the asserted interrupts.
	UARTIntClear(FT230_UART_BASE, ulStatus);

	if(UARTRxErrorGet(FT230_UART_BASE)) UARTRxErrorClear(FT230_UART_BASE);

	// Receive Interrupt Mask
	if(UART_INT_RX == ulStatus || UART_INT_RT == ulStatus)
	{

		for(time_out = 0; time_out < 15; time_out++)
		{
		    // Loop while there are characters in the receive FIFO.
            while(UARTCharsAvail(FT230_UART_BASE) && recv_buffer.index < SERIAL_BUF_SIZE)
            {
                recv_buffer.data[recv_buffer.index] = (uint8_t)UARTCharGet(FT230_UART_BASE);
                //recv_buffer.index++;
                recv_buffer.csum += recv_buffer.data[recv_buffer.index++];

                time_out = 0;

            }
		}


		sCarga = (recv_buffer.data[2]<<8) | recv_buffer.data[3];

		if(recv_buffer.index > sCarga + 4)
		{
		    //TaskSetNew(PROCESS_USB2SERIAL_MESSAGE);
		    usb2serial_process_data();
		}

        if(sCarga > SERIAL_BUF_SIZE)
        {
            recv_buffer.index = 0;
            recv_buffer.csum  = 0;
            send_buffer.index = 0;
            send_buffer.csum  = 0;
        }

	}
}

void
usb2serial_tx_handler(void)
{
	unsigned int i;

	// Prepare answer
	send_buffer.data[0] = SERIAL_MASTER_ADDRESS;
	send_buffer.csum    = 0;

	// Send packet

	for(i = 0; i < send_packet.len + SERIAL_HEADER; ++i)
	{
		// Wait until have space in the TX buffer
		while(!UARTSpaceAvail(FT230_UART_BASE));
		// CheckSum calc
		send_buffer.csum -= send_buffer.data[i];
		// Send Byte
		UARTCharPut(FT230_UART_BASE, send_buffer.data[i]);
	}
	// Wait until have space in the TX buffer
	while(!UARTSpaceAvail(FT230_UART_BASE));
	// Send Byte
	UARTCharPut(FT230_UART_BASE, send_buffer.csum);

}

void
usb2serial_process_data(void)
{
	// Received less than HEADER + CSUM bytes
	if(recv_buffer.index < (SERIAL_HEADER + SERIAL_CSUM))
		goto exit;

	// Checksum is not zero
	if(recv_buffer.csum)
		goto exit;

	// Packet is not for me
	if(recv_buffer.data[0] != SERIAL_ADDRESS && recv_buffer.data[0] != BCAST_ADDRESS)
		goto exit;

	recv_packet.len = recv_buffer.index - SERIAL_HEADER - SERIAL_CSUM;

	if(recv_buffer.data[0]==SERIAL_ADDRESS)
	{
	    // Library will process the packet
	    BSMPprocess(&recv_packet, &send_packet, 0);
	}

	usb2serial_tx_handler();

	exit:
	recv_buffer.index = 0;
	recv_buffer.csum  = 0;
	send_buffer.index = 0;
	send_buffer.csum  = 0;
}

void
set_usb2serial_address(uint8_t addr)
{
	if(addr < 33 && addr > 0 && addr != SERIAL_ADDRESS)
	{
		SERIAL_ADDRESS = addr;
		SaveRs485Add(SERIAL_ADDRESS);
	}
}

uint8_t
get_usb2serial_address(void)
{
	return SERIAL_ADDRESS;
}

void
init_usb2serial(void)
{
	// Load address from EEPROM and config it
	//SetUsb2SerialAddress(1);

	// serial configuration, operation mode 8-N-1
	UARTConfigSetExpClk(FT230_UART_BASE, SysCtlClockGet(SYSTEM_CLOCK_SPEED), 115200,
						(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
						UART_CONFIG_PAR_NONE));

	UARTFIFOEnable(FT230_UART_BASE);
	UARTFIFOLevelSet(FT230_UART_BASE,UART_FIFO_TX1_8,UART_FIFO_RX1_8);

	//Habilita interrupção pela UART0
	IntRegister(FT230_INT, isr_usb2serial);
	UARTIntEnable(FT230_UART_BASE, UART_INT_RX | UART_INT_RT);

	//Seta niveis de prioridade entre as interrupcoes
	IntPrioritySet(FT230_INT, 0);

	// Enable the UART
	UARTEnable(FT230_UART_BASE);

	IntEnable(FT230_INT);
}


