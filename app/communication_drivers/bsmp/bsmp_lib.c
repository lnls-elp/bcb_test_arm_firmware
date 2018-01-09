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
 * @file bsmp_lib.c
 * @brief BSMP protocol for UDC board.
 *
 * Treat BSMP messages in UDC board.
 *
 * @author joao.rosa
 *
 * @date 09/06/2015
 *
 */

#include <stdint.h>
#include <stdarg.h>
#include <string.h>

#include "communication_drivers/i2c_onboard/eeprom.h"
#include "communication_drivers/system_task/system_task.h"
#include "communication_drivers/ipc/ipc_lib.h"
#include "communication_drivers/can/can_bkp.h"
#include "communication_drivers/rs485/rs485.h"
#include "communication_drivers/i2c_onboard/exio.h"

#include "inc/hw_memmap.h"
#include "inc/hw_ipc.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/ipc.h"
#include "driverlib/systick.h"

#include "bsmp/include/server.h"
#include "bsmp_lib.h"

#include "board_drivers/hardware_def.h"

#define NUMBER_OF_BSMP_SERVERS  4

volatile unsigned long ulTimeout;

//#pragma CODE_SECTION(BSMPprocess, "ramfuncs");

bsmp_server_t bsmp[NUMBER_OF_BSMP_SERVERS];
uint16_t TIMEOUT_VALUE = 20;


//*****************************************************************************
//                  Clear BCB POF Transmitters
//*****************************************************************************
uint8_t clear_bcb_pof_tx (uint8_t *input, uint8_t *output)
{
    GPIOPinWrite(PWM_1TO8_BASE, 0x00000FF, OFF);
    GPIOPinWrite(PWM_9TO16_BASE, 0x00000FF, OFF);
    GPIOPinWrite(EPWMSYNC_BASE, EPWMSYNCO_PIN, OFF);

    *output = 0;
    return *output;
}

static struct bsmp_func clear_bcb_pof_tx_t = {
    .func_p           = clear_bcb_pof_tx,
    .info.input_size  = 0,       // Nothing is read from the input parameter
    .info.output_size = 1,       // command_ack
};


//*****************************************************************************
//                  Set BCB POF Transmitters
//*****************************************************************************
uint8_t set_bcb_pof_tx (uint8_t *input, uint8_t *output)
{
    GPIOPinWrite(PWM_1TO8_BASE, 0x00000FF, ON);
    GPIOPinWrite(PWM_9TO16_BASE, 0x00000FF, ON);
    GPIOPinWrite(EPWMSYNC_BASE, EPWMSYNCO_PIN, ON);

    *output = 0;
    return *output;
}

static struct bsmp_func set_bcb_pof_tx_t = {
    .func_p           = set_bcb_pof_tx,
    .info.input_size  = 0,       // Nothing is read from the input parameter
    .info.output_size = 1,       // command_ack
};


//*****************************************************************************
//                  Read BCB POF Receivers
//*****************************************************************************
uint8_t read_bcb_pof_rx (uint8_t *input, uint8_t *output)
{
    static uint8_t rx_status;

    rx_status = 0;

    rx_status |= GPIOPinRead(INT_ARM_BASE, INT_ARM_PIN) >> 4;
    rx_status |= GPIOPinRead(INT_C28_BASE, INT_C28_PIN) >> 4;
    rx_status |= GPIOPinRead(EPWMSYNC_BASE, EPWMSYNCI_PIN) >> 4;
    rx_status |= GPIOPinRead(INT_GENERAL_BASE, INT_GENERAL_PIN) >> 4;

    *output = rx_status;
    return 0;
}

static struct bsmp_func read_bcb_pof_rx_t = {
    .func_p           = read_bcb_pof_rx,
    .info.input_size  = 0,       // Nothing is read from the input parameter
    .info.output_size = 1,       // POF receivers status
};

//*****************************************************************************
//                          Enable Buzzer
//*****************************************************************************
uint8_t enable_buzzer (uint8_t *input, uint8_t *output)
{
    sound_sel_ctrl(1);
    *output = 0;
    return *output;
}

static struct bsmp_func enable_buzzer_t = {
    .func_p           = enable_buzzer,
    .info.input_size  = 0,       // Nothing is read from the input parameter
    .info.output_size = 1,       // command_ack
};

//*****************************************************************************
//                          Disable Buzzer
//*****************************************************************************
uint8_t disable_buzzer (uint8_t *input, uint8_t *output)
{
    sound_sel_ctrl(0);
    *output = 0;
    return *output;
}

static struct bsmp_func disable_buzzer_t = {
    .func_p           = disable_buzzer,
    .info.input_size  = 0,       // Nothing is read from the input parameter
    .info.output_size = 1,       // command_ack
};

//*****************************************************************************
//                          DUMMY Variables Memory
//*****************************************************************************
static uint8_t dummy_float_memory[4];
static uint8_t dummy_u32_memory[4];
static uint8_t dummy_u16_memory[2];

//*****************************************************************************
//                              BSMP Variables
//*****************************************************************************
static struct bsmp_var ps_status[NUMBER_OF_BSMP_SERVERS];
static struct bsmp_var ps_setpoint[NUMBER_OF_BSMP_SERVERS];


static void init_bsmp_var(struct bsmp_var *p_var, uint8_t size, uint8_t *p_dummy, bool writable);
/*
 * @brief Initialize BSMP module.
 *
 * Initialize BMSP functions, variables and curves
 *
 * @param uint8_t Server id to be initialized.
 */
void bsmp_init(uint8_t server)
{
    //********************************************
    // Initialize communications library
    bsmp_server_init(&bsmp[server]);
    //bsmp_register_hook(&bsmp, hook);

    //*************************************************************************
    //                      BSMP Function Register
    //*************************************************************************
    bsmp_register_function(&bsmp[server], &clear_bcb_pof_tx_t);     // ID 0
    bsmp_register_function(&bsmp[server], &set_bcb_pof_tx_t);       // ID 1
    bsmp_register_function(&bsmp[server], &read_bcb_pof_rx_t);      // ID 2
    bsmp_register_function(&bsmp[server], &enable_buzzer_t);        // ID 3
    bsmp_register_function(&bsmp[server], &disable_buzzer_t);       // ID 4

    //*************************************************************************
    //                      BSMP Variable Register
    //*************************************************************************
    init_bsmp_var(&ps_status[server], 2, dummy_u16_memory, false);
    init_bsmp_var(&ps_setpoint[server], 4, dummy_float_memory, false);

    bsmp_register_variable(&bsmp[server], &ps_status[server]);       // ID 0
    bsmp_register_variable(&bsmp[server], &ps_setpoint[server]);     // ID 1

    //*************************************************************************
    //                  BSMP Variable Pointers Initialization
    //*************************************************************************
    set_bsmp_var_pointer(0, server, g_ipc_ctom.ps_module[server].ps_status.u8);
    set_bsmp_var_pointer(1, server, g_ipc_ctom.ps_module[server].ps_setpoint.u8);
}

/*
 * @brief Set BSMP variable pointer to specified variable
 *
 * Initialize BSMP variable for the specified server and point it to the
 * data region in memory.
 *
 * @param uint8_t ID for BSMP variable
 * @param uint8_t BSMP server to initialize
 * @param uint8_t* Pointer to memory region where BSM variable is stored
 */

void set_bsmp_var_pointer(uint8_t var_id, uint8_t server, volatile uint8_t *new_data)
{
    struct bsmp_var *var = bsmp[server].vars.list[var_id];
    var->data = new_data;
}

/*
 * @brief BSMP process data
 *
 * Send received data to BSMP server specified and process
 *
 * @param bsmp_raw_packet* Pointer to received packet
 * @param bsmp_raw_packet* Pointer to store response packet
 * @param uint8_t ID for BSMP server
 */

void BSMPprocess(struct bsmp_raw_packet *recv_packet,
                 struct bsmp_raw_packet *send_packet, uint8_t server)
{
    bsmp_process_packet(&bsmp[server], recv_packet, send_packet);
}

static void init_bsmp_var(struct bsmp_var *p_var, uint8_t size, uint8_t *p_dummy, bool writable)
{
    p_var->info.size     = size;
    p_var->info.writable = writable;
    p_var->data          = p_dummy;
    p_var->value_ok      = NULL;
}
