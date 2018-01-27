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
 * @file can_bkp.h
 * @brief Backplane CAN module.
 *
 * Module to process data in CAN BUS for backplane.
 *
 * @author joao.rosa
 *
 * @date 21/01/2016
 *
 */

#ifndef CAN_BKP_H_
#define CAN_BKP_H_

extern void init_can_bkp(void);
extern void can_check(void);
extern void send_can_message();
extern void clear_can_buffer();
extern uint8_t get_can_message();

#endif /* APP_COMMUNICATION_DRIVERS_CAN_CAN_BKP_H_ */
