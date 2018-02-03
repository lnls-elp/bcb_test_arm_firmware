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
 * @file i2c_offboard_isolated.h
 * @brief I2C offboard module.
 *
 * @author joao.rosa
 *
 * @date 15/07/2015
 *
 */

#include <stdint.h>

#ifndef I2C_OFFBOARD_ISOLATED_H_
#define I2C_OFFBOARD_ISOLATED_H_

#define	SINGLE_ADDRESS	0x01
#define	DOUBLE_ADDRESS	0x02

#define SLAVE_ADDRESS           0x3c
#define SLAVE_READ_REG_ADD      0x5c

extern void init_i2c_offboard_isolated(void);
extern void init_i2c_slave_offboard_isolated(void);

extern void read_i2c_offboard_isolated(uint8_t SLAVE_ADDR, uint8_t TYPE_REGISTER_ADDR, uint8_t MESSAGE_SIZE, uint8_t *data);
extern void write_i2c_offboard_isolated(uint8_t SLAVE_ADDR, uint8_t MESSAGE_SIZE, uint8_t *data);

extern uint8_t get_i2c_message();

#endif /* I2C_ONBOARD_H_ */
