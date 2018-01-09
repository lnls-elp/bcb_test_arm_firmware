/*
 * usb_to_serial.h
 *
 *  Created on: 23/01/2017
 *      Author: joao.rosa
 */

#include <stdint.h>

#ifndef USB_TO_SERIAL_USB_TO_SERIAL_H_
#define USB_TO_SERIAL_USB_TO_SERIAL_H_

extern void init_usb2serial(void);

extern void usb2serial_process_data(void);

extern uint8_t get_usb2serial_address(void);

extern void set_usb2serial_address(uint8_t addr);


#endif /* USB_TO_SERIAL_USB_TO_SERIAL_H_ */
