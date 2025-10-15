/*
 * PC_Connect.h
 *
 *  Created on: Oct 10, 2025
 *      Author: Gigabyte
 */

#ifndef INC_PC_CONNECT_H_
#define INC_PC_CONNECT_H_
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdint.h>

void USB_ProcessCommand(uint8_t *buf);


#endif /* INC_PC_CONNECT_H_ */
