/*
 * as5048a.h
 *
 *  Created on: Sep 26, 2025
 *      Author: Gigabyte
 */

#ifndef INC_AS5048A_H_
#define INC_AS5048A_H_

#include <stdint.h>

static uint16_t spiTransfer16(uint16_t tx);
uint16_t AS5048_ReadRaw(void);
float AS5048_ReadAngleDeg(void);





#endif /* INC_AS5048A_H_ */
