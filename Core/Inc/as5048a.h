/*
 * as5048a.h
 *
 *  Created on: Sep 28, 2025
 *      Author: Gigabyte
 */

#ifndef INC_AS5048A_H_
#define INC_AS5048A_H_
#include "stdint.h"
#include <math.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

uint16_t AS5048_Transfer(uint16_t data);
float AS5048_ReadAngle(void);


#endif /* INC_AS5048A_H_ */
