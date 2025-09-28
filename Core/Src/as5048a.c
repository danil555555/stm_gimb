/*
 * as5048a.c
 *
 *  Created on: Sep 26, 2025
 *      Author: Gigabyte
 */
#include "as5048a.h"
#include "stm32f4xx_hal.h"

#define AS5048_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define AS5048_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)


