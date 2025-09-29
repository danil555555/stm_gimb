/*
 * as5048a.c
 *
 *  Created on: Sep 28, 2025
 *      Author: Gigabyte
 */

#include "as5048a.h"
#define AS5048_CMD_ANGLE 0x3FFF


extern SPI_HandleTypeDef hspi1;

uint16_t AS5048_Transfer(uint16_t data)
{
    uint8_t tx[2] = { data >> 8, data & 0xFF };
    uint8_t rx[2];

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, &tx[0], &rx[0], 1, HAL_MAX_DELAY);
    HAL_SPI_TransmitReceive(&hspi1, &tx[1], &rx[1], 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    return ((uint16_t)rx[0] << 8) | rx[1];
}


float AS5048_ReadAngle(void)
{
	uint16_t res = AS5048_Transfer(AS5048_CMD_ANGLE | 0x4000);
	uint16_t res_transfer = res & 0x3FFF;
	//return ((res_transfer / 8192.0f)) * 360.0f;
	return ((res_transfer / 8192.0f) - 1.0f) * 360.0f;
}
