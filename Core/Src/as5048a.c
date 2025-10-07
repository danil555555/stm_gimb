/*
 * as5048a.c
 *
 *  Created on: Sep 26, 2025
 *      Author: Gigabyte
 */
#include "as5048a.h"
#include "stm32f4xx_hal.h"
#define AS5048_CMD_READ_ANGLE  0x3FFF  // адрес регистра угла

extern SPI_HandleTypeDef hspi1;

uint16_t AS5048_Transfer(uint16_t data)
{
    uint8_t tx[2] = { data >> 8, data & 0xFF };
    uint8_t rx[2] = {0};

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);   // CS HIGH

    return ((uint16_t)rx[0] << 8) | rx[1];
}

// Чтение угла
uint16_t AS5048_ReadAngle(void)
{
	AS5048_Transfer(AS5048_CMD_READ_ANGLE | 0x4000); // бит R=1 (чтение)
	    // Вторая транзакция — получение результата
	    uint16_t res = AS5048_Transfer(0x0000);
	    return res & 0x3FFF;
}
