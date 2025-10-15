/*
 * as5048a.c
 *
 *  Created on: Sep 26, 2025
 *      Author: Gigabyte
 */
#include "as5048a.h"
#include "drv8302.h"
#include "stm32f4xx_hal.h"
#define AS5048_CMD_READ_ANGLE  0x3FFF  // адрес регистра угла


extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;

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

float _normalizeAngle(float angle) {
    while (angle > 2.0f * (float)M_PI) {
        angle -= 2.0f * (float)M_PI;
    }
    while (angle < 0) {
        angle += 2.0f * (float)M_PI;
    }
    return angle;
}

void Sensor_Init()
{
    float mecAngleNum   = AS5048_ReadAngle() / 8192.0f * 360.0f;
    prev_angle_rad      = mecAngleNum * M_PI / 180.0f;
    full_rotations      = 0.0f;
}

void SensorUpdate()
{
    float mecAngleNum   = AS5048_ReadAngle() / 8192.0f * 360.0f;
    float angle_rad = mecAngleNum * M_PI / 180.0f;

    float delta_angle = angle_rad - prev_angle_rad;
    if (fabsf(delta_angle) > 0.8f * 2.0f * (float)M_PI) {
        full_rotations += (delta_angle > 0.0f) ? -1 : 1;
    }
    prev_angle_rad = angle_rad;

    angle_el = _normalizeAngle( (float)(sensor_direction * pole_pairs) * prev_angle_rad  - zero_electric_angle );

    now_angle = ( ( 2.0f * (float)M_PI ) * full_rotations ) + prev_angle_rad;
}

float electricalAngle(void){
	return angle_el;
}
float NowAngle(void)
{
	return now_angle;
}
float Sensor_GetAngleRad(void)
{
    return prev_angle_rad;
}
float Sensor_GetFullRotations(void)
{
    return full_rotations;
}


void alignSensor()
{
    foc_direct(0.0f, 1.0f, 3.0f * M_PI / 2.0f, &htim1);
    HAL_Delay(700);
    SensorUpdate();
    zero_electric_angle = 0;
    zero_electric_angle = electricalAngle();
    HAL_Delay(20);
    foc_direct(0.0f, 0.0f, 0.0f, &htim1);
    HAL_Delay(100);
}
