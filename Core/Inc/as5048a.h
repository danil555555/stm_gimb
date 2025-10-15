/*
 * as5048a.h
 *
 *  Created on: Sep 26, 2025
 *      Author: Gigabyte
 */

#ifndef INC_AS5048A_H_
#define INC_AS5048A_H_

#include <stdint.h>
#include <math.h>

static float prev_angle_rad  = 0.0f;
static float full_rotations  = 0.0f;
static float zero_electric_angle = 0.0f;
static float now_angle = 0.0f;
static float angle_el = 0.0f;



static int pole_pairs = 14;
static int sensor_direction = 1; //(1 or -1)

uint16_t AS5048_ReadAngle(void);
uint16_t AS5048_Transfer(uint16_t data);
float _normalizeAngle(float angle);
void Sensor_Init();
void SensorUpdate();
float electricalAngle(void);
float NowAngle(void);
float Sensor_GetAngleRad(void);
void alignSensor();




#endif /* INC_AS5048A_H_ */
