/*
 * mpu6050.h
 *
 *  Created on: Oct 8, 2025
 *      Author: Gigabyte
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdio.h>
#include "math.h"
#include "string.h"
#include <stdint.h>


#define MPU6050_ADDR 		0x68 << 1
#define SMPLRT_DIV_REG 		0x19
#define GYRO_CONFIG_REG 	0x1B
#define ACCEL_CONFIG_REG 	0x1C
#define ACCEL_XOUT_H_REG 	0x3B
#define TEMP_OUT_H_REG 		0x41
#define GYRO_XOUT_H_REG 	0x43
#define PWR_MGMT_1_REG 		0x6B
#define WHO_AM_I_REG 		0x75
#define RAD_TO_DEG 57.295779513082320876798154814105

typedef struct{

int16_t gyroX;
int16_t gyroY;
int16_t gyroZ;

int16_t accelX;
int16_t accelY;
int16_t accelZ;

float Ax;
float Ay;
float Az;

float Gx;
float Gy;
float Gz;


float Roll_gyro;
float Pitch_gyro;
float Yaw_gyro;

float correctForRoll;
float correctForPitch;

float KalmanAngleX;
float KalmanAngleY;
//float roll2;
} MPU6050_t;
typedef struct
{
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
} Kalman_t;

extern Kalman_t KalmanX;
extern Kalman_t KalmanY;
extern MPU6050_t MPU6050;
extern const float Accel_Z_corrector;
extern uint32_t timer;



void MPU6050_Init(MPU6050_t *MPUDataStruct);
float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt);
void MPU6050_Read_Gyro(MPU6050_t *MPUDataStruct);

#endif /* INC_MPU6050_H_ */
