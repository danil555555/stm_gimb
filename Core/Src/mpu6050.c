/*
 * mpu6050.c
 *
 *  Created on: Oct 8, 2025
 *      Author: Gigabyte
 */
#include "mpu6050.h"
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

Kalman_t KalmanX = {
        .Q_angle = 0.1f,
        .Q_bias = 0.3f,
        .R_measure = 0.3f
};

Kalman_t KalmanY = {
        .Q_angle = 0.1f,
        .Q_bias = 0.3f,
        .R_measure = 0.3f,
};

const float Accel_Z_corrector = 14418.0;
MPU6050_t MPU6050;
uint32_t timer;

void MPU6050_Init(MPU6050_t *MPUDataStruct)
{
	uint8_t check, data;
	uint8_t buffer[6];
	int sumX = 0;
	int sumY = 0;
    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 100);
    if (check == 0x68) {
        printf("MPU6050 connect!\n");
    } else {
        printf("MPU6050 no connect! (OUT: 0x%02X)\n", check);
        while(1);
    }
    data = 0;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 100);
    data = 0x07;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 100);
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 100);
	data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 100);

    for(int i = 0; i < 500; i++)
    {
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, buffer, 6, 100);
        MPUDataStruct->gyroX = (uint16_t)(buffer[0] << 8 | buffer[1]);
        MPUDataStruct->gyroY = (uint16_t)(buffer[2] << 8 | buffer[3]);
        sumX += MPUDataStruct->gyroX;
        sumY += MPUDataStruct->gyroY;
        HAL_Delay(10);
    }

    MPUDataStruct->correctForRoll = sumX / 500.0;
    MPUDataStruct->correctForPitch = sumY / 500.0;


}
float Kalman_getAngle(Kalman_t *Kalman, float newAngle, float newRate, float dt)
{
	float rate = newRate - Kalman->bias;
	Kalman->angle += dt * rate;

	Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
	Kalman->P[0][1] -= dt * Kalman->P[1][1];
	Kalman->P[1][0] -= dt * Kalman->P[1][1];
	Kalman->P[1][1] += Kalman->Q_bias * dt;

    float S = Kalman->P[0][0] + Kalman->R_measure;
    float K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    float y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    float P00_temp = Kalman->P[0][0];
    float P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
}


void MPU6050_Read_Gyro(MPU6050_t *MPUDataStruct) {
	uint8_t data[6];
	uint8_t buffer[6];


	HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, data, 6, 100);
	MPUDataStruct->accelX = (data[0] << 8) | data[1];
	MPUDataStruct->accelY = (data[2] << 8) | data[3];
	MPUDataStruct->accelZ = (data[4] << 8) | data[5];

	MPUDataStruct->Ax = MPUDataStruct->accelX / 16384.0;
	MPUDataStruct->Ay = MPUDataStruct->accelY / 16384.0;
	MPUDataStruct->Az = MPUDataStruct->accelZ / Accel_Z_corrector;

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x43, 1, buffer, 6, 100);
    MPUDataStruct->gyroX = (uint16_t)(buffer[0] << 8 | buffer[1]);
    MPUDataStruct->gyroY = (uint16_t)(buffer[2] << 8 | buffer[3]);
    MPUDataStruct->gyroZ = (uint16_t)(buffer[4] << 8 | buffer[5]);

    MPUDataStruct->Gx = (MPUDataStruct->gyroX - MPUDataStruct->correctForRoll) / 131.0;
    MPUDataStruct->Gy = (MPUDataStruct->gyroY - MPUDataStruct->correctForPitch) / 131.0;
    MPUDataStruct->Gz = MPUDataStruct->gyroZ / 131.0;

    float dt = (float) (HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
    // интегрируем гироскоп
    /*uint32_t now = HAL_GetTick();
    dt = (now - *lastTimeGyro) / 1000.0;
    *lastTimeGyro = now;
    MPUDataStruct->Roll_gyro += (MPUDataStruct->Gx * dt);
    MPUDataStruct->Pitch_gyro += (MPUDataStruct->Gy * dt);*/
    float roll;
    float rollSqrt = sqrt((MPUDataStruct->accelX * MPUDataStruct->accelX) + (MPUDataStruct->accelZ * MPUDataStruct->accelZ));
    if(rollSqrt != 0)
    {
    	roll = atan2(-MPUDataStruct->accelY, rollSqrt) * RAD_TO_DEG;
    }
    else
    {
    	roll = 0;
    }

    float pitchSqrt = sqrt((MPUDataStruct->accelY * MPUDataStruct->accelY) + (MPUDataStruct->accelZ * MPUDataStruct->accelZ));
    float pitch;
    if(pitchSqrt != 0)
    {
    	pitch = atan2(-MPUDataStruct->accelX, pitchSqrt) * RAD_TO_DEG;
    }
    else
    {
    	pitch = 0;
    }

    MPUDataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch,MPUDataStruct->Gx, dt);
    MPUDataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll,MPUDataStruct->Gy, dt);

}
