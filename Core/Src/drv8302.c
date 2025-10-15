/*
 * drv8302.c
 *
 *  Created on: Sep 29, 2025
 *      Author: Gigabyte
 */

#include "drv8302.h"
#include "stm32f4xx_hal.h"

#define dt  0.0001

#ifndef min
  #define min(a,b)  (( (a) < (b) ) ? (a) : (b))
#endif

#ifndef max
  #define max(a,b)  (( (a) > (b) ) ? (a) : (b))
#endif

#ifndef _constrain
  #define _constrain(x, min_val, max_val)  ( ((x)<(min_val)) ? (min_val) : (((x)>(max_val)) ? (max_val) : (x)) )
#endif



float Ualpha = 0;
float Ubeta = 0;
float Ua = 0;
float Ub = 0;
float Uc = 0;
#define _SQRT3_2  0.86602540378f
float center = 0;
#define VOLTAGE_POWER_SUPPLY 12.0f
#define VOLTAGE_LIMIT  (VOLTAGE_POWER_SUPPLY * 0.866f)
float dc_a = 0;
float dc_b = 0;
float dc_c = 0;

float LPF_current_q(float in)
{
    static float prev_q = 0.0f;   //syokai dake syokika
    float tau_q = 0.01f;         // tau > 0
    float alpha = dt / (tau_q + dt);
    alpha = 0.2;

    float out = prev_q + alpha * (in - prev_q);
    prev_q = out;
    return out;
}

float LPF_current_d(float in)
{
    static float prev_d = 0.0f;//syokai dake syokika
    float tau_d = 0.01f;         // tau > 0
    float alpha = dt / (tau_d + dt);
    alpha = 0.2;

    float out = prev_d + alpha * (in - prev_d);
    prev_d = out;
    return out;
}

Current foc_callback(uint16_t adc1_value, uint16_t adc2_value, float theta_electric_rad)
{
	Current current;
	float voltageA = (adc1_value/4095.0) * 3.265 - 3.265/2.0;
    float voltageB = (adc2_value/4095.0) * 3.265 - 3.265/2.0;
	float currentA_measure = voltageA/(0.005 * 40.0);
	float currentB_measure = voltageB/(0.005 * 40.0);

    current.value1 = currentA_measure;
	current.value2 = currentB_measure;
	float phasa_B_callback = current.value2;
	float phasa_A_callback = current.value1;
	float phasa_C_callback = -(phasa_B_callback + phasa_A_callback);

	float mid = (1.f/3)*(phasa_B_callback + phasa_A_callback + phasa_C_callback);
	phasa_A_callback = phasa_A_callback - mid;
	phasa_B_callback = phasa_B_callback - mid;

	float alpha_callback = phasa_A_callback;
	float betta_callback = 1.0/sqrt(3)*(phasa_A_callback + 2.0*phasa_B_callback);


	current.id_callback = alpha_callback * cosf(theta_electric_rad) + betta_callback * sinf(theta_electric_rad);
	current.iq_callback = -alpha_callback * sinf(theta_electric_rad) + betta_callback * cosf(theta_electric_rad);

	current.iq_callback = LPF_current_q(current.iq_callback);
	current.id_callback = LPF_current_d(current.id_callback);

	return current;
}

Phasa foc_direct(float id_ref, float iq_ref, float angle_electric_rad, TIM_HandleTypeDef *htim)
{
	Phasa phasa;
	float alpha = (id_ref * cosf(angle_electric_rad)) - (iq_ref * sinf(angle_electric_rad));
	float betta = (id_ref * sinf(angle_electric_rad)) + (iq_ref * cosf(angle_electric_rad));

	phasa.phasa_A = alpha;
	phasa.phasa_B = (-0.5 * alpha) + (sqrt(3)/2 * betta);
	phasa.phasa_C = (-0.5 * alpha) - (sqrt(3)/2 * betta);

	//phasa.phasa_A = phasa.phasa_A * 0.5 + 0.5;
	//phasa.phasa_B = phasa.phasa_B * 0.5 + 0.5;
	//phasa.phasa_C = phasa.phasa_C * 0.5 + 0.5;

    center = VOLTAGE_LIMIT / 2;
    float Umin = min(Ua, min(Ub, Uc));
    float Umax = max(Ua, max(Ub, Uc));
    center -= (Umax + Umin) / 2;
    Ua += center;
    Ub += center;
    Uc += center;

    Ua = _constrain(phasa.phasa_A, 0, VOLTAGE_LIMIT);
    Ub = _constrain(phasa.phasa_B, 0, VOLTAGE_LIMIT);
    Uc = _constrain(phasa.phasa_C, 0, VOLTAGE_LIMIT);

    dc_a = _constrain(Ua / VOLTAGE_POWER_SUPPLY, 0.0f, 1.0f);
    dc_b = _constrain(Ub / VOLTAGE_POWER_SUPPLY, 0.0f, 1.0f);
    dc_c = _constrain(Uc / VOLTAGE_POWER_SUPPLY, 0.0f, 1.0f);

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim);



    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, (uint32_t)(dc_a * (float)period) );
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, (uint32_t)(dc_b * (float)period) );
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, (uint32_t)(dc_c * (float)period) );
	return phasa;


	//TIM1->CCR1 = TIM1->ARR*phasa_A;
	//TIM1->CCR2 = TIM1->ARR*phasa_B;
	//TIM1->CCR3 = TIM1->ARR*phasa_C;
}

