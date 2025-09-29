/*
 * drv8302.c
 *
 *  Created on: Sep 29, 2025
 *      Author: Gigabyte
 */

#include "drv8302.h"

Current foc_callback(uint16_t adc1_value, uint16_t adc2_value, float angle)
{
	Current current;
	float voltageA = (adc1_value/4095.0) * 3.265 - 3.265/2.0;
    float voltageB = (adc2_value/4095.0) * 3.265 - 3.265/2.0;
	float currentA_measure = voltageA/(0.165 * 40.0);
	float currentB_measure = voltageB/(0.165 * 40.0);

    float value1 = currentA_measure;
	float value2 = currentB_measure;
	float phasa_B_callback = value2;
	float phasa_A_callback = value1;

	float alpha_callback = phasa_A_callback;
	float betta_callback = 1.0/sqrt(3)*(phasa_A_callback + 2.0*phasa_B_callback);

	float theta_electric_rad = angle*M_PI/180;

	current.id_callback = alpha_callback * cosf(theta_electric_rad) + betta_callback * sinf(theta_electric_rad);
	current.iq_callback = -alpha_callback * sinf(theta_electric_rad) + betta_callback * cosf(theta_electric_rad);

	return current;
}

Phasa foc_direct(float id_ref, float iq_ref, float angle_electric_rad)
{
	Phasa phasa;
	float alpha = (id_ref * cosf(angle_electric_rad)) - (iq_ref * sinf(angle_electric_rad));
	float betta = (id_ref * sinf(angle_electric_rad)) + (iq_ref * cosf(angle_electric_rad));

	phasa.phasa_A = alpha;
	phasa.phasa_B = (-0.5 * alpha) + (sqrt(3)/2 * betta);
	phasa.phasa_C = (-0.5 * alpha) - (sqrt(3)/2 * betta);

	phasa.phasa_A = phasa.phasa_A * 0.5 + 0.5;
	phasa.phasa_B = phasa.phasa_B * 0.5 + 0.5;
	phasa.phasa_C = phasa.phasa_C * 0.5 + 0.5;
	return phasa;
	//TIM1->CCR1 = TIM1->ARR*phasa_A;
	//TIM1->CCR2 = TIM1->ARR*phasa_B;
	//TIM1->CCR3 = TIM1->ARR*phasa_C;
}

