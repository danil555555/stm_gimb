/*
 * drv8302.h
 *
 *  Created on: Sep 29, 2025
 *      Author: Gigabyte
 */

#ifndef INC_DRV8302_H_
#define INC_DRV8302_H_
#include <math.h>
#include <stdint.h>
#include <stdio.h>

typedef struct {
    float id_callback;
    float iq_callback;
} Current;

typedef struct {
	float phasa_A;
	float phasa_B;
	float phasa_C;
} Phasa;

Current foc_callback(uint16_t adc1_value, uint16_t adc2_value, float angle);
Phasa foc_direct(float id_ref, float iq_ref, float angle_electric_rad);


#endif /* INC_DRV8302_H_ */
