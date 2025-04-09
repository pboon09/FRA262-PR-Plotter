/*
 * WCS1700.h
 *
 *  Created on: Apr 28, 2024
 *      Author: Tim
 */

#ifndef INC_WCS1700_H_
#define INC_WCS1700_H_

#include <MathOperation.h>
#include "main.h"

typedef struct {
	ADC_HandleTypeDef *hadc; // ADC pointer
	uint16_t *pData; // Dynamically allocated data
	uint32_t lengthData;
	uint8_t rank;
	uint8_t index;
	float ADCResolution;
	float adcValue;
	float raw_voltage;
	float voltage;
	float current;
	float offset_voltage;



} WCS1700;

void WCS1700_init(WCS1700 *wcs1700,ADC_HandleTypeDef *hadc , uint16_t *data,uint32_t length, float res, uint8_t rank, uint8_t index, float offset_voltage);
void WCS1700_compute_data(WCS1700 *wcs1700);




#endif /* INC_WCS1700_H_ */
