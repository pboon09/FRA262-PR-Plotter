/*
 * wcs1700.c
 *
 *  Created on: Apr 28, 2024
 *      Author: User
 */

#include <WCS1700.h>
#include <stdlib.h>


void WCS1700_init(WCS1700 *wcs1700,ADC_HandleTypeDef *hadc , uint16_t *data,uint32_t length,float res, uint8_t rank, uint8_t index, float offset_voltage){
	wcs1700->hadc = hadc;
	wcs1700->pData = data;
	wcs1700->lengthData = length;
	wcs1700->ADCResolution = res;
	wcs1700->rank = rank;
	wcs1700->index = index;//1,2
	wcs1700->adcValue = 0;
	wcs1700->raw_voltage = 0;
	wcs1700->voltage = 0;
	wcs1700->current = 0;
	wcs1700->offset_voltage = offset_voltage;

	for (int i = 0; i < wcs1700->lengthData; i++) {
		wcs1700->pData[i] = 0;
	}

//	HAL_ADCEx_Calibration_Start(wcs1700->hadc, ADC_SINGLE_ENDED);
//	HAL_ADC_Start_DMA(wcs1700->hadc, wcs1700->pData, wcs1700->lengthData);

}

void WCS1700_compute_data(WCS1700 *wcs1700){
	uint32_t sum = 0;
	for (uint32_t j =  0; j < wcs1700->lengthData; j += wcs1700->rank) {
		if (j + wcs1700->index < wcs1700->lengthData) {
			sum += wcs1700->pData[j + wcs1700->index];
		}
	}
	wcs1700->adcValue = (float)sum / (wcs1700->lengthData/wcs1700->rank)  ;
	wcs1700->raw_voltage = mapf(wcs1700->adcValue,0,wcs1700->ADCResolution,0,3.3);
	wcs1700->voltage = wcs1700->raw_voltage - wcs1700->offset_voltage;
	wcs1700->current = 15.1793457908771*wcs1700->voltage - 24.8674344063837; //Linear Regression 9 points

}








