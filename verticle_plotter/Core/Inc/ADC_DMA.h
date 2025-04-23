/*
 * ADC_DMA.h
 *
 *  Created on: Apr 10, 2025
 *      Author: HP
 */

#ifndef INC_ADC_DMA_H_
#define INC_ADC_DMA_H_

#include "main.h"
#include <MathOperation.h>

typedef struct {
    ADC_HandleTypeDef *hadc;    // ADC handle
    uint16_t *dma_buffer;       // DMA buffer for ADC values
    uint32_t buffer_length;     // Length of the DMA buffer
    uint8_t num_channels;       // Number of ADC channels
    float adc_vref;             // ADC reference voltage (typically 3.3V)
    float adc_resolution;       // ADC resolution (4095 for 12-bit)
} ADC_DMA;

// Function prototypes
void ADC_DMA_Init(ADC_DMA *adc_dma, ADC_HandleTypeDef *hadc,
                 uint16_t *buffer, uint32_t buffer_length,
                 uint8_t num_channels, float vref, float resolution);

void ADC_DMA_Start(ADC_DMA *adc_dma);
void ADC_DMA_Stop(ADC_DMA *adc_dma);

// Get raw voltage from ADC channel
float ADC_DMA_GetValue(ADC_DMA *adc_dma, uint8_t channel_index);

// Get joystick X and Y values
float ADC_DMA_GetJoystick(ADC_DMA *adc_dma, uint8_t channel_index, float joydata);

// Compute current from specified channel
float ADC_DMA_ComputeCurrent(ADC_DMA *adc_dma, uint8_t channel_index, float offset_voltage);

#endif /* INC_ADC_DMA_H_ */
