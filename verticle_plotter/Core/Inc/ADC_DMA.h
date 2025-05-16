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

#include "main.h"
#include <MathOperation.h>

typedef struct {
    ADC_HandleTypeDef *hadc;    // ADC handle
    uint16_t *dma_buffer;       // DMA buffer for ADC values
    uint32_t buffer_length;     // Length of the DMA buffer
    uint8_t num_channels;       // Number of ADC channels
    float adc_vref;             // ADC reference voltage (typically 3.3V)
    float adc_resolution;       // ADC resolution (4095 for 12-bit)
    float center_point;         // Center point for joystick (default: 2048)
    float threshold;            // Threshold for input filtering
    uint8_t error_percentage;   // Error percentage for threshold calculation
} ADC_DMA;

// Function prototypes
void ADC_DMA_Init(ADC_DMA *adc_dma, ADC_HandleTypeDef *hadc,
                 uint16_t *buffer, uint32_t buffer_length,
                 uint8_t num_channels, float vref, float resolution);

void ADC_DMA_Start(ADC_DMA *adc_dma);
void ADC_DMA_Stop(ADC_DMA *adc_dma);

// Get raw value from ADC channel
float ADC_DMA_GetValue(ADC_DMA *adc_dma, uint8_t channel_index);

// Set center point and error percentage for threshold calculation
void ADC_DMA_SetCenterPoint(ADC_DMA *adc_dma, float center_point, uint8_t error_percentage);

// Get filtered joystick value with threshold and mapping applied
float ADC_DMA_GetJoystickValue(ADC_DMA *adc_dma, uint8_t channel_index, float min_output, float max_output);

// Compute current from specified channel
float ADC_DMA_ComputeCurrent(ADC_DMA *adc_dma, uint8_t channel_index, float offset_voltage);

#endif /* INC_ADC_DMA_H_ */
