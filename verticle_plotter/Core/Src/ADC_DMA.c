/*
 * ADC_DMA.c
 *
 *  Created on: Apr 10, 2025
 *      Author: HP
 */

#include "ADC_DMA.h"

void ADC_DMA_Init(ADC_DMA *adc_dma, ADC_HandleTypeDef *hadc,
                 uint16_t *buffer, uint32_t buffer_length,
                 uint8_t num_channels, float vref, float resolution) {

    adc_dma->hadc = hadc;
    adc_dma->dma_buffer = buffer;
    adc_dma->buffer_length = buffer_length;
    adc_dma->num_channels = num_channels;
    adc_dma->adc_vref = vref;
    adc_dma->adc_resolution = resolution;

    // Default center point and error percentage
    adc_dma->center_point = resolution / 2.0f;  // Typically 2048 for 12-bit ADC
    adc_dma->error_percentage = 5;              // 5% error by default
    adc_dma->threshold = (adc_dma->error_percentage / 100.0f) * adc_dma->center_point;

    // Initialize DMA buffer
    for (uint32_t i = 0; i < buffer_length; i++) {
        buffer[i] = 0;
    }
}

void ADC_DMA_Start(ADC_DMA *adc_dma) {
    // Perform ADC calibration
    HAL_ADCEx_Calibration_Start(adc_dma->hadc, ADC_SINGLE_ENDED);

    // Start ADC with DMA
    HAL_ADC_Start_DMA(adc_dma->hadc, (uint32_t*)adc_dma->dma_buffer, adc_dma->buffer_length);
}

void ADC_DMA_Stop(ADC_DMA *adc_dma) {
    HAL_ADC_Stop_DMA(adc_dma->hadc);
}

float ADC_DMA_GetValue(ADC_DMA *adc_dma, uint8_t channel_index) {
    uint32_t sum = 0;
    uint32_t samples = 0;

    // Average all samples for this channel
    for (uint32_t j = channel_index; j < adc_dma->buffer_length; j += adc_dma->num_channels) {
        sum += adc_dma->dma_buffer[j];
        samples++;
    }

    // Calculate raw ADC value
    if (samples > 0) {
        return (float)sum / samples;
    }

    return 0.0f;
}

void ADC_DMA_SetCenterPoint(ADC_DMA *adc_dma, float center_point, uint8_t error_percentage) {
    adc_dma->center_point = center_point;
    adc_dma->error_percentage = error_percentage;
    // Update threshold
    adc_dma->threshold = (adc_dma->error_percentage / 100.0f) * adc_dma->center_point;
}

float ADC_DMA_GetJoystickValue(ADC_DMA *adc_dma, uint8_t channel_index, float min_output, float max_output) {
    // Get raw value
    float value = ADC_DMA_GetValue(adc_dma, channel_index);

    // Apply threshold (dead zone) as in XYAnalog
    if (fabsf(value - adc_dma->center_point) < adc_dma->threshold) {
        value = adc_dma->center_point;
    }

    // Map to desired output range
    return mapf(value, 0.0f, adc_dma->adc_resolution, min_output, max_output);
}

float ADC_DMA_ComputeCurrent(ADC_DMA *adc_dma, uint8_t channel_index, float offset_voltage) {
    // Get raw value
    float value = ADC_DMA_GetValue(adc_dma, channel_index);

    // Convert to voltage
    float raw_voltage = mapf(value, 0, adc_dma->adc_resolution, 0, adc_dma->adc_vref);

    // Calculate offset voltage
    float voltage = raw_voltage - offset_voltage;

    // Compute current using WCS1700 formula
    return 15.1793457908771f * voltage - 24.8674344063837f;
}
