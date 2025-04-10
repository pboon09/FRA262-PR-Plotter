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

float ADC_DMA_GetChannelVoltage(ADC_DMA *adc_dma, uint8_t channel_index) {
    uint32_t sum = 0;
    uint32_t samples = 0;

    // Average all samples for this channel
    for (uint32_t j = 0; j < adc_dma->buffer_length; j += adc_dma->num_channels) {
        if ((j + channel_index) < adc_dma->buffer_length) {
            sum += adc_dma->dma_buffer[j + channel_index];
            samples++;
        }
    }

    // Calculate raw ADC value and convert to voltage
    if (samples > 0) {
        float raw_value = (float)sum / samples;
        return mapf(raw_value, 0, adc_dma->adc_resolution, 0, adc_dma->adc_vref);
    }

    return 0.0f;
}

float ADC_DMA_ComputeCurrent(ADC_DMA *adc_dma, uint8_t channel_index, float offset_voltage) {
    // Get raw voltage
    float raw_voltage = ADC_DMA_GetChannelVoltage(adc_dma, channel_index);

    // Calculate offset voltage
    float voltage = raw_voltage - offset_voltage;

    // Compute current using WCS1700 formula
    return 15.1793457908771 * voltage - 24.8674344063837;
}

float ADC_DMA_GetJoystickX(ADC_DMA *adc_dma, uint8_t channel_index) {
    float voltage = ADC_DMA_GetChannelVoltage(adc_dma, channel_index);

    return (voltage - (adc_dma->adc_vref / 2)) / (adc_dma->adc_vref / 2);
}

float ADC_DMA_GetJoystickY(ADC_DMA *adc_dma, uint8_t channel_index) {
    float voltage = ADC_DMA_GetChannelVoltage(adc_dma, channel_index);

    return (voltage - (adc_dma->adc_vref / 2)) / (adc_dma->adc_vref / 2);
}
