/*
 * FIR.c
 *
 *  Created on: Apr 5, 2025
 *      Author: transporter
 */
#include "FIR.h"
#include "arm_math.h"

void FIR_init(FIR *fir, uint16_t numTaps, float cutoffFreq, float samplingFreq) {
    // Make sure numTaps is odd
    if (numTaps % 2 == 0) {
        numTaps += 1;
    }

    fir->numTaps = numTaps;
    fir->bufferIndex = 0;

    // Allocate memory for coefficients and buffer
    fir->coeffs = (float*)malloc(numTaps * sizeof(float));
    fir->buffer = (float*)malloc(numTaps * sizeof(float));

    if (fir->coeffs != NULL && fir->buffer != NULL) {
        // Clear buffer
        for (uint16_t i = 0; i < numTaps; i++) {
            fir->buffer[i] = 0.0f;
        }

        // Calculate normalized cutoff frequency (0 to 0.5)
        float omega = cutoffFreq / samplingFreq;

        // Calculate filter coefficients (low-pass)
        int16_t half_taps = numTaps / 2;
        for (int16_t i = 0; i < numTaps; i++) {
            if (i == half_taps) {
                // Center tap
                fir->coeffs[i] = 2.0f * omega;
            } else {
                // Side taps
                int16_t n = i - half_taps;
                fir->coeffs[i] = sinf(2.0f * PI * omega * n) / (PI * n);
            }

            // Apply Hamming window
            fir->coeffs[i] *= (0.54f - 0.46f * cosf(2.0f * PI * i / (numTaps - 1)));
        }

        // Normalize gain
        float sum = 0.0f;
        for (uint16_t i = 0; i < numTaps; i++) {
            sum += fir->coeffs[i];
        }

        if (sum != 0.0f) {
            for (uint16_t i = 0; i < numTaps; i++) {
                fir->coeffs[i] /= sum;
            }
        }
    }
}

float FIR_process(FIR *fir, float input) {
    // Update circular buffer with new input
    fir->buffer[fir->bufferIndex] = input;

    // Compute output (convolution)
    float output = 0.0f;
    uint16_t index = fir->bufferIndex;

    for (uint16_t i = 0; i < fir->numTaps; i++) {
        output += fir->buffer[index] * fir->coeffs[i];

        // Move back in circular buffer
        if (index == 0) {
            index = fir->numTaps - 1;
        } else {
            index--;
        }
    }

    // Update buffer index for next input
    fir->bufferIndex++;
    if (fir->bufferIndex >= fir->numTaps) {
        fir->bufferIndex = 0;
    }

    return output;
}

void FIR_reset(FIR *fir) {
    // Reset buffer
    for (uint16_t i = 0; i < fir->numTaps; i++) {
        fir->buffer[i] = 0.0f;
    }
    fir->bufferIndex = 0;
}

void FIR_delete(FIR *fir) {
    // Free allocated memory
    if (fir->coeffs != NULL) {
        free(fir->coeffs);
        fir->coeffs = NULL;
    }

    if (fir->buffer != NULL) {
        free(fir->buffer);
        fir->buffer = NULL;
    }
}
