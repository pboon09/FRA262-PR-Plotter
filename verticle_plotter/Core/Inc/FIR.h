/*
 * FIR.h
 *
 *  Created on: Apr 5, 2025
 *      Author: transporter
 */

#ifndef INC_FIR_H_
#define INC_FIR_H_

#include "main.h"
#include <stdlib.h>
#include <math.h>

typedef struct {
    float *coeffs;           // Filter coefficients
    float *buffer;           // Circular buffer for input samples
    uint16_t numTaps;        // Number of filter taps
    uint16_t bufferIndex;    // Current position in circular buffer
} FIR;

// Function prototypes
void FIR_init(FIR *fir, uint16_t numTaps, float cutoffFreq, float samplingFreq);
float FIR_process(FIR *fir, float input);
void FIR_reset(FIR *fir);
void FIR_delete(FIR *fir);

#endif /* INC_FIR_H_ */
