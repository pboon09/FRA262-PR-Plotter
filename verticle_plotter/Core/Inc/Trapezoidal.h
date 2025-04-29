/*
 * Trapezoidal.h
 *
 *  Created on: Apr 10, 2025
 *      Author: HP
 */

#ifndef INC_TRAPEZOIDAL_H_
#define INC_TRAPEZOIDAL_H_

#include "main.h"
#include "stdlib.h"
#include <stdbool.h>
#include "arm_math.h"

typedef struct
{
    float32_t t1;          // Time to reach maximum acceleration
    float32_t t2;          // Time to reach constant velocity
    float32_t t3;          // Time to start deceleration
    float32_t time_total;  // Total trajectory time
    int8_t dir;            // Direction of motion
} Trapezoidal_GenStruct;

typedef struct
{
    float32_t setposition;     // Current position
    float32_t setvelocity;     // Current velocity
    float32_t setacceleration; // Current acceleration
    float32_t t;               // Current time
    bool isFinised;            // Flag indicating if trajectory is complete
} Trapezoidal_EvaStruct;

void Trapezoidal_Generator(volatile Trapezoidal_GenStruct *trapGen, float32_t initial_p, float32_t target_p, float32_t vmax, float32_t amax);
void Trapezoidal_Evaluated(volatile Trapezoidal_GenStruct *trapGen, volatile Trapezoidal_EvaStruct *evaTrapezoidal, float32_t initial_p, float32_t target_p, float32_t vmax, float32_t amax);

#endif /* INC_TRAPEZOIDAL_H_ */
