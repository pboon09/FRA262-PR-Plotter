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

// Structure for dual-joint trajectory parameters
typedef struct
{
    Trapezoidal_GenStruct joint1;
    Trapezoidal_GenStruct joint2;
    float32_t time_total;  // Total synchronized time
} Dual_Trapezoidal_GenStruct;

// Structure for dual-joint trajectory evaluation
typedef struct
{
    Trapezoidal_EvaStruct joint1;
    Trapezoidal_EvaStruct joint2;
    float32_t t;           // Current time
    bool isFinised;        // Flag indicating if trajectory is complete
} Dual_Trapezoidal_EvaStruct;

void Trajectory_Generator(volatile Trapezoidal_GenStruct *trapGen, float32_t initial_p, float32_t target_p, float32_t vmax, float32_t amax);
void Trajectory_Evaluated(volatile Trapezoidal_GenStruct *trapGen, volatile Trapezoidal_EvaStruct *evaTrapezoidal, float32_t initial_p, float32_t target_p, float32_t vmax, float32_t amax);

// Function to generate synchronized dual-joint trajectories
void Dual_Trajectory_Generator(
    volatile Dual_Trapezoidal_GenStruct *dualTrapGen,
    float32_t initial_p1, float32_t target_p1, float32_t vmax1, float32_t amax1,
    float32_t initial_p2, float32_t target_p2, float32_t vmax2, float32_t amax2);

// Function to evaluate synchronized dual-joint trajectories
void Dual_Trajectory_Evaluated(
    volatile Dual_Trapezoidal_GenStruct *dualTrapGen,
    volatile Dual_Trapezoidal_EvaStruct *dualEvaTrapezoidal,
    float32_t initial_p1, float32_t target_p1, float32_t vmax1, float32_t amax1,
    float32_t initial_p2, float32_t target_p2, float32_t vmax2, float32_t amax2);

// Function to initialize dual-joint trajectory evaluation structure
void Dual_Trajectory_Init(volatile Dual_Trapezoidal_EvaStruct *dualEvaTrapezoidal);

#endif /* INC_TRAPEZOIDAL_H_ */
