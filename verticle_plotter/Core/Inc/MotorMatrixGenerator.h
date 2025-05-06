/*
 * MotorMatrixGenerator.h
 *
 *  Created on: May 6, 2025
 *      Author: pboon
 */

#ifndef INC_MOTORMATRIXGENERATOR_H_
#define INC_MOTORMATRIXGENERATOR_H_

#include <stdint.h>
#include <stdio.h>

typedef float float32_t;

/*
 * Generate the system matrices for a DC motor Kalman filter
 *
 * Parameters:
 *   R_a - Armature resistance (Ohms)
 *   L_a - Armature inductance (H)
 *   J - Motor inertia (kg*m^2)
 *   b - Viscous friction coefficient (N*m*s)
 *   ke - Back-EMF constant (V*s/rad)
 *   kt - Torque constant (N*m/A)
 *   dt - Sample time (seconds)
 *   A - Output discrete state transition matrix (4x4)
 *   B - Output discrete input matrix (4x1)
 *   C - Output measurement matrix (1x4) - can be NULL if not needed
 *   G - Output process noise input matrix (4x1) - can be NULL if not needed
 */
void GenerateMotorMatrices(float32_t R_a, float32_t L_a, float32_t J, float32_t b,
                          float32_t ke, float32_t kt, float32_t dt,
                          float32_t *A, float32_t *B, float32_t *C, float32_t *G);

#endif /* INC_MOTORMATRIXGENERATOR_H_ */
