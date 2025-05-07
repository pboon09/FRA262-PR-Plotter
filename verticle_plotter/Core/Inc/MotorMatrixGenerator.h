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
#include "arm_math.h"

typedef float float32_t;

void GenerateMotorMatrices(float32_t R_a, float32_t L_a, float32_t J, float32_t b,
                          float32_t ke, float32_t kt, float32_t dt,
                          float32_t *A, float32_t *B);

#endif /* INC_MOTORMATRIXGENERATOR_H_ */
