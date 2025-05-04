/*
 * Calculator.h
 *
 *  Created on: May 1, 2024
 *      Author: User
 */

#ifndef INC_MATHOPERATION_H_
#define INC_MATHOPERATION_H_

#include "main.h"
#include "math.h"

float calculate_average(uint16_t *array, uint32_t length);
float mapf(float input, float min_input, float max_input, float min_output, float max_output);
float saturation(float input, float upper_limit, float lower_limit);

#endif /* INC_MATHOPERATION_H_ */
