/*
 * Controller.h
 *
 *  Created on: Mar 31, 2024
 *      Author: beamk
 */
#ifndef __CONTROLLER_H__
#define __CONTROLLER_H__

#include "stdint.h"

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float u_max;
	float ek_1;
	float ek_2;
	float u;
} PID_CONTROLLER;

int32_t PWM_Satuation(float _u, int32_t _upper_limit, int32_t _lower_limit);
void PID_CONTROLLER_Init(PID_CONTROLLER *controller, float _Kp, float _Ki, float _Kd, float _u_max);
float PID_CONTROLLER_Compute(PID_CONTROLLER *controller, float ek);

#endif /* INC_CONTROLLER_H_ */
