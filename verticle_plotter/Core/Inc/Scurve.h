/*
 * Scurve.h
 *
 *  Created on: May 29, 2023
 *      Author: pawee
 */

#ifndef INC_SCURVE_H_
#define INC_SCURVE_H_

#include "main.h"
#include "math.h"
#include "stdlib.h"
#include <stdbool.h>

typedef struct
{
	double t1;
	double t2;
	double t3;
	double t4;
	double t5;
	double t6;
	double t7;
	double time_total;
	int8_t dir;
} Scurve_GenStruct;

typedef struct
{
	double setposition;
	double setvelocity;
	double setacceleration;
	double t;
	bool isFinised;
} Scurve_EvaStruct;

void Trajectory_Generator(volatile Scurve_GenStruct *Scurve,float initial_p,float target_p,float vmax,float amax,float jmax);
void Trajectory_Evaluated(volatile Scurve_GenStruct *genScurve,volatile Scurve_EvaStruct *evaScurve,float initial_p,float target_p,float vmax,float amax,float jmax);

#endif /* INC_SCURVE_H_ */
