/*
 * ZGX45RGG_150RPM.c
 *
 *  Created on: Apr 18, 2025
 *      Author: HP
 */

/*
 * ZGX45RGG_150RPM.c
 *
 * Created on: Apr 18, 2025
 * Author: HP
 */

#include <DC_MOTOR.h>
#include <math.h>
#include "MathOperation.h"

Motor_Constant_Structure ZGX45RGG_150RPM_Constant = {
    .Ke = 0.485667365845868,
    .Kt = 0.485667365845868 * 0.458826928266350,
    .L = 0.157854,
    .R = 1.15,
    .J = 0.001384174297611,
    .B = 0.065057814635037,
    .V_max = 12.0,
    .U_max = 65535.0,
    .qd_max = 15.707963249999999, 			// rad/s
	.qdd_max =  15.707963249999999 * 0.4
};

Motor_Constant_Structure ZGX45RGG_400RPM_Constant = {
    .Ke = 0.195904590337507,
    .Kt = 0.195904590337507 * 0.406856794178969,
    .L = 0.149095,
    .R = 1.00,
    .J = 0.000461046026253,
    .B = 0.007841032240553,
    .V_max = 12.0,
    .U_max = 65535.0,
    .qd_max = 41.887902 					// rad/s
};

Environment Disturbance_Constant = {
    .g = 9.81,               			// m/s²
    .plotter_mass = 500e-3,  			// kg
	.slide_rail_mass = 4.24,
	.offset = 30.07e-3,
    .c = 24.13e-3,
    .prismatic_pulley_radius = 1.5915e-2 		// m
};

void REVOLUTE_MOTOR_FFD_Init(DC_MOTOR_FFeedward *motor, Motor_Constant_Structure *_Mx){
    motor->Mx = _Mx;
}

float REVOLUTE_MOTOR_FFD_Compute(DC_MOTOR_FFeedward *motor, float qd){
    float transfer_function = (motor->Mx->B * motor->Mx->R + motor->Mx->Ke * motor->Mx->Kt) / motor->Mx->Kt;

    float v = qd * transfer_function;

    return mapf(v, -motor->Mx->V_max, motor->Mx->V_max, -motor->Mx->U_max, motor->Mx->U_max);
}

void REVOLUTE_MOTOR_DFD_Init(DC_MOTOR_DFeedward *motor, Motor_Constant_Structure *_Mx, Environment *_En){
    motor->Mx = _Mx;
    motor->En = _En;
}

float REVOLUTE_MOTOR_DFD_Compute(DC_MOTOR_DFeedward *motor, float q, float s){
    float gravity_compensate_plotter = motor->En->plotter_mass * motor->En->g * sin(q) * (s + motor->En->offset);

    float gravity_compensate_rail = motor->En->slide_rail_mass * motor->En->g * sin(q) * (motor->En->c);

    float transfer_function = motor->Mx->R / motor->Mx->Kt;

    float v = (gravity_compensate_plotter - gravity_compensate_rail) * transfer_function;

    return mapf(v, -motor->Mx->V_max, motor->Mx->V_max, -motor->Mx->U_max, motor->Mx->U_max);
}

void PRISMATIC_MOTOR_FFD_Init(DC_MOTOR_FFeedward *motor, Motor_Constant_Structure *_Mx) {
    motor->Mx = _Mx;
}

float PRISMATIC_MOTOR_FFD_Compute(DC_MOTOR_FFeedward *motor, float sd) {
    float transfer_function = (motor->Mx->B * motor->Mx->R  + motor->Mx->Ke * motor->Mx->Kt) / motor->Mx->Kt;

    float v = sd * transfer_function;

    return mapf(v, -motor->Mx->V_max, motor->Mx->V_max, -motor->Mx->U_max, motor->Mx->U_max);
}

void PRISMATIC_MOTOR_DFD_Init(DC_MOTOR_DFeedward *motor, Motor_Constant_Structure *_Mx, Environment *_En){
    motor->Mx = _Mx;
    motor->En = _En;
}

float PRISMATIC_MOTOR_DFD_Compute(DC_MOTOR_DFeedward *motor, float q, float qd, float s){
    float gravity_compensate_plotter = motor->En->plotter_mass * motor->En->g * cos(q);

    float centrifugal_force = motor->En->plotter_mass * qd * qd * s;

    float transfer_function = (motor->Mx->R * motor->En->prismatic_pulley_radius) / motor->Mx->Kt;

    float v = (gravity_compensate_plotter + centrifugal_force) * transfer_function;

    return mapf(v, -motor->Mx->V_max, motor->Mx->V_max, -motor->Mx->U_max, motor->Mx->U_max);
}
