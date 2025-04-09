/*
 * RMX_Motor.c
 *
 *  Created on: Nov 29, 2024
 *      Author: beamk
 */

#include <plotter_config.h>

MDXX motor1;
MDXX motor2;
MDXX motor3;
MDXX motor4;

QEI encoder1;
QEI encoder2;
QEI encoder3;
QEI encoder4;

PID_CONTROLLER pid1;
PID_CONTROLLER pid2;
PID_CONTROLLER pid3;
PID_CONTROLLER pid4;

FIR lowPassFilter1;
FIR lowPassFilter2;
FIR lowPassFilter3;
FIR lowPassFilter4;

SKID_KINEMATIC robot_kinematics;

void transporter_begin() {
	SKID_init(&robot_kinematics, WHEEL_RADIUS, WHEEL_DISTANCE, MAX_LINEAR_VEL, MAX_ANGULAR_VEL);

	QEI_init(&encoder1, ENC_TIM1, ENC_PPR, ENC_FREQ, MOTOR_RATIO);
	QEI_init(&encoder2, ENC_TIM2, ENC_PPR, ENC_FREQ, MOTOR_RATIO);
	QEI_init(&encoder3, ENC_TIM3, ENC_PPR, ENC_FREQ, MOTOR_RATIO);
	QEI_init(&encoder4, ENC_TIM4, ENC_PPR, ENC_FREQ, MOTOR_RATIO);

	MDXX_GPIO_init(&motor1, MOTOR1_TIM, MOTOR1_TIM_CH, MOTOR1_GPIOx, MOTOR1_GPIO_Pin);
	MDXX_GPIO_init(&motor2, MOTOR2_TIM, MOTOR2_TIM_CH, MOTOR2_GPIOx, MOTOR2_GPIO_Pin);
	MDXX_GPIO_init(&motor3, MOTOR3_TIM, MOTOR3_TIM_CH, MOTOR3_GPIOx, MOTOR3_GPIO_Pin);
	MDXX_GPIO_init(&motor4, MOTOR4_TIM, MOTOR4_TIM_CH, MOTOR4_GPIOx, MOTOR4_GPIO_Pin);

	MDXX_set_range(&motor1, 2000, 0);
	MDXX_set_range(&motor2, 2000, 0);
	MDXX_set_range(&motor3, 2000, 0);
	MDXX_set_range(&motor4, 2000, 0);

	PID_CONTROLLER_Init(&pid1, 0, 0, 0, 65535);
	PID_CONTROLLER_Init(&pid2, 0, 0, 0, 65535);
	PID_CONTROLLER_Init(&pid3, 0, 0, 0, 65535);
	PID_CONTROLLER_Init(&pid4, 0, 0, 0, 65535);
//	PID_CONTROLLER_Init(&pid4, 500000, 7500, 0, 65535);

	FIR_init(&lowPassFilter1, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&lowPassFilter2, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&lowPassFilter3, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&lowPassFilter4, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);

	HAL_TIM_Base_Start_IT(CONTROL_TIM);
}
