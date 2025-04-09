/*
 * RMX_Motor.c
 *
 *  Created on: Nov 29, 2024
 *      Author: beamk
 */

#include <plotter_config.h>

MDXX prismatic_motor;
MDXX revolute_motor;

QEI prismatic_encoder;
QEI revolute_encoder;

PID_CONTROLLER prismatic_position_pid;
PID_CONTROLLER prismatic_velocity_pid;

PID_CONTROLLER revolute_position_pid;
PID_CONTROLLER revolute_velocity_pid;

void plotter_begin() {
	QEI_init(&prismatic_encoder, ENC_TIM1, ENC_PPR, ENC_FREQ, MOTOR_RATIO1);
	QEI_init(&revolute_encoder, ENC_TIM2, ENC_PPR, ENC_FREQ, MOTOR_RATIO2);

	MDXX_GPIO_init(&prismatic_motor, MOTOR1_TIM, MOTOR1_TIM_CH, MOTOR1_GPIOx, MOTOR1_GPIO_Pin);
	MDXX_GPIO_init(&revolute_motor, MOTOR2_TIM, MOTOR2_TIM_CH, MOTOR2_GPIOx, MOTOR2_GPIO_Pin);

	PWM_init(&servo, SERVO_TIM, SERVO_TIM_CH);

	MDXX_set_range(&prismatic_motor, 2000, 0);
	MDXX_set_range(&revolute_motor, 2000, 0);
	PWM_write_range(&servo, 2000, 0);

	PID_CONTROLLER_Init(&prismatic_position_pid, 0, 0, 0, 65535);
	PID_CONTROLLER_Init(&prismatic_velocity_pid, 0, 0, 0, 65535);

	PID_CONTROLLER_Init(&revolute_position_pid, 0, 0, 0, 65535);
	PID_CONTROLLER_Init(&revolute_velocity_pid, 0, 0, 0, 65535);

	HAL_TIM_Base_Start_IT(CONTROL_TIM);
}
