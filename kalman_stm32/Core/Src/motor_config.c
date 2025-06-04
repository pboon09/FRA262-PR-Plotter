/*
 * plotter_config.c
 *
 *  Created on: Apr 20, 2025
 *      Author: pboon09
 */

#include <motor_config.h>
MotorKalman motor_kalman;
KalmanFilter paweekorn_kalman;
float32_t A_d[16], B_d[4];

SignalGenerator sine_sg_pwm;
SignalGenerator square_sg_pwm;
SignalGenerator sine_sg;
SignalGenerator square_sg;

MDXX motor;

QEI encoder;

PID_CONTROLLER velocity_pid;

float Ke = 0.485667365845868;
float Kt = 0.485667365845868 * 0.458826928266350;
float L = 0.157854;
float R = 1.15;
float J = 0.001384174297611;
float B = 0.065057814635037;

void motor_begin() {

	SIGNAL_init(&sine_sg_pwm, SIGNAL_SINE);
	SIGNAL_config_sine(&sine_sg_pwm, 65535.0,
	SINE_FREQUENCY, SINE_PHASE,
	SINE_OFFSET, -65535.0, 65535.0);

	SIGNAL_init(&square_sg_pwm, SIGNAL_SQUARE);
	SIGNAL_config_square(&square_sg_pwm, 65535.0,
	SQUARE_FREQUENCY,
	SQUARE_DUTY_CYCLE, SQUARE_PHASE, SQUARE_OFFSET, -65535.0, 65535.0);

	SIGNAL_init(&sine_sg, SIGNAL_SINE);
	SIGNAL_config_sine(&sine_sg, 40.0,
	SINE_FREQUENCY, SINE_PHASE,
	SINE_OFFSET, -40.0, 40.0);

	SIGNAL_init(&square_sg, SIGNAL_SQUARE);
	SIGNAL_config_square(&square_sg, 40.0,
	SQUARE_FREQUENCY,
	SQUARE_DUTY_CYCLE, SQUARE_PHASE, SQUARE_OFFSET, -40.0, 40.0);

	QEI_init(&encoder, ENC_TIM, ENC_PPR, ENC_FREQ, MOTOR_RATIO, 0.0);

	PID_CONTROLLER_Init(&velocity_pid, 7000, 0, 0, 65535.0);

	MDXX_GPIO_init(&motor, MOTOR_TIM, MOTOR_TIM_CH, MOTOR_GPIOx,
	MOTOR_GPIO_Pin);

	MDXX_set_range(&motor, 2000, 0);

	GenerateMotorMatrices(R, L, J, B* 0.6, Ke, Kt, 1e-3, A_d, B_d);
	Kalman_Start(&paweekorn_kalman, A_d, B_d, 1.0, 0.1);

	MotorKalman_Init(&motor_kalman, 1e-3, J, B, Kt, Ke, R, L, 1.0, 0.1);

	HAL_TIM_Base_Start_IT(CONTROL_TIM);
}
