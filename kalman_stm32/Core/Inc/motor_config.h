/*
 * plotter_config.h
 *
 *  Created on: Apr 20, 2025
 *      Author: pboon09
 */

#ifndef INC_MOTOR_CONFIG_H_
#define INC_MOTOR_CONFIG_H_

#include "main.h"
#include "tim.h"
#include "gpio.h"

#include "stdio.h"
#include "string.h"
#include <stdbool.h>

#include "Cytron_MDXX.h"
#include "PWM.h"
#include "QEI.h"
#include "signal_generator.h"
#include "Controller.h"

#include "MotorMatrixGenerator.h"
#include "MotorKalman.h"
#include "kalman.h"

#include "MathOperation.h"

/*-------Configure Kalman Start------*/
extern MotorKalman motor_kalman;
extern KalmanFilter	paweekorn_kalman;
extern float32_t A_d[16], B_d[4];
/*-------Configure Kalman Stop------*/

/*-------Configure Signal Generator Start------*/
extern SignalGenerator sine_sg_pwm;
extern SignalGenerator square_sg_pwm;
extern SignalGenerator sine_sg;
extern SignalGenerator square_sg;

// Sine wave configuration
#define SINE_FREQUENCY       0.1f
#define SINE_PHASE           0.0f
#define SINE_OFFSET          0.0f

// Square wave configuration
#define SQUARE_FREQUENCY     0.1f
#define SQUARE_DUTY_CYCLE    0.5f
#define SQUARE_PHASE         0.0f
#define SQUARE_OFFSET        0.0f
/*-------Configure Signal Generator End------*/

/*-------Configure Motor Start------*/
extern TIM_HandleTypeDef htim8;
extern MDXX motor;
#define MOTOR_TIM &htim8
#define MOTOR_TIM_CH TIM_CHANNEL_2
#define MOTOR_GPIOx GPIOA
#define MOTOR_GPIO_Pin GPIO_PIN_9
/*-------Configure Motor End------*/

/*-----Configure Encoder Start-----*/
extern TIM_HandleTypeDef htim5; //For QEI Encoder
extern QEI encoder;
#define ENC_TIM &htim5
#define ENC_PPR 8192.0
#define ENC_FREQ 1000
#define MOTOR_RATIO 1.0f
/*-----Configure Encoder End-----*/

/*-------Configure Controller Start------*/
extern TIM_HandleTypeDef htim2; //For Control Loop
extern PID_CONTROLLER velocity_pid;
#define CONTROL_TIM &htim2
/*-------Configure Controller Stop------*/

extern float J, B, Kt, Ke, R, L;

void motor_begin();
#endif /* INC_MOTOR_CONFIG_H_ */
