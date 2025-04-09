/*
 * RMX_Motor.h
 *
 *  Created on: Nov 29, 2024
 *      Author: beamk
 */

#ifndef INC_PLOTTER_CONFIG_H_
#define INC_PLOTTER_CONFIG_H_

#include "main.h"
#include "tim.h"
#include "gpio.h"

#include "QEI.h"
#include "PWM.h"
#include "Cytron_MDXX.h"
#include "Controller.h"

/*-------Configure Prismatic Motor Start------*/
extern TIM_HandleTypeDef htim8;
extern MDXX prismatic_motor;
#define MOTOR1_TIM &htim8
#define MOTOR1_TIM_CH TIM_CHANNEL_1
#define MOTOR1_GPIOx GPIOC
#define MOTOR1_GPIO_Pin GPIO_PIN_8
/*-------Configure Prismatic Motor End------*/

/*-------Configure Revolute Motor Start------*/
extern TIM_HandleTypeDef htim8;
extern MDXX revolute_motor;
#define MOTOR2_TIM &htim8
#define MOTOR2_TIM_CH TIM_CHANNEL_2
#define MOTOR2_GPIOx GPIOA
#define MOTOR2_GPIO_Pin GPIO_PIN_9
/*-------Configure Revolute Motor End------*/

/*-------Configure Servo Start------*/
extern TIM_HandleTypeDef htim8;
extern PWM servo;
#define SERVO_TIM &htim8
#define SERVO_TIM_CH TIM_CHANNEL_3
/*-------Configure Servo End------*/

/*-----Configure Encoder Start-----*/
extern TIM_HandleTypeDef htim3; //For QEI Encoder
extern TIM_HandleTypeDef htim4; //For QEI Encoder
extern QEI prismatic_encoder;
extern QEI revolute_encoder;
#define ENC_TIM1 &htim3
#define ENC_TIM2 &htim4
#define ENC_PPR 8192.0
#define ENC_FREQ 1000
#define MOTOR_RATIO1 1.0f
#define MOTOR_RATIO2 0.5f
/*-----Configure Encoder End-----*/

/*-------Configure Controller Start------*/
extern TIM_HandleTypeDef htim2; //For Control Loop
#define CONTROL_TIM &htim2 //For Control Loop
extern PID_CONTROLLER prismatic_position_pid;
extern PID_CONTROLLER prismatic_velocity_pid;
extern PID_CONTROLLER revolute_position_pid;
extern PID_CONTROLLER revolute_velocity_pid;
/*-------Configure Controller Stop------*/

void plotter_begin();

#endif /* INC_PLOTTER_CONFIG_H_ */
