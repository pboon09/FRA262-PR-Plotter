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
#include <string.h>
#include "math.h"
#include "QEI.h"
#include "PWM.h"
#include "Cytron_MDXX.h"
#include "Controller.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "FIR.h"
#include  "skid_kinematic.h"

/*-------Configure RMX_Motor Start------*/
extern TIM_HandleTypeDef htim2; //For Control Loop
#define CONTROL_TIM &htim2 //For Control Loop
extern PID_CONTROLLER pid1;
extern PID_CONTROLLER pid2;
extern PID_CONTROLLER pid3;
extern PID_CONTROLLER pid4;
/*-------Configure RMX_Motor Stop------*/

///*-----Configure Motor 1 Start-----*/
extern TIM_HandleTypeDef htim8; //For Motor 1
extern MDXX motor1;
#define MOTOR1_TIM &htim8
#define MOTOR1_TIM_CH TIM_CHANNEL_1
#define MOTOR1_GPIOx GPIOB
#define MOTOR1_GPIO_Pin GPIO_PIN_5
///*-----Configure Motor End-----*/
//
///*-----Configure Motor 2 Start-----*/
extern TIM_HandleTypeDef htim8; //For Motor 1
extern MDXX motor2;
#define MOTOR2_TIM &htim8
#define MOTOR2_TIM_CH TIM_CHANNEL_2
#define MOTOR2_GPIOx GPIOB
#define MOTOR2_GPIO_Pin GPIO_PIN_4
///*-----Configure Motor End-----*/

/*-----Configure Motor 3 Start-----*/
extern TIM_HandleTypeDef htim8; //For Motor 1
extern MDXX motor3;
#define MOTOR3_TIM &htim8
#define MOTOR3_TIM_CH TIM_CHANNEL_3
#define MOTOR3_GPIOx GPIOA
#define MOTOR3_GPIO_Pin GPIO_PIN_8
/*-----Configure Motor End-----*/

/*-----Configure Motor 4 Start-----*/
extern TIM_HandleTypeDef htim8; //For Motor 1
extern MDXX motor4;
#define MOTOR4_TIM &htim8
#define MOTOR4_TIM_CH TIM_CHANNEL_4
#define MOTOR4_GPIOx GPIOB
#define MOTOR4_GPIO_Pin GPIO_PIN_10
/*-----Configure Motor End-----*/

/*-----Configure Encoder Start-----*/
extern TIM_HandleTypeDef htim3; //For QEI Encoder
extern TIM_HandleTypeDef htim4; //For QEI Encoder
extern TIM_HandleTypeDef htim5; //For QEI Encoder
extern TIM_HandleTypeDef htim20; //For QEI Encoder
extern QEI encoder1;
extern QEI encoder2;
extern QEI encoder3;
extern QEI encoder4;
#define ENC_TIM1 &htim3
#define ENC_TIM2 &htim4
#define ENC_TIM3 &htim5
#define ENC_TIM4 &htim20
#define ENC_PPR 8192.0
#define ENC_FREQ 1000
#define MOTOR_RATIO 0.33f
/*-----Configure Encoder End-----*/

extern FIR lowPassFilter1;
extern FIR lowPassFilter2;
extern FIR lowPassFilter3;
extern FIR lowPassFilter4;
#define NUM_TAPS 31
#define CUTOFF_FREQ 25.0
#define SAMPLE_RATE 1000.0f  // 1kHz

extern SKID_KINEMATIC robot_kinematics;
#define WHEEL_RADIUS 0.245 / 2.0
#define WHEEL_DISTANCE 3.8
#define MAX_LINEAR_VEL 0.085
#define MAX_ANGULAR_VEL 0.7 /3.0

void transporter_begin();

#endif /* INC_PLOTTER_CONFIG_H_ */
