/*
 * plotter_config.h
 *
 *  Created on: Apr 20, 2025
 *      Author: pboon09
 */

#ifndef INC_PLOTTER_CONFIG_H_
#define INC_PLOTTER_CONFIG_H_

#include "main.h"
#include "tim.h"
#include "gpio.h"

#include "stdio.h"
#include "string.h"
#include <stdbool.h>

#include "DC_MOTOR.h"

#include "signal_generator.h"
#include "UnitConverter.h"
#include "ADC_DMA.h"

#include "Cytron_MDXX.h"
#include "PWM.h"
#include "QEI.h"
#include "Controller.h"

#include "FIR.h"
#include "MotorMatrixGenerator.h"
#include "MotorKalman.h"
#include "kalman.h"

#include "ModBusRTU.h"

/*-------Configure Kalman Start------*/
extern MotorKalman prismatic_kalman;
extern KalmanFilter revolute_kalman;
extern float32_t revolute_A[16];
extern float32_t revolute_B[4];
#define REVOLUTE_Q 1.0f
#define REVOLUTE_R 0.05f
/*-------Configure Kalman Stop------*/

/*-------Configure Signal Generator Start------*/
extern SignalGenerator sine_sg_PWM;
extern SignalGenerator square_sg_PWM;
extern SignalGenerator sine_sg_cascade;
extern SignalGenerator square_sg_cascade;
extern SignalGenerator sine_sg_prismatic;
extern SignalGenerator square_sg_prismatic;
extern SignalGenerator sine_sg_revolute;
extern SignalGenerator square_sg_revolute;

// Sine wave configuration
#define SINE_AMPLITUDE       65535.0f
#define SINE_FREQUENCY       0.1f
#define SINE_PHASE           0.0f
#define SINE_OFFSET          0.0f
#define SINE_MIN_SETPOINT    -65535.0f
#define SINE_MAX_SETPOINT    65535.0f

// Square wave configuration
#define SQUARE_AMPLITUDE     65535.0f
#define SQUARE_FREQUENCY     0.1f
#define SQUARE_DUTY_CYCLE    0.5f
#define SQUARE_PHASE         0.0f
#define SQUARE_OFFSET        0.0f
#define SQUARE_MIN_SETPOINT  -65535.0f
#define SQUARE_MAX_SETPOINT  65535.0f

// Sine wave configuration
#define SINE_AMPLITUDE_CASCADE       200.0f
#define SINE_MIN_SETPOINT_CASCADE   -200.0f
#define SINE_MAX_SETPOINT_CASCADE    200.0f

// Square wave configuration
#define SQUARE_AMPLITUDE_CASCADE     25.0f
#define SQUARE_MIN_SETPOINT_CASCADE -25.0f
#define SQUARE_MAX_SETPOINT_CASCADE  25.0f
/*-------Configure Signal Generator End------*/

/*-------Configure UnitConverterSystem Start------*/
extern UnitConverterSystem converter_system;
/*-------Configure UnitConverterSystem End------*/

/*-------Configure Prismatic Motor Start------*/
extern TIM_HandleTypeDef htim8;
extern MDXX prismatic_motor;
#define MOTOR1_TIM &htim8
#define MOTOR1_TIM_CH TIM_CHANNEL_2
#define MOTOR1_GPIOx GPIOA
#define MOTOR1_GPIO_Pin GPIO_PIN_9
/*-------Configure Prismatic Motor End------*/

/*-------Configure Revolute Motor Start------*/
extern TIM_HandleTypeDef htim8;
extern MDXX revolute_motor;
#define MOTOR2_TIM &htim8
#define MOTOR2_TIM_CH TIM_CHANNEL_1
#define MOTOR2_GPIOx GPIOC
#define MOTOR2_GPIO_Pin GPIO_PIN_8
/*-------Configure Revolute Motor End------*/

/*-------Configure Servo Start------*/
extern TIM_HandleTypeDef htim1;
extern PWM servo;
#define SERVO_TIM &htim1
#define SERVO_TIM_CH TIM_CHANNEL_4
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
#define MOTOR1_RATIO 1.0f
#define MOTOR2_RATIO 1.0f
#define MOTOR2_PULLEY_DIAMETER 0.0f //mm
/*-----Configure Encoder End-----*/

/*-------Configure Controller Start------*/
extern TIM_HandleTypeDef htim2; //For Control Loop
extern PID_CONTROLLER prismatic_position_pid;
extern PID_CONTROLLER prismatic_velocity_pid;
extern PID_CONTROLLER revolute_position_pid;
extern PID_CONTROLLER revolute_velocity_pid;
extern PID_CONTROLLER revolute_joy_pid;
#define CONTROL_TIM &htim2
/*-------Configure Controller Stop------*/

/*------- Configure DC Motor Feedforward/Disturbance Control Start ------*/
extern DC_MOTOR_FFeedward prismatic_motor_ffd;
extern DC_MOTOR_DFeedward prismatic_motor_dfd;
extern DC_MOTOR_FFeedward revolute_motor_ffd;
extern DC_MOTOR_DFeedward revolute_motor_dfd;

/*------- Configure DC Motor Feedforward/Disturbance Control End ------*/

/*-------Configure ADC DMA Start------*/
extern ADC_HandleTypeDef hadc1;  // Your ADC handle
extern ADC_DMA joystick;

// Define channel indices for ADC
#define JOYSTICK_X_CHANNEL 6      // ADC Channel 6
#define JOYSTICK_Y_CHANNEL 7      // ADC Channel 7

// Number of ADC channels and samples
#define ADC_CHANNELS 2
#define SAMPLES_PER_CHANNEL 50
#define ADC_BUFFER_SIZE (ADC_CHANNELS * SAMPLES_PER_CHANNEL)
extern uint16_t joystick_buffer[ADC_BUFFER_SIZE];

#define ADC_VREF 3.3f
#define ADC_RESOLUTION 4095.0f
#define ADC_CENTERPOINT 2048.0f
#define ADC_ERROR 5

// Current sensor offsets
#define PRISMATIC_CURRENT_OFFSET 1.65f
#define REVOLUTE_CURRENT_OFFSET 1.65f

extern float joystick_x;
extern float joystick_y;
extern float prismatic_current;
extern float revolute_current;

void update_sensors(void);
/*-------Configure ADC DMA End------*/

/*-------Configure FIR Start------*/
extern FIR prismatic_lp_accel;
extern FIR revolute_lp_accel;
#define NUM_TAPS 31
#define CUTOFF_FREQ 10.0
#define SAMPLE_RATE 1000.0f  // 1kHz
/*-------Configure FIR Stop------*/

///*-------Configure Kalman Start------*/
//extern MotorKalman prismatic_kalman;
//extern MotorKalman revolute_kalman;
///*-------Configure Kalman Stop------*/

/*----- Configure ModBus Start -----*/
extern ModbusHandleTypedef ModBus; // ModBus OOP
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim16;
extern u16u8_t registerFrame[200];
#define MODBUS_USART &huart2
#define MODBUS_DATA_SENDING_PERIOD_TIM &htim16
#define MODBUS_SLAVE_ADDRESS 0x15
#define MODBUS_REGISTER_FRAME_SIZE 200

#define Heartbeat_Protocol 0x00 // Read/Write
#define BaseSystem_Status 0x01 // Write
#define LimitSwitch_Status 0x03 // Read
#define Servo_UP 0x04 //Write
#define Servo_Down 0x05 //Write

#define R_Theta_Status 0x10 // Read
#define R_Axis_Actual_Position 0x11 //Write
#define Theta_Axis_Actual_Position 0x12 //Write
#define R_Axis_Actual_Speed 0x13 //Write
#define Theta_Axis_Actual_Speed 0x14 //Write
#define R_Axis_Acceleration 0x15 //Write
#define Theta_Axis_Acceleration 0x16 //Write

#define Target_PosR_1 0x20 // Read
#define Target_PosT_1 0x21 // Read
#define Target_PosR_2 0x22 // Read
#define Target_PosT_2 0x23 // Read
#define Target_PosR_3 0x24 // Read
#define Target_PosT_3 0x25 // Read
#define Target_PosR_4 0x26 // Read
#define Target_PosT_4 0x27 // Read
#define Target_PosR_5 0x28 // Read
#define Target_PosT_5 0x29 // Read
#define Target_PosR_6 0x30 // Read
#define Target_PosT_6 0x31 // Read
#define Target_PosR_7 0x32 // Read
#define Target_PosT_7 0x33 // Read
#define Target_PosR_8 0x34 // Read
#define Target_PosT_8 0x35 // Read
#define Target_PosR_9 0x36 // Read
#define Target_PosT_9 0x37 // Read
#define Target_PosR_10 0x38 // Read
#define Target_PosT_10 0x39 // Read

#define Goal_R 0x40 // Read
#define Goal_Theta 0x41 // Read
/*----- Configure ModBus End -----*/

/*----- Sensor State Variable Start -----*/
extern int up_lim, low_lim, b1, b2, b3, b4;

extern bool homing_active;
extern volatile bool up_photo;
extern volatile bool low_photo;
/*----- Sensor State Variable End -----*/

void plotter_begin();
void plotter_reset();
void plotter_update_sensors();
void plotter_pen_up();
void plotter_pen_down();
#endif /* INC_PLOTTER_CONFIG_H_ */
