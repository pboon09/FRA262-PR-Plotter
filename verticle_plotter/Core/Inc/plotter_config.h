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

#include "stdio.h"
#include "string.h"

#include "QEI.h"
#include "PWM.h"
#include "Cytron_MDXX.h"
#include "Controller.h"
#include "ADC_DMA.h"
#include "signal_generator.h"
#include "FIR.h"
#include "kalman.h"
#include "ModBusRTU.h"
#include "DC_MOTOR.h"

/*-------Configure Signal Generator Start------*/
extern SignalGenerator sine_sg;
extern SignalGenerator chirp_linear_sg;
extern SignalGenerator chirp_log_sg;
extern SignalGenerator square_sg;
extern SignalGenerator ramp_sg;

// Sine wave configuration
#define SINE_AMPLITUDE       (0.7f / 3.0f)
#define SINE_FREQUENCY       0.05f
#define SINE_PHASE           0.0f
#define SINE_OFFSET          0.0f
#define SINE_MIN_SETPOINT    (-0.7f / 3.0f)
#define SINE_MAX_SETPOINT    (0.7f / 3.0f)

// Chirp linear configuration
#define CHIRP_LINEAR_AMPLITUDE    (0.7f / 3.0f)
#define CHIRP_LINEAR_START_FREQ   1.0f
#define CHIRP_LINEAR_END_FREQ     10.0f
#define CHIRP_LINEAR_DURATION     5.0f
#define CHIRP_LINEAR_MIN_SETPOINT (-0.7f / 3.0f)
#define CHIRP_LINEAR_MAX_SETPOINT (0.7f / 3.0f)

// Chirp logarithmic configuration
#define CHIRP_LOG_AMPLITUDE    (0.7f / 3.0f)
#define CHIRP_LOG_START_FREQ   1.0f
#define CHIRP_LOG_END_FREQ     10.0f
#define CHIRP_LOG_DURATION     5.0f
#define CHIRP_LOG_MIN_SETPOINT (-0.7f / 3.0f)
#define CHIRP_LOG_MAX_SETPOINT (0.7f / 3.0f)

// Square wave configuration
#define SQUARE_AMPLITUDE     (0.7f / 3.0f)
#define SQUARE_FREQUENCY     2.0f
#define SQUARE_DUTY_CYCLE    0.3f
#define SQUARE_PHASE         0.0f
#define SQUARE_OFFSET        0.0f
#define SQUARE_MIN_SETPOINT  (-0.7f / 3.0f)
#define SQUARE_MAX_SETPOINT  (0.7f / 3.0f)

// Ramp wave configuration
#define RAMP_AMPLITUDE       (0.7f / 3.0f)
#define RAMP_FREQUENCY       1.0f
#define RAMP_START           (-0.7f / 3.0f)
#define RAMP_END             (0.7f / 3.0f)
#define RAMP_PHASE           0.0f
#define RAMP_OFFSET          0.0f
#define RAMP_MIN_SETPOINT    (-0.7f / 3.0f)
#define RAMP_MAX_SETPOINT    (0.7f / 3.0f)
/*-------Configure Signal Generator End------*/

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
#define ENC_TIM1 &htim4
#define ENC_TIM2 &htim3
#define ENC_PPR 8192.0
#define ENC_FREQ 1000
#define MOTOR_RATIO1 1.0f
#define MOTOR_RATIO2 1.0f
/*-----Configure Encoder End-----*/

/*-------Configure Controller Start------*/
extern TIM_HandleTypeDef htim2; //For Control Loop
extern PID_CONTROLLER prismatic_position_pid;
extern PID_CONTROLLER prismatic_velocity_pid;
extern PID_CONTROLLER revolute_position_pid;
extern PID_CONTROLLER revolute_velocity_pid;
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
extern ADC_DMA adc_dma;

// Define channel indices for ADC
#define JOYSTICK_X_CHANNEL 6      // ADC Channel 6
#define JOYSTICK_Y_CHANNEL 7      // ADC Channel 7
#define PRISMATIC_CURRENT_CHANNEL 8    // ADC Channel 8
#define REVOLUTE_CURRENT_CHANNEL 15    // ADC Channel 15

// Number of ADC channels and samples
#define ADC_CHANNELS 4
#define SAMPLES_PER_CHANNEL 10
#define ADC_BUFFER_SIZE (ADC_CHANNELS * SAMPLES_PER_CHANNEL)
extern uint16_t adc_dma_buffer[ADC_BUFFER_SIZE];

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
extern FIR LP_prismatic_velocity;
extern FIR LP_prismatic_current;
extern FIR LP_revolute_velocity;
extern FIR LP_revolute_current;
#define NUM_TAPS 31
#define CUTOFF_FREQ 25.0
#define SAMPLE_RATE 1000.0f  // 1kHz
/*-------Configure FIR Stop------*/

/*-------Configure Kalman Start------*/
extern KalmanFilter flit_prismatic_velocity;
extern float32_t prismatic_A[16];
extern float32_t prismatic_B[4];

extern KalmanFilter flit_revolute_velocity;
extern float32_t revolute_A[16];
extern float32_t revolute_B[4];
/*-------Configure Kalman Stop------*/

/*----- Config ModBus Start -----*/
extern ModbusHandleTypedef ModBus; // ModBus OOP
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim16;
extern u16u8_t registerFrame[200];
#define MODBUS_USART &huart2
#define MODBUS_DATA_SENDING_PERIOD_TIM &htim16
#define MODBUS_SLAVE_ADDRESS 0x15
#define MODBUS_REGISTER_FRAME_SIZE 200
/*----- Config ModBus End -----*/

void plotter_begin();
void update_sensors(void);

#endif /* INC_PLOTTER_CONFIG_H_ */
