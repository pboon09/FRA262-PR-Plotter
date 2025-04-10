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

PWM servo;

ADC_DMA adc_dma;

SignalGenerator sine_sg;
SignalGenerator chirp_linear_sg;
SignalGenerator chirp_log_sg;
SignalGenerator square_sg;
SignalGenerator ramp_sg;

FIR LP_prismatic_velocity;
FIR LP_prismatic_current;
FIR LP_revolute_velocity;
FIR LP_revolute_current;

KalmanFilter flit_prismatic_velocity;
KalmanFilter flit_revolute_velocity;

uint16_t adc_dma_buffer[ADC_BUFFER_SIZE];

float joystick_x = 0.0f;
float joystick_y = 0.0f;
float prismatic_current = 0.0f;
float revolute_current = 0.0f;

void plotter_begin() {
	SIGNAL_init(&sine_sg, SIGNAL_SINE);
	SIGNAL_config_sine(&sine_sg, SINE_AMPLITUDE, SINE_FREQUENCY, SINE_PHASE, SINE_OFFSET, SINE_MIN_SETPOINT, SINE_MAX_SETPOINT);

	SIGNAL_init(&chirp_linear_sg, SIGNAL_CHIRP);
	SIGNAL_config_chirp(&chirp_linear_sg, CHIRP_LINEAR_AMPLITUDE, CHIRP_LINEAR_START_FREQ,
	                   CHIRP_LINEAR_END_FREQ, CHIRP_LINEAR_DURATION, CHIRP_LINEAR,
	                   CHIRP_LINEAR_MIN_SETPOINT, CHIRP_LINEAR_MAX_SETPOINT);

	SIGNAL_init(&chirp_log_sg, SIGNAL_CHIRP);
	SIGNAL_config_chirp(&chirp_log_sg, CHIRP_LOG_AMPLITUDE, CHIRP_LOG_START_FREQ,
	                   CHIRP_LOG_END_FREQ, CHIRP_LOG_DURATION, CHIRP_LOGARITHMIC,
	                   CHIRP_LOG_MIN_SETPOINT, CHIRP_LOG_MAX_SETPOINT);

	SIGNAL_init(&square_sg, SIGNAL_SQUARE);
	SIGNAL_config_square(&square_sg, SQUARE_AMPLITUDE, SQUARE_FREQUENCY,
	                    SQUARE_DUTY_CYCLE, SQUARE_PHASE, SQUARE_OFFSET,
	                    SQUARE_MIN_SETPOINT, SQUARE_MAX_SETPOINT);

	SIGNAL_init(&ramp_sg, SIGNAL_RAMP);
	SIGNAL_config_ramp(&ramp_sg, RAMP_AMPLITUDE, RAMP_FREQUENCY,
	                  RAMP_START, RAMP_END, RAMP_PHASE, RAMP_OFFSET,
	                  RAMP_MIN_SETPOINT, RAMP_MAX_SETPOINT);

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

	ADC_DMA_Init(&adc_dma, &hadc1, adc_dma_buffer, ADC_BUFFER_SIZE, ADC_CHANNELS, 3.3f, 4095.0f);
	ADC_DMA_Start(&adc_dma);

	FIR_init(&LP_prismatic_current, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&LP_prismatic_velocity, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&LP_revolute_current, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&LP_revolute_velocity, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);

	Kalman_Start(&flit_prismatic_velocity);
	Kalman_Start(&flit_revolute_velocity);

	HAL_TIM_Base_Start_IT(CONTROL_TIM);
}

void update_sensors(void) {
    joystick_x = ADC_DMA_GetJoystickX(&adc_dma, JOYSTICK_X_CHANNEL);
    joystick_y = ADC_DMA_GetJoystickY(&adc_dma, JOYSTICK_Y_CHANNEL);
    prismatic_current = ADC_DMA_ComputeCurrent(&adc_dma, PRISMATIC_CURRENT_CHANNEL, PRISMATIC_CURRENT_OFFSET);
    revolute_current = ADC_DMA_ComputeCurrent(&adc_dma, REVOLUTE_CURRENT_CHANNEL, REVOLUTE_CURRENT_OFFSET);
}
