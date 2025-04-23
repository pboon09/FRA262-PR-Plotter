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

DC_MOTOR_FFeedward prismatic_motor_ffd;
DC_MOTOR_DFeedward prismatic_motor_dfd;

DC_MOTOR_FFeedward revolute_motor_ffd;
DC_MOTOR_DFeedward revolute_motor_dfd;

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
float32_t prismatic_A[16] = {
	    1.0f, 0.000991203202452616f, -7.58380272526136e-05f, 3.54373669483581e-05f,
	    0.0f, 0.982415142884365f,   -0.151228895527954f, 0.0705795807772549f,
	    0.0f, 0.0f, 1.0f, 0.0f,
	    0.0f, -0.00359965953788849f, 0.000275750228790085f, 0.992612408366667f
};

float32_t prismatic_B[4] = {
		7.49874875871698e-08f,
		0.000224494576940452f,
	    0.0f,
		0.00631167507255972f
};

KalmanFilter flit_revolute_velocity;
float32_t revolute_A[16] = {
	    1.0f, 0.000988562926927761f, -0.000649922017925031f, 0.000112742749409589f,
	    0.0f, 0.977162908316739f,   -1.29485853504780f, 0.224367910579471f,
	    0.0f, 0.0f, 1.0f, 0.0f,
	    0.0f, -0.00132500147466383f, 0.000872091882057674f, 0.993164026946268f
};

float32_t revolute_B[4] = {
		7.49874875871698e-08f,
		0.000224494576940452f,
	    0.0f,
		0.00631167507255972f
};

uint16_t adc_dma_buffer[ADC_BUFFER_SIZE];

ModbusHandleTypedef ModBus;
u16u8_t registerFrame[200];

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

	MDXX_set_range(&prismatic_motor, 1000, 0);
	MDXX_set_range(&revolute_motor, 1000, 0);
	PWM_write_range(&servo, 50, 0);

	PID_CONTROLLER_Init(&prismatic_position_pid, 0, 0, 0, 65535);
	PID_CONTROLLER_Init(&prismatic_velocity_pid, 0, 0, 0, 65535);

	PID_CONTROLLER_Init(&revolute_position_pid, 0, 0, 0, 65535);
	PID_CONTROLLER_Init(&revolute_velocity_pid, 0, 0, 0, 65535);

	REVOLUTE_MOTOR_FFD_Init(&revolute_motor_ffd, &ZGX45RGG_150RPM_Constant);
	PRISMATIC_MOTOR_FFD_Init(&prismatic_motor_ffd, &ZGX45RGG_400RPM_Constant);

	REVOLUTE_MOTOR_DFD_Init(&revolute_motor_dfd, &ZGX45RGG_150RPM_Constant, &Disturbance_Constant);
	PRISMATIC_MOTOR_DFD_Init(&prismatic_motor_dfd, &ZGX45RGG_400RPM_Constant, &Disturbance_Constant);

	ADC_DMA_Init(&adc_dma, &hadc1, adc_dma_buffer, ADC_BUFFER_SIZE, ADC_CHANNELS, 3.3f, 4095.0f);
	ADC_DMA_Start(&adc_dma);

	FIR_init(&LP_prismatic_current, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&LP_prismatic_velocity, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&LP_revolute_current, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&LP_revolute_velocity, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);

	Kalman_Start(&flit_revolute_velocity, revolute_A, revolute_B);
	Kalman_Start(&flit_prismatic_velocity, prismatic_A, prismatic_B);

	Modbus_init(&ModBus, MODBUS_USART, MODBUS_DATA_SENDING_PERIOD_TIM, registerFrame, MODBUS_SLAVE_ADDRESS, MODBUS_REGISTER_FRAME_SIZE);

	HAL_TIM_Base_Start_IT(CONTROL_TIM);
}

void update_sensors(void) {
    joystick_x = ADC_DMA_GetJoystick(&adc_dma, JOYSTICK_X_CHANNEL, 1.0);
    joystick_y = ADC_DMA_GetJoystick(&adc_dma, JOYSTICK_Y_CHANNEL, 1.0);
//    prismatic_current = ADC_DMA_ComputeCurrent(&adc_dma, PRISMATIC_CURRENT_CHANNEL, PRISMATIC_CURRENT_OFFSET);
//    revolute_current = ADC_DMA_ComputeCurrent(&adc_dma, REVOLUTE_CURRENT_CHANNEL, REVOLUTE_CURRENT_OFFSET);
}
