/*
 * plotter_config.c
 *
 *  Created on: Apr 20, 2025
 *      Author: pboon09
 */

#include <plotter_config.h>

RobotState current_state = IDLE;
RobotState previous_state = IDLE;

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

SignalGenerator sine_sg_PWM;
SignalGenerator square_sg_PWM;
SignalGenerator sine_sg_cascade;
SignalGenerator square_sg_cascade;
SignalGenerator sine_sg_prismatic;
SignalGenerator square_sg_prismatic;
SignalGenerator sine_sg_revolute;
SignalGenerator square_sg_revolute;

FIR prismatic_lp_velocity;
FIR prismatic_lp_current;
FIR revolute_lp_velocity;
FIR revolute_lp_current;

KalmanFilter prismatic_kalman;
//b * 0.6
float32_t prismatic_A[16] = { 1.0f, 0.000993096502229541f,
		-0.000651907539312034f, 0.000113087373016393f, 0.0f, 0.986174211076634f,
		-1.30079678997714f, 0.225398024082257f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
		-0.00133108479516514f, 0.000874757627317859f, 0.993163564257609f };

float32_t prismatic_B[4] = { 2.53264868595647e-07f, 0.000758492055510868f, 0.0f,
		0.00668435039056396f };

KalmanFilter revolute_kalman;
//b * 0.735
float32_t revolute_A[16] = { 1.0f, 0.000982843274975228f,
		-0.000357087554676877f, 7.93789751408048e-05f, 0.0f, 0.965802607949948f,
		-0.710057452064783f, 0.157648968323378f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
		-0.00301284940198517f, 0.00109597588320089f, 0.992497629717641f };

float32_t revolute_B[4] = { 1.68207942599771e-07f, 0.000502863247943066f, 0.0f,
		0.00631143131595644f };

uint16_t adc_dma_buffer[ADC_BUFFER_SIZE];

ModbusHandleTypedef ModBus;
u16u8_t registerFrame[200];

float joystick_x = 0.0f;
float joystick_y = 0.0f;
float prismatic_current = 0.0f;
float revolute_current = 0.0f;

int b1, b2, b3, b4, prox, emer, photo_pris, photo_revo, up_lim, low_lim;
float joy_x, joy_y;

void plotter_begin() {
	SIGNAL_init(&sine_sg_PWM, SIGNAL_SINE);
	SIGNAL_config_sine(&sine_sg_PWM, SINE_AMPLITUDE, SINE_FREQUENCY, SINE_PHASE,
	SINE_OFFSET, SINE_MIN_SETPOINT, SINE_MAX_SETPOINT);

	SIGNAL_init(&square_sg_PWM, SIGNAL_SQUARE);
	SIGNAL_config_square(&square_sg_PWM, SQUARE_AMPLITUDE, SQUARE_FREQUENCY,
	SQUARE_DUTY_CYCLE, SQUARE_PHASE, SQUARE_OFFSET,
	SQUARE_MIN_SETPOINT, SQUARE_MAX_SETPOINT);

	SIGNAL_init(&sine_sg_cascade, SIGNAL_SINE);
	SIGNAL_config_sine(&sine_sg_cascade, SINE_AMPLITUDE_CASCADE, SINE_FREQUENCY, SINE_PHASE,
	SINE_OFFSET, SINE_MIN_SETPOINT_CASCADE, SINE_MAX_SETPOINT_CASCADE);

	SIGNAL_init(&square_sg_cascade, SIGNAL_SQUARE);
	SIGNAL_config_square(&square_sg_cascade, SQUARE_AMPLITUDE_CASCADE, SQUARE_FREQUENCY,
	SQUARE_DUTY_CYCLE, SQUARE_PHASE, SQUARE_OFFSET,
	SQUARE_MIN_SETPOINT_CASCADE, SQUARE_MAX_SETPOINT_CASCADE);

	SIGNAL_init(&sine_sg_prismatic, SIGNAL_SINE);
	SIGNAL_config_sine(&sine_sg_prismatic, ZGX45RGG_400RPM_Constant.qd_max,
	SINE_FREQUENCY, SINE_PHASE,
	SINE_OFFSET, -ZGX45RGG_400RPM_Constant.qd_max,
			ZGX45RGG_400RPM_Constant.qd_max);

	SIGNAL_init(&square_sg_prismatic, SIGNAL_SQUARE);
	SIGNAL_config_square(&square_sg_prismatic, ZGX45RGG_400RPM_Constant.qd_max,
	SQUARE_FREQUENCY,
	SQUARE_DUTY_CYCLE, SQUARE_PHASE, SQUARE_OFFSET,
			-ZGX45RGG_400RPM_Constant.qd_max, ZGX45RGG_400RPM_Constant.qd_max);

	SIGNAL_init(&sine_sg_revolute, SIGNAL_SINE);
	SIGNAL_config_sine(&sine_sg_revolute, ZGX45RGG_150RPM_Constant.qd_max,
	SINE_FREQUENCY, SINE_PHASE,
	SINE_OFFSET, -ZGX45RGG_150RPM_Constant.qd_max,
			ZGX45RGG_150RPM_Constant.qd_max);

	SIGNAL_init(&square_sg_revolute, SIGNAL_SQUARE);
	SIGNAL_config_square(&square_sg_revolute, ZGX45RGG_150RPM_Constant.qd_max,
	SQUARE_FREQUENCY,
	SQUARE_DUTY_CYCLE, SQUARE_PHASE, SQUARE_OFFSET,
			-ZGX45RGG_150RPM_Constant.qd_max, ZGX45RGG_150RPM_Constant.qd_max);

	QEI_init(&prismatic_encoder, ENC_TIM1, ENC_PPR, ENC_FREQ, MOTOR_RATIO1);
	QEI_init(&revolute_encoder, ENC_TIM2, ENC_PPR, ENC_FREQ, MOTOR_RATIO2);

	MDXX_GPIO_init(&prismatic_motor, MOTOR1_TIM, MOTOR1_TIM_CH, MOTOR1_GPIOx,
	MOTOR1_GPIO_Pin);
	MDXX_GPIO_init(&revolute_motor, MOTOR2_TIM, MOTOR2_TIM_CH, MOTOR2_GPIOx,
	MOTOR2_GPIO_Pin);

	PWM_init(&servo, SERVO_TIM, SERVO_TIM_CH);

	MDXX_set_range(&prismatic_motor, 2000, 0);
	MDXX_set_range(&revolute_motor, 2000, 0);
	pen_up();

	PID_CONTROLLER_Init(&prismatic_position_pid, 500, 5, 70,
			ZGX45RGG_400RPM_Constant.qd_max);
	PID_CONTROLLER_Init(&prismatic_velocity_pid, 500, 7, 0,
			ZGX45RGG_400RPM_Constant.U_max);

	//unit test
	//25 1e-8 0
	//2000 80 0
	PID_CONTROLLER_Init(&revolute_position_pid, 500, 1e-9, 72,
			ZGX45RGG_150RPM_Constant.qd_max);
	PID_CONTROLLER_Init(&revolute_velocity_pid, 1000, 20, 0,
			ZGX45RGG_150RPM_Constant.U_max);

	REVOLUTE_MOTOR_FFD_Init(&revolute_motor_ffd, &ZGX45RGG_150RPM_Constant);
	PRISMATIC_MOTOR_FFD_Init(&prismatic_motor_ffd, &ZGX45RGG_400RPM_Constant);

	REVOLUTE_MOTOR_DFD_Init(&revolute_motor_dfd, &ZGX45RGG_150RPM_Constant,
			&Disturbance_Constant);
	PRISMATIC_MOTOR_DFD_Init(&prismatic_motor_dfd, &ZGX45RGG_400RPM_Constant,
			&Disturbance_Constant);

	ADC_DMA_Init(&adc_dma, &hadc1, adc_dma_buffer, ADC_BUFFER_SIZE,
	ADC_CHANNELS, 3.3f, 4095.0f);
	ADC_DMA_Start(&adc_dma);

	FIR_init(&prismatic_lp_current, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&prismatic_lp_velocity, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&revolute_lp_current, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&revolute_lp_velocity, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);

	Kalman_Start(&revolute_kalman, revolute_A, revolute_B, REVOLUTE_Q,
	REVOLUTE_R);
	Kalman_Start(&prismatic_kalman, prismatic_A, prismatic_B, PRISMATIC_Q,
	PRISMATIC_R);

	Modbus_init(&ModBus, MODBUS_USART, MODBUS_DATA_SENDING_PERIOD_TIM,
			registerFrame, MODBUS_SLAVE_ADDRESS, MODBUS_REGISTER_FRAME_SIZE);

	HAL_TIM_Base_Start_IT(CONTROL_TIM);
}

void update_sensors() {
	joystick_x = ADC_DMA_GetJoystick(&adc_dma, JOYSTICK_X_CHANNEL, 1.0);
	joystick_y = ADC_DMA_GetJoystick(&adc_dma, JOYSTICK_Y_CHANNEL, 1.0);

	b1 = !HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin);
	b2 = !HAL_GPIO_ReadPin(SAVE_GPIO_Port, SAVE_Pin);
	b3 = !HAL_GPIO_ReadPin(DELETE_GPIO_Port, DELETE_Pin);
	b4 = !HAL_GPIO_ReadPin(RESET_SYS_GPIO_Port, RESET_SYS_Pin);

	prox = HAL_GPIO_ReadPin(PROX_GPIO_Port, PROX_Pin);
	photo_pris = HAL_GPIO_ReadPin(PHOTO_PRIS_GPIO_Port, PHOTO_PRIS_Pin);
	photo_revo = HAL_GPIO_ReadPin(PHOTO_REVO_GPIO_Port, PHOTO_REVO_Pin);
	up_lim = HAL_GPIO_ReadPin(UPPER_LIM_GPIO_Port, UPPER_LIM_Pin);
	low_lim = HAL_GPIO_ReadPin(LOWER_LIM_GPIO_Port, LOWER_LIM_Pin);

	emer = HAL_GPIO_ReadPin(EMER_GPIO_Port, EMER_Pin);
//    prismatic_current = ADC_DMA_ComputeCurrent(&adc_dma, PRISMATIC_CURRENT_CHANNEL, PRISMATIC_CURRENT_OFFSET);
//    revolute_current = ADC_DMA_ComputeCurrent(&adc_dma, REVOLUTE_CURRENT_CHANNEL, REVOLUTE_CURRENT_OFFSET);
}

void test_sensors_motor_servo(float duty_pris, float duty_revo,
		float duty_servo) {
	update_sensors();

	QEI_get_diff_count(&revolute_encoder);
	QEI_compute_data(&revolute_encoder);

	QEI_get_diff_count(&prismatic_encoder);
	QEI_compute_data(&prismatic_encoder);

	MDXX_set_range(&prismatic_motor, 2000, duty_pris);
	MDXX_set_range(&revolute_motor, 2000, duty_revo);
	PWM_write_duty(&servo, 50, duty_servo);

	b1 = !HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin);
	b2 = !HAL_GPIO_ReadPin(SAVE_GPIO_Port, SAVE_Pin);
	b3 = !HAL_GPIO_ReadPin(DELETE_GPIO_Port, DELETE_Pin);
	b4 = !HAL_GPIO_ReadPin(RESET_SYS_GPIO_Port, RESET_SYS_Pin);

	prox = HAL_GPIO_ReadPin(PROX_GPIO_Port, PROX_Pin);
	photo_pris = HAL_GPIO_ReadPin(PHOTO_PRIS_GPIO_Port, PHOTO_PRIS_Pin);
	photo_revo = HAL_GPIO_ReadPin(PHOTO_REVO_GPIO_Port, PHOTO_REVO_Pin);
	up_lim = HAL_GPIO_ReadPin(UPPER_LIM_GPIO_Port, UPPER_LIM_Pin);
	low_lim = HAL_GPIO_ReadPin(LOWER_LIM_GPIO_Port, LOWER_LIM_Pin);

	emer = HAL_GPIO_ReadPin(EMER_GPIO_Port, EMER_Pin);

	joy_x = joystick_x;
	joy_y = joystick_y;
}

void pen_up() {
	PWM_write_duty(&servo, 50, 7);
}

void pen_down() {
	PWM_write_duty(&servo, 50, 12);
}

void KalmanTuning(){

}
