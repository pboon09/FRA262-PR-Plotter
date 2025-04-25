/*
 * plotter_config.c
 *
 *  Created on: Apr 20, 2025
 *      Author: pboon09
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
//b * 0.6
float32_t prismatic_A[16] = { 1.0f, 0.000993096502229541f,
		-0.000651907539312034f, 0.000113087373016393f, 0.0f, 0.986174211076634f,
		-1.30079678997714f, 0.225398024082257f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
		-0.00133108479516514f, 0.000874757627317859f, 0.993163564257609f };

float32_t prismatic_B[4] = { 2.53264868595647e-07f, 0.000758492055510868f, 0.0f,
		0.00668435039056396f };

KalmanFilter flit_revolute_velocity;
//float32_t revolute_A[16] = { 1.0f, 0.000988562926927761f,
//		-0.000649922017925031f, 0.000112742749409589f, 0.0f, 0.977162908316739f,
//		-1.29485853504780f, 0.224367910579471f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
//		-0.00132500147466383f, 0.000872091882057674f, 0.993164026946268f };
//float32_t revolute_B[4] = { 7.49874875871698e-08f, 0.000224494576940452f, 0.0f,
//		0.00631167507255972f };

//b * 1.4
//float32_t revolute_A[16] = {
//    1.0f, 0.000987730458216224f, -7.56607879232622e-05f, 3.53544968515789e-05f,
//    0.0f, 0.975518788206619f, -0.150699055355908f, 0.0703319998872943f,
//    0.0f, 0.0f, 1.0f, 0.0f,
//    0.0f, -0.00358703255849684f, 0.000275105388326063f, 0.992612709868349f
//};
//
//float32_t revolute_B[4] = {
//    7.48559161524874e-08f, 0.000223969597549501f, 0.0f, 0.00631167555142406f
//};

//b * 1.25
//float32_t revolute_A[16] = {
//    1.0f, 0.000989030829926645f, -7.57271796839252e-05f, 3.53855389991954e-05f,
//    0.0f, 0.978099229840933f, -0.150897454409761f, 0.0704247066562337f,
//    0.0f, 0.0f, 1.0f, 0.0f,
//    0.0f, -0.00359176073626956f, 0.000275346937855402f, 0.992612596929469f
//};
//
//float32_t revolute_B[4] = {
//    7.49052120833864e-08f, 0.000224166248553698f, 0.0f, 0.00631167537200777f
//};

//b * 1.3
float32_t revolute_A[16] = {
    1.0f, 0.000988597118708185f, -7.57050393755908e-05f, 3.53751870696359e-05f,
    0.0f, 0.977238325116089f, -0.150831282641567f, 0.0703937862818857f,
    0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, -0.00359018375296631f, 0.000275266385963705f, 0.992612634592311f
};

float32_t revolute_B[4] = {
    7.48887743305548e-08f, 0.000224100669413736f, 0.0f, 0.00631167543183423f
};

//b*1.275
//float32_t revolute_A[16] = {
//    1.0f, 0.000988813942552254f, -7.57161083140429e-05f, 3.53803624657881e-05f,
//    0.0f, 0.977668682719841f, -0.150864363679221f, 0.0704092442030598f,
//    0.0f, 0.0f, 1.0f, 0.0f,
//    0.0f, -0.00359097212904870f, 0.000275306657484869f, 0.992612615762959f
//};
//
//float32_t revolute_B[4] = {
//    7.48969924847229e-08f, 0.000224133455381480f, 0.0f, 0.00631167540192363f
//};
//
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
	SIGNAL_init(&sine_sg, SIGNAL_SINE);
	SIGNAL_config_sine(&sine_sg, SINE_AMPLITUDE, SINE_FREQUENCY, SINE_PHASE,
	SINE_OFFSET, SINE_MIN_SETPOINT, SINE_MAX_SETPOINT);

	SIGNAL_init(&chirp_linear_sg, SIGNAL_CHIRP);
	SIGNAL_config_chirp(&chirp_linear_sg, CHIRP_LINEAR_AMPLITUDE,
	CHIRP_LINEAR_START_FREQ,
	CHIRP_LINEAR_END_FREQ, CHIRP_LINEAR_DURATION, CHIRP_LINEAR,
	CHIRP_LINEAR_MIN_SETPOINT, CHIRP_LINEAR_MAX_SETPOINT);

	SIGNAL_init(&chirp_log_sg, SIGNAL_CHIRP);
	SIGNAL_config_chirp(&chirp_log_sg, CHIRP_LOG_AMPLITUDE,
	CHIRP_LOG_START_FREQ,
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

	MDXX_GPIO_init(&prismatic_motor, MOTOR1_TIM, MOTOR1_TIM_CH, MOTOR1_GPIOx,
	MOTOR1_GPIO_Pin);
	MDXX_GPIO_init(&revolute_motor, MOTOR2_TIM, MOTOR2_TIM_CH, MOTOR2_GPIOx,
	MOTOR2_GPIO_Pin);

	PWM_init(&servo, SERVO_TIM, SERVO_TIM_CH);

	MDXX_set_range(&prismatic_motor, 2000, 0);
	MDXX_set_range(&revolute_motor, 2000, 0);
	pen_up();

//  Low Pass PID Control
	PID_CONTROLLER_Init(&prismatic_position_pid, 2, 1e-7, 1, ZGX45RGG_400RPM_Constant.U_max);
	PID_CONTROLLER_Init(&prismatic_velocity_pid, 500, 25, 0, ZGX45RGG_400RPM_Constant.U_max);

	PID_CONTROLLER_Init(&revolute_position_pid, 2, 1e-7, 1, ZGX45RGG_150RPM_Constant.U_max);
	PID_CONTROLLER_Init(&revolute_velocity_pid, 1000, 50, 0, ZGX45RGG_150RPM_Constant.U_max);

	REVOLUTE_MOTOR_FFD_Init(&revolute_motor_ffd, &ZGX45RGG_150RPM_Constant);
	PRISMATIC_MOTOR_FFD_Init(&prismatic_motor_ffd, &ZGX45RGG_400RPM_Constant);

	REVOLUTE_MOTOR_DFD_Init(&revolute_motor_dfd, &ZGX45RGG_150RPM_Constant,
			&Disturbance_Constant);
	PRISMATIC_MOTOR_DFD_Init(&prismatic_motor_dfd, &ZGX45RGG_400RPM_Constant,
			&Disturbance_Constant);

	ADC_DMA_Init(&adc_dma, &hadc1, adc_dma_buffer, ADC_BUFFER_SIZE,
			ADC_CHANNELS, 3.3f, 4095.0f);
	ADC_DMA_Start(&adc_dma);

	FIR_init(&LP_prismatic_current, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&LP_prismatic_velocity, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&LP_revolute_current, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&LP_revolute_velocity, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);

	Kalman_Start(&flit_revolute_velocity, revolute_A, revolute_B);
	Kalman_Start(&flit_prismatic_velocity, prismatic_A, prismatic_B);

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
