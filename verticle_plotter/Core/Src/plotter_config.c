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

PID_CONTROLLER revolute_joy_pid;

DC_MOTOR_FFeedward prismatic_motor_ffd;
DC_MOTOR_DFeedward prismatic_motor_dfd;

DC_MOTOR_FFeedward revolute_motor_ffd;
DC_MOTOR_DFeedward revolute_motor_dfd;

PWM servo;

ADC_DMA joystick;

SignalGenerator sine_sg_PWM;
SignalGenerator square_sg_PWM;
SignalGenerator sine_sg_cascade;
SignalGenerator square_sg_cascade;
SignalGenerator sine_sg_prismatic;
SignalGenerator square_sg_prismatic;
SignalGenerator sine_sg_revolute;
SignalGenerator square_sg_revolute;

UnitConverterSystem converter_system;

FIR prismatic_lp_accel;
FIR revolute_lp_accel;

FIR prismatic_lp;
FIR revolute_lp;

uint16_t joystick_buffer[ADC_BUFFER_SIZE];

ModbusHandleTypedef ModBus;
u16u8_t registerFrame[200];

MotorKalman prismatic_kalman;

KalmanFilter revolute_kalman;
float32_t revolute_A[16];
float32_t revolute_B[4];

float joystick_x = 0.0f;
float joystick_y = 0.0f;
float prismatic_current = 0.0f;
float revolute_current = 0.0f;

int up_lim, low_lim, b1, b2, b3, b4;

void plotter_begin() {
	ZGX45RGG_400RPM_Constant.sd_max = ZGX45RGG_400RPM_Constant.qd_max
			* Disturbance_Constant.prismatic_pulley_radius * 1000;
	ZGX45RGG_400RPM_Constant.sdd_max = ZGX45RGG_400RPM_Constant.sd_max * 2;

	ZGX45RGG_400RPM_Constant.traject_sd_max = 500.0;
	ZGX45RGG_400RPM_Constant.traject_sdd_max = 1000.0;

	ZGX45RGG_150RPM_Constant.qd_max = ZGX45RGG_150RPM_Constant.qd_max
			* (24.0 / 36.0);

	ZGX45RGG_150RPM_Constant.traject_qd_max = 4.0;
	ZGX45RGG_150RPM_Constant.traject_qdd_max = 2.5;

	SIGNAL_init(&sine_sg_PWM, SIGNAL_SINE);
	SIGNAL_config_sine(&sine_sg_PWM, SINE_AMPLITUDE, SINE_FREQUENCY, SINE_PHASE,
	SINE_OFFSET, SINE_MIN_SETPOINT, SINE_MAX_SETPOINT);

	SIGNAL_init(&square_sg_PWM, SIGNAL_SQUARE);
	SIGNAL_config_square(&square_sg_PWM, SQUARE_AMPLITUDE, SQUARE_FREQUENCY,
	SQUARE_DUTY_CYCLE, SQUARE_PHASE, SQUARE_OFFSET,
	SQUARE_MIN_SETPOINT, SQUARE_MAX_SETPOINT);

	SIGNAL_init(&sine_sg_cascade, SIGNAL_SINE);
	SIGNAL_config_sine(&sine_sg_cascade, SINE_AMPLITUDE_CASCADE, SINE_FREQUENCY,
	SINE_PHASE,
	SINE_OFFSET, SINE_MIN_SETPOINT_CASCADE, SINE_MAX_SETPOINT_CASCADE);

	SIGNAL_init(&square_sg_cascade, SIGNAL_SQUARE);
	SIGNAL_config_square(&square_sg_cascade, SQUARE_AMPLITUDE_CASCADE,
	SQUARE_FREQUENCY,
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

	UnitConverter_init(&converter_system);

	QEI_init(&prismatic_encoder, ENC_TIM1, ENC_PPR, ENC_FREQ, MOTOR1_RATIO,
			Disturbance_Constant.prismatic_pulley_radius * 2.0 * 1000.0);
	QEI_init(&revolute_encoder, ENC_TIM2, ENC_PPR, ENC_FREQ, MOTOR2_RATIO,
	MOTOR2_PULLEY_DIAMETER);

	MDXX_GPIO_init(&prismatic_motor, MOTOR1_TIM, MOTOR1_TIM_CH, MOTOR1_GPIOx,
	MOTOR1_GPIO_Pin);
	MDXX_GPIO_init(&revolute_motor, MOTOR2_TIM, MOTOR2_TIM_CH, MOTOR2_GPIOx,
	MOTOR2_GPIO_Pin);

	PWM_init(&servo, SERVO_TIM, SERVO_TIM_CH);

	MDXX_set_range(&prismatic_motor, 2000, 0);
	MDXX_set_range(&revolute_motor, 2000, 0);

//	PID_CONTROLLER_Init(&prismatic_position_pid, 75, 1e-10, 0.0001, //103 (D)
//			ZGX45RGG_400RPM_Constant.sd_max);
//	PID_CONTROLLER_Init(&prismatic_velocity_pid, 150, 1e-5, 100, //150 (P)
//			ZGX45RGG_400RPM_Constant.U_max);
//
//	PID_CONTROLLER_Init(&revolute_position_pid, 25, 0.001, 700,
//			ZGX45RGG_150RPM_Constant.qd_max);
//	PID_CONTROLLER_Init(&revolute_velocity_pid, 5000, 30, 2000, //2280->2275 (P), 25->30 (I), 3000 -> 2996 (D)
//			ZGX45RGG_150RPM_Constant.U_max);

	PID_CONTROLLER_Init(&prismatic_position_pid, 75, 1e-10, 80,
			ZGX45RGG_400RPM_Constant.sd_max);
	PID_CONTROLLER_Init(&prismatic_velocity_pid, 150, 1e-5, 0,
			ZGX45RGG_400RPM_Constant.U_max);

	PID_CONTROLLER_Init(&revolute_position_pid, 200, 0.001, 700,
			ZGX45RGG_150RPM_Constant.qd_max);
	PID_CONTROLLER_Init(&revolute_velocity_pid, 1400, 30, 1000,
			ZGX45RGG_150RPM_Constant.U_max);

//	PID_CONTROLLER_Init(&prismatic_position_pid, 75, 1e-10, 103, //103 (D)
//			ZGX45RGG_400RPM_Constant.sd_max);
//	PID_CONTROLLER_Init(&prismatic_velocity_pid, 150, 1e-5, 0, //150 (P)
//			ZGX45RGG_400RPM_Constant.U_max);
//
//	PID_CONTROLLER_Init(&revolute_position_pid, 250, 0.001, 700,
//			ZGX45RGG_150RPM_Constant.qd_max);
//	PID_CONTROLLER_Init(&revolute_velocity_pid, 2275, 30.5, 2996, //2280->2275 (P), 25->30 (I), 3000 -> 2996 (D)
//			ZGX45RGG_150RPM_Constant.U_max);

	PID_CONTROLLER_Init(&revolute_joy_pid, 7500, 100, 2000,
			ZGX45RGG_150RPM_Constant.U_max);

	REVOLUTE_MOTOR_FFD_Init(&revolute_motor_ffd, &ZGX45RGG_150RPM_Constant);
	PRISMATIC_MOTOR_FFD_Init(&prismatic_motor_ffd, &ZGX45RGG_400RPM_Constant);

	REVOLUTE_MOTOR_DFD_Init(&revolute_motor_dfd, &ZGX45RGG_150RPM_Constant,
			&Disturbance_Constant);
	PRISMATIC_MOTOR_DFD_Init(&prismatic_motor_dfd, &ZGX45RGG_400RPM_Constant,
			&Disturbance_Constant);

	ADC_DMA_Init(&joystick, &hadc1, joystick_buffer, ADC_BUFFER_SIZE,
	ADC_CHANNELS, ADC_VREF, ADC_RESOLUTION);
	ADC_DMA_SetCenterPoint(&joystick, ADC_CENTERPOINT, ADC_ERROR);
	ADC_DMA_Start(&joystick);

	FIR_init(&prismatic_lp_accel, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&revolute_lp_accel, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);

	FIR_init(&prismatic_lp, NUM_TAPS, 25, SAMPLE_RATE);
	FIR_init(&revolute_lp, NUM_TAPS, 25, SAMPLE_RATE);

	MotorKalman_Init(&prismatic_kalman, 1e-3, ZGX45RGG_400RPM_Constant.J,
			ZGX45RGG_400RPM_Constant.B, ZGX45RGG_400RPM_Constant.Kt,
			ZGX45RGG_400RPM_Constant.Ke, ZGX45RGG_400RPM_Constant.R,
			ZGX45RGG_400RPM_Constant.L, 1.0, 1.0);

	GenerateMotorMatrices(ZGX45RGG_150RPM_Constant.R,
			ZGX45RGG_150RPM_Constant.L, ZGX45RGG_150RPM_Constant.J,
			ZGX45RGG_150RPM_Constant.B * 2.2, ZGX45RGG_150RPM_Constant.Ke,
			ZGX45RGG_150RPM_Constant.Kt, 0.001, &revolute_A, &revolute_B);

	Kalman_Start(&revolute_kalman, revolute_A, revolute_B, REVOLUTE_Q,
	REVOLUTE_R);

	Modbus_init(&ModBus, MODBUS_USART, MODBUS_DATA_SENDING_PERIOD_TIM,
			registerFrame, MODBUS_SLAVE_ADDRESS, MODBUS_REGISTER_FRAME_SIZE);

	HAL_TIM_Base_Start_IT(CONTROL_TIM);

	plotter_reset();
}

void plotter_reset() {
	QEI_reset(&prismatic_encoder);
	QEI_reset(&prismatic_encoder);

	prismatic_encoder.diff_counts = 0;
	prismatic_encoder.rpm = 0;
	prismatic_encoder.pulses = 0;
	prismatic_encoder.revs = 0;
	prismatic_encoder.rads = 0;
	prismatic_encoder.mm = 0;

	revolute_encoder.diff_counts = 0;
	revolute_encoder.rpm = 0;
	revolute_encoder.pulses = 0;
	revolute_encoder.revs = 0;
	revolute_encoder.rads = 0;
	revolute_encoder.mm = 0;
}

void plotter_update_sensors() {
	joystick_x = ADC_DMA_GetJoystickValue(&joystick, JOYSTICK_X_CHANNEL, -50,
			50);
	joystick_y = ADC_DMA_GetJoystickValue(&joystick, JOYSTICK_Y_CHANNEL, -50,
			50);

	b1 = !HAL_GPIO_ReadPin(J1_GPIO_Port, J1_Pin);
	b2 = !HAL_GPIO_ReadPin(J2_GPIO_Port, J2_Pin);
	b3 = !HAL_GPIO_ReadPin(J3_GPIO_Port, J3_Pin);
	b4 = !HAL_GPIO_ReadPin(J4_GPIO_Port, J4_Pin);

	up_lim = HAL_GPIO_ReadPin(UPPER_LIM_GPIO_Port, UPPER_LIM_Pin);
	low_lim = HAL_GPIO_ReadPin(LOWER_LIM_GPIO_Port, LOWER_LIM_Pin);

	extern bool homing_active;
	if (!homing_active) {
		extern volatile bool up_photo, low_photo;
		up_photo = HAL_GPIO_ReadPin(upperphoto_GPIO_Port, upperphoto_Pin);
		low_photo = HAL_GPIO_ReadPin(LOWER_PHOTO_GPIO_Port, LOWER_PHOTO_Pin);
	}
}

void plotter_pen_up() {
	PWM_write_duty(&servo, 50, 7);
}

void plotter_pen_down() {
	PWM_write_duty(&servo, 50, 10);
}
