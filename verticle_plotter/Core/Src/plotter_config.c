/*
 * plotter_config.c
 *
 *  Created on: Apr 20, 2025
 *      Author: pboon09
 */

#include <plotter_config.h>

RobotState rs_current_state = RS_RETURN_TO_HOME;
RobotState rs_previous_state = RS_RETURN_TO_HOME;
SetPointState setpoint_state = POINT_IDLE;
MovingThroghPointState moving_state = MOVING_IDLE;
WriteLetterState writing_state = WRITE_IDLE;
JoyStickState joy_state = JOY_IDLE;
PrismaticPosition prismatic_state = PP_UNKNOWN;
RevolutePosition revolute_state = RP_UNKNOWN;
ServoState servo_state = PEN_IDLE;
EmergencyState emer_state = DEFAULT;

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

FIR prismatic_lp_velocity;
FIR prismatic_lp_current;
FIR revolute_lp_velocity;
FIR revolute_lp_current;

MotorKalman prismatic_kalman;
MotorKalman revolute_kalman;

uint16_t joystick_buffer[ADC_BUFFER_SIZE];

ModbusHandleTypedef ModBus;
u16u8_t registerFrame[200];

float joystick_x = 0.0f;
float joystick_y = 0.0f;
float prismatic_current = 0.0f;
float revolute_current = 0.0f;

int prox, emer, up_photo, low_photo, up_lim, low_lim, b1, b2, b3, b4;

void plotter_begin() {
	ZGX45RGG_400RPM_Constant.sd_max = ZGX45RGG_400RPM_Constant.qd_max
			* Disturbance_Constant.prismatic_pulley_radius * 1000;
	ZGX45RGG_400RPM_Constant.sdd_max = ZGX45RGG_400RPM_Constant.sd_max * 0.5;

	ZGX45RGG_150RPM_Constant.qd_max = ZGX45RGG_150RPM_Constant.qd_max * (24.0 /36.0) * 0.3;
	ZGX45RGG_150RPM_Constant.qdd_max = ZGX45RGG_150RPM_Constant.qd_max * 0.4;

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
	plotter_pen_up();

	PID_CONTROLLER_Init(&prismatic_position_pid, 75, 1e-10, 75,
			ZGX45RGG_400RPM_Constant.sd_max);
	PID_CONTROLLER_Init(&prismatic_velocity_pid, 150, 1e-5, 0,
			ZGX45RGG_400RPM_Constant.U_max);

	PID_CONTROLLER_Init(&revolute_position_pid, 25, 1e-10, 10, // 100 1000
			ZGX45RGG_150RPM_Constant.qd_max);
	PID_CONTROLLER_Init(&revolute_velocity_pid, 3000, 200, 0,
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

	FIR_init(&prismatic_lp_current, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&prismatic_lp_velocity, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&revolute_lp_current, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);
	FIR_init(&revolute_lp_velocity, NUM_TAPS, CUTOFF_FREQ, SAMPLE_RATE);

	MotorKalman_Init(&prismatic_kalman, 1e-3, ZGX45RGG_400RPM_Constant.J,
			ZGX45RGG_400RPM_Constant.B, ZGX45RGG_400RPM_Constant.Kt,
			ZGX45RGG_400RPM_Constant.Ke, ZGX45RGG_400RPM_Constant.R,
			ZGX45RGG_400RPM_Constant.L, 1.0, 1.0);

	MotorKalman_Init(&revolute_kalman, 1e-3, ZGX45RGG_150RPM_Constant.J,
			ZGX45RGG_150RPM_Constant.B, ZGX45RGG_150RPM_Constant.Kt,
			ZGX45RGG_150RPM_Constant.Ke, ZGX45RGG_150RPM_Constant.R,
			ZGX45RGG_150RPM_Constant.L, 1.0, 1.0);

	Modbus_init(&ModBus, MODBUS_USART, MODBUS_DATA_SENDING_PERIOD_TIM,
			registerFrame, MODBUS_SLAVE_ADDRESS, MODBUS_REGISTER_FRAME_SIZE);

	HAL_TIM_Base_Start_IT(CONTROL_TIM);
}

void plotter_reset() {
	QEI_reset(&prismatic_encoder);
	QEI_reset(&revolute_encoder);
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

	prox = HAL_GPIO_ReadPin(PROX_GPIO_Port, PROX_Pin);
	up_photo = HAL_GPIO_ReadPin(UPPER_PHOTO_GPIO_Port, UPPER_PHOTO_Pin);
	low_photo = HAL_GPIO_ReadPin(LOWER_PHOTO_GPIO_Port, LOWER_PHOTO_Pin);
	up_lim = HAL_GPIO_ReadPin(UPPER_LIM_GPIO_Port, UPPER_LIM_Pin);
	low_lim = HAL_GPIO_ReadPin(LOWER_LIM_GPIO_Port, LOWER_LIM_Pin);

	emer = !HAL_GPIO_ReadPin(EMER_GPIO_Port, EMER_Pin);

	if (up_lim) {
		servo_state = PEN_UP;
	} else if (low_lim) {
		servo_state = PEN_DOWN;
	} else {
		servo_state = PEN_IDLE;
	}

	if (up_photo) {
		prismatic_state = PP_AT_TOP_END_POSITION;
	} else if (low_photo) {
		prismatic_state = PP_AT_BOTTOM_END_POSITION;
	}

	if (prox) {
		revolute_state = RP_AT_HOME_POSITION;
	}

//    prismatic_current = ADC_DMA_ComputeCurrent(&adc_dma, PRISMATIC_CURRENT_CHANNEL, PRISMATIC_CURRENT_OFFSET);
//    revolute_current = ADC_DMA_ComputeCurrent(&adc_dma, REVOLUTE_CURRENT_CHANNEL, REVOLUTE_CURRENT_OFFSET);
}

void plotter_pen_up() {
	PWM_write_duty(&servo, 50, 7);
}

void plotter_pen_down() {
	PWM_write_duty(&servo, 50, 12);
}

void test_sensors_motor_servo(float duty_pris, float duty_revo,
		float duty_servo) {
	plotter_update_sensors();

	QEI_get_diff_count(&revolute_encoder);
	QEI_compute_data(&revolute_encoder);

	QEI_get_diff_count(&prismatic_encoder);
	QEI_compute_data(&prismatic_encoder);

	MDXX_set_range(&prismatic_motor, 2000, duty_pris);
	MDXX_set_range(&revolute_motor, 2000, duty_revo);
	PWM_write_duty(&servo, 50, duty_servo);
}

//bool check_prismatic_limit() {
//	// If this is a valid operation that's supposed to reach the limit,
//	// then don't report it as a limit violation
//	if (prismatic_state == PP_GOING_TOP_END
//			|| prismatic_state == PP_AT_TOP_END_POSITION
//			|| prismatic_state == PP_AT_BOTTOM_END_POSITION) {
//		return false;
//	}
//
//	// Otherwise, if either condition indicates we're at a limit, report it
//	return (prismatic_encoder.mm >= PRISMATIC_MAX_POS || up_photo
//			|| prismatic_encoder.mm <= PRISMATIC_MIN_POS || low_photo);
//}
//
//bool check_revolute_limit() {
//	// For homing operations, don't count limits as violations
//	if (revolute_state == RP_GOING_HOME) {
//		return false;
//	}
//
//	// For normal operation, check limits
//	return (revolute_encoder.rads >= REVOLUTE_MAX_POS
//			|| revolute_encoder.rads <= REVOLUTE_MIN_POS);
//}
//
//bool is_valid_target(float32_t pris_tgt, float32_t rev_tgt) {
//	// Check if targets are within physical limits
//	if (pris_tgt > PRISMATIC_MAX_POS || pris_tgt < PRISMATIC_MIN_POS) {
//		return false;
//	}
//
//	if (rev_tgt > REVOLUTE_MAX_POS || rev_tgt < REVOLUTE_MIN_POS) {
//		return false;
//	}
//
//	return true;
//}
//
//void plotter_move() {
//	pris_pos[0] = prismatic_encoder.mm;
//
//	pris_pos_error = prismatic_pos - prismatic_encoder.mm;
//
//	pris_cmd_vx = saturation(
//			PID_CONTROLLER_Compute(&prismatic_position_pid, pris_pos_error),
//			ZGX45RGG_400RPM_Constant.sd_max, -ZGX45RGG_400RPM_Constant.sd_max);
//
//	pris_vel_error = pris_cmd_vx + prismatic_vel - pris_kal_filt;
//
//	pris_cmd_ux = PWM_Satuation(
//			PID_CONTROLLER_Compute(&prismatic_velocity_pid, pris_vel_error),
//			ZGX45RGG_400RPM_Constant.U_max, -ZGX45RGG_400RPM_Constant.U_max);
//
//	if (check_prismatic_limit()) {
//		MDXX_set_range(&prismatic_motor, 2000, 0);
//		pristrajectoryActive = false;
//		return;
//	}
//
//	MDXX_set_range(&prismatic_motor, 2000, pris_cmd_ux);
//
//	if (pris_pos[0] - pris_pos[1] > 0) {
//		prismatic_state = PP_GO_UP;
//	} else {
//		prismatic_state = PP_GO_DOWN;
//	}
//
///////////////////////////////////////////////////////
//
//	rev_pos[0] = revolute_encoder.rads;
//
//	rev_pos_error = revolute_pos - revolute_encoder.rads;
//
//	rev_cmd_vx = saturation(
//			PID_CONTROLLER_Compute(&revolute_position_pid, rev_pos_error),
//			ZGX45RGG_150RPM_Constant.qd_max, -ZGX45RGG_150RPM_Constant.qd_max);
//
//	rev_vel_error = rev_cmd_vx + revolute_vel - rev_kal_filt;
//
//	rev_cmd_ux = PWM_Satuation(
//			PID_CONTROLLER_Compute(&revolute_velocity_pid, rev_vel_error),
//			ZGX45RGG_150RPM_Constant.U_max, -ZGX45RGG_150RPM_Constant.U_max);
//
//	if (check_revolute_limit()) {
//		MDXX_set_range(&revolute_motor, 2000, 0);
//		revtrajectoryActive = false;
//		return;
//	}
//
//	MDXX_set_range(&revolute_motor, 2000, rev_cmd_ux);
//
//	if (rev_pos[0] - rev_pos[1] > 0) {
//		revolute_state = RP_GO_CLOCKWISE;
//	} else {
//		revolute_state = RP_GO_COUNTER_CLOCKWISE;
//	}
//
//	pris_pos[1] = pris_pos[0];
//	rev_pos[1] = rev_pos[0];
//}
//
//void plotter_joymove() {
//	static float32_t pris_joy, rev_joy;
//
//// Prismatic limits check
//	if (check_prismatic_limit()) {
//		pris_joy = 0.0f;
//	} else {
//		if (joystick_y > 0.7) {
//			pris_joy = (joystick_y - 0.7) * 25000.0f;
//			prismatic_state = PP_GO_UP;
//		} else if (joystick_y < -0.7) {
//			pris_joy = (joystick_y + 0.7) * 25000.0f;
//			prismatic_state = PP_GO_DOWN;
//		} else {
//			pris_joy = 0.0;
//			prismatic_state = PP_UNKNOWN;
//		}
//	}
//
//// Revolute limits check
//	if (check_revolute_limit()) {
//		rev_joy = 0.0f;
//	} else {
//		if (joystick_x > 0.7) {
//			rev_joy = (joystick_x - 0.7) * 25000.0f;
//			revolute_state = RP_GO_COUNTER_CLOCKWISE;
//		} else if (joystick_x < -0.7) {
//			rev_joy = (joystick_x + 0.7) * 25000.0f;
//			revolute_state = RP_GO_CLOCKWISE;
//		} else {
//			rev_joy = 0.0;
//			revolute_state = RP_UNKNOWN;
//		}
//	}
//
//	MDXX_set_range(&prismatic_motor, 2000, pris_joy);
//	MDXX_set_range(&revolute_motor, 2000, rev_joy);
//}
//
//void plotter_handle_state_transition() {
//// Save previous state
//
//// Only process base system commands if not in emergency mode
//	if (rs_current_state != RS_EMERGENCY_TRIGGED) {
//		// Process base system commands
//		if (registerFrame[BaseSystem_Status].U16 == 1) {
//			// Set Home Command
//			rs_current_state = RS_RETURN_TO_HOME;
//			registerFrame[R_Theta_Status].U16 = 1;
//		} else if (registerFrame[BaseSystem_Status].U16 == 2) {
//			// Joystick Mode
//			rs_current_state = RS_JOG_MODE;
//			registerFrame[R_Theta_Status].U16 = 2;
//		} else if (registerFrame[BaseSystem_Status].U16 == 4) {
//			// Point Mode (Just Receive Input)
//			rs_current_state = RS_POINT_MODE;
//			registerFrame[R_Theta_Status].U16 = 4;
//		} else if (registerFrame[BaseSystem_Status].U16 == 8) {
//			// Go to Point Mode
//			rs_current_state = RS_MOVING;
//			registerFrame[R_Theta_Status].U16 = 8;
//		}
//	}
//
//// Handle state entry/exit actions if state has changed
//	if (rs_previous_state != rs_current_state) {
//		// Exit actions for previous state
//		switch (rs_previous_state) {
//		case RS_JOG_MODE:
//			// Clean up any joystick control resources
//			joy_state = JOY_IDLE;
//			HAL_GPIO_WritePin(PILOT_GPIO_Port, PILOT_Pin, 0);
//			MDXX_set_range(&prismatic_motor, 2000, 0);
//			MDXX_set_range(&revolute_motor, 2000, 0);
//			break;
//
//		case RS_MOVING:
//			// Stop trajectory generators
//			pristrajectoryActive = false;
//			revtrajectoryActive = false;
//			break;
//
//		case RS_EMERGENCY_TRIGGED:
//			// Exit emergency state
//			emer_state = DEFAULT;
//			break;
//
//		default:
//			break;
//		}
//
//		// Entry actions for new state
//		switch (rs_current_state) {
//		case RS_JOG_MODE:
//			// Initialize joystick control
//			joy_state = A1B1_MODE;
//			HAL_GPIO_WritePin(PILOT_GPIO_Port, PILOT_Pin, 1);
//			break;
//
//		case RS_MOVING:
//			// Initialize trajectory generator
//			pris_initial_p = prismatic_encoder.mm;
//			rev_initial_p = revolute_encoder.rads;
//			pris_target_p = registerFrame[Goal_R].U16;
//			rev_target_p = registerFrame[Goal_Theta].U16;
//			pristrajectoryActive = true;
//			revtrajectoryActive = true;
//			moving_state = MOVING_GO_TO_POINT;
//			break;
//
//		case RS_EMERGENCY_TRIGGED:
//			// Enter emergency state
//			joy_state = EMERGENCY_MODE;
////			registerFrame[R_Theta_Status].U16 = 16;
//			// Immediately stop all motors
//			MDXX_set_range(&prismatic_motor, 2000, 0);
//			MDXX_set_range(&revolute_motor, 2000, 0);
//			break;
//
//		case RS_RETURN_TO_HOME:
//			// Initialize homing sequence
//			plotter_pen_up(); // First ensure pen is up
//			break;
//
//		case RS_POINT_MODE:
//			MDXX_set_range(&prismatic_motor, 2000, 0);
//			MDXX_set_range(&revolute_motor, 2000, 0);
//			break;
//
//		default:
//			break;
//		}
//	}
//	rs_previous_state = rs_current_state;
//}
//
//void plotter_process_jog_mode() {
//// block any joystick actions while in emergency
//	if (rs_current_state == RS_EMERGENCY_TRIGGED
//			|| joy_state == EMERGENCY_MODE) {
//		return;
//	}
//
//// Handle state transitions for A1B1_MODE
//	if (joy_state == A1B1_MODE) {
//		if (b1[0] == 0 && b1[1] == 1) {
//			joy_state = A1B1_SETPOINT;
//		} else if (b2[0] == 0 && b2[1] == 1) {
//			joy_state = A1B1_MOVING;
//			moving_state = MOVING_GO_TO_POINT;
//			if (total_setpoints > 0) {
//				joy_state = A1B1_MOVING;
//				moving_state = MOVING_GO_TO_POINT;
//				move_index = 0;            // start at first point
//			}
//
//		}
//	}
//// Handle transitions back to A1B1_MODE
//	else if ((joy_state == A1B1_SETPOINT && b2[0] == 0 && b2[1] == 1)
//			|| (joy_state == A1B1_MOVING && b1[0] == 0 && b1[1] == 1)) {
//		joy_state = A1B1_MODE;
//	}
//
//// Handle state transitions for A2B2_MODE
//	if (joy_state == A2B2_MODE) {
//		if (b1[0] == 0 && b1[1] == 1) {
//			joy_state = A2B2_WRITING;
//			writing_state = WRITE_IDLE;
//		} else if (b2[0] == 0 && b2[1] == 1) {
//			joy_state = A2B2_GOTO_HOME;
//		}
//	}
//// Handle transitions back to A2B2_MODE
//	else if ((joy_state == A2B2_WRITING && b2[0] == 0 && b2[1] == 1)
//			|| (joy_state == A2B2_GOTO_HOME && b1[0] == 0 && b1[1] == 1)) {
//		joy_state = A2B2_MODE;
//	}
//	if (b4[0] == 0 && b4[1] == 1) {
//		if (joy_state == A1B1_MODE) {
//			joy_state = A2B2_MODE;
//		} else if (joy_state == A2B2_MODE) {
//			joy_state = A1B1_MODE;
//		}
//	}
//
//// Execute state-specific actions
//	switch (joy_state) {
//	case A1B1_MOVING:
//		if (total_setpoints == 0) {
//			joy_state = A1B1_MODE;
//		}
//		static uint8_t current_point = 0;
//
//		// Initialize target for first point if needed
//		if (current_point == 0) {
//			// Get register for first point
//			uint16_t r_reg = getPointRegisterR(current_point);
//			uint16_t t_reg = getPointRegisterT(current_point);
//
//			// Set targets
//			pris_target_p = registerFrame[r_reg].U16;
//			rev_target_p = registerFrame[t_reg].U16;
//
//			// Send to trajectory control
//			plotter_process_moving_mode(pris_target_p, rev_target_p);
//		}
//
//		// Check if point reached (trajectories complete)
//		if (!pristrajectoryActive && !revtrajectoryActive) {
//			// Move to next point
//			current_point++;
//
//			// Check if all points traversed
//			if (current_point >= getNumberOfSetPoints()) {
//				current_point = 0; // Reset for next time
//				joy_state = A1B1_MODE;
//			} else {
//				moving_state = MOVING_GO_TO_POINT;
//				// Set target for next point
//				uint16_t r_reg = getPointRegisterR(current_point);
//				uint16_t t_reg = getPointRegisterT(current_point);
//
//				// Set targets
//				pris_target_p = registerFrame[r_reg].U16;
//				rev_target_p = registerFrame[t_reg].U16;
//
//				// Send to trajectory control
//				plotter_process_moving_mode(pris_target_p, rev_target_p);
//			}
//		}
//		break;
//
//	case A1B1_SETPOINT:
//		plotter_joymove();
//
//		// When b1 is pressed, save the current position
//		if (b1[0] == 0 && b1[1] == 1) {
//			// Determine which point to set based on current state
//			uint16_t r_reg = 0, t_reg = 0;
//			uint8_t point_index = 0;
//
//			// Convert enum to numerical index
//			if (setpoint_state == NO_POINT_SET || setpoint_state == POINT_IDLE)
//				point_index = 0;
//			else if (setpoint_state == POINT_10_SET) {
//				// All points are set, return to A1B1_MODE
//				joy_state = A1B1_MODE;
//				return;
//			} else
//				point_index = (uint8_t) setpoint_state;
//
//			// Map index to register addresses (using helper function)
//			r_reg = getPointRegisterR(point_index);
//			t_reg = getPointRegisterT(point_index);
//
//			// Update setpoint state to next value
//			switch (setpoint_state) {
//			case NO_POINT_SET:
//			case POINT_IDLE:
//				setpoint_state = POINT_1_SET;
//				break;
//			case POINT_1_SET:
//				setpoint_state = POINT_2_SET;
//				break;
//			case POINT_2_SET:
//				setpoint_state = POINT_3_SET;
//				break;
//			case POINT_3_SET:
//				setpoint_state = POINT_4_SET;
//				break;
//			case POINT_4_SET:
//				setpoint_state = POINT_5_SET;
//				break;
//			case POINT_5_SET:
//				setpoint_state = POINT_6_SET;
//				break;
//			case POINT_6_SET:
//				setpoint_state = POINT_7_SET;
//				break;
//			case POINT_7_SET:
//				setpoint_state = POINT_8_SET;
//				break;
//			case POINT_8_SET:
//				setpoint_state = POINT_9_SET;
//				break;
//			case POINT_9_SET:
//				setpoint_state = POINT_10_SET;
//				break;
//			case POINT_10_SET:
//				// Return to menu if all points set
//				joy_state = A1B1_MODE;
//				break;
//			}
//
//			// Save current position to the registers
//			registerFrame[r_reg].U16 = prismatic_encoder.mm * 10.0;
//			registerFrame[t_reg].U16 = revolute_encoder.rads * 10.0;
//
//			total_setpoints = point_index + 1;
//		}
//		break;
//
//	case A2B2_GOTO_HOME:
//		// Transition to home state
//		plotter_process_return_to_home();
//		break;
//
//	case A2B2_WRITING:
//		plotter_process_writing_state();
//		break;
//
//	default:
//		// Stop motors in other states
//		MDXX_set_range(&prismatic_motor, 2000, 0);
//		MDXX_set_range(&revolute_motor, 2000, 0);
//		break;
//	}
//}
//
//void plotter_process_writing_state() {
//// Do nothing in writing state
//
//// Reset motors to ensure safety
//	MDXX_set_range(&prismatic_motor, 2000, 0);
//	MDXX_set_range(&revolute_motor, 2000, 0);
//
//	writing_state = WRITE_IDLE;
//	joy_state = A2B2_MODE;
//}
//
//void plotter_process_moving_mode(float32_t target_p_pris,
//		float32_t target_p_rev) {
//// Process moving through points using MovingThroghPointState
//	switch (moving_state) {
//	case MOVING_GO_TO_POINT:
//		plotter_process_trajectory_control(target_p_pris, target_p_rev);
//
//		if (!pristrajectoryActive && !revtrajectoryActive) {
//			moving_state = MOVING_DOWN;
//		}
//		break;
//
//	case MOVING_DOWN:
//		// Lower pen at target position
//		plotter_pen_down();
//		// Wait for pen to complete movement
//		if (servo_state == PEN_DOWN) {
//			moving_state = MOVING_UP;
//		}
//		break;
//
//	case MOVING_UP:
//		// Raise pen after touching point
//		plotter_pen_up();
//		// Wait for pen to complete movement
//		if (servo_state == PEN_UP) {
//			moving_state = MOVING_COMPLETE;
//		}
//		break;
//
//	case MOVING_COMPLETE:
//		moving_state = MOVING_IDLE;
//		break;
//
//	case MOVING_IDLE:
//		// Stop motors
//		MDXX_set_range(&prismatic_motor, 2000, 0);
//		MDXX_set_range(&revolute_motor, 2000, 0);
//		break;
//	}
//}
//
//void plotter_process_return_to_home() {
//// First ensure pen is up
//	if (servo_state != PEN_UP) {
//		plotter_pen_up();
////		return;
//	}
//
//// Home revolute axis first
//	if (revolute_state != RP_AT_HOME_POSITION) {
//		revolute_state = RP_GOING_HOME;
//
//		// Move revolute axis until proximity sensor triggered
//		MDXX_set_range(&revolute_motor, 2000, 1000);
//
//		// Check if home reached
//		if (prox) {
//			revolute_state = RP_AT_HOME_POSITION;
//			MDXX_set_range(&revolute_motor, 2000, 0);
//		}
//	}
//// Then home prismatic axis if revolute is done
//	else if (prismatic_state != PP_AT_TOP_END_POSITION) {
//		prismatic_state = PP_GOING_TOP_END;
//
//		// Move prismatic upward
//		MDXX_set_range(&prismatic_motor, 2000, 1000);
//
//		// Check if top position reached
//		if (up_photo) {
//			prismatic_state = PP_AT_TOP_END_POSITION;
//			MDXX_set_range(&prismatic_motor, 2000, 0);
//		}
//	}
//// Both axes homed
//	if (prismatic_state == PP_AT_TOP_END_POSITION
//			&& revolute_state == RP_AT_HOME_POSITION) {
//		plotter_reset();
//
//		pris_pos[0] = 0.0;
//		pris_pos[1] = 0.0;
//
//		rev_pos[0] = 0.0;
//		rev_pos[1] = 0.0;
//
//		pristrajectoryActive = false;
//		revtrajectoryActive = false;
//
//		pris_target_p = 0.0f;
//		rev_target_p = 0.0f;
//
//		if (joy_state == A2B2_GOTO_HOME) {
//			joy_state = A2B2_MODE;
//			rs_current_state = RS_JOG_MODE;
//		} else {
//			rs_current_state = RS_IDLE;
//		}
//	}
//}
//
//void plotter_process_emergency() {
//// Emergency stop - immediately cut power to motors
//	MDXX_set_range(&prismatic_motor, 2000, 0);
//	MDXX_set_range(&revolute_motor, 2000, 0);
//
//// Exit emergency mode only if button pressed and emergency switch released
////	if (joy_state == EMERGENCY_MODE && b1 && !emer) {
////		rs_current_state = RS_RETURN_TO_HOME;
////		emer_state = DEFAULT;
////	}
//}
//
//void plotter_process_trajectory_control(float32_t pris_tgt, float32_t rev_tgt) {
//	if (!is_valid_target(pris_tgt, rev_tgt)) {
//		// If target is invalid, don't start trajectory
//		pristrajectoryActive = false;
//		revtrajectoryActive = false;
//		return;
//	}
//
//// Setup target points
//	pris_target_p = pris_tgt;
//	rev_target_p = rev_tgt;
//
//// Set initial positions from current encoder readings
//	pris_initial_p = prismatic_encoder.mm;
//	rev_initial_p = revolute_encoder.rads;
//
//// Reset trajectory timers
//	prisEva.t = 0.0f;
//	prisEva.isFinised = false;
//	revEva.t = 0.0f;
//	revEva.isFinised = false;
//
//// Generate prismatic trajectory
//	Trapezoidal_Generator(&prisGen, pris_initial_p, pris_target_p,
//			ZGX45RGG_400RPM_Constant.sd_max, ZGX45RGG_400RPM_Constant.sdd_max);
//
//// Generate revolute trajectory
//	Trapezoidal_Generator(&revGen, rev_initial_p, rev_target_p,
//			ZGX45RGG_150RPM_Constant.qd_max, ZGX45RGG_150RPM_Constant.qdd_max);
//
//// Activate trajectories
//	pristrajectoryActive = true;
//	revtrajectoryActive = true;
//}
//
//void plotter_update_trajectories() {
//// Evaluate prismatic trajectory
//	if (pristrajectoryActive) {
//		Trapezoidal_Evaluated(&prisGen, &prisEva, pris_initial_p, pris_target_p,
//				ZGX45RGG_400RPM_Constant.sd_max,
//				ZGX45RGG_400RPM_Constant.sdd_max);
//
//		// Update reference signals
//		prismatic_pos = prisEva.setposition;
//		prismatic_vel = prisEva.setvelocity;
//		prismatic_accel = prisEva.setacceleration;
//
//		// Check if trajectory is complete
//		if (prisEva.isFinised) {
//			pristrajectoryActive = false;
//			prismatic_state = PP_TARGET_REACH;
//		}
//	}
//
//// Evaluate revolute trajectory
//	if (revtrajectoryActive) {
//		Trapezoidal_Evaluated(&revGen, &revEva, rev_initial_p, rev_target_p,
//				ZGX45RGG_150RPM_Constant.qd_max,
//				ZGX45RGG_150RPM_Constant.qdd_max);
//
//		// Update reference signals
//		revolute_pos = revEva.setposition;
//		revolute_vel = revEva.setvelocity;
//		revolute_accel = revEva.setacceleration;
//
//		// Check if trajectory is complete
//		if (revEva.isFinised) {
//			revtrajectoryActive = false;
//			revolute_state = RP_TARGET_REACH;
//		}
//	}
//
//// If trajectories are active, apply motion control
//	if (pristrajectoryActive || revtrajectoryActive) {
//		plotter_move();
//	}
//}
//
//uint16_t getPointRegisterR(uint8_t point_index) {
//	switch (point_index) {
//	case 0:
//		return Target_PosR_1;
//	case 1:
//		return Target_PosR_2;
//	case 2:
//		return Target_PosR_3;
//	case 3:
//		return Target_PosR_4;
//	case 4:
//		return Target_PosR_5;
//	case 5:
//		return Target_PosR_6;
//	case 6:
//		return Target_PosR_7;
//	case 7:
//		return Target_PosR_8;
//	case 8:
//		return Target_PosR_9;
//	case 9:
//		return Target_PosR_10;
//	default:
//		return Target_PosR_1;
//	}
//}
//
//uint16_t getPointRegisterT(uint8_t point_index) {
//	switch (point_index) {
//	case 0:
//		return Target_PosT_1;
//	case 1:
//		return Target_PosT_2;
//	case 2:
//		return Target_PosT_3;
//	case 3:
//		return Target_PosT_4;
//	case 4:
//		return Target_PosT_5;
//	case 5:
//		return Target_PosT_6;
//	case 6:
//		return Target_PosT_7;
//	case 7:
//		return Target_PosT_8;
//	case 8:
//		return Target_PosT_9;
//	case 9:
//		return Target_PosT_10;
//	default:
//		return Target_PosT_1;
//	}
//}
//
//uint8_t getNumberOfSetPoints() {
//	switch (setpoint_state) {
//	case NO_POINT_SET:
//	case POINT_IDLE:
//		return 0;
//	case POINT_1_SET:
//		return 1;
//	case POINT_2_SET:
//		return 2;
//	case POINT_3_SET:
//		return 3;
//	case POINT_4_SET:
//		return 4;
//	case POINT_5_SET:
//		return 5;
//	case POINT_6_SET:
//		return 6;
//	case POINT_7_SET:
//		return 7;
//	case POINT_8_SET:
//		return 8;
//	case POINT_9_SET:
//		return 9;
//	case POINT_10_SET:
//		return 10;
//	default:
//		return 0;
//	}
//}
