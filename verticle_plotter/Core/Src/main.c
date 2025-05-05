/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "plotter_config.h"
#include "Trapezoidal.h"
#include "serial_frame.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRISMATIC_MAX_POS 300.0f  // mm
#define PRISMATIC_MIN_POS 0.0f
#define REVOLUTE_MAX_POS (2*PI)   // 1 revolution
#define REVOLUTE_MIN_POS 0.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SerialFrame serial_frame;

Trapezoidal_GenStruct prisGen;
Trapezoidal_EvaStruct prisEva;
bool pristrajectoryActive = false;
float32_t pris_initial_p = 0.0f;
float32_t pris_target_p = 0.0f;
float32_t prismatic_pos, prismatic_vel, prismatic_accel;
float pris_pos_error, pris_vel_error, pris_kal_filt, pris_vin;
float pris_cmd_ux, pris_cmd_vx;

Trapezoidal_GenStruct revGen;
Trapezoidal_EvaStruct revEva;
bool revtrajectoryActive = false;
float32_t rev_initial_p = 0.0f;
float32_t rev_target_p = 0.0f;
float32_t revolute_pos, revolute_vel, revolute_accel;
float rev_pos_error, rev_vel_error, rev_kal_filt, rev_vin;
float rev_cmd_ux, rev_cmd_vx;

uint8_t total_setpoints, move_index;

float pris_pos[2], rev_pos[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
bool check_prismatic_limit();
bool check_revolute_limit();

void plotter_move();
void plotter_joymove();
void plotter_handle_state_transition();
void plotter_process_jog_mode();
void plotter_process_writing_state();
void plotter_process_moving_mode(float32_t target_p_pris,
		float32_t target_p_rev);
void plotter_process_return_to_home();
void plotter_process_emergency();
void plotter_process_trajectory_control(float32_t pris_tgt, float32_t rev_tgt);
void plotter_update_trajectories();

uint16_t getPointRegisterR(uint8_t point_index);
uint16_t getPointRegisterT(uint8_t point_index);
uint8_t getNumberOfSetPoints();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM5_Init();
	MX_TIM8_Init();
	MX_USART2_UART_Init();
	MX_TIM16_Init();
	MX_TIM1_Init();
	MX_LPUART1_UART_Init();
	/* USER CODE BEGIN 2 */
	plotter_begin();

//	SerialFrame_Init(&serial_frame, &hlpuart1, 37, 'N');
//	SerialFrame_StartReceive(&serial_frame);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		plotter_update_sensors();
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
bool check_prismatic_limit() {
	return prismatic_encoder.mm >= PRISMATIC_MAX_POS
			|| prismatic_encoder.mm <= PRISMATIC_MIN_POS || up_photo
			|| low_photo;
}

bool check_revolute_limit() {
	return revolute_encoder.rads >= REVOLUTE_MAX_POS
			|| revolute_encoder.rads <= REVOLUTE_MIN_POS;
}

void plotter_move() {
	pris_pos[0] = prismatic_encoder.mm;

	pris_pos_error = prismatic_pos - prismatic_encoder.mm;

	pris_cmd_vx = saturation(
			PID_CONTROLLER_Compute(&prismatic_position_pid, pris_pos_error),
			ZGX45RGG_400RPM_Constant.sd_max, -ZGX45RGG_400RPM_Constant.sd_max);

	pris_vel_error = pris_cmd_vx + prismatic_vel - pris_kal_filt;

	pris_cmd_ux = PWM_Satuation(
			PID_CONTROLLER_Compute(&prismatic_velocity_pid, pris_vel_error),
			ZGX45RGG_400RPM_Constant.U_max, -ZGX45RGG_400RPM_Constant.U_max);

	if (check_prismatic_limit()) {
		MDXX_set_range(&prismatic_motor, 2000, 0);
		pristrajectoryActive = false;
		return;
	}

	MDXX_set_range(&prismatic_motor, 2000, pris_cmd_ux);

	if (pris_pos[0] - pris_pos[1] > 0) {
		prismatic_state = PP_GO_UP;
	} else {
		prismatic_state = PP_GO_DOWN;
	}

/////////////////////////////////////////////////////

	rev_pos[0] = revolute_encoder.rads;

	rev_pos_error = revolute_pos - revolute_encoder.rads;

	rev_cmd_vx = saturation(
			PID_CONTROLLER_Compute(&revolute_position_pid, rev_pos_error),
			ZGX45RGG_150RPM_Constant.qd_max, -ZGX45RGG_150RPM_Constant.qd_max);

	rev_vel_error = rev_cmd_vx + revolute_vel - rev_kal_filt;

	rev_cmd_ux = PWM_Satuation(
			PID_CONTROLLER_Compute(&revolute_velocity_pid, rev_vel_error),
			ZGX45RGG_150RPM_Constant.U_max, -ZGX45RGG_150RPM_Constant.U_max);

	if (check_revolute_limit()) {
		MDXX_set_range(&revolute_motor, 2000, 0);
		revtrajectoryActive = false;
		return;
	}

	MDXX_set_range(&revolute_motor, 2000, rev_cmd_ux);

	if (rev_pos[0] - rev_pos[1] > 0) {
		revolute_state = RP_GO_CLOCKWISE;
	} else {
		revolute_state = RP_GO_COUNTER_CLOCKWISE;
	}

	pris_pos[1] = pris_pos[0];
	rev_pos[1] = rev_pos[0];
}

void plotter_joymove() {
	static float32_t pris_joy, rev_joy;

// Prismatic limits check
	if (check_prismatic_limit()) {
		pris_joy = 0.0f;
	} else {
		if (joystick_y > 0.7) {
			pris_joy = (joystick_y - 0.7) * 25000.0f;
			prismatic_state = PP_GO_UP;
		} else if (joystick_y < -0.7) {
			pris_joy = (joystick_y + 0.7) * 25000.0f;
			prismatic_state = PP_GO_DOWN;
		} else {
			pris_joy = 0.0;
			prismatic_state = PP_UNKNOWN;
		}
	}

// Revolute limits check
	if (check_revolute_limit()) {
		rev_joy = 0.0f;
	} else {
		if (joystick_x > 0.7) {
			rev_joy = (joystick_x - 0.7) * 25000.0f;
			revolute_state = RP_GO_COUNTER_CLOCKWISE;
		} else if (joystick_x < -0.7) {
			rev_joy = (joystick_x + 0.7) * 25000.0f;
			revolute_state = RP_GO_CLOCKWISE;
		} else {
			rev_joy = 0.0;
			revolute_state = RP_UNKNOWN;
		}
	}

	MDXX_set_range(&prismatic_motor, 2000, pris_joy);
	MDXX_set_range(&revolute_motor, 2000, rev_joy);
}

void plotter_handle_state_transition() {
	// Save previous state
	rs_previous_state = rs_current_state;

	// Only process base system commands if not in emergency mode
	if (rs_current_state != RS_EMERGENCY_TRIGGED) {
		// Process base system commands
		if (registerFrame[BaseSystem_Status].U16 == 1) {
			// Set Home Command
			rs_current_state = RS_RETURN_TO_HOME;
			registerFrame[R_Theta_Status].U16 = 1;
		} else if (registerFrame[BaseSystem_Status].U16 == 2) {
			// Joystick Mode
			rs_current_state = RS_JOG_MODE;
			registerFrame[R_Theta_Status].U16 = 2;
		} else if (registerFrame[BaseSystem_Status].U16 == 4) {
			// Point Mode (Just Receive Input)
			rs_current_state = RS_POINT_MODE;
			registerFrame[R_Theta_Status].U16 = 4;
		} else if (registerFrame[BaseSystem_Status].U16 == 8) {
			// Go to Point Mode
			rs_current_state = RS_MOVING;
			registerFrame[R_Theta_Status].U16 = 8;
		}
	}

	// Handle state entry/exit actions if state has changed
	if (rs_previous_state != rs_current_state) {
		// Exit actions for previous state
		switch (rs_previous_state) {
		case RS_JOG_MODE:
			// Clean up any joystick control resources
			joy_state = JOY_IDLE;
			MDXX_set_range(&prismatic_motor, 2000, 0);
			MDXX_set_range(&revolute_motor, 2000, 0);
			break;

		case RS_MOVING:
			// Stop trajectory generators
			pristrajectoryActive = false;
			revtrajectoryActive = false;
			break;

		case RS_EMERGENCY_TRIGGED:
			// Exit emergency state
			emer_state = DEFAULT;
			break;

		default:
			break;
		}

		// Entry actions for new state
		switch (rs_current_state) {
		case RS_JOG_MODE:
			// Initialize joystick control
			joy_state = A1B1_MODE;
			break;

		case RS_MOVING:
			// Initialize trajectory generator
			pris_initial_p = prismatic_encoder.mm;
			rev_initial_p = revolute_encoder.rads;
			pris_target_p = registerFrame[Goal_R].U16;
			rev_target_p = registerFrame[Goal_Theta].U16;
			pristrajectoryActive = true;
			revtrajectoryActive = true;
			moving_state = MOVING_GO_TO_POINT;
			break;

		case RS_EMERGENCY_TRIGGED:
			// Enter emergency state
			joy_state = EMERGENCY_MODE;
			registerFrame[R_Theta_Status].U16 = 16;
			// Immediately stop all motors
			MDXX_set_range(&prismatic_motor, 2000, 0);
			MDXX_set_range(&revolute_motor, 2000, 0);
			break;

		case RS_RETURN_TO_HOME:
			// Initialize homing sequence
			plotter_pen_up(); // First ensure pen is up
			break;

		case RS_POINT_MODE:
			MDXX_set_range(&prismatic_motor, 2000, 0);
			MDXX_set_range(&revolute_motor, 2000, 0);
			break;

		default:
			break;
		}
	}
}

void plotter_process_jog_mode() {
	// block any joystick actions while in emergency
	if (rs_current_state == RS_EMERGENCY_TRIGGED
			|| joy_state == EMERGENCY_MODE) {
		return;
	}

	// Handle state transitions for A1B1_MODE
	if (joy_state == A1B1_MODE) {
		if (b1) {
			joy_state = A1B1_SETPOINT;
		} else if (b2) {
			joy_state = A1B1_MOVING;
			moving_state = MOVING_GO_TO_POINT;
			if (total_setpoints > 0) {
				joy_state = A1B1_MOVING;
				moving_state = MOVING_GO_TO_POINT;
				move_index = 0;            // start at first point
			}

		} else if (b4) {
			joy_state = A2B2_MODE;
		}
	}
	// Handle transitions back to A1B1_MODE
	else if ((joy_state == A1B1_SETPOINT && b2)
			|| (joy_state == A1B1_MOVING && b1)) {
		joy_state = A1B1_MODE;
	}

	// Handle state transitions for A2B2_MODE
	if (joy_state == A2B2_MODE) {
		if (b1) {
			joy_state = A2B2_WRITING;
			writing_state = WRITE_IDLE;
		} else if (b2) {
			joy_state = A2B2_GOTO_HOME;
		} else if (b4) {
			joy_state = A1B1_MODE;
		}
	}
	// Handle transitions back to A2B2_MODE
	else if ((joy_state == A2B2_WRITING && b2)
			|| (joy_state == A2B2_GOTO_HOME && b1)) {
		joy_state = A2B2_MODE;
	}

	// Execute state-specific actions
	switch (joy_state) {
	case A1B1_MOVING:
		if (total_setpoints == 0) {
			joy_state = A1B1_MODE;
		}
		static uint8_t current_point = 0;

		// Initialize target for first point if needed
		if (current_point == 0) {
			// Get register for first point
			uint16_t r_reg = getPointRegisterR(current_point);
			uint16_t t_reg = getPointRegisterT(current_point);

			// Set targets
			pris_target_p = registerFrame[r_reg].U16;
			rev_target_p = registerFrame[t_reg].U16;

			// Send to trajectory control
			plotter_process_moving_mode(pris_target_p, rev_target_p);
		}

		// Check if point reached (trajectories complete)
		if (!pristrajectoryActive && !revtrajectoryActive) {
			// Move to next point
			current_point++;

			// Check if all points traversed
			if (current_point >= getNumberOfSetPoints()) {
				current_point = 0; // Reset for next time
				joy_state = A1B1_MODE;
			} else {
				moving_state = MOVING_GO_TO_POINT;
				// Set target for next point
				uint16_t r_reg = getPointRegisterR(current_point);
				uint16_t t_reg = getPointRegisterT(current_point);

				// Set targets
				pris_target_p = registerFrame[r_reg].U16;
				rev_target_p = registerFrame[t_reg].U16;

				// Send to trajectory control
				plotter_process_moving_mode(pris_target_p, rev_target_p);
			}
		}
		break;

	case A1B1_SETPOINT:
		plotter_joymove();

		// When b1 is pressed, save the current position
		if (b1) {
			// Determine which point to set based on current state
			uint16_t r_reg = 0, t_reg = 0;
			uint8_t point_index = 0;

			// Convert enum to numerical index
			if (setpoint_state == NO_POINT_SET || setpoint_state == POINT_IDLE)
				point_index = 0;
			else if (setpoint_state == POINT_10_SET) {
				// All points are set, return to A1B1_MODE
				joy_state = A1B1_MODE;
				return;
			} else
				point_index = (uint8_t) setpoint_state;

			// Map index to register addresses (using helper function)
			r_reg = getPointRegisterR(point_index);
			t_reg = getPointRegisterT(point_index);

			// Update setpoint state to next value
			switch (setpoint_state) {
			case NO_POINT_SET:
			case POINT_IDLE:
				setpoint_state = POINT_1_SET;
				break;
			case POINT_1_SET:
				setpoint_state = POINT_2_SET;
				break;
			case POINT_2_SET:
				setpoint_state = POINT_3_SET;
				break;
			case POINT_3_SET:
				setpoint_state = POINT_4_SET;
				break;
			case POINT_4_SET:
				setpoint_state = POINT_5_SET;
				break;
			case POINT_5_SET:
				setpoint_state = POINT_6_SET;
				break;
			case POINT_6_SET:
				setpoint_state = POINT_7_SET;
				break;
			case POINT_7_SET:
				setpoint_state = POINT_8_SET;
				break;
			case POINT_8_SET:
				setpoint_state = POINT_9_SET;
				break;
			case POINT_9_SET:
				setpoint_state = POINT_10_SET;
				break;
			case POINT_10_SET:
				// Return to menu if all points set
				joy_state = A1B1_MODE;
				break;
			}

			// Save current position to the registers
			registerFrame[r_reg].U16 = prismatic_encoder.mm * 10.0;
			registerFrame[t_reg].U16 = revolute_encoder.rads * 10.0;

			total_setpoints = point_index + 1;
		}
		break;

	case A2B2_GOTO_HOME:
		// Transition to home state
		rs_current_state = RS_RETURN_TO_HOME; //////////////////////////////////
		break;

	case A2B2_WRITING:
		plotter_process_writing_state();
		break;

	default:
		// Stop motors in other states
		MDXX_set_range(&prismatic_motor, 2000, 0);
		MDXX_set_range(&revolute_motor, 2000, 0);
		break;
	}
}

void plotter_process_writing_state() {
// Do nothing in writing state

// Reset motors to ensure safety
	MDXX_set_range(&prismatic_motor, 2000, 0);
	MDXX_set_range(&revolute_motor, 2000, 0);

	writing_state = WRITE_IDLE;
	joy_state = A2B2_MODE;
}

void plotter_process_moving_mode(float32_t target_p_pris,
		float32_t target_p_rev) {
// Process moving through points using MovingThroghPointState
	switch (moving_state) {
	case MOVING_GO_TO_POINT:
		plotter_process_trajectory_control(target_p_pris, target_p_rev);

		if (!pristrajectoryActive && !revtrajectoryActive) {
			moving_state = MOVING_DOWN;
		}
		break;

	case MOVING_DOWN:
		// Lower pen at target position
		plotter_pen_down();
		// Wait for pen to complete movement
		if (servo_state == PEN_DOWN) {
			moving_state = MOVING_UP;
		}
		break;

	case MOVING_UP:
		// Raise pen after touching point
		plotter_pen_up();
		// Wait for pen to complete movement
		if (servo_state == PEN_UP) {
			moving_state = MOVING_COMPLETE;
		}
		break;

	case MOVING_COMPLETE:
		moving_state = MOVING_IDLE;
		break;

	case MOVING_IDLE:
		// Stop motors
		MDXX_set_range(&prismatic_motor, 2000, 0);
		MDXX_set_range(&revolute_motor, 2000, 0);
		break;
	}
}

void plotter_process_return_to_home() {
// First ensure pen is up
	if (servo_state != PEN_UP) {
		plotter_pen_up();
		return;
	}

// Home revolute axis first
	if (revolute_state != RP_AT_HOME_POSITION) {
		revolute_state = RP_GOING_HOME;

		// Move revolute axis until proximity sensor triggered
		MDXX_set_range(&revolute_motor, 2000, 1000);

		// Check if home reached
		if (prox) {
			revolute_state = RP_AT_HOME_POSITION;
			MDXX_set_range(&revolute_motor, 2000, 0);
		}
	}
// Then home prismatic axis if revolute is done
	else if (prismatic_state != PP_AT_TOP_END_POSITION) {
		prismatic_state = PP_GOING_TOP_END;

		// Move prismatic upward
		MDXX_set_range(&prismatic_motor, 2000, 1000);

		// Check if top position reached
		if (up_photo) {
			prismatic_state = PP_AT_TOP_END_POSITION;
			MDXX_set_range(&prismatic_motor, 2000, 0);
		}
	}
// Both axes homed
	else {
		plotter_reset();

		pris_pos[0] = 0.0;
		pris_pos[1] = 0.0;

		rev_pos[0] = 0.0;
		rev_pos[1] = 0.0;

		if (joy_state == A2B2_GOTO_HOME) {
			joy_state = A2B2_MODE;
			rs_current_state = RS_JOG_MODE;
		} else {
			rs_current_state = RS_IDLE;
		}
	}
}

void plotter_process_emergency() {
// Emergency stop - immediately cut power to motors
	MDXX_set_range(&prismatic_motor, 2000, 0);
	MDXX_set_range(&revolute_motor, 2000, 0);

// Exit emergency mode only if button pressed and emergency switch released
	if (joy_state == EMERGENCY_MODE && b1 && !emer) {
		rs_current_state = RS_RETURN_TO_HOME;
		emer_state = DEFAULT;
	}
}

void plotter_process_trajectory_control(float32_t pris_tgt, float32_t rev_tgt) {
	if (pris_tgt > PRISMATIC_MAX_POS || pris_tgt < PRISMATIC_MIN_POS) {
		return;
	}

	if (rev_tgt > REVOLUTE_MAX_POS || rev_tgt < REVOLUTE_MIN_POS) {
		return;
	}

// Setup target points
	pris_target_p = pris_tgt;
	rev_target_p = rev_tgt;

// Set initial positions from current encoder readings
	pris_initial_p = prismatic_encoder.mm;
	rev_initial_p = revolute_encoder.rads;

// Reset trajectory timers
	prisEva.t = 0.0f;
	prisEva.isFinised = false;
	revEva.t = 0.0f;
	revEva.isFinised = false;

// Generate prismatic trajectory
	Trapezoidal_Generator(&prisGen, pris_initial_p, pris_target_p,
			ZGX45RGG_400RPM_Constant.sd_max, ZGX45RGG_400RPM_Constant.sdd_max);

// Generate revolute trajectory
	Trapezoidal_Generator(&revGen, rev_initial_p, rev_target_p,
			ZGX45RGG_150RPM_Constant.qd_max, ZGX45RGG_150RPM_Constant.qdd_max);

// Activate trajectories
	pristrajectoryActive = true;
	revtrajectoryActive = true;
}

void plotter_update_trajectories() {
// Evaluate prismatic trajectory
	if (pristrajectoryActive) {
		Trapezoidal_Evaluated(&prisGen, &prisEva, pris_initial_p, pris_target_p,
				ZGX45RGG_400RPM_Constant.sd_max,
				ZGX45RGG_400RPM_Constant.sdd_max);

		// Update reference signals
		prismatic_pos = prisEva.setposition;
		prismatic_vel = prisEva.setvelocity;
		prismatic_accel = prisEva.setacceleration;

		// Check if trajectory is complete
		if (prisEva.isFinised) {
			pristrajectoryActive = false;
			prismatic_state = PP_TARGET_REACH;
		}
	}

// Evaluate revolute trajectory
	if (revtrajectoryActive) {
		Trapezoidal_Evaluated(&revGen, &revEva, rev_initial_p, rev_target_p,
				ZGX45RGG_150RPM_Constant.qd_max,
				ZGX45RGG_150RPM_Constant.qdd_max);

		// Update reference signals
		revolute_pos = revEva.setposition;
		revolute_vel = revEva.setvelocity;
		revolute_accel = revEva.setacceleration;

		// Check if trajectory is complete
		if (revEva.isFinised) {
			revtrajectoryActive = false;
			revolute_state = RP_TARGET_REACH;
		}
	}

// If trajectories are active, apply motion control
	if (pristrajectoryActive || revtrajectoryActive) {
		plotter_move();
	}
}

uint16_t getPointRegisterR(uint8_t point_index) {
	switch (point_index) {
	case 0:
		return Target_PosR_1;
	case 1:
		return Target_PosR_2;
	case 2:
		return Target_PosR_3;
	case 3:
		return Target_PosR_4;
	case 4:
		return Target_PosR_5;
	case 5:
		return Target_PosR_6;
	case 6:
		return Target_PosR_7;
	case 7:
		return Target_PosR_8;
	case 8:
		return Target_PosR_9;
	case 9:
		return Target_PosR_10;
	default:
		return Target_PosR_1;
	}
}

uint16_t getPointRegisterT(uint8_t point_index) {
	switch (point_index) {
	case 0:
		return Target_PosT_1;
	case 1:
		return Target_PosT_2;
	case 2:
		return Target_PosT_3;
	case 3:
		return Target_PosT_4;
	case 4:
		return Target_PosT_5;
	case 5:
		return Target_PosT_6;
	case 6:
		return Target_PosT_7;
	case 7:
		return Target_PosT_8;
	case 8:
		return Target_PosT_9;
	case 9:
		return Target_PosT_10;
	default:
		return Target_PosT_1;
	}
}

uint8_t getNumberOfSetPoints() {
	switch (setpoint_state) {
	case NO_POINT_SET:
	case POINT_IDLE:
		return 0;
	case POINT_1_SET:
		return 1;
	case POINT_2_SET:
		return 2;
	case POINT_3_SET:
		return 3;
	case POINT_4_SET:
		return 4;
	case POINT_5_SET:
		return 5;
	case POINT_6_SET:
		return 6;
	case POINT_7_SET:
		return 7;
	case POINT_8_SET:
		return 8;
	case POINT_9_SET:
		return 9;
	case POINT_10_SET:
		return 10;
	default:
		return 0;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == EMER_Pin) {
		rs_current_state = RS_EMERGENCY_TRIGGED;
		emer_state = PUSHED;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &hlpuart1) {

	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		Modbus_Protocal_Worker();

		QEI_get_diff_count(&prismatic_encoder);
		QEI_compute_data(&prismatic_encoder);

		pris_vin = pris_cmd_ux * ZGX45RGG_400RPM_Constant.V_max
				/ ZGX45RGG_400RPM_Constant.U_max;

		pris_kal_filt = SteadyStateKalmanFilter(&prismatic_kalman, pris_vin,
				prismatic_encoder.rads)
				* Disturbance_Constant.prismatic_pulley_radius;

		QEI_get_diff_count(&revolute_encoder);
		QEI_compute_data(&revolute_encoder);

		rev_vin = rev_cmd_ux * ZGX45RGG_150RPM_Constant.V_max
				/ ZGX45RGG_150RPM_Constant.U_max;

		rev_kal_filt = SteadyStateKalmanFilter(&revolute_kalman, rev_vin,
				revolute_encoder.rads);

		// Heartbeat and pen commands
		registerFrame[Heartbeat_Protocol].U16 = 22881;

		if (registerFrame[Servo_UP].U16 == 1) {
			plotter_pen_up();
		} else if (registerFrame[Servo_Down].U16 == 1) {
			plotter_pen_down();
		}

		// Update limit switch status
		if (servo_state == PEN_UP) {
			registerFrame[LimitSwitch_Status].U16 = 1;
		} else if (servo_state == PEN_DOWN) {
			registerFrame[LimitSwitch_Status].U16 = 2;
		}

		if (pristrajectoryActive || revtrajectoryActive) {
			plotter_update_trajectories();
		}

		plotter_handle_state_transition();

		if (check_prismatic_limit() || check_revolute_limit()) {
			pristrajectoryActive = false;
			revtrajectoryActive = false;
			rs_current_state = RS_EMERGENCY_TRIGGED;
		}

		switch (rs_current_state) {
		case RS_JOG_MODE:
			plotter_process_jog_mode();
			break;

		case RS_POINT_MODE:
			MDXX_set_range(&prismatic_motor, 2000, 0);
			MDXX_set_range(&revolute_motor, 2000, 0);
			break;

		case RS_MOVING:
			static bool point_initialized = false;
			if (!point_initialized) {
				plotter_process_moving_mode(pris_target_p, rev_target_p);
				point_initialized = true;
			}

			if (!pristrajectoryActive && !revtrajectoryActive) {
				point_initialized = false;
				rs_current_state = RS_IDLE;
			}
			break;

		case RS_RETURN_TO_HOME:
			plotter_process_return_to_home();
			break;

		case RS_EMERGENCY_TRIGGED:
			plotter_process_emergency();
			break;

		case RS_IDLE:
		default:
			MDXX_set_range(&prismatic_motor, 2000, 0);
			MDXX_set_range(&revolute_motor, 2000, 0);
			break;
		}

		registerFrame[R_Axis_Actual_Position].U16 = prismatic_encoder.mm * 10.0;
		registerFrame[Theta_Axis_Actual_Position].U16 = revolute_encoder.rads
				* 10.0;
		registerFrame[R_Axis_Actual_Speed].U16 = pris_kal_filt * 10.0;
		registerFrame[Theta_Axis_Actual_Speed].U16 = rev_kal_filt * 10.0;
		registerFrame[R_Axis_Acceleration].U16 = prismatic_encoder.mmpss * 10.0;
		registerFrame[Theta_Axis_Acceleration].U16 = revolute_encoder.radpss
				* 10.0;

		if (rs_current_state == RS_IDLE) {
			// If move is done, reset status registers
			registerFrame[BaseSystem_Status].U16 = 0;
			registerFrame[R_Theta_Status].U16 = 0;
		}
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
