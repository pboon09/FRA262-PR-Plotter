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
float32_t prismatic_pos = 0.0f, prismatic_vel = 0.0f, prismatic_accel = 0.0f;
float pris_pos_error = 0.0f, pris_vel_error = 0.0f, pris_kal_filt = 0.0f,
		pris_vin = 0.0f;
float pris_cmd_ux = 0.0f, pris_cmd_vx = 0.0f;

Trapezoidal_GenStruct revGen;
Trapezoidal_EvaStruct revEva;
bool revtrajectoryActive = false;
float32_t rev_initial_p = 0.0f;
float32_t rev_target_p = 0.0f;
float32_t revolute_pos = 0.0f, revolute_vel = 0.0f, revolute_accel = 0.0f;
float rev_pos_error = 0.0f, rev_vel_error = 0.0f, rev_kal_filt, rev_vin = 0.0f;
float rev_cmd_ux = 0.0f, rev_cmd_vx = 0.0f;

uint8_t total_setpoints = 0, move_index = 0;

float pris_pos[2] = { 0.0f }, rev_pos[2] = { 0.0f };

float rev_dfd, rev_ffd, pris_dfd, pris_ffd;

bool homing_in_progress = false;

// Existing revolute backlash variables
#define BACKLASH_COMPENSATION_GAIN 0.2f
float revolute_backlash = 0.01f;
float revolute_last_cmd_direction = 0.0f;
float revolute_backlash_state = 0.0f;

// New prismatic backlash variables
#define PRISMATIC_BACKLASH_COMPENSATION_GAIN 0.1f
float prismatic_backlash = 0.03f;  // Backlash in mm
float prismatic_last_cmd_direction = 0.0f;
float prismatic_backlash_state = 0.0f;

int home;

float signal;

uint8_t trajectory_sequence_index = 0;
bool sequence_active = false;
const float32_t sequence_pris_points[4] = { 175.0f, 95.0f, 275.0f, 0.0f }; // Sequence of setpoints
const float32_t sequence_rev_points[4] = { 175.0f, 195.0f, 95.0f, 0.0f }; // Sequence of setpoints

uint8_t button_pressed_previous = 0;

float cur_pos;

float dfd, ffd;

float deg;
float normalized_current;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void start_combined_trajectory(float prismatic_target_mm,
		float revolute_target_deg);
void start_homing();
void update_control_loops();
void velocity_control(float prismatic_target_mmps, float revolute_target_rads);
float revolute_backlash_compensator(float cmd_vel);
float prismatic_backlash_compensator(float cmd_vel);
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

	//	SerialFrame_StartReceive(&serial_frame);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (b1 && !button_pressed_previous && !revtrajectoryActive
				&& !pristrajectoryActive) {
			start_combined_trajectory(
					sequence_pris_points[trajectory_sequence_index],
					sequence_rev_points[trajectory_sequence_index]);
			trajectory_sequence_index = (trajectory_sequence_index + 1) % 4;
		}

		button_pressed_previous = b1;
		if (b2) {
			start_homing();
		}

		if (b3) {
			home = 99;
		}

		if (b4) {
			NVIC_SystemReset();
		}
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
void start_combined_trajectory(float prismatic_target_mm,
		float revolute_target_deg) {
	// Get current positions
	float pris_current = prismatic_encoder.mm;
	float rev_current = revolute_encoder.rads;

	// Reset trajectory evaluation structs
	prisEva.t = 0.0f;
	prisEva.isFinised = false;
	revEva.t = 0.0f;
	revEva.isFinised = false;

	// Save initial positions
	pris_initial_p = pris_current;
	rev_initial_p = rev_current;

	// For prismatic axis - direct target
	pris_target_p = prismatic_target_mm;

	// For revolute axis - handle path planning
	// Normalize current position to [0, 2π]
	float normalized_current = fmodf(rev_current, 2 * PI);
	if (normalized_current < 0) {
		normalized_current += 2 * PI;
	}

	// Convert target to radians
	float target_radians = UnitConverter_angle(&converter_system,
			revolute_target_deg, UNIT_DEGREE, UNIT_RADIAN);

	// Normalize target to [0, 2π]
	float normalized_target = fmodf(target_radians, 2 * PI);
	if (normalized_target < 0) {
		normalized_target += 2 * PI;
	}

	// Convert to degrees for easier logic
	float current_deg = normalized_current * 180.0f / PI;
	float target_deg = revolute_target_deg;

	// Define explicit rules for movements
	float movement_deg = 0.0f;

	// If both angles are on the same side of 180°
	if ((current_deg < 180 && target_deg < 180)
			|| (current_deg >= 180 && target_deg >= 180)) {

		// Simple case - take shortest path
		movement_deg = target_deg - current_deg;

		// Ensure shortest path
		if (movement_deg > 180)
			movement_deg -= 360;
		if (movement_deg < -180)
			movement_deg += 360;
	}
	// If we need to cross the 180° boundary
	else {
		// Explicitly determine direction to avoid crossing 180°
		if (current_deg < 180) {
			// Current < 180, target > 180
			// Go counterclockwise through 0°
			if (current_deg < target_deg - 180) {
				movement_deg = -(current_deg + (360 - target_deg)); // Negative = clockwise
			} else {
				movement_deg = -(current_deg - target_deg + 360); // Negative = clockwise
			}
		} else {
			// Current > 180, target < 180
			// Go clockwise through 0°
			if (target_deg < current_deg - 180) {
				movement_deg = 360 - current_deg + target_deg; // Positive = counterclockwise
			} else {
				movement_deg = target_deg - current_deg + 360; // Positive = counterclockwise
			}
		}
	}

	// Convert to radians and apply to absolute position
	float movement_rad = movement_deg * PI / 180.0f;
	rev_target_p = rev_initial_p + movement_rad;

	// Generate trajectories
	Trapezoidal_Generator(&prisGen, pris_initial_p, pris_target_p,
			ZGX45RGG_400RPM_Constant.sd_max, ZGX45RGG_400RPM_Constant.sdd_max);

	Trapezoidal_Generator(&revGen, rev_initial_p, rev_target_p,
			ZGX45RGG_150RPM_Constant.qd_max, ZGX45RGG_150RPM_Constant.qdd_max);

	plotter_pen_up();

	pristrajectoryActive = true;
	revtrajectoryActive = true;
}

void start_homing() {
	// Initialize homing state
	homing_in_progress = true;

	// First ensure pen is up
	plotter_pen_up();

	// Enter homing state
	home = 1; // This triggers your existing homing sequence in the timer callback
}

void update_control_loops() {
	// Normalize revolute position
	cur_pos = fmodf(revolute_encoder.rads, 2 * PI);
	if (cur_pos < 0) {
		cur_pos += 2 * PI;
	}

	// Calculate angle in degrees for display/debugging
	deg = UnitConverter_angle(&converter_system, cur_pos, UNIT_RADIAN,
			UNIT_DEGREE);

	// Update prismatic trajectory if active
	if (pristrajectoryActive && !prisEva.isFinised) {
		Trapezoidal_Evaluated(&prisGen, &prisEva, pris_initial_p, pris_target_p,
				ZGX45RGG_400RPM_Constant.sd_max,
				ZGX45RGG_400RPM_Constant.sdd_max);

		prismatic_pos = prisEva.setposition;
		prismatic_vel = prisEva.setvelocity;

		if (prisEva.isFinised) {
			pristrajectoryActive = false;
			prismatic_pos = prisEva.setposition;
			prismatic_vel = 0.0f;
			pris_dfd = 0.0;
			pris_ffd = 0.0;
		}
	}

	// Update revolute trajectory if active
	if (revtrajectoryActive && !revEva.isFinised) {
		Trapezoidal_Evaluated(&revGen, &revEva, rev_initial_p, rev_target_p,
				ZGX45RGG_150RPM_Constant.qd_max,
				ZGX45RGG_150RPM_Constant.qdd_max);

		revolute_pos = revEva.setposition;
		revolute_vel = revEva.setvelocity;

		if (revEva.isFinised) {
			revtrajectoryActive = false;
			revolute_pos = revEva.setposition;
			revolute_vel = 0.0f;
			rev_dfd = 0.0;
			rev_ffd = 0.0;
		}
	}

	// Calculate control signals for prismatic axis
	pris_vin = mapf(pris_cmd_ux, -65535.0, 65535.0, -12.0, 12.0);

	pris_kal_filt = MotorKalman_Estimate(&prismatic_kalman, pris_vin,
			prismatic_encoder.rads)
			* Disturbance_Constant.prismatic_pulley_radius * 1000;

	if (isnan(pris_kal_filt)) {
		pris_kal_filt = 0.0f;
	}

	pris_pos_error = prismatic_pos - prismatic_encoder.mm;

	pris_cmd_vx = PWM_Satuation(
			PID_CONTROLLER_Compute(&prismatic_position_pid, pris_pos_error),
			ZGX45RGG_400RPM_Constant.sd_max, -ZGX45RGG_400RPM_Constant.sd_max);

	// Add velocity feedforward for trajectory
	if (pristrajectoryActive) {
		pris_vel_error = pris_cmd_vx + prismatic_vel - pris_kal_filt;
	} else {
		pris_vel_error = pris_cmd_vx - pris_kal_filt;
	}

	pris_cmd_ux = PWM_Satuation(
			PID_CONTROLLER_Compute(&prismatic_velocity_pid, pris_vel_error),
			ZGX45RGG_400RPM_Constant.U_max, -ZGX45RGG_400RPM_Constant.U_max);

	// Calculate control signals for revolute axis
	rev_vin = mapf(rev_cmd_ux, -65535.0, 65535.0, -12.0, 12.0);

	rev_kal_filt = MotorKalman_Estimate(&revolute_kalman, rev_vin,
			revolute_encoder.rads);

	if (isnan(rev_kal_filt)) {
		rev_kal_filt = 0.0f;
	}

	rev_pos_error = revolute_pos - cur_pos;

	// Ensure error uses the shortest path for control
	if (rev_pos_error > PI) {
		rev_pos_error -= 2 * PI;
	}
	if (rev_pos_error < -PI) {
		rev_pos_error += 2 * PI;
	}

	rev_cmd_vx = PWM_Satuation(
			PID_CONTROLLER_Compute(&revolute_position_pid, rev_pos_error),
			ZGX45RGG_150RPM_Constant.qd_max, -ZGX45RGG_150RPM_Constant.qd_max);

	// Add velocity feedforward for trajectory
	if (revtrajectoryActive) {
		rev_vel_error = rev_cmd_vx + revolute_vel - rev_kal_filt;
	} else {
		rev_vel_error = rev_cmd_vx - rev_kal_filt;
	}

	rev_cmd_ux = PWM_Satuation(
			PID_CONTROLLER_Compute(&revolute_velocity_pid, rev_vel_error),
			ZGX45RGG_150RPM_Constant.U_max, -ZGX45RGG_150RPM_Constant.U_max);

	// Add feed-forward compensation
	pris_ffd = PRISMATIC_MOTOR_FFD_Compute(&prismatic_motor_ffd,
			prismatic_vel / 1000.0);

	pris_dfd = PRISMATIC_MOTOR_DFD_Compute(&prismatic_motor_dfd, cur_pos,
			revolute_vel, prismatic_encoder.mm / 1000.0);

	float pris_backlash_compensation = prismatic_backlash_compensator(
			pris_cmd_vx);

	pris_cmd_ux = pris_cmd_ux + pris_backlash_compensation + pris_dfd
			+ pris_ffd;

	rev_dfd = REVOLUTE_MOTOR_DFD_Compute(&revolute_motor_dfd, cur_pos, 0.0,
			prismatic_encoder.mm / 1000.0);
	rev_ffd = REVOLUTE_MOTOR_FFD_Compute(&revolute_motor_ffd, revolute_vel);

	float rev_backlash_compensation = revolute_backlash_compensator(rev_cmd_vx);

	rev_cmd_ux = rev_cmd_ux + rev_backlash_compensation + rev_dfd + rev_ffd;

	// Final saturation
	pris_cmd_ux = PWM_Satuation(pris_cmd_ux, ZGX45RGG_400RPM_Constant.U_max,
			-ZGX45RGG_400RPM_Constant.U_max);

	rev_cmd_ux = PWM_Satuation(rev_cmd_ux, ZGX45RGG_150RPM_Constant.U_max,
			-ZGX45RGG_150RPM_Constant.U_max);

	// Apply commands to motors
	MDXX_set_range(&prismatic_motor, 2000, pris_cmd_ux);
	MDXX_set_range(&revolute_motor, 2000, rev_cmd_ux);

	if (revEva.isFinised && prisEva.isFinised) {
		static uint32_t timer_count = 0;
		timer_count++;

		if (timer_count > 500) {
			plotter_pen_down();
			timer_count = 0;
		}
	}
}

void velocity_control(float prismatic_target_mmps, float revolute_target_rads) {
	// Get current velocity estimates from Kalman filter
	pris_vin = mapf(pris_cmd_ux, -65535.0, 65535.0, -12.0, 12.0);
	pris_kal_filt = MotorKalman_Estimate(&prismatic_kalman, pris_vin,
			prismatic_encoder.rads)
			* Disturbance_Constant.prismatic_pulley_radius * 1000;

	if (isnan(pris_kal_filt)) {
		pris_kal_filt = 0.0f;
	}

	rev_vin = mapf(rev_cmd_ux, -65535.0, 65535.0, -12.0, 12.0);
	rev_kal_filt = MotorKalman_Estimate(&revolute_kalman, rev_vin,
			revolute_encoder.rads);

	if (isnan(rev_kal_filt)) {
		rev_kal_filt = 0.0f;
	}

	// Calculate velocity errors (bypassing position cascade)
	pris_vel_error = prismatic_target_mmps - pris_kal_filt;
	rev_vel_error = revolute_target_rads - rev_kal_filt;

	// Compute control signals using velocity PID only
	pris_cmd_ux = PWM_Satuation(
			PID_CONTROLLER_Compute(&prismatic_velocity_pid, pris_vel_error),
			ZGX45RGG_400RPM_Constant.U_max, -ZGX45RGG_400RPM_Constant.U_max);

	rev_cmd_ux = PWM_Satuation(
			PID_CONTROLLER_Compute(&revolute_velocity_pid, rev_vel_error),
			ZGX45RGG_150RPM_Constant.U_max, -ZGX45RGG_150RPM_Constant.U_max);

	// Add feed-forward compensation for prismatic axis
	pris_ffd = PRISMATIC_MOTOR_FFD_Compute(&prismatic_motor_ffd,
			prismatic_target_mmps / 1000.0); // Convert mm/s to m/s

	// Get current revolute position for feed-forward compensation
	cur_pos = fmodf(revolute_encoder.rads, 2 * PI);
	if (cur_pos < 0) {
		cur_pos += 2 * PI;
	}

	// Add disturbance feed-forward for prismatic axis
	pris_dfd = PRISMATIC_MOTOR_DFD_Compute(&prismatic_motor_dfd, cur_pos,
			revolute_target_rads, prismatic_encoder.mm / 1000.0);

	// Add backlash compensation for prismatic axis
	float pris_backlash_compensation = prismatic_backlash_compensator(
			prismatic_target_mmps);

	// Add feed-forward terms to prismatic control signal
	pris_cmd_ux = pris_cmd_ux + pris_backlash_compensation + pris_dfd
			+ pris_ffd;

	// Add feed-forward compensation for revolute axis
	rev_ffd = REVOLUTE_MOTOR_FFD_Compute(&revolute_motor_ffd,
			revolute_target_rads);

	// Add disturbance feed-forward for revolute axis
	rev_dfd = REVOLUTE_MOTOR_DFD_Compute(&revolute_motor_dfd, cur_pos, 0.0,
			prismatic_encoder.mm / 1000.0);

	// Add backlash compensation for revolute axis
	float rev_backlash_compensation = revolute_backlash_compensator(
			revolute_target_rads);

	// Add feed-forward terms to revolute control signal
	rev_cmd_ux = rev_cmd_ux + rev_backlash_compensation + rev_dfd + rev_ffd;

	// Final saturation
	pris_cmd_ux = PWM_Satuation(pris_cmd_ux, ZGX45RGG_400RPM_Constant.U_max,
			-ZGX45RGG_400RPM_Constant.U_max);

	rev_cmd_ux = PWM_Satuation(rev_cmd_ux, ZGX45RGG_150RPM_Constant.U_max,
			-ZGX45RGG_150RPM_Constant.U_max);

	// Apply commands to motors
	MDXX_set_range(&prismatic_motor, 2000, pris_cmd_ux);
	MDXX_set_range(&revolute_motor, 2000, rev_cmd_ux);
}

float prismatic_backlash_compensator(float cmd_vel) {
	// Determine current direction
	float current_direction =
			(cmd_vel > 0.0f) ? 1.0f : ((cmd_vel < 0.0f) ? -1.0f : 0.0f);

	// If stopped, maintain last direction
	if (current_direction == 0.0f) {
		current_direction = prismatic_last_cmd_direction;
	}

	// Detect direction change
	if (current_direction != prismatic_last_cmd_direction
			&& current_direction != 0.0f) {
		// If direction changed, update backlash state
		prismatic_backlash_state = current_direction * prismatic_backlash;
		prismatic_last_cmd_direction = current_direction;
	}

	// Apply adaptive compensation based on velocity
	float compensation = prismatic_backlash_state
			* PRISMATIC_BACKLASH_COMPENSATION_GAIN;

	// Gradually reduce backlash state as we overcome the backlash
	// This simulates the physical process of taking up the slack
	float backlash_decay_rate = 0.01f * fabsf(cmd_vel); // Proportional to velocity
	if (prismatic_backlash_state > 0.0f) {
		prismatic_backlash_state = fmaxf(0.0f,
				prismatic_backlash_state - backlash_decay_rate);
	} else if (prismatic_backlash_state < 0.0f) {
		prismatic_backlash_state = fminf(0.0f,
				prismatic_backlash_state + backlash_decay_rate);
	}

	return compensation;
}

float revolute_backlash_compensator(float cmd_vel) {
	// Determine current direction
	float current_direction =
			(cmd_vel > 0.0f) ? 1.0f : ((cmd_vel < 0.0f) ? -1.0f : 0.0f);

	// If stopped, maintain last direction
	if (current_direction == 0.0f) {
		current_direction = revolute_last_cmd_direction;
	}

	// Detect direction change
	if (current_direction != revolute_last_cmd_direction
			&& current_direction != 0.0f) {
		// If direction changed, update backlash state
		revolute_backlash_state = current_direction * revolute_backlash;
		revolute_last_cmd_direction = current_direction;
	}

	// Apply adaptive compensation based on velocity
	float compensation = revolute_backlash_state * BACKLASH_COMPENSATION_GAIN;

	// Gradually reduce backlash state as we overcome the backlash
	// This simulates the physical process of taking up the slack
	float backlash_decay_rate = 0.01f * fabsf(cmd_vel); // Proportional to velocity
	if (revolute_backlash_state > 0.0f) {
		revolute_backlash_state = fmaxf(0.0f,
				revolute_backlash_state - backlash_decay_rate);
	} else if (revolute_backlash_state < 0.0f) {
		revolute_backlash_state = fminf(0.0f,
				revolute_backlash_state + backlash_decay_rate);
	}

	return compensation;
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
		plotter_update_sensors();

		QEI_get_diff_count(&prismatic_encoder);
		QEI_compute_data(&prismatic_encoder);

		QEI_get_diff_count(&revolute_encoder);
		QEI_compute_data(&revolute_encoder);

		if (home == 99) {
			plotter_pen_down();

			pristrajectoryActive = false;
			revtrajectoryActive = false;

			if (up_photo && joystick_x > 40) {
				MDXX_set_range(&prismatic_motor, 2000, 0);
			} else if (low_photo && joystick_x < -40) {
				MDXX_set_range(&prismatic_motor, 2000, 0);
			} else if (joystick_x > 40) {
				MDXX_set_range(&prismatic_motor, 2000, -12000);
			} else if (joystick_x < -40) {
				MDXX_set_range(&prismatic_motor, 2000, 12000);
			} else {
				MDXX_set_range(&prismatic_motor, 2000, 0);
			}

			float revolute_deg = UnitConverter_angle(&converter_system,
					revolute_encoder.rads, UNIT_RADIAN, UNIT_DEGREE);

			if ((revolute_deg > 175.0f && joystick_y > 40)
					|| (revolute_deg < -175.0f && joystick_y < -40)) {
				MDXX_set_range(&revolute_motor, 2000, 0);
			} else if (joystick_y > 40) {
				MDXX_set_range(&revolute_motor, 2000, 25000);
			} else if (joystick_y < -40) {
				MDXX_set_range(&revolute_motor, 2000, -25000);
			} else {
				MDXX_set_range(&revolute_motor, 2000, 0);
			}
		}

		else if (home > 0) {
			homing_in_progress = true;

			if (home == 1) {
				// Move prismatic motor down to lower limit
				MDXX_set_range(&prismatic_motor, 2000, 10000);

				if (low_photo) {
					home = 2;
					MDXX_set_range(&prismatic_motor, 2000, 0);
				}
			} else if (home == 2) {
				static int prox_count = 0;
				static bool prox_previous = false;
				static bool initialized = false;

				// Initialize on first entry
				if (!initialized) {
					prox_previous = prox;
					prox_count = 0;
					initialized = true;
				}

				// Move revolute motor clockwise at constant speed
				MDXX_set_range(&revolute_motor, 2000, 12000);

				// Count proximity sensor triggers (rising edge detection)
				if (prox && !prox_previous) {
					prox_count++;
				}
				prox_previous = prox;

				// After reaching home, stop motor
				if (prox_count >= 1) {
					MDXX_set_range(&revolute_motor, 2000, 0);
					initialized = false;  // Reset for next time
					home = 3;
				}
			} else if (home == 3) {
				// Move prismatic motor up to upper limit
				MDXX_set_range(&prismatic_motor, 2000, -10000);

				if (up_photo) {
					MDXX_set_range(&prismatic_motor, 2000, 0);
					home = 4;
				}
			} else if (home == 4) {
				plotter_reset();
				home = 0;
				homing_in_progress = false;

			}
		} else {
			update_control_loops();
		}
	}
}

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
//	if (htim == &htim2) {
//		Modbus_Protocal_Worker();
//
//		QEI_get_diff_count(&prismatic_encoder);
//		QEI_compute_data(&prismatic_encoder);
//
//		pris_vin = pris_cmd_ux * ZGX45RGG_400RPM_Constant.V_max
//				/ ZGX45RGG_400RPM_Constant.U_max;
//
//		pris_kal_filt = SteadyStateKalmanFilter(&prismatic_kalman, pris_vin,
//				prismatic_encoder.rads)
//				* Disturbance_Constant.prismatic_pulley_radius;
//
//		QEI_get_diff_count(&revolute_encoder);
//		QEI_compute_data(&revolute_encoder);
//
//		rev_vin = rev_cmd_ux * ZGX45RGG_150RPM_Constant.V_max
//				/ ZGX45RGG_150RPM_Constant.U_max;
//
//		rev_kal_filt = SteadyStateKalmanFilter(&revolute_kalman, rev_vin,
//				revolute_encoder.rads);
//
//		// Heartbeat and pen commands
//		registerFrame[Heartbeat_Protocol].U16 = 22881;
//
//		if (registerFrame[Servo_UP].U16 == 1) {
//			plotter_pen_up();
//		} else if (registerFrame[Servo_Down].U16 == 1) {
//			plotter_pen_down();
//		}
//
//		// Update limit switch status
//		if (servo_state == PEN_UP) {
//			registerFrame[LimitSwitch_Status].U16 = 1;
//		} else if (servo_state == PEN_DOWN) {
//			registerFrame[LimitSwitch_Status].U16 = 2;
//		}
//
//		if (pristrajectoryActive || revtrajectoryActive) {
//			plotter_update_trajectories();
//		}
//
//		plotter_handle_state_transition();
//
////		if (check_prismatic_limit() || check_revolute_limit()) {
////			pristrajectoryActive = false;
////			revtrajectoryActive = false;
////			rs_current_state = RS_EMERGENCY_TRIGGED;
////		}
//
//		switch (rs_current_state) {
//		case RS_JOG_MODE:
//			plotter_process_jog_mode();
//			b1[1]==b1[0];
//			b2[1]==b2[0];
//			b3[1]==b3[0];
//			b4[1]==b4[0];
//			break;
//
//		case RS_POINT_MODE:
//			MDXX_set_range(&prismatic_motor, 2000, 0);
//			MDXX_set_range(&revolute_motor, 2000, 0);
//			break;
//
//		case RS_MOVING:
//			static bool point_initialized = false;
//			if (!point_initialized) {
//				plotter_process_moving_mode(pris_target_p, rev_target_p);
//				point_initialized = true;
//			}
//
//			if (!pristrajectoryActive && !revtrajectoryActive) {
//				point_initialized = false;
//				rs_current_state = RS_IDLE;
//			}
//			break;
//
//		case RS_RETURN_TO_HOME:
//			plotter_process_return_to_home();
//			break;
//
//		case RS_EMERGENCY_TRIGGED:
//			plotter_process_emergency();
//			break;
//
//		case RS_IDLE:
//		default:
//			MDXX_set_range(&prismatic_motor, 2000, 0);
//			MDXX_set_range(&revolute_motor, 2000, 0);
//			break;
//		}
//
//		registerFrame[R_Axis_Actual_Position].U16 = prismatic_encoder.mm * 10.0;
//		registerFrame[Theta_Axis_Actual_Position].U16 = revolute_encoder.rads
//				* 10.0;
//		registerFrame[R_Axis_Actual_Speed].U16 = pris_kal_filt * 10.0;
//		registerFrame[Theta_Axis_Actual_Speed].U16 = rev_kal_filt * 10.0;
//		registerFrame[R_Axis_Acceleration].U16 = prismatic_encoder.mmpss * 10.0;
//		registerFrame[Theta_Axis_Acceleration].U16 = revolute_encoder.radpss
//				* 10.0;
//
//		if (rs_current_state == RS_IDLE) {
//			// If move is done, reset status registers
//			registerFrame[BaseSystem_Status].U16 = 0;
//			registerFrame[R_Theta_Status].U16 = 0;
//		}
//	}
//}
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
