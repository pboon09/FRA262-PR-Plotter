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
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	HOMING_IDLE = 0,
	HOMING_PRIS_DOWN,
	HOMING_REV_RESET,
	HOMING_PRIS_UP,
	HOMING_COMPLETE
} HomingState_t;

typedef enum {
	JOY_IDLE = 0,         // Not in joystick control mode
	JOY_ACTIVE,           // Actively controlling with joystick
	JOY_EXIT_REQUESTED    // Exit requested, preparing to maintain position
} JoystickState_t;

typedef struct {
	float32_t position;        // Current position setpoint
	float32_t velocity;        // Current velocity setpoint
	float32_t acceleration;    // Current acceleration setpoint
	float32_t initial_pos;     // Start position for trajectory
	float32_t target_pos;      // Target position for trajectory
	float32_t pos_error;       // Position error for control
	float32_t vel_error;       // Velocity error for control
	float32_t kalman_velocity; // Kalman filtered velocity
	float32_t input_voltage;   // Motor input voltage
	float32_t command_pos;     // Position command
	float32_t command_vel;     // Velocity command
	float32_t dfd;             // Disturbance feedforward
	float32_t ffd;             // Velocity feedforward
	bool trajectory_active;    // Flag for active trajectory

	float32_t target;
	float32_t mm;
	float32_t deg;
} AxisState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRISMATIC_MAX_POS 300.0f  // mm
#define PRISMATIC_MIN_POS 0.0f
#define REVOLUTE_MAX_POS (2.0f*PI)   // 1 revolution
#define REVOLUTE_MIN_POS 0.0f

#define BACKLASH_COMPENSATION_GAIN 0.2f
#define PRISMATIC_BACKLASH_COMPENSATION_GAIN 0.5f
#define BACKLASH_DECAY_FACTOR 0.01f
#define SEQUENCE_MAX_POINTS 4
#define JOYSTICK_THRESHOLD 40
#define PEN_SETTLE_TIME 500 // Delay before pen down (in timer ticks)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SerialFrame serial_frame;

// Trajectory generation structures
Trapezoidal_GenStruct prisGen, revGen;
Trapezoidal_EvaStruct prisEva, revEva;

// Axis state structures
AxisState_t prismatic_axis = { 0 };
AxisState_t revolute_axis = { 0 };

// Backlash compensation
float revolute_backlash = 0.01f;
float revolute_last_cmd_direction = 0.0f;
float revolute_backlash_state = 0.0f;

float prismatic_backlash = 0.50f;  // Backlash in mm
float prismatic_last_cmd_direction = 0.0f;
float prismatic_backlash_state = 0.0f;

// System state variables
HomingState_t homing_state = HOMING_IDLE;
uint8_t trajectory_sequence_index = 0;
bool sequence_active = false;
bool button_pressed_previous = false;
const float32_t sequence_pris_points[SEQUENCE_MAX_POINTS] = { 175.0f, 95.0f,
		275.0f, 0.0f };
const float32_t sequence_rev_points[SEQUENCE_MAX_POINTS] = { 175.0f, 195.0f,
		95.0f, 0.0f };

JoystickState_t joystick_state = JOY_IDLE;

float pris_velocity_target = 0.0f;
float rev_velocity_target = 0.0f;

// Temporary variables used in functions
float normalized_position;
float movement_deg;
float angle_deg;

volatile bool prox_rising_edge = false;
volatile uint32_t prox_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void start_combined_trajectory(float prismatic_target_mm,
		float revolute_target_deg);
void start_homing(void);
void update_control_loops(void);
float revolute_backlash_compensator(float cmd_vel);
float prismatic_backlash_compensator(float cmd_vel);
float normalize_angle(float angle_rad);
float calculate_movement_deg(float current_deg, float target_deg);
void manual_control_mode(void);
void process_joystick_control(void);
void velocity_control(float prismatic_target_mmps, float revolute_target_rads);
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

	// Initialize position setpoints to current position to prevent startup errors
	prismatic_axis.position = prismatic_encoder.mm;
	revolute_axis.position = revolute_encoder.rads;

	// Reset all error terms and flags
	prismatic_axis.pos_error = 0.0f;
	prismatic_axis.vel_error = 0.0f;
	revolute_axis.pos_error = 0.0f;
	revolute_axis.vel_error = 0.0f;
	prismatic_axis.trajectory_active = false;
	revolute_axis.trajectory_active = false;
	//	SerialFrame_StartReceive(&serial_frame);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (b1 && !button_pressed_previous && !revolute_axis.trajectory_active
				&& !prismatic_axis.trajectory_active) {
			start_combined_trajectory(
					sequence_pris_points[trajectory_sequence_index],
					sequence_rev_points[trajectory_sequence_index]);
			trajectory_sequence_index = (trajectory_sequence_index + 1)
					% SEQUENCE_MAX_POINTS;
		}
		button_pressed_previous = b1;

		// Process button 2 - start homing procedure
		if (b2 && homing_state == HOMING_IDLE) {
			start_homing();
		}

		// Process button 3 - enter manual control mode
		if (b3) {
			manual_control_mode();
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

/* USER CODE BEGIN 4 *//**
 * @brief Normalizes an angle to the range [0, 2π]
 * @param angle_rad Angle in radians
 * @return Normalized angle in radians
 */
float normalize_angle(float angle_rad) {
	float result = fmodf(angle_rad, 2.0f * PI);
	if (result < 0.0f) {
		result += 2.0f * PI;
	}
	return result;
}

/**
 * @brief Calculates shortest angular movement in degrees
 * @param current_deg Current angle in degrees
 * @param target_deg Target angle in degrees
 * @return Movement required in degrees
 */
float calculate_movement_deg(float current_deg, float target_deg) {
	float movement = 0.0f;

	// If both angles are on the same side of 180°
	if ((current_deg < 180.0f && target_deg < 180.0f)
			|| (current_deg >= 180.0f && target_deg >= 180.0f)) {
		// Simple case - take shortest path
		movement = target_deg - current_deg;

		// Ensure shortest path
		if (movement > 180.0f)
			movement -= 360.0f;
		if (movement < -180.0f)
			movement += 360.0f;
	}
	// If we need to cross the 180° boundary
	else {
		// Explicitly determine direction to avoid crossing 180°
		if (current_deg < 180.0f) {
			// Current < 180, target > 180
			// Go counterclockwise through 0°
			if (current_deg < target_deg - 180.0f) {
				movement = -(current_deg + (360.0f - target_deg)); // Negative = clockwise
			} else {
				movement = -(current_deg - target_deg + 360.0f); // Negative = clockwise
			}
		} else {
			// Current > 180, target < 180
			// Go clockwise through 0°
			if (target_deg < current_deg - 180.0f) {
				movement = 360.0f - current_deg + target_deg; // Positive = counterclockwise
			} else {
				movement = target_deg - current_deg + 360.0f; // Positive = counterclockwise
			}
		}
	}

	return movement;
}

/**
 * @brief Starts a combined trajectory for both prismatic and revolute axes
 * @param prismatic_target_mm Target position for prismatic axis in mm
 * @param revolute_target_deg Target position for revolute axis in degrees
 */
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
	prismatic_axis.initial_pos = pris_current;
	revolute_axis.initial_pos = rev_current;

	// For prismatic axis - direct target with bounds checking
	prismatic_axis.target_pos = fminf(
			fmaxf(prismatic_target_mm, PRISMATIC_MIN_POS), PRISMATIC_MAX_POS);

	// For revolute axis - handle path planning
	// Normalize current position to [0, 2π]
	float normalized_current = normalize_angle(rev_current);

	// Convert target to radians
	float target_radians = UnitConverter_angle(&converter_system,
			revolute_target_deg, UNIT_DEGREE, UNIT_RADIAN);

	// Normalize target to [0, 2π]
	float normalized_target = normalize_angle(target_radians);

	// Convert to degrees for movement calculation
	float current_deg = normalized_current * 180.0f / PI;
	float target_deg = revolute_target_deg;

	// Calculate movement in degrees
	float movement_deg = calculate_movement_deg(current_deg, target_deg);

	// Convert to radians and apply to absolute position
	float movement_rad = movement_deg * PI / 180.0f;
	revolute_axis.target_pos = revolute_axis.initial_pos + movement_rad;

	// Generate trajectories
	Trapezoidal_Generator(&prisGen, prismatic_axis.initial_pos,
			prismatic_axis.target_pos, ZGX45RGG_400RPM_Constant.sd_max,
			ZGX45RGG_400RPM_Constant.sdd_max);

	Trapezoidal_Generator(&revGen, revolute_axis.initial_pos,
			revolute_axis.target_pos, ZGX45RGG_150RPM_Constant.qd_max,
			ZGX45RGG_150RPM_Constant.qdd_max);

	// Lift pen during movement
	plotter_pen_up();

	// Set trajectory active flags
	prismatic_axis.trajectory_active = true;
	revolute_axis.trajectory_active = true;
}

/**
 * @brief Starts the homing sequence for both axes
 */
void start_homing(void) {
	// Ensure pen is up for safety
	plotter_pen_up();

	// Set initial homing state
	homing_state = HOMING_PRIS_DOWN;
}

/**
 * @brief Updates control loops for both axes
 */
void update_control_loops(void) {
	// Normalize revolute position
	normalized_position = normalize_angle(revolute_encoder.rads);
	angle_deg = normalize_angle(revolute_axis.target_pos);

	// Update prismatic trajectory if active
	if (prismatic_axis.trajectory_active && !prisEva.isFinised) {
		Trapezoidal_Evaluated(&prisGen, &prisEva, prismatic_axis.initial_pos,
				prismatic_axis.target_pos, ZGX45RGG_400RPM_Constant.sd_max,
				ZGX45RGG_400RPM_Constant.sdd_max);

		prismatic_axis.position = prisEva.setposition;
		prismatic_axis.velocity = prisEva.setvelocity;

		if (prisEva.isFinised) {
			prismatic_axis.trajectory_active = false;
			prismatic_axis.position = prisEva.setposition;
			prismatic_axis.velocity = 0.0f;
			prismatic_axis.dfd = 0.0f;
			prismatic_axis.ffd = 0.0f;
		}
	}

	// Update revolute trajectory if active
	if (revolute_axis.trajectory_active && !revEva.isFinised) {
		Trapezoidal_Evaluated(&revGen, &revEva, revolute_axis.initial_pos,
				revolute_axis.target_pos, ZGX45RGG_150RPM_Constant.qd_max,
				ZGX45RGG_150RPM_Constant.qdd_max);

		revolute_axis.position = revEva.setposition;
		revolute_axis.velocity = revEva.setvelocity;

		if (revEva.isFinised) {
			revolute_axis.trajectory_active = false;
			revolute_axis.position = revEva.setposition;
			revolute_axis.velocity = 0.0f;
			revolute_axis.dfd = 0.0f;
			revolute_axis.ffd = 0.0f;
		}
	}

	// Calculate control signals for prismatic axis
	prismatic_axis.input_voltage = mapf(prismatic_axis.command_pos, -65535.0f,
			65535.0f, -12.0f, 12.0f);

	prismatic_axis.kalman_velocity = MotorKalman_Estimate(&prismatic_kalman,
			prismatic_axis.input_voltage, prismatic_encoder.rads)
			* Disturbance_Constant.prismatic_pulley_radius * 1000.0f;

	if (isnan(prismatic_axis.kalman_velocity)) {
		prismatic_axis.kalman_velocity = 0.0f;
	}

	prismatic_axis.pos_error = prismatic_axis.position - prismatic_encoder.mm;

	prismatic_axis.command_vel = PWM_Satuation(
			PID_CONTROLLER_Compute(&prismatic_position_pid,
					prismatic_axis.pos_error), ZGX45RGG_400RPM_Constant.sd_max,
			-ZGX45RGG_400RPM_Constant.sd_max);

	// Add velocity feedforward for trajectory
	if (prismatic_axis.trajectory_active) {
		prismatic_axis.vel_error = prismatic_axis.command_vel
				+ prismatic_axis.velocity - prismatic_axis.kalman_velocity;
	} else {
		prismatic_axis.vel_error = prismatic_axis.command_vel
				- prismatic_axis.kalman_velocity;
	}

	prismatic_axis.command_pos = PWM_Satuation(
			PID_CONTROLLER_Compute(&prismatic_velocity_pid,
					prismatic_axis.vel_error), ZGX45RGG_400RPM_Constant.U_max,
			-ZGX45RGG_400RPM_Constant.U_max);

	// Calculate control signals for revolute axis
	revolute_axis.input_voltage = mapf(revolute_axis.command_pos, -65535.0f,
			65535.0f, -12.0f, 12.0f);

	revolute_axis.kalman_velocity = MotorKalman_Estimate(&revolute_kalman,
			revolute_axis.input_voltage, revolute_encoder.rads);

	if (isnan(revolute_axis.kalman_velocity)) {
		revolute_axis.kalman_velocity = 0.0f;
	}

	revolute_axis.pos_error = revolute_axis.position - normalized_position;

	// Ensure error uses the shortest path for control
	if (revolute_axis.pos_error > PI) {
		revolute_axis.pos_error -= 2.0f * PI;
	}
	if (revolute_axis.pos_error < -PI) {
		revolute_axis.pos_error += 2.0f * PI;
	}

	revolute_axis.command_vel = PWM_Satuation(
			PID_CONTROLLER_Compute(&revolute_position_pid,
					revolute_axis.pos_error), ZGX45RGG_150RPM_Constant.qd_max,
			-ZGX45RGG_150RPM_Constant.qd_max);

	// Add velocity feedforward for trajectory
	if (revolute_axis.trajectory_active) {
		revolute_axis.vel_error = revolute_axis.command_vel
				+ revolute_axis.velocity - revolute_axis.kalman_velocity;
	} else {
		revolute_axis.vel_error = revolute_axis.command_vel
				- revolute_axis.kalman_velocity;
	}

	revolute_axis.command_pos = PWM_Satuation(
			PID_CONTROLLER_Compute(&revolute_velocity_pid,
					revolute_axis.vel_error), ZGX45RGG_150RPM_Constant.U_max,
			-ZGX45RGG_150RPM_Constant.U_max);

	// Add feed-forward compensation
	if (prismatic_axis.trajectory_active) {
		// Only compute FFD during active movement
		prismatic_axis.ffd = PRISMATIC_MOTOR_FFD_Compute(&prismatic_motor_ffd,
				prismatic_axis.velocity / 1000.0f);

		// Only compute DFD during active movement
		prismatic_axis.dfd = PRISMATIC_MOTOR_DFD_Compute(&prismatic_motor_dfd,
				normalized_position, revolute_axis.velocity,
				prismatic_encoder.mm / 1000.0f);
	} else {
		// Zero when not moving
		prismatic_axis.ffd = 0.0f;
		prismatic_axis.dfd = 0.0f;
	}

	float pris_backlash_compensation = prismatic_backlash_compensator(
			prismatic_axis.command_vel);

	prismatic_axis.command_pos = prismatic_axis.command_pos
			+ pris_backlash_compensation + prismatic_axis.dfd
			+ prismatic_axis.ffd;

	revolute_axis.dfd = REVOLUTE_MOTOR_DFD_Compute(&revolute_motor_dfd,
			normalized_position, 0.0f, prismatic_encoder.mm / 1000.0f);

	revolute_axis.ffd = REVOLUTE_MOTOR_FFD_Compute(&revolute_motor_ffd,
			revolute_axis.velocity);

	float rev_backlash_compensation = revolute_backlash_compensator(
			revolute_axis.command_vel);

	revolute_axis.command_pos = revolute_axis.command_pos
			+ rev_backlash_compensation + revolute_axis.dfd + revolute_axis.ffd;

	// Final saturation
	prismatic_axis.command_pos = PWM_Satuation(prismatic_axis.command_pos,
			ZGX45RGG_400RPM_Constant.U_max, -ZGX45RGG_400RPM_Constant.U_max);

	revolute_axis.command_pos = PWM_Satuation(revolute_axis.command_pos,
			ZGX45RGG_150RPM_Constant.U_max, -ZGX45RGG_150RPM_Constant.U_max);

	// Apply commands to motors
	MDXX_set_range(&prismatic_motor, 2000, prismatic_axis.command_pos);
	MDXX_set_range(&revolute_motor, 2000, revolute_axis.command_pos);

	prismatic_axis.mm = prismatic_encoder.mm;
	prismatic_axis.target = prismatic_axis.target_pos;

	revolute_axis.deg = UnitConverter_angle(&converter_system,
			normalized_position, UNIT_RADIAN, UNIT_DEGREE);
	;
	revolute_axis.target = UnitConverter_angle(&converter_system, angle_deg,
			UNIT_RADIAN, UNIT_DEGREE);

	// Put pen down when both trajectories are complete
	static uint32_t pen_down_timer = 0;
	if (revEva.isFinised && prisEva.isFinised) {
		pen_down_timer++;
		if (pen_down_timer > PEN_SETTLE_TIME) {
			plotter_pen_down();
			pen_down_timer = 0;
		}
	} else {
		pen_down_timer = 0;
	}
}

/**
 * @brief Compensation for revolute axis backlash
 * @param cmd_vel Command velocity
 * @return Compensation value
 */
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
	float backlash_decay_rate = BACKLASH_DECAY_FACTOR * fabsf(cmd_vel);
	if (revolute_backlash_state > 0.0f) {
		revolute_backlash_state = fmaxf(0.0f,
				revolute_backlash_state - backlash_decay_rate);
	} else if (revolute_backlash_state < 0.0f) {
		revolute_backlash_state = fminf(0.0f,
				revolute_backlash_state + backlash_decay_rate);
	}

	return compensation;
}

/**
 * @brief Compensation for prismatic axis backlash
 * @param cmd_vel Command velocity
 * @return Compensation value
 */
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
	float backlash_decay_rate = BACKLASH_DECAY_FACTOR * fabsf(cmd_vel);
	if (prismatic_backlash_state > 0.0f) {
		prismatic_backlash_state = fmaxf(0.0f,
				prismatic_backlash_state - backlash_decay_rate);
	} else if (prismatic_backlash_state < 0.0f) {
		prismatic_backlash_state = fminf(0.0f,
				prismatic_backlash_state + backlash_decay_rate);
	}

	return compensation;
}

/**
 * @brief Optimized velocity control implementation for both axes
 * @param prismatic_target_mmps Target velocity for prismatic axis in mm/s
 * @param revolute_target_rads Target velocity for revolute axis in rad/s
 */
void velocity_control(float prismatic_target_mmps, float revolute_target_rads) {
	// -- Prismatic axis control --
	// Calculate current velocity from Kalman filter
	float pris_vin = mapf(prismatic_axis.command_pos, -65535.0f, 65535.0f,
			-12.0f, 12.0f);
	float pris_vel = MotorKalman_Estimate(&prismatic_kalman, pris_vin,
			prismatic_encoder.rads)
			* Disturbance_Constant.prismatic_pulley_radius * 1000.0f;

	// Safety check for NaN
	pris_vel = isnan(pris_vel) ? 0.0f : pris_vel;

	// Calculate velocity error and PID output
	float pris_vel_error = prismatic_target_mmps - pris_vel;
	prismatic_axis.command_pos = PID_CONTROLLER_Compute(&prismatic_velocity_pid,
			pris_vel_error);

	// -- Revolute axis control --
	// Calculate current velocity from Kalman filter
	float rev_vin = mapf(revolute_axis.command_pos, -65535.0f, 65535.0f, -12.0f,
			12.0f);
	float rev_vel = MotorKalman_Estimate(&revolute_kalman, rev_vin,
			revolute_encoder.rads);

	// Safety check for NaN
	rev_vel = isnan(rev_vel) ? 0.0f : rev_vel;

	// Calculate velocity error and PID output
	float rev_vel_error = revolute_target_rads - rev_vel;
	revolute_axis.command_pos = PID_CONTROLLER_Compute(&revolute_velocity_pid,
			rev_vel_error);

	// Get normalized angle once for all feed-forward calculations
	float normalized_angle = normalize_angle(revolute_encoder.rads);
	float pris_position_m = prismatic_encoder.mm / 1000.0f; // Convert mm to m, calculate once

	// -- Add feed-forward terms --
	// Prismatic axis feed-forward
	prismatic_axis.command_pos += PRISMATIC_MOTOR_FFD_Compute(
			&prismatic_motor_ffd, prismatic_target_mmps / 1000.0f) // FFD (velocity)
			+ prismatic_backlash_compensator(prismatic_target_mmps); // Backlash

			// Revolute axis feed-forward
	revolute_axis.command_pos += REVOLUTE_MOTOR_FFD_Compute(&revolute_motor_ffd,
			revolute_target_rads) // FFD (velocity)
			+ REVOLUTE_MOTOR_DFD_Compute(&revolute_motor_dfd, normalized_angle,
					0.0f, pris_position_m) // DFD (disturbance)
			+ revolute_backlash_compensator(revolute_target_rads); // Backlash

			// Apply saturation to both axes in one step
	prismatic_axis.command_pos = PWM_Satuation(prismatic_axis.command_pos,
			ZGX45RGG_400RPM_Constant.U_max, -ZGX45RGG_400RPM_Constant.U_max);
	revolute_axis.command_pos = PWM_Satuation(revolute_axis.command_pos,
			ZGX45RGG_150RPM_Constant.U_max, -ZGX45RGG_150RPM_Constant.U_max);

	// Apply commands to motors
	MDXX_set_range(&prismatic_motor, 2000, prismatic_axis.command_pos);
	MDXX_set_range(&revolute_motor, 2000, revolute_axis.command_pos);
}

/**
 * @brief Process joystick control in timer callback function
 */
void process_joystick_control(void) {
	// State machine for joystick control
	switch (joystick_state) {
	case JOY_IDLE:
		// Not in joystick mode, nothing to do
		rev_velocity_target = 0.0f;
		pris_velocity_target = 0.0f;
		break;

	case JOY_ACTIVE:
        prismatic_axis.position = prismatic_encoder.mm;
        revolute_axis.position = revolute_encoder.rads;;

		// Check if exit is requested
		HAL_GPIO_WritePin(PILOT_GPIO_Port, PILOT_Pin, 1);

		if (b4) {
			// Immediately stop motors for safety
			pris_velocity_target = 0.0f;
			rev_velocity_target = 0.0f;
			velocity_control(pris_velocity_target, rev_velocity_target);

			joystick_state = JOY_EXIT_REQUESTED;
			break;
		}

		// Process prismatic axis joystick control
		if (up_photo && joystick_x > JOYSTICK_THRESHOLD) {
			pris_velocity_target = 0.0f;
		} else if (low_photo && joystick_x < -JOYSTICK_THRESHOLD) {
			pris_velocity_target = 0.0f;
		} else if (joystick_x > JOYSTICK_THRESHOLD) {
			pris_velocity_target = -ZGX45RGG_400RPM_Constant.sd_max / 2.0f;
		} else if (joystick_x < -JOYSTICK_THRESHOLD) {
			pris_velocity_target = ZGX45RGG_400RPM_Constant.sd_max / 2.0f;
		} else {
			pris_velocity_target = 0.0f;
		}

		// Process revolute axis joystick control
		float revolute_deg = UnitConverter_angle(&converter_system,
				revolute_encoder.rads, UNIT_RADIAN, UNIT_DEGREE);

		if ((revolute_deg > 175.0f && joystick_y > JOYSTICK_THRESHOLD)
				|| (revolute_deg < -175.0f && joystick_y < -JOYSTICK_THRESHOLD)) {
			rev_velocity_target = 0.0f;
		} else if (joystick_y > JOYSTICK_THRESHOLD) {
			rev_velocity_target = ZGX45RGG_150RPM_Constant.qd_max;
		} else if (joystick_y < -JOYSTICK_THRESHOLD) {
			rev_velocity_target = -ZGX45RGG_150RPM_Constant.qd_max;
		} else {
			rev_velocity_target = 0.0f;
		}

        velocity_control(pris_velocity_target, rev_velocity_target);

        prismatic_axis.position = prismatic_encoder.mm;
        revolute_axis.position = revolute_encoder.rads;

		break;

    case JOY_EXIT_REQUESTED:
    	velocity_control(0.0f, 0.0f);

        // Reset joystick state and velocity targets
        pris_velocity_target = 0.0f;
        rev_velocity_target = 0.0f;

        // Turn off indicator light
        HAL_GPIO_WritePin(PILOT_GPIO_Port, PILOT_Pin, 0);

        // Return to normal control mode
        joystick_state = JOY_IDLE;
        break;

	default:
		// Unexpected state - reset to idle
		joystick_state = JOY_IDLE;
		break;
	}
}

/**
 * @brief Simplified manual control mode function to start joystick control
 */
void manual_control_mode(void) {
	// Only enter joystick mode if not already in it
	if (joystick_state == JOY_IDLE) {
		// Put pen down for drawing
		plotter_pen_down();

		// Stop any active trajectories
		prismatic_axis.trajectory_active = false;
		revolute_axis.trajectory_active = false;

		// Set joystick state to active
		joystick_state = JOY_ACTIVE;
	}
}

/**
 * @brief GPIO external interrupt callback
 * @param GPIO_Pin Pin that triggered the interrupt
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == EMER_Pin) {
		rs_current_state = RS_EMERGENCY_TRIGGED;
		emer_state = PUSHED;
	}
}

/**
 * @brief UART receive complete callback
 * @param huart UART handle
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// Handle UART reception if needed
}

/**
 * @brief Timer period elapsed callback - main control loop
 * @param htim Timer handle
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		// Update sensor readings
		plotter_update_sensors();

		// Update encoder readings
		QEI_get_diff_count(&prismatic_encoder);
		QEI_compute_data(&prismatic_encoder);
		QEI_get_diff_count(&revolute_encoder);
		QEI_compute_data(&revolute_encoder);

		// Handle different system states
		if (homing_state != HOMING_IDLE) {
			// Homing state machine
			switch (homing_state) {
			case HOMING_PRIS_DOWN:
				// Move prismatic motor down to lower limit
				pris_velocity_target = ZGX45RGG_400RPM_Constant.sd_max / 3.0f;
				if (low_photo) {
					pris_velocity_target = 0.0f;
					homing_state = HOMING_REV_RESET;
				}

		        prismatic_axis.position = prismatic_encoder.mm;
		        revolute_axis.position = revolute_encoder.rads;

		        velocity_control(pris_velocity_target, 0);
				break;

			case HOMING_REV_RESET: {
//				static int prox_count = 0;
//				static bool prox_previous = false;
//				static bool initialized = false;
//
//				// Initialize on first entry
//				if (!initialized) {
//					prox_previous = prox;
//					prox_count = 0;
//					initialized = true;
//				}
//
//				// Move revolute motor clockwise at constant speed
//				rev_velocity_target = ZGX45RGG_150RPM_Constant.qd_max / 2;
//
//				// Count proximity sensor triggers (rising edge detection)
//				if (prox && !prox_previous) {
//					prox_count++;
//				}
//				prox_previous = prox;
//
//				// After reaching home, stop motor
//				if (prox_count >= 2) {
//					rev_velocity_target = 0.0f;
//					initialized = false;  // Reset for next time
//					homing_state = HOMING_PRIS_UP;
//				}
//
//		        prismatic_axis.position = prismatic_encoder.mm;
//		        revolute_axis.position = revolute_encoder.rads;
//
//		        velocity_control(0, rev_velocity_target);

				homing_state = HOMING_PRIS_UP;

				break;
			}

			case HOMING_PRIS_UP:
				// Move prismatic motor up to upper limit
				pris_velocity_target = -ZGX45RGG_400RPM_Constant.sd_max / 3.0f;

				if (up_photo) {
					pris_velocity_target = 0.0f;
					homing_state = HOMING_COMPLETE;
				}

		        prismatic_axis.position = prismatic_encoder.mm;
		        revolute_axis.position = revolute_encoder.rads;

		        velocity_control(pris_velocity_target, 0);
				break;

			case HOMING_COMPLETE:
				plotter_reset();

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

				// Reset ALL control variables
				revolute_axis.pos_error = 0.0f;
				revolute_axis.vel_error = 0.0f;
				prismatic_axis.pos_error = 0.0f;
				prismatic_axis.vel_error = 0.0f;

				// Reset position setpoints to current position
				prismatic_axis.position = prismatic_encoder.mm;
				revolute_axis.position = revolute_encoder.rads;

				// Reset trajectories and state
				prismatic_axis.trajectory_active = false;
				revolute_axis.trajectory_active = false;

				pris_velocity_target = 0.0f;
				rev_velocity_target = 0.0f;

				homing_state = HOMING_IDLE;
				break;

			default:
				// Unexpected state - reset to idle
				homing_state = HOMING_IDLE;
				break;
			}
		} else if (joystick_state != JOY_IDLE) {
			// Process joystick control if active
			process_joystick_control();
		} else {
			// Normal operation - update control loops
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
