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
typedef struct {
	float32_t position;
	float32_t velocity;
	float32_t acceleration;
	float32_t initial_pos;
	float32_t target_pos;
	float32_t pos_error;
	float32_t vel_error;
	float32_t kalman_velocity;
	float32_t input_voltage;
	float32_t command_pos;
	float32_t command_vel;
	float32_t dfd;
	float32_t ffd;
	bool trajectory_active;
	float32_t mm;
	float32_t deg;
} AxisState_t;

typedef enum {
	MOTION_IDLE = 0,
	MOTION_PEN_UP_DELAY,
	MOTION_PRISMATIC_ACTIVE,
	MOTION_REVOLUTE_ACTIVE,
	MOTION_PEN_DOWN_DELAY,
	MOTION_COMPLETE
} MotionSequenceState_t;

typedef enum {
	HOMING_IDLE = 0,
	HOMING_PEN_UP,
	HOMING_DELAY_AFTER_PEN_UP,
	HOMING_PRIS_DOWN_TO_LOW_PHOTO,
	HOMING_DELAY_AFTER_LOW_PHOTO,
	HOMING_PRIS_UP_TO_UP_PHOTO,
	HOMING_DELAY_AFTER_UP_PHOTO,
	HOMING_REV_TO_ZERO_DEG,
	HOMING_DELAY_AFTER_ZERO_DEG,
	HOMING_REV_CW_TO_PROX1,
	HOMING_DELAY_AFTER_PROX,
	HOMING_COMPLETE
} HomingState_t;

typedef enum {
	SAFETY_NORMAL = 0, SAFETY_SOFTWARE_EMERGENCY, SAFETY_HARDWARE_EMERGENCY
} SafetyState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRISMATIC_MAX_POS 300.0f
#define PRISMATIC_MIN_POS 0.0f
#define SEQUENCE_MAX_POINTS 6

#define HOMING_PRIS_VELOCITY 250.0f
#define HOMING_REV_VELOCITY 2.0f
#define HOMING_BACKUP_VELOCITY 50.0f

#define SAFETY_TOGGLE_PERIOD 1000
#define POSITION_CONTROL_DIVIDER 10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
AxisState_t prismatic_axis = { 0 };
AxisState_t revolute_axis = { 0 };

MotionSequenceState_t motion_sequence_state = MOTION_IDLE;
Trapezoidal_GenStruct prisGen, revGen;
Trapezoidal_EvaStruct prisEva, revEva;

uint8_t trajectory_sequence_index = 0;
const float32_t sequence_pris_points[SEQUENCE_MAX_POINTS] = { 175.0f, 95.0f,
		231.0f, 200.0f, 300.0f, 0.0f };
const float32_t sequence_rev_points[SEQUENCE_MAX_POINTS] = { 175.0f, 195.0f,
		95.0f, 300.0f, 150.0f, 0.0f };

volatile uint32_t motion_delay_timer = 0;
volatile uint32_t prox_count = 0;
volatile bool up_photo = false;
volatile bool low_photo = false;

HomingState_t homing_state = HOMING_IDLE;
bool first_startup = true;
bool homing_active = false;

SafetyState_t safety_state = SAFETY_NORMAL;
volatile uint32_t safety_toggle_timer = 0;
volatile bool pilot_light_state = false;
volatile bool hardware_emergency_triggered = false;

static bool rev_to_zero_trajectory_started = false;
static Trapezoidal_EvaStruct revZeroEva;
static Trapezoidal_GenStruct revZeroGen;
static float rev_zero_initial_pos;
static float rev_zero_target_pos;

volatile uint32_t position_control_tick = 0;
bool tuning_mode = true;

float normalized_position;
float movement_deg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void start_homing_sequence(bool is_startup);
void update_homing_sequence(void);

void start_combined_trajectory(float prismatic_target_mm,
		float revolute_target_deg);
void update_position_control(void);
void update_velocity_control(void);
void update_control_loops(void);
float normalize_angle(float angle_rad);
float calculate_movement_deg(float current_deg, float target_deg);

void check_safety_conditions(void);
void trigger_software_emergency(void);
void trigger_hardware_emergency(void);
void clear_emergency_state(void);
void update_safety_system(void);
bool is_emergency_active(void);
void emergency_stop_all_motors(void);
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

	prismatic_axis.position = prismatic_encoder.mm;
	revolute_axis.position = revolute_encoder.rads;

	safety_state = SAFETY_NORMAL;
	hardware_emergency_triggered = false;
	pilot_light_state = false;
	safety_toggle_timer = 0;
	position_control_tick = 0;

	if (first_startup) {
		start_homing_sequence(true);
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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
void start_homing_sequence(bool is_startup) {
	if (homing_active)
		return;

	// Check if already homed (up_photo and prox available)
	bool up_photo_detected = HAL_GPIO_ReadPin(UPPER_PHOTO_GPIO_Port,
	UPPER_PHOTO_Pin);

	// If system already homed and sensors are available, skip homing
	if (!is_startup && !first_startup && up_photo_detected && prox_count > 0) {
		return; // Already homed, no need to home again
	}

	homing_active = true;
	motion_sequence_state = MOTION_IDLE;
	prox_count = 0;
	up_photo = false;
	low_photo = false;

	if (is_startup || first_startup) {
		// STARTUP SEQUENCE: Skip HOMING_REV_TO_ZERO_DEG
		homing_state = HOMING_PEN_UP;
	} else {
		// MANUAL HOMING: Include HOMING_REV_TO_ZERO_DEG
		// Manual homing - check if already at up photo
		if (up_photo_detected) {
			homing_state = HOMING_REV_TO_ZERO_DEG; // Go to zero first, then find prox
		} else {
			homing_state = HOMING_PEN_UP;
		}
	}
}

void update_homing_sequence(void) {
	if (!homing_active)
		return;

	switch (homing_state) {
	case HOMING_PEN_UP:
		// Ensure pen is up
		plotter_pen_up();
		prismatic_axis.command_pos = 0.0f;
		revolute_axis.command_pos = 0.0f;
		motion_delay_timer = 0;
		homing_state = HOMING_DELAY_AFTER_PEN_UP;
		break;

	case HOMING_DELAY_AFTER_PEN_UP:
		// Stop motors and wait
		prismatic_axis.command_pos = 0.0f;
		revolute_axis.command_pos = 0.0f;
		motion_delay_timer++;
		if (motion_delay_timer >= 500) {
			// Check if already at low photo sensor
			low_photo = HAL_GPIO_ReadPin(LOWER_PHOTO_GPIO_Port,
			LOWER_PHOTO_Pin);

			if (low_photo) {
				// Already at low photo, skip moving down and go directly to delay
				motion_delay_timer = 0;
				homing_state = HOMING_DELAY_AFTER_LOW_PHOTO;
				low_photo = false; // Reset flag
				up_photo = false;  // Reset for next detection
			} else {
				// Not at low photo, need to move down
				homing_state = HOMING_PRIS_DOWN_TO_LOW_PHOTO;
			}
		}
		break;

	case HOMING_PRIS_DOWN_TO_LOW_PHOTO:
		// Move prismatic down at constant velocity
		prismatic_axis.vel_error = HOMING_PRIS_VELOCITY
				- prismatic_axis.kalman_velocity;
		prismatic_axis.command_pos = PWM_Satuation(
				PID_CONTROLLER_Compute(&prismatic_velocity_pid,
						prismatic_axis.vel_error),
				ZGX45RGG_400RPM_Constant.U_max,
				-ZGX45RGG_400RPM_Constant.U_max);

		// Add feedforward compensation during homing
		prismatic_axis.ffd = PRISMATIC_MOTOR_FFD_Compute(&prismatic_motor_ffd,
		HOMING_PRIS_VELOCITY / 1000.0f);
		prismatic_axis.dfd = PRISMATIC_MOTOR_DFD_Compute(&prismatic_motor_dfd,
				revolute_encoder.rads, 0.0f, prismatic_encoder.mm / 1000.0f);
		prismatic_axis.command_pos += prismatic_axis.ffd + prismatic_axis.dfd;

		prismatic_axis.command_pos = PWM_Satuation(prismatic_axis.command_pos,
				ZGX45RGG_400RPM_Constant.U_max,
				-ZGX45RGG_400RPM_Constant.U_max);

		if (low_photo) {
			// Found low photo, stop and start delay
			prismatic_axis.command_pos = 0.0f;
			revolute_axis.command_pos = 0.0f;
			motion_delay_timer = 0;
			homing_state = HOMING_DELAY_AFTER_LOW_PHOTO;
			low_photo = false; // Reset flag after use
			up_photo = false;  // Reset for next detection
		}
		break;

	case HOMING_DELAY_AFTER_LOW_PHOTO:
		// Stop motors and wait
		prismatic_axis.command_pos = 0.0f;
		revolute_axis.command_pos = 0.0f;
		motion_delay_timer++;
		if (motion_delay_timer >= 500) {
			homing_state = HOMING_PRIS_UP_TO_UP_PHOTO;
		}
		break;

	case HOMING_PRIS_UP_TO_UP_PHOTO:
		// Move prismatic up at constant velocity
		prismatic_axis.vel_error = -HOMING_PRIS_VELOCITY
				- prismatic_axis.kalman_velocity;
		prismatic_axis.command_pos = PWM_Satuation(
				PID_CONTROLLER_Compute(&prismatic_velocity_pid,
						prismatic_axis.vel_error),
				ZGX45RGG_400RPM_Constant.U_max,
				-ZGX45RGG_400RPM_Constant.U_max);

		// Add feedforward compensation during homing
		prismatic_axis.ffd = PRISMATIC_MOTOR_FFD_Compute(&prismatic_motor_ffd,
				-HOMING_PRIS_VELOCITY / 1000.0f);
		prismatic_axis.dfd = PRISMATIC_MOTOR_DFD_Compute(&prismatic_motor_dfd,
				revolute_encoder.rads, 0.0f, prismatic_encoder.mm / 1000.0f);
		prismatic_axis.command_pos += prismatic_axis.ffd + prismatic_axis.dfd;

		prismatic_axis.command_pos = PWM_Satuation(prismatic_axis.command_pos,
				ZGX45RGG_400RPM_Constant.U_max,
				-ZGX45RGG_400RPM_Constant.U_max);

		if (up_photo) {
			// Found up photo, stop and start delay before backup
			prismatic_axis.command_pos = 0.0f;
			revolute_axis.command_pos = 0.0f;
			motion_delay_timer = 0;
			homing_state = HOMING_DELAY_AFTER_UP_PHOTO;
			up_photo = false; // Reset flag after use
		}
		break;

	case HOMING_DELAY_AFTER_UP_PHOTO:
		// Stop motors and wait before starting backup procedure
		prismatic_axis.command_pos = 0.0f;
		revolute_axis.command_pos = 0.0f;
		motion_delay_timer++;
		if (motion_delay_timer >= 500) {
			if (first_startup) {
				// STARTUP: Skip REV_TO_ZERO_DEG, go directly to find prox sensor
				homing_state = HOMING_REV_CW_TO_PROX1;
				prox_count = 0; // Reset prox counter
			} else {
				// MANUAL HOMING: Go to 0° first (we know where it is from previous homing)
				homing_state = HOMING_REV_TO_ZERO_DEG;
				// Initialize trajectory variables for zero degree movement
				rev_to_zero_trajectory_started = false;
			}
		}
		break;

	case HOMING_REV_TO_ZERO_DEG:
		if (!rev_to_zero_trajectory_started) {
			// Clear/initialize the static structures
			memset(&revZeroEva, 0, sizeof(Trapezoidal_EvaStruct));
			memset(&revZeroGen, 0, sizeof(Trapezoidal_GenStruct));

			// Get current position and calculate shortest path to 0 degrees
			float current_rev_pos = revolute_encoder.rads;
			float normalized_current = normalize_angle(current_rev_pos);
			float current_deg = normalized_current * 180.0f / PI;

			// Calculate shortest movement to 0 degrees
			float target_deg = 0.0f;
			float movement_deg = calculate_movement_deg(current_deg,
					target_deg);

			// Convert movement to radians and apply to absolute position
			float movement_rad = movement_deg * PI / 180.0f;

			// Store initial and target positions
			rev_zero_initial_pos = current_rev_pos;
			rev_zero_target_pos = current_rev_pos + movement_rad;

			// Generate trajectory from current position to calculated target
			Trapezoidal_Generator(&revZeroGen, rev_zero_initial_pos,
					rev_zero_target_pos,
					ZGX45RGG_150RPM_Constant.traject_qd_max,
					ZGX45RGG_150RPM_Constant.traject_qdd_max);

			// Reset trajectory evaluation
			revZeroEva.t = 0.0f;
			revZeroEva.isFinised = false;

			rev_to_zero_trajectory_started = true;
		}

		// Update trajectory
		if (!revZeroEva.isFinised) {
			Trapezoidal_Evaluated(&revZeroGen, &revZeroEva,
					rev_zero_initial_pos, rev_zero_target_pos,
					ZGX45RGG_150RPM_Constant.traject_qd_max,
					ZGX45RGG_150RPM_Constant.traject_qdd_max);

			revolute_axis.position = revZeroEva.setposition;
			revolute_axis.velocity = revZeroEva.setvelocity;

			// Use normal revolute control with trajectory
			revolute_axis.pos_error = revolute_axis.position
					- normalize_angle(revolute_encoder.rads);

			// Ensure error uses the shortest path for control
			if (revolute_axis.pos_error > PI) {
				revolute_axis.pos_error -= 2.0f * PI;
			}
			if (revolute_axis.pos_error < -PI) {
				revolute_axis.pos_error += 2.0f * PI;
			}

			revolute_axis.command_vel = PWM_Satuation(
					PID_CONTROLLER_Compute(&revolute_position_pid,
							revolute_axis.pos_error),
					ZGX45RGG_150RPM_Constant.qd_max,
					-ZGX45RGG_150RPM_Constant.qd_max);

			// Add velocity feedforward for trajectory
			revolute_axis.vel_error = revolute_axis.command_vel
					+ revolute_axis.velocity - revolute_axis.kalman_velocity;

			revolute_axis.command_pos = PWM_Satuation(
					PID_CONTROLLER_Compute(&revolute_velocity_pid,
							revolute_axis.vel_error),
					ZGX45RGG_150RPM_Constant.U_max,
					-ZGX45RGG_150RPM_Constant.U_max);

			// Add feedforward compensation
			revolute_axis.ffd = REVOLUTE_MOTOR_FFD_Compute(&revolute_motor_ffd,
					revolute_axis.velocity);
			revolute_axis.dfd = REVOLUTE_MOTOR_DFD_Compute(&revolute_motor_dfd,
					revolute_encoder.rads, prismatic_encoder.mm / 1000.0f);
			revolute_axis.command_pos += revolute_axis.ffd + revolute_axis.dfd;

			revolute_axis.command_pos = PWM_Satuation(revolute_axis.command_pos,
					ZGX45RGG_150RPM_Constant.U_max,
					-ZGX45RGG_150RPM_Constant.U_max);

			if (revZeroEva.isFinised) {
				// Trajectory complete, stop and start delay
				revolute_axis.command_pos = 0.0f;
				prismatic_axis.command_pos = 0.0f;
				revolute_axis.velocity = 0.0f;
				revolute_axis.ffd = 0.0f;
				revolute_axis.dfd = 0.0f;

				motion_delay_timer = 0;
				homing_state = HOMING_DELAY_AFTER_ZERO_DEG;
				prox_count = 0; // Reset prox counter for next stage
				rev_to_zero_trajectory_started = false; // Reset for next time
			}
		}
		break;

	case HOMING_DELAY_AFTER_ZERO_DEG:
		// Stop motors and wait
		prismatic_axis.command_pos = 0.0f;
		revolute_axis.command_pos = 0.0f;
		motion_delay_timer++;
		if (motion_delay_timer >= 500) {
			homing_state = HOMING_REV_CW_TO_PROX1;
		}
		break;

	case HOMING_REV_CW_TO_PROX1:
		// Move revolute clockwise with velocity control until prox count = 1
		revolute_axis.vel_error = -HOMING_REV_VELOCITY
				- revolute_axis.kalman_velocity;
		revolute_axis.command_pos = PWM_Satuation(
				PID_CONTROLLER_Compute(&revolute_velocity_pid,
						revolute_axis.vel_error),
				ZGX45RGG_150RPM_Constant.U_max,
				-ZGX45RGG_150RPM_Constant.U_max);

		// Add feedforward compensation during homing
		revolute_axis.ffd = REVOLUTE_MOTOR_FFD_Compute(&revolute_motor_ffd,
				-HOMING_REV_VELOCITY);
		revolute_axis.dfd = REVOLUTE_MOTOR_DFD_Compute(&revolute_motor_dfd,
				revolute_encoder.rads, prismatic_encoder.mm / 1000.0f);
		revolute_axis.command_pos += revolute_axis.ffd + revolute_axis.dfd;

		revolute_axis.command_pos = PWM_Satuation(revolute_axis.command_pos,
				ZGX45RGG_150RPM_Constant.U_max,
				-ZGX45RGG_150RPM_Constant.U_max);

		if (prox_count >= 1) {
			// Found prox sensor, stop and start delay
			prismatic_axis.command_pos = 0.0f;
			revolute_axis.command_pos = 0.0f;
			motion_delay_timer = 0;
			homing_state = HOMING_DELAY_AFTER_PROX;
		}
		break;

	case HOMING_DELAY_AFTER_PROX:
		// Stop motors and wait
		prismatic_axis.command_pos = 0.0f;
		revolute_axis.command_pos = 0.0f;
		motion_delay_timer++;
		if (motion_delay_timer >= 500) {
			homing_state = HOMING_COMPLETE;
		}
		break;

	case HOMING_COMPLETE:
		memset(&prismatic_axis, 0, sizeof(AxisState_t));
		memset(&revolute_axis, 0, sizeof(AxisState_t));

		Kalman_Reset(&revolute_kalman);
		MotorKalman_Reset(&prismatic_kalman);

		// Prismatic: 0.0mm (up photo position - after backup)
		prismatic_encoder.mm = 0.0f;
		prismatic_encoder.pulses = 0;
		prismatic_encoder.revs = 0.0f;
		prismatic_encoder.rads = 0.0f;

		// Revolute: -5.18° (prox sensor position)
		float target_home_deg = -5.18f;
		float target_home_rad = target_home_deg * PI / 180.0f;
		revolute_encoder.rads = target_home_rad;
		revolute_encoder.revs = target_home_rad / (2.0f * PI);
		revolute_encoder.pulses = (int32_t) (target_home_rad * ENC_PPR
				* MOTOR2_RATIO / (2.0f * PI));
		revolute_encoder.mm = 0.0f;

		// Set axis position setpoints to match encoder values
		prismatic_axis.position = 0.0f;
		revolute_axis.position = target_home_rad;

		// Complete homing sequence
		homing_active = false;
		first_startup = false;
		homing_state = HOMING_IDLE;

		// Reset all flags and counters
		up_photo = false;
		low_photo = false;
		prox_count = 0;
		break;

	case HOMING_IDLE:
	default:
		break;
	}
}

float normalize_angle(float angle_rad) {
	float result = fmodf(angle_rad, 2.0f * PI);
	if (result < 0.0f) {
		result += 2.0f * PI;
	}
	return result;
}

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
		// If we need to cross the 180° boundary
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

void start_combined_trajectory(float prismatic_target_mm,
		float revolute_target_deg) {
	if (is_emergency_active() || homing_active) {
		return; // Don't start trajectory if in emergency or homing
	}

	float pris_current = prismatic_encoder.mm;
	float rev_current = revolute_encoder.rads;

	prisEva.t = 0.0f;
	prisEva.isFinised = false;
	revEva.t = 0.0f;
	revEva.isFinised = false;

	prismatic_axis.initial_pos = pris_current;
	revolute_axis.initial_pos = rev_current;
	prismatic_axis.target_pos = fminf(
			fmaxf(prismatic_target_mm, PRISMATIC_MIN_POS), PRISMATIC_MAX_POS);

	float normalized_current = normalize_angle(rev_current);
	float current_deg = normalized_current * 180.0f / PI;
	movement_deg = calculate_movement_deg(current_deg, revolute_target_deg);
	float movement_rad = movement_deg * PI / 180.0f;
	revolute_axis.target_pos = revolute_axis.initial_pos + movement_rad;

	Trapezoidal_Generator(&prisGen, prismatic_axis.initial_pos,
			prismatic_axis.target_pos, ZGX45RGG_400RPM_Constant.traject_sd_max,
			ZGX45RGG_400RPM_Constant.traject_sdd_max);

	prismatic_axis.trajectory_active = true;
	revolute_axis.trajectory_active = false;

	plotter_pen_up();
	motion_delay_timer = 0;
	motion_sequence_state = MOTION_PEN_UP_DELAY;
}

void update_position_control(void) {
	prismatic_axis.pos_error = prismatic_axis.position - prismatic_encoder.mm;
	prismatic_axis.command_vel = PWM_Satuation(
			PID_CONTROLLER_Compute(&prismatic_position_pid,
					prismatic_axis.pos_error), ZGX45RGG_400RPM_Constant.sd_max,
			-ZGX45RGG_400RPM_Constant.sd_max);

	float normalized_position = normalize_angle(revolute_encoder.rads);
	revolute_axis.pos_error = revolute_axis.position - normalized_position;

	if (revolute_axis.pos_error > PI)
		revolute_axis.pos_error -= 2.0f * PI;
	if (revolute_axis.pos_error < -PI)
		revolute_axis.pos_error += 2.0f * PI;

	revolute_axis.command_vel = PWM_Satuation(
			PID_CONTROLLER_Compute(&revolute_position_pid,
					revolute_axis.pos_error), ZGX45RGG_150RPM_Constant.qd_max,
			-ZGX45RGG_150RPM_Constant.qd_max);
}

void update_velocity_control(void) {
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

	if (prismatic_axis.trajectory_active) {
		prismatic_axis.ffd = PRISMATIC_MOTOR_FFD_Compute(&prismatic_motor_ffd,
				prismatic_axis.velocity / 1000.0f);
		prismatic_axis.dfd = PRISMATIC_MOTOR_DFD_Compute(&prismatic_motor_dfd,
				revolute_encoder.rads, revolute_axis.velocity,
				prismatic_encoder.mm / 1000.0f);
	} else {
		prismatic_axis.ffd = 0.0f;
		prismatic_axis.dfd = 0.0f;
	}

	prismatic_axis.command_pos += prismatic_axis.ffd + prismatic_axis.dfd;
	prismatic_axis.command_pos = PWM_Satuation(prismatic_axis.command_pos,
			ZGX45RGG_400RPM_Constant.U_max, -ZGX45RGG_400RPM_Constant.U_max);

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

	if (revolute_axis.trajectory_active) {
		revolute_axis.ffd = REVOLUTE_MOTOR_FFD_Compute(&revolute_motor_ffd,
				revolute_axis.velocity);
		revolute_axis.dfd = REVOLUTE_MOTOR_DFD_Compute(&revolute_motor_dfd,
				revolute_encoder.rads, prismatic_encoder.mm / 1000.0f);
	} else {
		revolute_axis.ffd = 0.0f;
		revolute_axis.dfd = REVOLUTE_MOTOR_DFD_Compute(&revolute_motor_dfd,
				revolute_encoder.rads, prismatic_encoder.mm / 1000.0f);
	}

	revolute_axis.command_pos += revolute_axis.ffd + revolute_axis.dfd;
	revolute_axis.command_pos = PWM_Satuation(revolute_axis.command_pos,
			ZGX45RGG_150RPM_Constant.U_max, -ZGX45RGG_150RPM_Constant.U_max);

	MDXX_set_range(&prismatic_motor, 2000, prismatic_axis.command_pos);
	MDXX_set_range(&revolute_motor, 2000, revolute_axis.command_pos);
}

void update_control_loops(void) {
	normalized_position = normalize_angle(revolute_encoder.rads);

	if (is_emergency_active()) {
		emergency_stop_all_motors();
		prismatic_axis.mm = prismatic_encoder.mm;
		revolute_axis.deg = UnitConverter_angle(&converter_system,
				normalized_position, UNIT_RADIAN, UNIT_DEGREE);
		return;
	}

	if (homing_active) {
		update_homing_sequence();
		MDXX_set_range(&prismatic_motor, 2000, prismatic_axis.command_pos);
		MDXX_set_range(&revolute_motor, 2000, revolute_axis.command_pos);
		prismatic_axis.mm = prismatic_encoder.mm;
		revolute_axis.deg = UnitConverter_angle(&converter_system,
				normalized_position, UNIT_RADIAN, UNIT_DEGREE);
		return;
	}

	switch (motion_sequence_state) {
	case MOTION_PEN_UP_DELAY:
		if (++motion_delay_timer >= 1500) {
			motion_sequence_state = MOTION_PRISMATIC_ACTIVE;
		}
		break;

	case MOTION_PRISMATIC_ACTIVE:
		if (prismatic_axis.trajectory_active && !prisEva.isFinised) {
			Trapezoidal_Evaluated(&prisGen, &prisEva,
					prismatic_axis.initial_pos, prismatic_axis.target_pos,
					ZGX45RGG_400RPM_Constant.traject_sd_max,
					ZGX45RGG_400RPM_Constant.traject_sdd_max);

			prismatic_axis.position = prisEva.setposition;
			prismatic_axis.velocity = prisEva.setvelocity;

			if (prisEva.isFinised) {
				prismatic_axis.trajectory_active = false;
				prismatic_axis.position = prisEva.setposition;
				prismatic_axis.velocity = 0.0f;

				Trapezoidal_Generator(&revGen, revolute_axis.initial_pos,
						revolute_axis.target_pos,
						ZGX45RGG_150RPM_Constant.traject_qd_max,
						ZGX45RGG_150RPM_Constant.traject_qdd_max);

				revolute_axis.trajectory_active = true;
				motion_sequence_state = MOTION_REVOLUTE_ACTIVE;
			}
		}
		break;

	case MOTION_REVOLUTE_ACTIVE:
		if (revolute_axis.trajectory_active && !revEva.isFinised) {
			Trapezoidal_Evaluated(&revGen, &revEva, revolute_axis.initial_pos,
					revolute_axis.target_pos,
					ZGX45RGG_150RPM_Constant.traject_qd_max,
					ZGX45RGG_150RPM_Constant.traject_qdd_max);

			revolute_axis.position = revEva.setposition;
			revolute_axis.velocity = revEva.setvelocity;

			if (revEva.isFinised) {
				revolute_axis.trajectory_active = false;
				revolute_axis.position = revEva.setposition;
				revolute_axis.velocity = 0.0f;

				PID_CONTROLLER_Reset(&revolute_position_pid);
				PID_CONTROLLER_Reset(&revolute_velocity_pid);

				motion_delay_timer = 0;
				motion_sequence_state = MOTION_PEN_DOWN_DELAY;
			}
		}
		break;

	case MOTION_PEN_DOWN_DELAY:
		if (++motion_delay_timer >= 1500) {
			plotter_pen_down();
			motion_sequence_state = MOTION_COMPLETE;
		}
		break;

	case MOTION_COMPLETE:
		motion_sequence_state = MOTION_IDLE;
		break;

	default:
		break;
	}

	prismatic_axis.mm = prismatic_encoder.mm;
	revolute_axis.deg = UnitConverter_angle(&converter_system,
			normalized_position, UNIT_RADIAN, UNIT_DEGREE);
}

void check_safety_conditions(void) {
	if (tuning_mode || safety_state != SAFETY_NORMAL || homing_active)
		return;

	if (up_photo && prismatic_axis.command_pos < 0.0f) {
		trigger_software_emergency();
		return;
	}

	if (low_photo && prismatic_axis.command_pos > 0.0f) {
		trigger_software_emergency();
		return;
	}
}

void trigger_software_emergency(void) {
	if (safety_state == SAFETY_NORMAL) {
		safety_state = SAFETY_SOFTWARE_EMERGENCY;
		emergency_stop_all_motors();
		safety_toggle_timer = 0;
		pilot_light_state = false;
		motion_sequence_state = MOTION_IDLE;
		prismatic_axis.trajectory_active = false;
		revolute_axis.trajectory_active = false;
	}
}

void trigger_hardware_emergency(void) {
	safety_state = SAFETY_HARDWARE_EMERGENCY;
	hardware_emergency_triggered = true;
	emergency_stop_all_motors();
	safety_toggle_timer = 0;
	pilot_light_state = false;
	homing_active = false;
	homing_state = HOMING_IDLE;
	motion_sequence_state = MOTION_IDLE;
	prismatic_axis.trajectory_active = false;
	revolute_axis.trajectory_active = false;
}

void clear_emergency_state(void) {
	safety_state = SAFETY_NORMAL;
	hardware_emergency_triggered = false;
	safety_toggle_timer = 0;
	pilot_light_state = false;
	HAL_GPIO_WritePin(PILOT_GPIO_Port, PILOT_Pin, GPIO_PIN_RESET);
	up_photo = false;
	low_photo = false;
}

void emergency_stop_all_motors(void) {
	MDXX_set_range(&prismatic_motor, 2000, 0);
	MDXX_set_range(&revolute_motor, 2000, 0);

	prismatic_axis.command_pos = 0.0f;
	revolute_axis.command_pos = 0.0f;
	prismatic_axis.command_vel = 0.0f;
	revolute_axis.command_vel = 0.0f;

	PID_CONTROLLER_Reset(&prismatic_position_pid);
	PID_CONTROLLER_Reset(&prismatic_velocity_pid);
	PID_CONTROLLER_Reset(&revolute_position_pid);
	PID_CONTROLLER_Reset(&revolute_velocity_pid);
}

void update_safety_system(void) {
	if (safety_state == SAFETY_SOFTWARE_EMERGENCY) {
		if (++safety_toggle_timer >= SAFETY_TOGGLE_PERIOD) {
			HAL_GPIO_TogglePin(PILOT_GPIO_Port, PILOT_Pin);
			pilot_light_state = !pilot_light_state;
			safety_toggle_timer = 0;
		}
	}

	if (safety_state == SAFETY_HARDWARE_EMERGENCY) {
		if (HAL_GPIO_ReadPin(EMER_GPIO_Port, EMER_Pin) == GPIO_PIN_SET) {
			if (++safety_toggle_timer >= SAFETY_TOGGLE_PERIOD) {
				HAL_GPIO_TogglePin(PILOT_GPIO_Port, PILOT_Pin);
				pilot_light_state = !pilot_light_state;
				safety_toggle_timer = 0;
			}
		} else {
			HAL_GPIO_WritePin(PILOT_GPIO_Port, PILOT_Pin, GPIO_PIN_RESET);
			pilot_light_state = false;
			safety_toggle_timer = 0;
		}
	}

	if (safety_state == SAFETY_NORMAL) {
		HAL_GPIO_WritePin(PILOT_GPIO_Port, PILOT_Pin, GPIO_PIN_RESET);
		pilot_light_state = false;
		safety_toggle_timer = 0;
	}
}

bool is_emergency_active(void) {
	return (safety_state != SAFETY_NORMAL);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == PROX_Pin) {
		prox_count++;
		return;
	}

	if (GPIO_Pin == UPPER_PHOTO_Pin) {
		up_photo = true;
		return;
	}

	if (GPIO_Pin == LOWER_PHOTO_Pin) {
		low_photo = true;
		return;
	}

	if (GPIO_Pin == EMER_Pin) {
		trigger_hardware_emergency();
		return;
	}

	if (GPIO_Pin == J1_Pin) {
		if (!is_emergency_active() && !homing_active
				&& motion_sequence_state == MOTION_IDLE) {
			start_combined_trajectory(
					sequence_pris_points[trajectory_sequence_index],
					sequence_rev_points[trajectory_sequence_index]);
			trajectory_sequence_index = (trajectory_sequence_index + 1)
					% SEQUENCE_MAX_POINTS;
		}
		return;
	}

	if (GPIO_Pin == J3_Pin) {
		if (!is_emergency_active() && !homing_active
				&& motion_sequence_state == MOTION_IDLE) {
			start_homing_sequence(false);
		}
		return;
	}

	if (GPIO_Pin == J4_Pin) {
		if (is_emergency_active()) {
			clear_emergency_state();
			first_startup = true;
			start_homing_sequence(true);
		}
		return;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		plotter_update_sensors();

		QEI_get_diff_count(&prismatic_encoder);
		QEI_compute_data(&prismatic_encoder);
		QEI_get_diff_count(&revolute_encoder);
		QEI_compute_data(&revolute_encoder);

		revolute_axis.input_voltage = mapf(revolute_axis.command_pos, -65535.0f,
				65535.0f, -12.0f, 12.0f);
		revolute_axis.kalman_velocity = SteadyStateKalmanFilter(
				&revolute_kalman, revolute_axis.input_voltage,
				revolute_encoder.rads);

		if (isnan(revolute_axis.kalman_velocity)) {
			revolute_axis.kalman_velocity = 0.0f;
		}

		prismatic_axis.input_voltage = mapf(prismatic_axis.command_pos,
				-65535.0f, 65535.0f, -12.0f, 12.0f);
		prismatic_axis.kalman_velocity = MotorKalman_Estimate(&prismatic_kalman,
				prismatic_axis.input_voltage, prismatic_encoder.rads)
				* Disturbance_Constant.prismatic_pulley_radius * 1000.0f;

		if (isnan(prismatic_axis.kalman_velocity)) {
			prismatic_axis.kalman_velocity = 0.0f;
		}

		if (++position_control_tick >= POSITION_CONTROL_DIVIDER) {
			position_control_tick = 0;

			if (!homing_active && (!is_emergency_active() || tuning_mode)) {
				update_position_control();
			}
		}

		if (!homing_active && (!is_emergency_active() || tuning_mode)) {
			update_velocity_control();
		}

		update_safety_system();

		if (!is_emergency_active() || tuning_mode) {
			check_safety_conditions();
		}

		update_control_loops();
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
