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
	float32_t accel_show;
} AxisState_t;

typedef enum {
	MOTION_IDLE = 0,
	MOTION_PEN_UP_DELAY,
	MOTION_PRISMATIC_ACTIVE,
	MOTION_REVOLUTE_ACTIVE,
	MOTION_BOTH_AXES_ACTIVE,
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

typedef enum {
	JOY_MODE_IDLE = 0, JOY_MODE_INITIAL_CONTROL, // New state: manual control before saving starts
	JOY_MODE_MANUAL_CONTROL,     // Saving positions state
	JOY_MODE_POSITION_SAVED,
	JOY_MODE_PLAYBACK,
	JOY_MODE_COMPLETE
} JoyModeState_t;

typedef struct {
	float prismatic_pos;
	float revolute_pos;
} SavedPosition_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRISMATIC_MAX_POS 300.0f
#define PRISMATIC_MIN_POS 0.0f
#define SEQUENCE_MAX_POINTS 6

#define HOMING_PRIS_VELOCITY 250.0f
#define HOMING_REV_VELOCITY 2.0f

#define SAFETY_TOGGLE_PERIOD 250
#define POSITION_CONTROL_DIVIDER 10

#define JOY_MODE_MAX_POSITIONS 10
#define JOY_MODE_VELOCITY_THRESHOLD 40.0f
#define JOY_MODE_CONSTANT_VELOCITY_PRIS 150.0f
#define JOY_MODE_CONSTANT_VELOCITY_REV 3.0f
#define JOY_MODE_PILOT_TOGGLE_PERIOD 1000
#define JOY_MODE_PLAYBACK_DELAY 2000
#define JOY_MODE_B2_DEBOUNCE_TIME 50
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

//const float32_t sequence_pris_points[SEQUENCE_MAX_POINTS] = { 0.0f };

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

volatile uint32_t position_control_tick = 0;
bool tuning_mode = true;

float normalized_position;
float movement_deg;

JoyModeState_t joy_mode_state = JOY_MODE_IDLE;
bool joy_mode_active = false;
SavedPosition_t saved_positions[JOY_MODE_MAX_POSITIONS];
uint8_t saved_position_count = 0;
uint8_t playback_position_index = 0;
volatile uint32_t joy_mode_pilot_timer = 0;
volatile bool joy_mode_pilot_state = false;
volatile uint32_t joy_mode_playback_timer = 0;
bool joy_mode_b2_pressed = false;
bool joy_mode_b2_last_state = false;

int check[10];
uint16_t b2S[2];

//100 point
uint8_t j1_cycle_count = 0;
bool j1_going_to_target = true;
bool j1_active = false;
const float32_t J1_TARGET_PRIS = 200.0f;
const float32_t J1_TARGET_REV = 90.0f;
static uint32_t j1_interrupt_last_time = 0;
const uint32_t J1_INTERRUPT_DEBOUNCE_MS = 500;
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

void check_emergency_button(void);
void check_safety_conditions(void);
void trigger_software_emergency(void);
void trigger_hardware_emergency(void);
void clear_emergency_state(void);
void update_safety_system(void);
bool is_emergency_active(void);
void emergency_stop_all_motors(void);

void enter_joy_mode(void);
void exit_joy_mode(void);
void update_joy_mode(void);
void save_current_position(void);
void start_position_playback(void);
void update_joy_mode_pilot_light(void);
void reset_joy_mode_data(void);
void update_joy_mode_velocity_control(void);
void handle_b2_button_polling(void);

void modbus_working(void);
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
		// Check if already at home position
		bool up_photo_detected = HAL_GPIO_ReadPin(UPPER_PHOTO_GPIO_Port,
		UPPER_PHOTO_Pin);
		bool prox_detected = HAL_GPIO_ReadPin(PROX_GPIO_Port, PROX_Pin);

		if (up_photo_detected && prox_detected) {
			// Already at home position - no need to home
			first_startup = false;
			homing_active = false;
			homing_state = HOMING_IDLE;

			// Clear any sensor flags
			up_photo = false;
			low_photo = false;
			prox_count = 0;

			// Set motion to idle
			motion_sequence_state = MOTION_IDLE;

		} else {
			start_homing_sequence(true);
		}
	}
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		handle_b2_button_polling();
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
	//modbus set home state
	registerFrame[R_Theta_Status].U16 = 1;
	if (homing_active)
		return;

	// Check current sensor states
	bool up_photo_detected = HAL_GPIO_ReadPin(UPPER_PHOTO_GPIO_Port,
	UPPER_PHOTO_Pin);
	bool prox_detected = HAL_GPIO_ReadPin(PROX_GPIO_Port, PROX_Pin);

	// Different logic for startup vs manual homing
	if (is_startup || first_startup) {
		// STARTUP LOGIC: Skip homing if already at home position
		if (up_photo_detected && prox_detected) {
			// Already at home position - no need to home
			homing_active = false;
			homing_state = HOMING_IDLE;
			first_startup = false;

			// Clear sensor flags
			up_photo = false;
			low_photo = false;
			prox_count = 0;

			// Set motion to idle
			motion_sequence_state = MOTION_IDLE;
			return;
		}

		// Not at home - start startup homing sequence (skip zero degrees)
		homing_active = true;
		motion_sequence_state = MOTION_IDLE;
		prox_count = 0;
		up_photo = false;
		low_photo = false;
		homing_state = HOMING_PEN_UP;

	} else {
		// MANUAL HOMING LOGIC: More sophisticated behavior
		if (up_photo_detected && prox_detected) {
			// Already perfectly homed - skip homing completely
			homing_active = false;
			homing_state = HOMING_IDLE;
			first_startup = false;  // ← ADDED THIS LINE
			up_photo = false;
			low_photo = false;
			prox_count = 0;
			motion_sequence_state = MOTION_IDLE;
			return;

			// Option B: Still run zero-degree calibration (uncomment below instead)
			/*
			 homing_active = true;
			 motion_sequence_state = MOTION_IDLE;
			 prox_count = 0;
			 up_photo = false;
			 low_photo = false;
			 homing_state = HOMING_REV_TO_ZERO_DEG;
			 rev_to_zero_trajectory_started = false;
			 */
		} else if (up_photo_detected && !prox_detected) {
			// At up photo but not at prox - go to zero degrees first
			homing_active = true;
			motion_sequence_state = MOTION_IDLE;
			prox_count = 0;
			up_photo = false;
			low_photo = false;
			homing_state = HOMING_REV_TO_ZERO_DEG;
			rev_to_zero_trajectory_started = false;
		} else {
			// Not at up photo - start full homing sequence
			homing_active = true;
			motion_sequence_state = MOTION_IDLE;
			prox_count = 0;
			up_photo = false;
			low_photo = false;
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
				// STARTUP: Check if prox is already detected before searching
				bool prox_detected = HAL_GPIO_ReadPin(PROX_GPIO_Port, PROX_Pin);

				if (prox_detected) {
					// Already at prox - skip search and go to completion
					motion_delay_timer = 0;
					homing_state = HOMING_DELAY_AFTER_PROX;
					prox_count = 1; // Set count to indicate prox found
				} else {
					// Not at prox - search for it
					homing_state = HOMING_REV_CW_TO_PROX1;
					prox_count = 0; // Reset prox counter
				}
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
			// Get current prismatic position (keep it where it is)
			float current_pris_pos = prismatic_encoder.mm;

			// Start combined trajectory to move revolute to 0° while keeping prismatic position
			start_combined_trajectory(current_pris_pos, 0.0f);

			rev_to_zero_trajectory_started = true;
		}

		// Wait for trajectory to complete
		if (motion_sequence_state == MOTION_IDLE) {
			// Trajectory completed, move to next homing state
			motion_delay_timer = 0;
			homing_state = HOMING_DELAY_AFTER_ZERO_DEG;
			prox_count = 0; // Reset prox counter for next stage
			rev_to_zero_trajectory_started = false; // Reset for next time
		}
		break;

	case HOMING_DELAY_AFTER_ZERO_DEG:
		// Stop motors and wait - let normal control handle this
		motion_delay_timer++;
		if (motion_delay_timer >= 500) {
			// CHECK IF PROX IS ALREADY DETECTED BEFORE STARTING SEARCH
			bool prox_detected = HAL_GPIO_ReadPin(PROX_GPIO_Port, PROX_Pin);

			if (prox_detected) {
				// Already at proximity sensor - skip search and go directly to completion
				motion_delay_timer = 0;
				homing_state = HOMING_DELAY_AFTER_PROX;
				prox_count = 1; // Set count to indicate prox found
			} else {
				// Not at prox - need to search for it
				homing_state = HOMING_REV_CW_TO_PROX1;
				prox_count = 0; // Reset counter for search
			}
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
		revolute_axis.dfd = 0.0;

		revolute_axis.command_pos += revolute_axis.ffd;

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
		NVIC_SystemReset();
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


	bool allow_during_homing = (homing_active
			&& homing_state == HOMING_REV_TO_ZERO_DEG);

	if (is_emergency_active() || (homing_active && !allow_during_homing)) {
		return;
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

	// Check if we're in HOMING_REV_TO_ZERO_DEG mode
	bool is_homing_zero_deg = (homing_active
			&& homing_state == HOMING_REV_TO_ZERO_DEG);

	if (is_homing_zero_deg) {

		// HOMING_REV_TO_ZERO_DEG: Only generate revolute trajectory, skip prismatic

		// Don't generate prismatic trajectory at all
		prismatic_axis.trajectory_active = false;
		prismatic_axis.position = pris_current;  // Hold current position
		prismatic_axis.velocity = 0.0f;
		// Only generate revolute trajectory
		check[4]++;
		Trapezoidal_Generator(&revGen, revolute_axis.initial_pos,
				revolute_axis.target_pos,
				ZGX45RGG_150RPM_Constant.traject_qd_max,
				ZGX45RGG_150RPM_Constant.traject_qdd_max);

		revolute_axis.trajectory_active = false;  // Will be activated later

		// Set motion sequence to skip prismatic phase
		plotter_pen_up();
		motion_delay_timer = 0;
		motion_sequence_state = MOTION_PEN_UP_DELAY; // Will skip to revolute directly

	} else {

		// NORMAL TRAJECTORY: Generate both prismatic and revolute trajectories
		// Generate prismatic trajectory
		check[5]++;

		Trapezoidal_Generator(&prisGen, prismatic_axis.initial_pos,
				prismatic_axis.target_pos,
				ZGX45RGG_400RPM_Constant.traject_sd_max,
				ZGX45RGG_400RPM_Constant.traject_sdd_max);

		Trapezoidal_Generator(&revGen, revolute_axis.initial_pos,
		            revolute_axis.target_pos,
		            ZGX45RGG_150RPM_Constant.traject_qd_max,
		            ZGX45RGG_150RPM_Constant.traject_qdd_max);

		prismatic_axis.trajectory_active = false;
		revolute_axis.trajectory_active = false;

		plotter_pen_up();
		motion_delay_timer = 0;
		motion_sequence_state = MOTION_PEN_UP_DELAY;

		//modbus reset state
		registerFrame[BaseSystem_Status].U16 = 0;
		registerFrame[R_Theta_Status].U16 = 0;
	}
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

	static float ffd_filtered = 0.0f;
	static float dfd_filtered = 0.0f;

	ffd_filtered = 0.8f * ffd_filtered + 0.2f * revolute_axis.ffd;
	dfd_filtered = 0.8f * dfd_filtered + 0.2f * revolute_axis.dfd;

	revolute_axis.command_pos += 0.01 * (dfd_filtered + ffd_filtered);

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

	// Handle all homing states except HOMING_REV_TO_ZERO_DEG with direct motor control
	if (homing_active && homing_state != HOMING_REV_TO_ZERO_DEG) {
		update_homing_sequence();
		MDXX_set_range(&prismatic_motor, 2000, prismatic_axis.command_pos);
		MDXX_set_range(&revolute_motor, 2000, revolute_axis.command_pos);
		prismatic_axis.mm = prismatic_encoder.mm;
		revolute_axis.deg = UnitConverter_angle(&converter_system,
				normalized_position, UNIT_RADIAN, UNIT_DEGREE);
		return;
	}

	// Handle HOMING_REV_TO_ZERO_DEG: use trajectory system + check completion
	if (homing_active && homing_state == HOMING_REV_TO_ZERO_DEG) {
		// First, update the homing sequence to handle trajectory start/completion
		update_homing_sequence();

		// If still in HOMING_REV_TO_ZERO_DEG after update, continue with trajectory control
		if (homing_state == HOMING_REV_TO_ZERO_DEG) {
			// Let the trajectory system handle the motion
			// Fall through to the switch statement below
		} else {
			// Homing sequence advanced to next state, return
			prismatic_axis.mm = prismatic_encoder.mm;
			revolute_axis.deg = UnitConverter_angle(&converter_system,
					normalized_position, UNIT_RADIAN, UNIT_DEGREE);
			return;
		}
	}
	//100 point
	if (j1_active && motion_sequence_state == MOTION_IDLE) {
		if (j1_going_to_target) {

			// check if finish goto target then goto 0
			j1_going_to_target = false;
			start_combined_trajectory(0.0f, 0.0f);
		} else {

			j1_cycle_count++;

			if (j1_cycle_count >= 100) {
				//finish 100
				j1_active = false;
				j1_cycle_count = 0;
			} else {
				// start again
				j1_going_to_target = true;
				start_combined_trajectory(J1_TARGET_PRIS, J1_TARGET_REV);
			}
		}
	}

	// Motion sequence handling
	switch (motion_sequence_state) {
	case MOTION_PEN_UP_DELAY:
		if (++motion_delay_timer >= 1500) {
			// Check if we're in HOMING_REV_TO_ZERO_DEG mode
			if (homing_active && homing_state == HOMING_REV_TO_ZERO_DEG) {
				// Skip prismatic phase entirely - go directly to revolute motion
				revolute_axis.trajectory_active = true;
				motion_sequence_state = MOTION_REVOLUTE_ACTIVE;

				// Ensure prismatic stays stationary
				prismatic_axis.trajectory_active = false;
				prismatic_axis.position = prismatic_encoder.mm;
				prismatic_axis.velocity = 0.0f;
			} else {
				// Normal trajectory - activate prismatic first
				prismatic_axis.trajectory_active = true;
				revolute_axis.trajectory_active = true;
				motion_sequence_state = MOTION_BOTH_AXES_ACTIVE;
			}
		}
		break;

	case MOTION_BOTH_AXES_ACTIVE:
	        {
	            bool pris_finished = true;  // Default to true for homing case
	            bool rev_finished = false;

	            // Handle prismatic axis (skip if in homing mode)
	            if (!(homing_active && homing_state == HOMING_REV_TO_ZERO_DEG)) {
	                if (prismatic_axis.trajectory_active && !prisEva.isFinised) {
	                    Trapezoidal_Evaluated(&prisGen, &prisEva,
	                            prismatic_axis.initial_pos, prismatic_axis.target_pos,
	                            ZGX45RGG_400RPM_Constant.traject_sd_max,
	                            ZGX45RGG_400RPM_Constant.traject_sdd_max);

	                    prismatic_axis.position = prisEva.setposition;
	                    prismatic_axis.velocity = prisEva.setvelocity;
	                    pris_finished = prisEva.isFinised;

	                    if (prisEva.isFinised) {
	                        prismatic_axis.trajectory_active = false;
	                        prismatic_axis.position = prisEva.setposition;
	                        prismatic_axis.velocity = 0.0f;
	                    }
	                }
	            }

	            // Handle revolute axis
	            if (revolute_axis.trajectory_active && !revEva.isFinised) {
	                Trapezoidal_Evaluated(&revGen, &revEva, revolute_axis.initial_pos,
	                        revolute_axis.target_pos,
	                        ZGX45RGG_150RPM_Constant.traject_qd_max,
	                        ZGX45RGG_150RPM_Constant.traject_qdd_max);

	                revolute_axis.position = revEva.setposition;
	                revolute_axis.velocity = revEva.setvelocity;
	                rev_finished = revEva.isFinised;

	                if (revEva.isFinised) {
	                    revolute_axis.trajectory_active = false;
	                    revolute_axis.position = revEva.setposition;
	                    revolute_axis.velocity = 0.0f;
	                }
	            }

	            // Check if BOTH axes are finished (or only revolute for homing)
	            if (pris_finished && rev_finished) {
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

void check_emergency_button(void) {
	// Read current state of emergency button
	bool emer_pressed = HAL_GPIO_ReadPin(EMER_GPIO_Port, EMER_Pin);

	// If emergency button is pressed (assuming active high)
	// Adjust the logic based on your hardware:
	// - If button is active HIGH when pressed: use == GPIO_PIN_SET
	// - If button is active LOW when pressed: use == GPIO_PIN_RESET

	if (emer_pressed == GPIO_PIN_RESET) {  // Assuming active high
		// Emergency button is pressed - trigger hardware emergency
		if (safety_state != SAFETY_HARDWARE_EMERGENCY) {
			trigger_hardware_emergency();
		}
	}

	// Optional: If you want to auto-clear when button is released
	// (usually not recommended for safety reasons)
	/*
	 else {
	 // Emergency button is released
	 if (safety_state == SAFETY_HARDWARE_EMERGENCY && hardware_emergency_triggered) {
	 // Auto-clear emergency when button released (NOT RECOMMENDED)
	 // clear_emergency_state();
	 }
	 }
	 */
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

		// Exit joy mode if active
		if (joy_mode_active) {
			exit_joy_mode();
		}
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

	// Exit joy mode if active
	if (joy_mode_active) {
		exit_joy_mode();
	}
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
	PID_CONTROLLER_Reset(&revolute_velocity_pid);
}

void update_safety_system(void) {
	// Don't control pilot light if joy mode is active
	if (joy_mode_active) {
		return; // Let joy mode handle pilot light
	}

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

void enter_joy_mode(void) {
	if (is_emergency_active() || homing_active || joy_mode_active) {
		return;
	}

	joy_mode_active = true;
	joy_mode_state = JOY_MODE_INITIAL_CONTROL; // Start in initial control state

	// Reset all joy mode data
	reset_joy_mode_data();

	// Turn on pilot light to indicate joy mode
	HAL_GPIO_WritePin(PILOT_GPIO_Port, PILOT_Pin, GPIO_PIN_SET);
	joy_mode_pilot_state = true;
	joy_mode_pilot_timer = 0;

	// Stop any current motion
	motion_sequence_state = MOTION_IDLE;
	prismatic_axis.trajectory_active = false;
	revolute_axis.trajectory_active = false;

	// Initialize position holding at current positions
	prismatic_axis.position = prismatic_encoder.mm;
	revolute_axis.position = revolute_encoder.rads;

	// Reset motor commands
	prismatic_axis.command_pos = 0.0f;
	revolute_axis.command_pos = 0.0f;
	prismatic_axis.command_vel = 0.0f;
	revolute_axis.command_vel = 0.0f;

	// Reset PID controllers
	PID_CONTROLLER_Reset(&prismatic_position_pid);
	PID_CONTROLLER_Reset(&prismatic_velocity_pid);
	PID_CONTROLLER_Reset(&revolute_position_pid);
	PID_CONTROLLER_Reset(&revolute_velocity_pid);
}

/* Updated exit joy mode to handle cleanup properly */
void exit_joy_mode(void) {
	joy_mode_active = false;
	joy_mode_state = JOY_MODE_IDLE;

	// Reset all data
	reset_joy_mode_data();

	// Turn off pilot light
	HAL_GPIO_WritePin(PILOT_GPIO_Port, PILOT_Pin, GPIO_PIN_RESET);
	joy_mode_pilot_state = false;
	joy_mode_pilot_timer = 0;

	// DON'T stop motors - hold current position
	// Set current positions as target positions for holding
	prismatic_axis.position = prismatic_encoder.mm;
	revolute_axis.position = revolute_encoder.rads;

	// Reset velocities and feedforward terms
	prismatic_axis.command_vel = 0.0f;
	revolute_axis.command_vel = 0.0f;
	prismatic_axis.ffd = 0.0f;
	prismatic_axis.dfd = 0.0f;
	revolute_axis.ffd = 0.0f;

	// Keep DFD for revolute axis (gravity compensation)
	revolute_axis.dfd = REVOLUTE_MOTOR_DFD_Compute(&revolute_motor_dfd,
			revolute_encoder.rads, prismatic_encoder.mm / 1000.0f);

	// Reset PID controllers
	PID_CONTROLLER_Reset(&prismatic_position_pid);
	PID_CONTROLLER_Reset(&prismatic_velocity_pid);
	PID_CONTROLLER_Reset(&revolute_position_pid);
	PID_CONTROLLER_Reset(&revolute_velocity_pid);
	PID_CONTROLLER_Reset(&revolute_velocity_pid);

	// Reset motion state
	motion_sequence_state = MOTION_IDLE;
	prismatic_axis.trajectory_active = false;
	revolute_axis.trajectory_active = false;

	//modbus reset state
	registerFrame[BaseSystem_Status].U16 = 0;
	registerFrame[R_Theta_Status].U16 = 0;
}

void save_current_position(void) {
	if (saved_position_count < JOY_MODE_MAX_POSITIONS) {
		float current_pris = prismatic_encoder.mm;
		float current_rev = revolute_encoder.rads;

		bool too_similar = false;
		if (saved_position_count > 0) {
			float last_pris =
					saved_positions[saved_position_count - 1].prismatic_pos;
			float last_rev =
					saved_positions[saved_position_count - 1].revolute_pos;

			if (fabsf(current_pris - last_pris) < 5.0f
					&& fabsf(current_rev - last_rev) < 0.1f) {
				too_similar = true;
				// DEBUG: Position too similar, not saving
				return;// Exit early if too similar
			}
		}

		// Only reach here if position should be saved
		saved_positions[saved_position_count].prismatic_pos = current_pris;
		saved_positions[saved_position_count].revolute_pos = current_rev;

		uint8_t r_addr = 0x20 + saved_position_count * 2;
		uint8_t t_addr = r_addr + 1;

		if (r_addr <= 0x38 && t_addr <= 0x39) {
			int16_t r_mm_fixed = (int16_t) (current_pris * 10.0);
			int16_t t_deg_fixed = (int16_t) (revolute_axis.deg * 10.0);

			registerFrame[r_addr].U16 = r_mm_fixed;
			registerFrame[t_addr].U16 = t_deg_fixed;
		}

		saved_position_count++;

		if (saved_position_count >= JOY_MODE_MAX_POSITIONS) {
			joy_mode_state = JOY_MODE_POSITION_SAVED;
			joy_mode_pilot_timer = 0;
		}
	}
}

void start_position_playback(void) {
	check[3]++;
	if (saved_position_count > 0) {
		check[2]++;
		playback_position_index = 0;
		joy_mode_playback_timer = 0;
		joy_mode_state = JOY_MODE_PLAYBACK;
		// Keep pilot light ON during playback (don't turn it off)
		HAL_GPIO_WritePin(PILOT_GPIO_Port, PILOT_Pin, GPIO_PIN_SET);
		joy_mode_pilot_state = true;

		// Start first trajectory
		float target_pris = saved_positions[0].prismatic_pos;
		float target_rev_rad = saved_positions[0].revolute_pos;
		float target_rev_deg = target_rev_rad * 180.0f / PI;

		start_combined_trajectory(target_pris, target_rev_deg);

	}
}

void reset_joy_mode_data(void) {
	// Reset saved position count
	saved_position_count = 0;

	// Reset playback index
	playback_position_index = 0;

	// Clear all saved positions
	for (int i = 0; i < JOY_MODE_MAX_POSITIONS; i++) {
		saved_positions[i].prismatic_pos = 0.0f;
		saved_positions[i].revolute_pos = 0.0f;
	}

	// Reset pilot light timers
	joy_mode_pilot_timer = 0;
	joy_mode_pilot_state = false;

	// Reset playback timer
	joy_mode_playback_timer = 0;

	// Reset button states
	joy_mode_b2_pressed = false;
	joy_mode_b2_last_state = false;
}

void update_joy_mode_velocity_control(void) {
	// Read current photo sensor states directly
	bool up_photo_detected = HAL_GPIO_ReadPin(UPPER_PHOTO_GPIO_Port,
	UPPER_PHOTO_Pin);
	bool low_photo_detected = HAL_GPIO_ReadPin(LOWER_PHOTO_GPIO_Port,
	LOWER_PHOTO_Pin);

	// Prismatic axis control based on joystick_x
	float pris_command_vel = 0.0f;
	bool pris_moving = false;
	static bool pris_was_moving = false; // Track previous moving state

	// Process prismatic axis joystick control (FIXED LOGIC)
	if (up_photo_detected && joystick_x > JOY_MODE_VELOCITY_THRESHOLD) {
		// At up photo and trying to go up (negative direction) - block movement
		pris_command_vel = 0.0f;
	} else if (low_photo_detected && joystick_x < -JOY_MODE_VELOCITY_THRESHOLD) {
		// At low photo and trying to go down (positive direction) - block movement
		pris_command_vel = 0.0f;
	} else if (joystick_x < -JOY_MODE_VELOCITY_THRESHOLD) {
		// Moving down (positive direction)
		pris_command_vel = JOY_MODE_CONSTANT_VELOCITY_PRIS;
		pris_moving = true;
		// Clear flags when moving away from sensors
		if (!low_photo_detected) {
			up_photo = false;
		}
	} else if (joystick_x > JOY_MODE_VELOCITY_THRESHOLD) {
		// Moving up (negative direction)
		pris_command_vel = -JOY_MODE_CONSTANT_VELOCITY_PRIS;
		pris_moving = true;
		// Clear flags when moving away from sensors
		if (!up_photo_detected) {
			low_photo = false;
		}
	} else {
		// Joystick in deadband - hold position
		pris_command_vel = 0.0f;
		pris_moving = false;
	}

	// Revolute axis control based on joystick_y
	float rev_command_vel = 0.0f;
	bool rev_moving = false;

	// Get current revolute position in degrees for limit checking
	float revolute_deg = UnitConverter_angle(&converter_system,
			revolute_encoder.rads, UNIT_RADIAN, UNIT_DEGREE);

	// Process revolute axis joystick control with limits
	if ((revolute_deg > 175.0f && joystick_y > JOY_MODE_VELOCITY_THRESHOLD)
			|| (revolute_deg < -175.0f
					&& joystick_y < -JOY_MODE_VELOCITY_THRESHOLD)) {
		// At revolute limits - block movement
		rev_command_vel = 0.0f;
		rev_moving = false;
	} else if (joystick_y > JOY_MODE_VELOCITY_THRESHOLD) {
		rev_command_vel = JOY_MODE_CONSTANT_VELOCITY_REV;
		rev_moving = true;
	} else if (joystick_y < -JOY_MODE_VELOCITY_THRESHOLD) {
		rev_command_vel = -JOY_MODE_CONSTANT_VELOCITY_REV;
		rev_moving = true;
	} else {
		// Joystick in deadband - hold position
		rev_command_vel = 0.0f;
		rev_moving = false;
	}

	/* PRISMATIC AXIS CONTROL - Keep existing PID-based control */
	// Detect transition from moving to stopped
	if (pris_was_moving && !pris_moving) {
		// Just stopped moving - capture current position as target
		prismatic_axis.position = prismatic_encoder.mm;
	}

	if (pris_moving) {
		// Moving - use velocity control
		prismatic_axis.vel_error = pris_command_vel
				- prismatic_axis.kalman_velocity;
		prismatic_axis.command_pos = PWM_Satuation(
				PID_CONTROLLER_Compute(&prismatic_velocity_pid,
						prismatic_axis.vel_error),
				ZGX45RGG_400RPM_Constant.U_max,
				-ZGX45RGG_400RPM_Constant.U_max);

		// Add feedforward for moving
		prismatic_axis.ffd = PRISMATIC_MOTOR_FFD_Compute(&prismatic_motor_ffd,
				pris_command_vel / 1000.0f);
		prismatic_axis.dfd = PRISMATIC_MOTOR_DFD_Compute(&prismatic_motor_dfd,
				revolute_encoder.rads, 0.0f, prismatic_encoder.mm / 1000.0f);

		// Continuously update target position while moving
		prismatic_axis.position = prismatic_encoder.mm;
	} else {
		// Not moving - hold target position with position control
		prismatic_axis.pos_error = prismatic_axis.position
				- prismatic_encoder.mm;
		prismatic_axis.command_vel = PWM_Satuation(
				PID_CONTROLLER_Compute(&prismatic_position_pid,
						prismatic_axis.pos_error),
				ZGX45RGG_400RPM_Constant.sd_max,
				-ZGX45RGG_400RPM_Constant.sd_max);

		prismatic_axis.vel_error = prismatic_axis.command_vel
				- prismatic_axis.kalman_velocity;
		prismatic_axis.command_pos = PWM_Satuation(
				PID_CONTROLLER_Compute(&prismatic_velocity_pid,
						prismatic_axis.vel_error),
				ZGX45RGG_400RPM_Constant.U_max,
				-ZGX45RGG_400RPM_Constant.U_max);

		// No feedforward when holding position
		prismatic_axis.ffd = 0.0f;
		prismatic_axis.dfd = 0.0f;
	}

	// Update previous state for next iteration
	pris_was_moving = pris_moving;

	prismatic_axis.command_pos += prismatic_axis.ffd + prismatic_axis.dfd;
	prismatic_axis.command_pos = PWM_Satuation(prismatic_axis.command_pos,
			ZGX45RGG_400RPM_Constant.U_max, -ZGX45RGG_400RPM_Constant.U_max);

	/* REVOLUTE AXIS CONTROL - SIMPLIFIED PURE PWM WITH FFD + DFD */
	float base_pwm = 0.0f;

	if (rev_moving) {
		// Calculate base PWM proportional to joystick input
		// Scale joystick input (-50 to +50) to PWM range
		float joystick_normalized = joystick_y / 50.0f; // -1.0 to +1.0
		base_pwm = joystick_normalized
				* (ZGX45RGG_150RPM_Constant.U_max * 0.3f); // Limit to 30% max PWM for safety

		// Add velocity feedforward
		revolute_axis.ffd = REVOLUTE_MOTOR_FFD_Compute(&revolute_motor_ffd,
				rev_command_vel);
	} else {
		// Joystick released - no base PWM, only compensation
		base_pwm = 0.0f;
		revolute_axis.ffd = 0.0f;
	}

	// Always add gravity/disturbance compensation
	revolute_axis.dfd = REVOLUTE_MOTOR_DFD_Compute(&revolute_motor_dfd,
			revolute_encoder.rads, prismatic_encoder.mm / 1000.0f);

	// Apply filtering to feedforward terms for stability
	static float ffd_filtered = 0.0f;
	static float dfd_filtered = 0.0f;

	ffd_filtered = 0.8f * ffd_filtered + 0.2f * revolute_axis.ffd;
	dfd_filtered = 0.8f * dfd_filtered + 0.2f * revolute_axis.dfd;

	// Combine base PWM with feedforward compensation
	revolute_axis.command_pos = base_pwm
			+ 0.01f * (ffd_filtered + dfd_filtered);

	// Saturate final command
	revolute_axis.command_pos = PWM_Satuation(revolute_axis.command_pos,
			ZGX45RGG_150RPM_Constant.U_max, -ZGX45RGG_150RPM_Constant.U_max);

	// Apply motor commands
	MDXX_set_range(&prismatic_motor, 2000, prismatic_axis.command_pos);
	MDXX_set_range(&revolute_motor, 2000, revolute_axis.command_pos);
}

void update_joy_mode_pilot_light(void) {
	if (joy_mode_state == JOY_MODE_POSITION_SAVED) {
		// Toggle pilot light every 1 second when 10 positions are saved
		joy_mode_pilot_timer++;
		if (joy_mode_pilot_timer >= JOY_MODE_PILOT_TOGGLE_PERIOD) {
			HAL_GPIO_TogglePin(PILOT_GPIO_Port, PILOT_Pin);
			joy_mode_pilot_state = !joy_mode_pilot_state;
			joy_mode_pilot_timer = 0;
		}
	}
}

void update_joy_mode(void) {
	if (!joy_mode_active) {
		return;
	}

	// ALWAYS update position display values when in joy mode
	normalized_position = normalize_angle(revolute_encoder.rads);
	prismatic_axis.mm = prismatic_encoder.mm;
	revolute_axis.deg = UnitConverter_angle(&converter_system,
			normalized_position, UNIT_RADIAN, UNIT_DEGREE);

	switch (joy_mode_state) {
	case JOY_MODE_INITIAL_CONTROL:
		// Initial manual joystick control - no position saving yet
		update_joy_mode_velocity_control();
		break;

	case JOY_MODE_MANUAL_CONTROL:
		// Manual joystick control with position saving enabled
		update_joy_mode_velocity_control();
		break;

	case JOY_MODE_POSITION_SAVED:
		revolute_axis.position = revolute_encoder.rads;
		// 10 positions saved, pilot light toggling, waiting for B2 to start playback
		update_joy_mode_pilot_light();
		break;

	case JOY_MODE_PLAYBACK:
//		revolute_axis.position = revolute_encoder.rads;
		// Playing back saved positions - PILOT LIGHT STAYS ON
		// Handle trajectory sequence states for joy mode playback
		switch (motion_sequence_state) {
		case MOTION_IDLE:
			// Current trajectory finished, wait before starting next

			joy_mode_playback_timer++;
			if (joy_mode_playback_timer >= JOY_MODE_PLAYBACK_DELAY) {
				playback_position_index++;
				if (playback_position_index < saved_position_count) {
					// Start next trajectory
					float target_pris =
							saved_positions[playback_position_index].prismatic_pos;
					float target_rev_rad =
							saved_positions[playback_position_index].revolute_pos;
					float target_rev_deg = target_rev_rad * 180.0f / PI;
					check[1]++;
					start_combined_trajectory(target_pris, target_rev_deg);
					joy_mode_playback_timer = 0;
				} else {
					// All positions played back - JUST EXIT JOY MODE (NO HOMING)
					check[9]++;
					exit_joy_mode();

					// Optional: Add some indication that playback is complete
					// You could flash the pilot light or set a status flag here
				}
			}
			break;

		case MOTION_PEN_UP_DELAY:
			// INCREMENT motion_delay_timer here for joy mode
			if (++motion_delay_timer >= 1500) {
				prismatic_axis.trajectory_active = true;
				motion_sequence_state = MOTION_PRISMATIC_ACTIVE;
			}
			break;

		case MOTION_PRISMATIC_ACTIVE:
			check[6]++;
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
			check[7]++;
			if (revolute_axis.trajectory_active && !revEva.isFinised) {
				Trapezoidal_Evaluated(&revGen, &revEva,
						revolute_axis.initial_pos, revolute_axis.target_pos,
						ZGX45RGG_150RPM_Constant.traject_qd_max,
						ZGX45RGG_150RPM_Constant.traject_qdd_max);

				revolute_axis.position = revEva.setposition;
				revolute_axis.velocity = revEva.setvelocity;

				if (revEva.isFinised) {
					revolute_axis.trajectory_active = false;
					revolute_axis.position = revEva.setposition;
					revolute_axis.velocity = 0.0f;

//					PID_CONTROLLER_Reset(&revolute_position_pid);
//					PID_CONTROLLER_Reset(&revolute_velocity_pid);

					motion_delay_timer = 0;
					motion_sequence_state = MOTION_PEN_DOWN_DELAY;
				}
			}
			break;

		case MOTION_PEN_DOWN_DELAY:
			// INCREMENT motion_delay_timer here for joy mode
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

		// Update position/velocity control for trajectory playback
		if (motion_sequence_state != MOTION_IDLE) {
			// Run position control if not in manual control
			if (position_control_tick >= POSITION_CONTROL_DIVIDER) {
				update_position_control();
			}
			// Always run velocity control during trajectory
			update_velocity_control();
		}
		break;

	case JOY_MODE_COMPLETE:
		exit_joy_mode();
		break;

	default:
		break;
	}
}

void handle_b2_button_polling(void) {
	// Read current B2 button state (assuming active low like other buttons)
	bool b2_current_state = !HAL_GPIO_ReadPin(J2_GPIO_Port, J2_Pin);

//	b2S[0] = !HAL_GPIO_ReadPin(J2_GPIO_Port, J2_Pin);
	static uint32_t last_press_time = 0;
	static uint32_t press_counter = 0;
	const uint32_t DEBOUNCE_TIME = 200; // 200ms debounce time

	press_counter++; // Increment every timer tick (assuming 1ms timer)

//	 Edge detection with debouncing
	if (b2_current_state && !joy_mode_b2_last_state) {
//	 Button just pressed - check if enough time has passed since last press
		if ((press_counter - last_press_time) >= DEBOUNCE_TIME) {
			// Button press is valid - trigger action
			joy_mode_b2_pressed = true;
			last_press_time = press_counter;

//	if (b2S[0] != b2S[1] && b2S[0] == 1) {
			// Handle B2 button press logic
			if (!is_emergency_active() && !homing_active
					&& motion_sequence_state == MOTION_IDLE) {
				if (!joy_mode_active) {
					// Enter joy mode (starts in JOY_MODE_INITIAL_CONTROL)
					enter_joy_mode();
				} else {
					// Joy mode is active, handle button press based on current state
					if (joy_mode_state == JOY_MODE_INITIAL_CONTROL) {
						// First B2 press in joy mode - start position saving mode
						joy_mode_state = JOY_MODE_MANUAL_CONTROL;
					} else if (joy_mode_state == JOY_MODE_MANUAL_CONTROL) {

						save_current_position();

						// Save current position
					} else if (joy_mode_state == JOY_MODE_POSITION_SAVED) {
						start_position_playback();
						// Start playback of saved positions
					}

					// Note: During JOY_MODE_PLAYBACK, B2 does nothing (ignore button press)
					// This prevents accidental interruption of playback
				}
			}
		}
//	b2S[1] = b2S[0];
		// If not enough time has passed, ignore this button press
	}

// Update last state
	joy_mode_b2_last_state = b2_current_state;
//
//// Reset pressed flag when button is released
	if (!b2_current_state) {
		joy_mode_b2_pressed = false;
	}
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

		uint32_t current_time = HAL_GetTick();
		if ((current_time - j1_interrupt_last_time) < 200) {
		    return; // ignore ถ้ายังไม่ครบ 200ms
		}
		j1_interrupt_last_time = current_time;

		if (!is_emergency_active() && !homing_active && !joy_mode_active
				&& !first_startup ) {
			check[0]++;
			if (!j1_active) {
				// start Again
				j1_active = true;
				j1_cycle_count = 0;
				j1_going_to_target = true;

				// go to target
				start_combined_trajectory(J1_TARGET_PRIS, J1_TARGET_REV);
			} else {
				// stop

				j1_active = false;
				j1_cycle_count = 0;
			}
		}
	}
// J2 is NOT handled here anymore - it's polled in the main loop

		if (GPIO_Pin == J3_Pin) {
			if (!is_emergency_active() && !joy_mode_active
					&& motion_sequence_state == MOTION_IDLE) {
				start_homing_sequence(false);
			}
			return;
		}

// Modified J4 button handler for joy mode exit
		if (GPIO_Pin == J4_Pin) {
			if (joy_mode_active) {
				// Exit joy mode and hold current position (don't move)
				exit_joy_mode();
			} else if (is_emergency_active()) {
				clear_emergency_state();
				start_homing_sequence(true);
			}
			return;
		}
	}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		Modbus_Protocal_Worker();
		modbus_working();

		plotter_update_sensors();
		check_emergency_button();

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

		// Position control update - Allow during HOMING_REV_TO_ZERO_DEG
		if (++position_control_tick >= POSITION_CONTROL_DIVIDER) {
			position_control_tick = 0;

			if ((!homing_active || homing_state == HOMING_REV_TO_ZERO_DEG) // ← FIXED
					&& (!joy_mode_active
							|| (joy_mode_state != JOY_MODE_MANUAL_CONTROL
									&& joy_mode_state
											!= JOY_MODE_INITIAL_CONTROL
									&& joy_mode_state != JOY_MODE_POSITION_SAVED))
					&& (!is_emergency_active() || tuning_mode)) {
				update_position_control();
			}
		}

		// Velocity control update - Allow during HOMING_REV_TO_ZERO_DEG
		if ((!homing_active || homing_state == HOMING_REV_TO_ZERO_DEG) // ← FIXED
				&& (!joy_mode_active
						|| (joy_mode_state != JOY_MODE_MANUAL_CONTROL
								&& joy_mode_state != JOY_MODE_INITIAL_CONTROL
								&& joy_mode_state != JOY_MODE_POSITION_SAVED))
				&& (!is_emergency_active() || tuning_mode)) {
			update_velocity_control();
		}

		update_safety_system();

		if (!is_emergency_active() || tuning_mode) {
			check_safety_conditions();
		}

		// Control loops - joy mode handles its own control
		if (!joy_mode_active) {
			update_control_loops();
		} else {
			update_joy_mode();
		}

		// ALWAYS update display values
		if (!joy_mode_active) {
			// Update display values for normal operation
			normalized_position = normalize_angle(revolute_encoder.rads);
			prismatic_axis.mm = prismatic_encoder.mm;
			revolute_axis.deg = UnitConverter_angle(&converter_system,
					normalized_position, UNIT_RADIAN, UNIT_DEGREE);
		}
		// Note: joy mode updates its own display values in update_joy_mode()

		prismatic_axis.accel_show = FIR_process(&prismatic_lp_accel,
				prismatic_encoder.mmpss);
		revolute_axis.accel_show = FIR_process(&revolute_lp_accel,
				revolute_encoder.radpss);
	}
}

//modbus
void modbus_working(void) {

	uint16_t limit_switch_status = 0;
//heartbeat
	registerFrame[Heartbeat_Protocol].U16 = 22881;
//servo write
	if (registerFrame[Servo_UP].U16 == 1) {
		plotter_pen_up();
	} else if (registerFrame[Servo_Down].U16 == 1) {
		plotter_pen_down();
	}

//limitSW
	if (up_lim == 1) {
		limit_switch_status |= 0x02;  // Bit 1 = Limit UP
	}
	if (low_lim == 1) {
		limit_switch_status |= 0x01;  // Bit 0 = Limit DOWN
	}
	registerFrame[LimitSwitch_Status].U16 = limit_switch_status;

	if (registerFrame[BaseSystem_Status].U16 == 1) {
		exit_joy_mode();
		start_homing_sequence(true);
	} else if (registerFrame[BaseSystem_Status].U16 == 2) {
		registerFrame[R_Theta_Status].U16 = 2;
		enter_joy_mode();
	} else if (registerFrame[BaseSystem_Status].U16 == 4) {
		exit_joy_mode();
	} else if (registerFrame[BaseSystem_Status].U16 == 8) {
		registerFrame[R_Theta_Status].U16 = 8;
		exit_joy_mode();
		float goal_r_mm = (float) (int16_t) registerFrame[Goal_R].U16 / 10.0;
		float goal_theta_deg = (float) (int16_t) registerFrame[Goal_Theta].U16
				/ 10.0;

		start_combined_trajectory(goal_r_mm, goal_theta_deg);
	}

	registerFrame[R_Axis_Actual_Position].U16 = prismatic_encoder.mm * 10.0f;
	registerFrame[R_Axis_Actual_Speed].U16 = prismatic_axis.kalman_velocity
			* 10.0f;

	float pris_accel = FIR_process(&prismatic_lp_accel,
			prismatic_encoder.mmpss);
	registerFrame[R_Axis_Acceleration].U16 = pris_accel * 10.0f;

	registerFrame[Theta_Axis_Actual_Position].U16 = revolute_axis.deg * 10.0f;

	float rev_theta_vel = UnitConverter_angle(&converter_system,
			revolute_axis.kalman_velocity, UNIT_RADIAN, UNIT_DEGREE);
	registerFrame[Theta_Axis_Actual_Speed].U16 = rev_theta_vel * 10.0f;

	float rev_theta_accel = UnitConverter_angle(&converter_system,
			revolute_encoder.radpss, UNIT_RADIAN, UNIT_DEGREE);
	registerFrame[Theta_Axis_Acceleration].U16 = rev_theta_accel * 10.0f;

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
