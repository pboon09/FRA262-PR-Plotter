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

typedef enum {
    MOTION_IDLE = 0,
    MOTION_PRISMATIC_ACTIVE,
    MOTION_REVOLUTE_ACTIVE,
    MOTION_COMPLETE
} MotionSequenceState_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define PRISMATIC_MAX_POS 300.0f
#define PRISMATIC_MIN_POS 0.0f
#define SEQUENCE_MAX_POINTS 6
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Axis state structures
AxisState_t prismatic_axis = { 0 };
AxisState_t revolute_axis = { 0 };

MotionSequenceState_t motion_sequence_state = MOTION_IDLE;

// Trajectory generation structures
Trapezoidal_GenStruct prisGen, revGen;
Trapezoidal_EvaStruct prisEva, revEva;

// System state variables
uint8_t trajectory_sequence_index = 0;
//const float32_t sequence_pris_points[SEQUENCE_MAX_POINTS] = { 0.0 };
const float32_t sequence_pris_points[SEQUENCE_MAX_POINTS] = { 175.0f, 95.0f,
		231.0f, 200.0f, 300.0f, 0.0f };
const float32_t sequence_rev_points[SEQUENCE_MAX_POINTS] = { 175.0f, 195.0f,
		95.0f, 300.0f, 150.0f, 0.0f };

// Helper variables
float normalized_position;
float movement_deg;
float angle_deg;

volatile uint32_t prox_count = 0;
volatile bool up_photo = false;
volatile bool low_photo = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void start_combined_trajectory(float prismatic_target_mm,
		float revolute_target_deg);
void update_control_loops(void);
float normalize_angle(float angle_rad);
float calculate_movement_deg(float current_deg, float target_deg);
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
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		static uint8_t button_pressed_previous = 0;

		if (b1 && !button_pressed_previous && motion_sequence_state == MOTION_IDLE) {
		    start_combined_trajectory(
		        sequence_pris_points[trajectory_sequence_index],
		        sequence_rev_points[trajectory_sequence_index]);
		    trajectory_sequence_index = (trajectory_sequence_index + 1) % SEQUENCE_MAX_POINTS;
		}
		button_pressed_previous = b1;
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

void start_combined_trajectory(float prismatic_target_mm, float revolute_target_deg) {
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
    prismatic_axis.target_pos = fminf(fmaxf(prismatic_target_mm, PRISMATIC_MIN_POS), PRISMATIC_MAX_POS);

    // For revolute axis - calculate target but don't start yet
    // Normalize current position to [0, 2π]
    float normalized_current = normalize_angle(rev_current);

    // Convert to degrees for movement calculation
    float current_deg = normalized_current * 180.0f / PI;

    // Calculate movement in degrees
    movement_deg = calculate_movement_deg(current_deg, revolute_target_deg);

    // Convert to radians and apply to absolute position
    float movement_rad = movement_deg * PI / 180.0f;
    revolute_axis.target_pos = revolute_axis.initial_pos + movement_rad;

    // Generate ONLY prismatic trajectory first
    Trapezoidal_Generator(&prisGen, prismatic_axis.initial_pos,
                         prismatic_axis.target_pos,
                         ZGX45RGG_400RPM_Constant.traject_sd_max,
                         ZGX45RGG_400RPM_Constant.traject_sdd_max);

    // Start with prismatic motion only
    prismatic_axis.trajectory_active = true;
    revolute_axis.trajectory_active = false;  // Don't start revolute yet

    // Set sequence state
    motion_sequence_state = MOTION_PRISMATIC_ACTIVE;
}

void update_control_loops(void) {
    // Normalize revolute position
    normalized_position = normalize_angle(revolute_encoder.rads);
    angle_deg = normalize_angle(revolute_axis.target_pos);

    // Handle motion sequence state machine
    switch (motion_sequence_state) {
        case MOTION_PRISMATIC_ACTIVE:
            // Update prismatic trajectory
            if (prismatic_axis.trajectory_active && !prisEva.isFinised) {
                Trapezoidal_Evaluated(&prisGen, &prisEva, prismatic_axis.initial_pos,
                                     prismatic_axis.target_pos,
                                     ZGX45RGG_400RPM_Constant.traject_sd_max,
                                     ZGX45RGG_400RPM_Constant.traject_sdd_max);

                prismatic_axis.position = prisEva.setposition;
                prismatic_axis.velocity = prisEva.setvelocity;

                if (prisEva.isFinised) {
                    // Prismatic motion complete - start revolute motion
                    prismatic_axis.trajectory_active = false;
                    prismatic_axis.position = prisEva.setposition;
                    prismatic_axis.velocity = 0.0f;
                    prismatic_axis.dfd = 0.0f;
                    prismatic_axis.ffd = 0.0f;

                    // Reset prismatic control variables
                    prismatic_axis.pos_error = 0.0f;
                    prismatic_axis.vel_error = 0.0f;
                    prismatic_axis.command_vel = 0.0f;
                    prismatic_axis.command_pos = 0.0f;

                    // Now start revolute trajectory
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
            // Update revolute trajectory
            if (revolute_axis.trajectory_active && !revEva.isFinised) {
                Trapezoidal_Evaluated(&revGen, &revEva, revolute_axis.initial_pos,
                                     revolute_axis.target_pos,
                                     ZGX45RGG_150RPM_Constant.traject_qd_max,
                                     ZGX45RGG_150RPM_Constant.traject_qdd_max);

                revolute_axis.position = revEva.setposition;
                revolute_axis.velocity = revEva.setvelocity;

                if (revEva.isFinised) {
                    // Revolute motion complete
                    revolute_axis.trajectory_active = false;
                    revolute_axis.position = revEva.setposition;
                    revolute_axis.velocity = 0.0f;
                    revolute_axis.dfd = 0.0f;
                    revolute_axis.ffd = 0.0f;

                    // Reset revolute control variables
                    revolute_axis.pos_error = 0.0f;
                    revolute_axis.vel_error = 0.0f;
                    revolute_axis.command_vel = 0.0f;
                    revolute_axis.command_pos = 0.0f;

                    motion_sequence_state = MOTION_COMPLETE;
                }
            }
            break;

        case MOTION_COMPLETE:
            // Both motions complete - ready for next command
            motion_sequence_state = MOTION_IDLE;
            break;

        case MOTION_IDLE:
        default:
            // No active motion
            break;
    }

    // *** PRISMATIC CONTROL ***
    prismatic_axis.pos_error = prismatic_axis.position - prismatic_encoder.mm;

    prismatic_axis.command_vel = PWM_Satuation(
        PID_CONTROLLER_Compute(&prismatic_position_pid, prismatic_axis.pos_error),
        ZGX45RGG_400RPM_Constant.sd_max, -ZGX45RGG_400RPM_Constant.sd_max);

    // Add velocity feedforward for trajectory
    if (prismatic_axis.trajectory_active) {
        prismatic_axis.vel_error = prismatic_axis.command_vel
                                  + prismatic_axis.velocity - prismatic_axis.kalman_velocity;
    } else {
        prismatic_axis.vel_error = prismatic_axis.command_vel - prismatic_axis.kalman_velocity;
    }

    prismatic_axis.command_pos = PWM_Satuation(
        PID_CONTROLLER_Compute(&prismatic_velocity_pid, prismatic_axis.vel_error),
        ZGX45RGG_400RPM_Constant.U_max, -ZGX45RGG_400RPM_Constant.U_max);

    // *** REVOLUTE CONTROL ***
    revolute_axis.pos_error = revolute_axis.position - normalized_position;

    // Ensure error uses the shortest path for control
    if (revolute_axis.pos_error > PI) {
        revolute_axis.pos_error -= 2.0f * PI;
    }
    if (revolute_axis.pos_error < -PI) {
        revolute_axis.pos_error += 2.0f * PI;
    }

    revolute_axis.command_vel = PWM_Satuation(
        PID_CONTROLLER_Compute(&revolute_position_pid, revolute_axis.pos_error),
        ZGX45RGG_150RPM_Constant.qd_max, -ZGX45RGG_150RPM_Constant.qd_max);

    // Add velocity feedforward for trajectory
    if (revolute_axis.trajectory_active) {
        revolute_axis.vel_error = revolute_axis.command_vel
                                 + revolute_axis.velocity - revolute_axis.kalman_velocity;
    } else {
        revolute_axis.vel_error = revolute_axis.command_vel - revolute_axis.kalman_velocity;
    }

    revolute_axis.command_pos = PWM_Satuation(
        PID_CONTROLLER_Compute(&revolute_velocity_pid, revolute_axis.vel_error),
        ZGX45RGG_150RPM_Constant.U_max, -ZGX45RGG_150RPM_Constant.U_max);

    // *** FEEDFORWARD COMPENSATION ***
    // Add feed-forward compensation for prismatic axis
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

    // Add feed-forward compensation for revolute axis
    if (revolute_axis.trajectory_active) {
        revolute_axis.ffd = REVOLUTE_MOTOR_FFD_Compute(&revolute_motor_ffd, revolute_axis.velocity);
        revolute_axis.dfd = REVOLUTE_MOTOR_DFD_Compute(&revolute_motor_dfd,
                                                      revolute_encoder.rads, 0.0f,
                                                      prismatic_encoder.mm / 1000.0f);
    } else {
        revolute_axis.ffd = 0.0f;
        revolute_axis.dfd = 0.0f;
    }

    // Add feedforward terms to commands
    prismatic_axis.command_pos += prismatic_axis.dfd + prismatic_axis.ffd;
    revolute_axis.command_pos += revolute_axis.dfd + revolute_axis.ffd;

    // Final saturation
    prismatic_axis.command_pos = PWM_Satuation(prismatic_axis.command_pos,
                                              ZGX45RGG_400RPM_Constant.U_max,
                                              -ZGX45RGG_400RPM_Constant.U_max);
    revolute_axis.command_pos = PWM_Satuation(revolute_axis.command_pos,
                                             ZGX45RGG_150RPM_Constant.U_max,
                                             -ZGX45RGG_150RPM_Constant.U_max);

    // Apply commands to motors
    MDXX_set_range(&prismatic_motor, 2000, prismatic_axis.command_pos);
    MDXX_set_range(&revolute_motor, 2000, revolute_axis.command_pos);

    // Update display values
    prismatic_axis.mm = prismatic_encoder.mm;
    prismatic_axis.target = prismatic_axis.target_pos;

    revolute_axis.deg = UnitConverter_angle(&converter_system, normalized_position,
                                          UNIT_RADIAN, UNIT_DEGREE);
    revolute_axis.target = UnitConverter_angle(&converter_system, angle_deg,
                                             UNIT_RADIAN, UNIT_DEGREE);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == EMER_Pin) {
		rs_current_state = RS_EMERGENCY_TRIGGED;
		emer_state = PUSHED;
	}

	if (GPIO_Pin == PROX_Pin) {
		prox_count++;
	}

	if (GPIO_Pin == UPPER_PHOTO_Pin) {
		up_photo = true;
	}

	if (GPIO_Pin == LOWER_PHOTO_Pin) {
		low_photo = true;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// Handle UART reception if needed
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		// Update sensor readings
		plotter_update_sensors();

		QEI_get_diff_count(&prismatic_encoder);
		QEI_compute_data(&prismatic_encoder);
		QEI_get_diff_count(&revolute_encoder);
		QEI_compute_data(&revolute_encoder);

		// Calculate control signals for revolute axis
		revolute_axis.input_voltage = mapf(revolute_axis.command_pos, -65535.0f,
				65535.0f, -12.0f, 12.0f);

		revolute_axis.kalman_velocity = SteadyStateKalmanFilter(
				&revolute_kalman, revolute_axis.input_voltage,
				revolute_encoder.rads);

		if (isnan(revolute_axis.kalman_velocity)) {
			revolute_axis.kalman_velocity = 0.0f;
		}

		// Calculate control signals for prismatic axis
		prismatic_axis.input_voltage = mapf(prismatic_axis.command_pos,
				-65535.0f, 65535.0f, -12.0f, 12.0f);

		prismatic_axis.kalman_velocity = MotorKalman_Estimate(&prismatic_kalman,
				prismatic_axis.input_voltage, prismatic_encoder.rads)
				* Disturbance_Constant.prismatic_pulley_radius * 1000.0f;

		if (isnan(prismatic_axis.kalman_velocity)) {
			prismatic_axis.kalman_velocity = 0.0f;
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
