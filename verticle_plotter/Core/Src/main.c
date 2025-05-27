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
	MOTION_REVOLUTE_ACTIVE,
	MOTION_COMPLETE
} MotionSequenceState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define POSITION_CONTROL_DIVIDER 10
#define SEQUENCE_MAX_POINTS 6
#define PI 3.14159265359f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
AxisState_t revolute_axis = { 0 };
MotionSequenceState_t motion_sequence_state = MOTION_IDLE;
Trapezoidal_GenStruct revGen;
Trapezoidal_EvaStruct revEva;

uint8_t trajectory_sequence_index = 0;
const float32_t sequence_rev_points[SEQUENCE_MAX_POINTS] = { 175.0f, 195.0f,
		95.0f, 300.0f, 150.0f, 0.0f };
volatile uint32_t position_control_tick = 0;
float temp;
float normalized_position;
float movement_deg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void update_revolute_velocity_control(void);
void start_revolute_trajectory(float revolute_target_deg);
void update_revolute_control_loops(void);
void update_revolute_position_control(void);
float normalize_angle(float angle_rad);
float calculate_movement_deg(float current_deg, float target_deg);/* USER CODE END PFP */

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

	revolute_axis.position = revolute_encoder.rads;
	motion_sequence_state = MOTION_IDLE;
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

void start_revolute_trajectory(float revolute_target_deg) {
	float rev_current = revolute_encoder.rads;

	// Reset trajectory evaluation
	revEva.t = 0.0f;
	revEva.isFinised = false;

	revolute_axis.initial_pos = rev_current;

	// Calculate shortest path to target
	float normalized_current = normalize_angle(rev_current);
	float current_deg = normalized_current * 180.0f / PI;
	movement_deg = calculate_movement_deg(current_deg, revolute_target_deg);
	float movement_rad = movement_deg * PI / 180.0f;
	revolute_axis.target_pos = revolute_axis.initial_pos + movement_rad;

	// Generate trapezoidal trajectory
	Trapezoidal_Generator(&revGen, revolute_axis.initial_pos,
			revolute_axis.target_pos,
			ZGX45RGG_150RPM_Constant.traject_qd_max,
			ZGX45RGG_150RPM_Constant.traject_qdd_max);

	revolute_axis.trajectory_active = true;
	motion_sequence_state = MOTION_REVOLUTE_ACTIVE;
}

void update_revolute_position_control(void) {
	float normalized_position = normalize_angle(revolute_encoder.rads);
	revolute_axis.pos_error = revolute_axis.position - normalized_position;

	// Handle angle wrapping
	if (revolute_axis.pos_error > PI)
		revolute_axis.pos_error -= 2.0f * PI;
	if (revolute_axis.pos_error < -PI)
		revolute_axis.pos_error += 2.0f * PI;

	revolute_axis.command_vel = PWM_Satuation(
			PID_CONTROLLER_Compute(&revolute_position_pid,
					revolute_axis.pos_error), ZGX45RGG_150RPM_Constant.qd_max,
			-ZGX45RGG_150RPM_Constant.qd_max);
}

void update_revolute_velocity_control(void) {
	if (revolute_axis.trajectory_active) {
		revolute_axis.vel_error = revolute_axis.command_vel
				+ revolute_axis.velocity - revolute_axis.kalman_velocity;
	} else {
		revolute_axis.vel_error = revolute_axis.command_vel
				- revolute_axis.kalman_velocity;
	}

	revolute_axis.command_pos = PWM_Satuation(
			PID_CONTROLLER_Compute(&revolute_velocity_pid, revolute_axis.vel_error),
			ZGX45RGG_150RPM_Constant.U_max, -ZGX45RGG_150RPM_Constant.U_max);

	if (revolute_axis.trajectory_active) {
		revolute_axis.ffd = REVOLUTE_MOTOR_FFD_Compute(&revolute_motor_ffd,
				revolute_axis.velocity);
		revolute_axis.dfd = REVOLUTE_MOTOR_DFD_Compute(&revolute_motor_dfd,
				revolute_encoder.rads, 300.0f / 1000.0f);
	} else {
		revolute_axis.ffd = 0.0f;
		revolute_axis.dfd = REVOLUTE_MOTOR_DFD_Compute(&revolute_motor_dfd,
				revolute_encoder.rads, 300.0f / 1000.0f);
	}

    static float ffd_filtered = 0.0f;
    static float dfd_filtered = 0.0f;

    ffd_filtered = 0.8f * ffd_filtered + 0.2f * revolute_axis.ffd;
    dfd_filtered = 0.8f * dfd_filtered + 0.2f * revolute_axis.dfd;

    revolute_axis.command_pos += 0.01 * (dfd_filtered + ffd_filtered);

//	revolute_axis.command_pos += revolute_axis.ffd + revolute_axis.dfd;
	revolute_axis.command_pos = PWM_Satuation(revolute_axis.command_pos,
			ZGX45RGG_150RPM_Constant.U_max, -ZGX45RGG_150RPM_Constant.U_max);

	MDXX_set_range(&revolute_motor, 2000, revolute_axis.command_pos);
}

void update_revolute_control_loops(void) {
	normalized_position = normalize_angle(revolute_encoder.rads);

	switch (motion_sequence_state) {
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

				motion_sequence_state = MOTION_COMPLETE;
			}
		}
		break;

	case MOTION_COMPLETE:
		motion_sequence_state = MOTION_IDLE;
		break;

	default:
		break;
	}

	revolute_axis.deg = UnitConverter_angle(&converter_system,
			normalized_position, UNIT_RADIAN, UNIT_DEGREE);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	// B1 button (J1_Pin) - Start revolute trajectory sequence
	if (GPIO_Pin == J1_Pin) {
		if (motion_sequence_state == MOTION_IDLE) {
			start_revolute_trajectory(sequence_rev_points[trajectory_sequence_index]);
			trajectory_sequence_index = (trajectory_sequence_index + 1) % SEQUENCE_MAX_POINTS;
		}
		return;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
        QEI_get_diff_count(&revolute_encoder);
        QEI_compute_data(&revolute_encoder);

        revolute_axis.input_voltage = mapf(revolute_axis.command_pos, -65535.0f,
                65535.0f, -12.0f, 12.0f);
        revolute_axis.kalman_velocity = SteadyStateKalmanFilter(
                &revolute_kalman, revolute_axis.input_voltage, revolute_encoder.rads);

        if (isnan(revolute_axis.kalman_velocity)) {
            revolute_axis.kalman_velocity = 0.0f;
        }

        // Update position control
        if (++position_control_tick >= POSITION_CONTROL_DIVIDER) {
            position_control_tick = 0;
            update_revolute_position_control();
        }

        // Velocity control at 1000 Hz (every iteration)
        update_revolute_velocity_control();

        // Update control loops and trajectory
        update_revolute_control_loops();
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
