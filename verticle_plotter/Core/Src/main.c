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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BACKLASH_COMPENSATION_GAIN 0.1f
#define PRISMATIC_BACKLASH_COMPENSATION_GAIN 0.1f
#define BACKLASH_DECAY_FACTOR 0.1f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Axis state structures
AxisState_t prismatic_axis = { 0 };
AxisState_t revolute_axis = { 0 };

float rev_velocity_target;

volatile uint32_t prox_count = 0;
volatile bool up_photo = false;
volatile bool low_photo = false;

float revolute_backlash = 10.0f;
float revolute_last_cmd_direction = 0.0f;
float revolute_backlash_state = 0.0f;

Trapezoidal_GenStruct prisGen;
Trapezoidal_EvaStruct prisEva;
bool trajectoryActive = false;
float32_t initial_p = 0.0f;   // Initial position (mm)
float32_t target_p = 0.0f;
volatile float32_t current_position;   // Current position (mm)
volatile float32_t current_velocity;   // Current velocity (mm/s)

uint8_t trajectory_sequence_index = 0;
bool sequence_active = false;
const float32_t trajectory_sequence[4] = { 3.14f, 1.24f, 5.63f, 0.0f }; // Sequence of setpoints

float rp, rv;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void control_motor_revolute(float pos_target, float vel_target);

float revolute_backlash_compensator(float cmd_vel);

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
//		if (joystick_y > 40.0) {
//			rev_velocity_target = ZGX45RGG_150RPM_Constant.qd_max - 1;
//		} else if (joystick_y < -40.0) {
//			rev_velocity_target = -ZGX45RGG_150RPM_Constant.qd_max + 1;
//		} else {
//			rev_velocity_target = 0.0f;
//		}
		static uint8_t button_pressed_previous = 0;

		if (b1 && !button_pressed_previous && !trajectoryActive) {
			prisEva.t = 0.0f;
			prisEva.isFinised = false;

			initial_p = revolute_encoder.rads;

			target_p = trajectory_sequence[trajectory_sequence_index];

			//			Trapezoidal_Generator(&prisGen, initial_p, target_p,
			//					ZGX45RGG_400RPM_Constant.qd_max,
			//					ZGX45RGG_400RPM_Constant.qd_max * 3.0);

			Trapezoidal_Generator(&prisGen, initial_p, target_p,
					ZGX45RGG_150RPM_Constant.traject_qd_max,
					ZGX45RGG_150RPM_Constant.traject_qdd_max);

			trajectoryActive = true;

			trajectory_sequence_index = (trajectory_sequence_index + 1) % 4;
		}
		button_pressed_previous = b1;

//		if (b1 && !button_pressed_previous && !revolute_axis.trajectory_active
//				&& !prismatic_axis.trajectory_active) {
//			start_combined_trajectory(
//					sequence_pris_points[trajectory_sequence_index],
//					sequence_rev_points[trajectory_sequence_index]);
//			trajectory_sequence_index = (trajectory_sequence_index + 1)
//					% SEQUENCE_MAX_POINTS;
//		}
//		button_pressed_previous = b1;
//
//		// Process button 2 - start homing procedure
//		if (b2 && homing_state == HOMING_IDLE) {
//			start_homing();
//		}
//
//		// Process button 3 - enter manual control mode
//		if (b3) {
//			manual_control_mode();
//		}
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
void control_motor_revolute(float pos_target, float vel_target) {
	revolute_axis.input_voltage = mapf(revolute_axis.command_pos, -65535.0f,
			65535.0f, -12.0f, 12.0f);

	revolute_axis.kalman_velocity = SteadyStateKalmanFilter(&revolute_kalman,
			revolute_axis.input_voltage, revolute_encoder.rads);

	if (isnan(revolute_axis.kalman_velocity)) {
		revolute_axis.kalman_velocity = 0.0f;
	}

	revolute_axis.pos_error = pos_target - revolute_encoder.rads;

	revolute_axis.command_vel = PWM_Satuation(
			PID_CONTROLLER_Compute(&revolute_position_pid,
					revolute_axis.pos_error), ZGX45RGG_150RPM_Constant.qd_max,
			-ZGX45RGG_150RPM_Constant.qd_max);

	if (pos_target == 0) {
		revolute_axis.vel_error = vel_target - revolute_axis.kalman_velocity;
	} else {
		revolute_axis.vel_error = vel_target + revolute_axis.command_vel
				- revolute_axis.kalman_velocity;
	}

	revolute_axis.command_pos = PWM_Satuation(
			PID_CONTROLLER_Compute(&revolute_velocity_pid,
					revolute_axis.vel_error), ZGX45RGG_150RPM_Constant.U_max,
			-ZGX45RGG_150RPM_Constant.U_max);

	MDXX_set_range(&revolute_motor, 2000, revolute_axis.command_pos);
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

		if (trajectoryActive && !prisEva.isFinised) {
			Trapezoidal_Evaluated(&prisGen, &prisEva, initial_p, target_p,
					ZGX45RGG_150RPM_Constant.traject_qd_max,
					ZGX45RGG_150RPM_Constant.traject_qdd_max);

			current_position = prisEva.setposition;
			current_velocity = prisEva.setvelocity;

			control_motor_revolute(current_position, current_velocity);
		} else {
			trajectoryActive = false;

		    revolute_axis.pos_error = 0.0f;
		    revolute_axis.vel_error = 0.0f;
		    revolute_axis.command_vel = 0.0f;
		    revolute_axis.command_pos = 0.0f;

		    revolute_axis.input_voltage = 0.0f;

			revolute_axis.kalman_velocity = SteadyStateKalmanFilter(&revolute_kalman,
						revolute_axis.input_voltage, revolute_encoder.rads);

			control_motor_revolute(current_position, 0);
		}



//		rv = SIGNAL_generate(&sine_sg_revolute, 0.001);
//		rp += rv * 0.001;
//		control_motor_revolute(rp, rv);
	}
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
