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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "plotter_config.h"
#include "Trapezoidal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Trapezoidal_GenStruct prisGen;
Trapezoidal_EvaStruct prisEva;
bool trajectoryActive = false;
float32_t initial_p = 0.0f;   // Initial position (mm)
float32_t target_p = 0.0f;
volatile float32_t current_position;   // Current position (mm)
volatile float32_t current_velocity;   // Current velocity (mm/s)

uint8_t trajectory_sequence_index = 0;
bool sequence_active = false;
const float32_t trajectory_sequence[4] = { 100.0f, 400.0f, 200.0f, 300.0f }; // Sequence of setpoints

float setpoint_pos, setpoint_vel, error, kal_flit, lp_filt;
float cmd_ux, cmd_vx;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
	/* USER CODE BEGIN 2 */
	plotter_begin();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		static uint8_t button_pressed_previous = 0;

		if (b1 && !button_pressed_previous && !trajectoryActive) {
			prisEva.t = 0.0f;
			prisEva.isFinised = false;

			initial_p = current_position;

			target_p = trajectory_sequence[trajectory_sequence_index];

			Trapezoidal_Generator(&prisGen, initial_p, target_p,
					ZGX45RGG_400RPM_Constant.qd_max,
					ZGX45RGG_400RPM_Constant.qd_max * 3.0);

			trajectoryActive = true;

			trajectory_sequence_index = (trajectory_sequence_index + 1) % 4;
		}
		button_pressed_previous = b1;

		HAL_GPIO_WritePin(PILOT_GPIO_Port, PILOT_Pin, b2);

		if (b3) {
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		update_sensors();

		if (trajectoryActive && !prisEva.isFinised) {
			Trapezoidal_Evaluated(&prisGen, &prisEva, initial_p, target_p,
					ZGX45RGG_400RPM_Constant.qd_max,
					ZGX45RGG_400RPM_Constant.qd_max * 3.0);

			current_position = prisEva.setposition;
			current_velocity = prisEva.setvelocity;

			setpoint_pos = current_position;
			setpoint_vel = current_velocity;
			QEI_get_diff_count(&prismatic_encoder);
			QEI_compute_data(&prismatic_encoder);

			lp_filt = FIR_process(&LP_prismatic_velocity,
					prismatic_encoder.radps);

			cmd_vx = PID_CONTROLLER_Compute(&prismatic_position_pid,
					setpoint_pos - prismatic_encoder.rads);
			cmd_ux = PWM_Satuation(
					PID_CONTROLLER_Compute(&prismatic_velocity_pid,
							cmd_vx + setpoint_vel - lp_filt), 65535, -65535);
		} else {
			trajectoryActive = false;
			cmd_ux = 0;
		}

		MDXX_set_range(&prismatic_motor, 2000, cmd_ux);
	}
}

//if (trajectoryActive && !prisEva.isFinised) {
//	// Evaluate trajectory
//	Trapezoidal_Evaluated(&prisGen, &prisEva, initial_p, target_p, vmax, amax);
//
//	// Update variables for CubeMonitor
//	current_position = prisEva.setposition;
//	current_velocity = prisEva.setvelocity;
//	current_acceleration = prisEva.setacceleration;
//
//} else {
//	current_duty = 0.0f;
//	trajectoryActive = false;
//}
//
//if (b4) {
//	// No longer system reset - now it's a move to target 4
//	if (!trajectoryActive) {
//		// Initialize evaluator
//		prisEva.t = 0.0f;
//		prisEva.isFinised = false;
//
//		// Set initial position to current position
//		initial_p = current_position;
//
//		// Set target to position 4
//		target_p = TARGET_POS_4;
//
//		// Generate trapezoidal trajectory
//		Trapezoidal_Generator(&prisGen, initial_p, target_p, vmax,
//				amax);
//		trajectoryActive = true;
//	}
//} else if (b3) {
//	if (!trajectoryActive) {
//		// Initialize evaluator
//		prisEva.t = 0.0f;
//		prisEva.isFinised = false;
//
//		// Set initial position to current position
//		initial_p = current_position;
//
//		// Set target to position 3
//		target_p = TARGET_POS_3;
//
//		// Generate trapezoidal trajectory
//		Trapezoidal_Generator(&prisGen, initial_p, target_p, vmax,
//				amax);
//		trajectoryActive = true;
//	}
//} else if (b2) {
//	if (!trajectoryActive) {
//		// Initialize evaluator
//		prisEva.t = 0.0f;
//		prisEva.isFinised = false;
//
//		// Set initial position to current position
//		initial_p = current_position;
//
//		// Set target to position 2
//		target_p = TARGET_POS_2;
//
//		// Generate trapezoidal trajectory
//		Trapezoidal_Generator(&prisGen, initial_p, target_p, vmax,
//				amax);
//		trajectoryActive = true;
//	}
//} else if (b1) {
//	if (!trajectoryActive) {
//		// Initialize evaluator
//		prisEva.t = 0.0f;
//		prisEva.isFinised = false;
//
//		// Set initial position to current position
//		initial_p = current_position;
//
//		// Set target to position 1
//		target_p = TARGET_POS_1;
//
//		// Generate trapezoidal trajectory
//		Trapezoidal_Generator(&prisGen, initial_p, target_p, vmax,
//				amax);
//		trajectoryActive = true;
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
