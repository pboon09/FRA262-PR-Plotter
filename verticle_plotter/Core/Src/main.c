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
#define TARGET_POS_1 100.0f  // Target position for button 1 (mm)
#define TARGET_POS_2 200.0f  // Target position for button 2 (mm)
#define TARGET_POS_3 300.0f  // Target position for button 3 (mm)
#define TARGET_POS_4 400.0f  // Target position for button 4 (mm)
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
float32_t target_p = 100.0f;  // Target position (mm)
float32_t vmax = 50.0f;       // Max velocity (mm/s)
float32_t amax = 10.0f;       // Max acceleration (mm/s²)
volatile float32_t current_position;   // Current position (mm)
volatile float32_t current_velocity;   // Current velocity (mm/s)
volatile float32_t current_acceleration; // Current acceleration (mm/s²)
volatile float32_t current_duty;       // Motor duty cycle (%)

float duty_pris;
float duty_revo;
int b1, b2, b3, b4, prox, photo_pris, photo_revo, up_lim, low_lim;
int joy_x, joy_y;
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
int main(void)
{

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
  /* USER CODE BEGIN 2 */
	plotter_begin();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		b1 = !HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin);
		b2 = !HAL_GPIO_ReadPin(SAVE_GPIO_Port, SAVE_Pin);
		b3 = !HAL_GPIO_ReadPin(DELETE_GPIO_Port, DELETE_Pin);
		b4 = !HAL_GPIO_ReadPin(RESET_SYS_GPIO_Port, RESET_SYS_Pin);

		prox = HAL_GPIO_ReadPin(PROX_GPIO_Port, PROX_Pin);
		photo_pris = HAL_GPIO_ReadPin(PHOTO_PRIS_GPIO_Port, PHOTO_PRIS_Pin);
		photo_revo = HAL_GPIO_ReadPin(PHOTO_REVO_GPIO_Port, PHOTO_REVO_Pin);
		up_lim = HAL_GPIO_ReadPin(UPPER_LIM_GPIO_Port, UPPER_LIM_Pin);
		low_lim = HAL_GPIO_ReadPin(LOWER_LIM_GPIO_Port, LOWER_LIM_Pin);

		HAL_GPIO_WritePin(PILOT_GPIO_Port, PILOT_Pin, b1);

		if (b4) {
			NVIC_SystemReset();
		}
//
//		if (b1 && !trajectoryActive) {
//			// Initialize evaluator
//			prisEva.t = 0.0f;
//			prisEva.isFinised = false;
//
//			// Generate trapezoidal trajectory
//			Trapezoidal_Generator(&prisGen, initial_p, target_p, vmax, amax);
//			trajectoryActive = true;
//		}
//		if (b4) {
//			// No longer system reset - now it's a move to target 4
//			if (!trajectoryActive) {
//				// Initialize evaluator
//				prisEva.t = 0.0f;
//				prisEva.isFinised = false;
//
//				// Set initial position to current position
//				initial_p = current_position;
//
//				// Set target to position 4
//				target_p = TARGET_POS_4;
//
//				// Generate trapezoidal trajectory
//				Trapezoidal_Generator(&prisGen, initial_p, target_p, vmax,
//						amax);
//				trajectoryActive = true;
//			}
//		} else if (b3) {
//			if (!trajectoryActive) {
//				// Initialize evaluator
//				prisEva.t = 0.0f;
//				prisEva.isFinised = false;
//
//				// Set initial position to current position
//				initial_p = current_position;
//
//				// Set target to position 3
//				target_p = TARGET_POS_3;
//
//				// Generate trapezoidal trajectory
//				Trapezoidal_Generator(&prisGen, initial_p, target_p, vmax,
//						amax);
//				trajectoryActive = true;
//			}
//		} else if (b2) {
//			if (!trajectoryActive) {
//				// Initialize evaluator
//				prisEva.t = 0.0f;
//				prisEva.isFinised = false;
//
//				// Set initial position to current position
//				initial_p = current_position;
//
//				// Set target to position 2
//				target_p = TARGET_POS_2;
//
//				// Generate trapezoidal trajectory
//				Trapezoidal_Generator(&prisGen, initial_p, target_p, vmax,
//						amax);
//				trajectoryActive = true;
//			}
//		} else if (b1) {
//			if (!trajectoryActive) {
//				// Initialize evaluator
//				prisEva.t = 0.0f;
//				prisEva.isFinised = false;
//
//				// Set initial position to current position
//				initial_p = current_position;
//
//				// Set target to position 1
//				target_p = TARGET_POS_1;
//
//				// Generate trapezoidal trajectory
//				Trapezoidal_Generator(&prisGen, initial_p, target_p, vmax,
//						amax);
//				trajectoryActive = true;
//			}
//		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
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
		QEI_get_diff_count(&prismatic_encoder);
		QEI_compute_data(&prismatic_encoder);

		QEI_get_diff_count(&revolute_encoder);
		QEI_compute_data(&revolute_encoder);

		update_sensors();

		joy_x = ADC_DMA_GetValue(&adc_dma, JOYSTICK_X_CHANNEL);
		joy_y = ADC_DMA_GetValue(&adc_dma, JOYSTICK_Y_CHANNEL);

		MDXX_set_range(&prismatic_motor, 1000, duty_pris);
		MDXX_set_range(&revolute_motor, 1000, duty_revo);
		PWM_write_range(&servo, 50, duty_revo);

		if (trajectoryActive && !prisEva.isFinised) {
			// Evaluate trajectory
			Trapezoidal_Evaluated(&prisGen, &prisEva, initial_p, target_p, vmax,
					amax);

			// Update variables for CubeMonitor
			current_position = prisEva.setposition;
			current_velocity = prisEva.setvelocity;
			current_acceleration = prisEva.setacceleration;

		} else {
			current_duty = 0.0f;
			trajectoryActive = false;
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
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
