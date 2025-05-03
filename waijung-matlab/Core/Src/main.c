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
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include <string.h>
#include "serial_frame.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define TX_LEN   20u
//#define RX_LEN   TX_LEN
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//uint8_t TxBuffer[TX_LEN] = { 0 };
//uint8_t RxBuffer[RX_LEN] = { 0 };
float dum_float1, dum_float2, dum_float3;
int16_t dum_int1, dum_int2, dum_int3;

float rev_float1, rev_float2, rev_float3;
int16_t rev_int1, rev_int2, rev_int3;

SerialFrame serial_frame;
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
	MX_LPUART1_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
//	HAL_UART_Transmit_DMA(&hlpuart1, TxBuffer, 20);
//	HAL_UART_Receive_DMA(&hlpuart1, RxBuffer, 20);
	SerialFrame_Init(&serial_frame, &hlpuart1, 37, 'N');

	SerialFrame_AddTxField(&serial_frame, SERIAL_TYPE_FLOAT, &dum_float1,
			"Float1");
	SerialFrame_AddTxField(&serial_frame, SERIAL_TYPE_FLOAT, &dum_float2,
			"Float2");
	SerialFrame_AddTxField(&serial_frame, SERIAL_TYPE_FLOAT, &dum_float3,
			"Float3");
	SerialFrame_AddTxField(&serial_frame, SERIAL_TYPE_INT16, &dum_int1, "Int1");
	SerialFrame_AddTxField(&serial_frame, SERIAL_TYPE_INT16, &dum_int2, "Int2");
	SerialFrame_AddTxField(&serial_frame, SERIAL_TYPE_INT16, &dum_int3, "Int3");

	// Add receive fields
	SerialFrame_AddRxField(&serial_frame, SERIAL_TYPE_FLOAT, &rev_float1,
			"RevFloat1");
	SerialFrame_AddRxField(&serial_frame, SERIAL_TYPE_FLOAT, &rev_float2,
			"RevFloat2");
	SerialFrame_AddRxField(&serial_frame, SERIAL_TYPE_FLOAT, &rev_float3,
			"RevFloat3");
	SerialFrame_AddRxField(&serial_frame, SERIAL_TYPE_INT16, &rev_int1,
			"RevInt1");
	SerialFrame_AddRxField(&serial_frame, SERIAL_TYPE_INT16, &rev_int2,
			"RevInt2");
	SerialFrame_AddRxField(&serial_frame, SERIAL_TYPE_INT16, &rev_int3,
			"RevInt3");

	SerialFrame_StartReceive(&serial_frame);

	HAL_TIM_Base_Start_IT(&htim2);
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
//		TxBuffer[0] = 37; /* header        */
//		memcpy(&TxBuffer[1], &dum_float1, sizeof(float)); /*  1 –  4       */
//		memcpy(&TxBuffer[5], &dum_float2, sizeof(float)); /*  5 –  8       */
//		memcpy(&TxBuffer[9], &dum_float3, sizeof(float)); /*  9 – 12       */
//		memcpy(&TxBuffer[13], &dum_int1, sizeof(int16_t)); /* 13 – 14       */
//		memcpy(&TxBuffer[15], &dum_int2, sizeof(int16_t)); /* 15 – 16       */
//		memcpy(&TxBuffer[17], &dum_int3, sizeof(int16_t)); /* 17 – 18       */
//		TxBuffer[19] = 'N'; /* terminator    */
//
//		HAL_UART_Transmit_DMA(&hlpuart1, TxBuffer, TX_LEN);
		SerialFrame_BuildTxFrame(&serial_frame);
		SerialFrame_Transmit(&serial_frame);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart == &hlpuart1) {
//		if (RxBuffer[0] == 37)
//		{
//			memcpy(&rev_float1, &RxBuffer[1], sizeof(float));
//			memcpy(&rev_float2, &RxBuffer[5], sizeof(float));
//			memcpy(&rev_float3, &RxBuffer[9], sizeof(float));
//			memcpy(&rev_int1, &RxBuffer[13], sizeof(int16_t));
//			memcpy(&rev_int2, &RxBuffer[15], sizeof(int16_t));
//			memcpy(&rev_int3, &RxBuffer[17], sizeof(int16_t));
//		}
//
//		HAL_UART_Receive_DMA(&hlpuart1, RxBuffer, RX_LEN);
		// Parse received frame
		SerialFrame_ParseRxFrame(&serial_frame);

		// Start receiving again
		SerialFrame_StartReceive(&serial_frame);
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
