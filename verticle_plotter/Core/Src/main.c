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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ProcessReceivedCommands(void);

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
	/* USER CODE BEGIN 2 */
	plotter_begin();

	for (int i = 0; i < 0x40; i++) {
	   registerFrame[i].U16 = 0;
	}
	registerFrame[0x00].U16 = 22881;
	registerFrame[0x03].U16 = 0x01;
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		Modbus_Protocal_Worker();
		ProcessReceivedCommands();
		registerFrame[0x00].U16 = 22881;
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
void ProcessReceivedCommands(void) {
    static uint32_t motionStartTime = 0;
    static uint8_t motionActive = 0;

    if ((registerFrame[0x01].U16 & 0x01) && !motionActive) {
        registerFrame[0x10].U16 = 0x01;

        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

        registerFrame[0x11].U16 = 500;
        registerFrame[0x12].U16 = 300;
        registerFrame[0x13].U16 = 50;
        registerFrame[0x14].U16 = 100;
        registerFrame[0x15].U16 = 20;
        registerFrame[0x16].U16 = 40;

        // Start motion timer
        motionStartTime = HAL_GetTick();
        motionActive = 1;
    }

    if ((registerFrame[0x01].U16 & 0x08) && !motionActive) {
        registerFrame[0x10].U16 = 0x08;

        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

        registerFrame[0x13].U16 = 12;
        registerFrame[0x14].U16 = 123;
        registerFrame[0x15].U16 = 21;
        registerFrame[0x16].U16 = 20;

        // Start motion timer
        motionStartTime = HAL_GetTick();
        motionActive = 1;
    }

    // Process Pen UP command
    if (registerFrame[0x04].U16 == 1) {
        // Update limit switch status
        registerFrame[0x03].U16 = 0x02;

        // Clear the command
        registerFrame[0x04].U16 = 0;
    }

    // Process Pen DOWN command
    if (registerFrame[0x05].U16 == 1) {
        // Update limit switch status
        registerFrame[0x03].U16 = 0x01;

        registerFrame[0x05].U16 = 0;
    }

    // Handle motion simulation
    if (motionActive) {
        // After 3 seconds, complete the motion
        if (HAL_GetTick() - motionStartTime > 3000) {
            // Clear the motion status - set to Idle
            registerFrame[0x10].U16 = 0x00;

            if (registerFrame[0x01].U16 & 0x01) {
                registerFrame[0x11].U16 = 0;
                registerFrame[0x12].U16 = 0;
                registerFrame[0x13].U16 = 0;
                registerFrame[0x14].U16 = 0;
                registerFrame[0x15].U16 = 0;
                registerFrame[0x16].U16 = 0;

                // Clear the Home command
                registerFrame[0x01].U16 &= ~0x01;
            }

            // If it was Go To Target, set position to exactly the target
            if (registerFrame[0x01].U16 & 0x08) {
                registerFrame[0x11].U16 = 100;
                registerFrame[0x12].U16 = 200;

                // Clear the Go To Target command
                registerFrame[0x01].U16 &= ~0x08;
            }

            // Reset motion state
            motionActive = 0;

            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {

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
