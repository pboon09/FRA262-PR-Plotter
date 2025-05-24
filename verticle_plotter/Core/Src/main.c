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
	float32_t position_setpoint;   // Desired position
	float32_t velocity_setpoint;   // Desired velocity
	float32_t kalman_velocity;     // Kalman filtered velocity
	float32_t input_voltage;       // Motor input voltage
	float32_t pos_error;           // Position error for control
	float32_t vel_error;           // Velocity error for control
	float32_t command_vel;         // Velocity command from position controller
	float32_t command_pos;         // PWM command output
	float32_t dfd;                 // Disturbance feedforward
	float32_t ffd;                 // Velocity feedforward
} revoluteAxis_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Axis state structures
revoluteAxis_t  revolute_axis = { 0 };

volatile uint32_t position_control_tick = 0;
const uint32_t POSITION_CONTROL_DIVIDER = 10; // 1000Hz / 10 = 100Hz
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void update_revolute_position_control(void);
void update_revolute_velocity_control(void);
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

	// Initialize revolute axis
	revolute_axis.position_setpoint = 0.0f;
	revolute_axis.velocity_setpoint = 0.0f;
	revolute_axis.pos_error = 0.0f;
	revolute_axis.vel_error = 0.0f;
	revolute_axis.command_vel = 0.0f;
	revolute_axis.command_pos = 0.0f;
	revolute_axis.dfd = 0.0f;
	revolute_axis.ffd = 0.0f;

	// Initialize position control tick counter
	position_control_tick = 0;
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
void update_revolute_position_control(void) {
	// Calculate position error
	revolute_axis.pos_error = revolute_axis.position_setpoint - revolute_encoder.mm;

	// Position PID control to generate velocity command
	revolute_axis.command_vel = PWM_Satuation(
			PID_CONTROLLER_Compute(&revolute_position_pid, revolute_axis.pos_error),
			ZGX45RGG_150RPM_Constant.qd_max,
			-ZGX45RGG_150RPM_Constant.qd_max);
}

void update_revolute_velocity_control(void) {
	// Calculate velocity error (combines position command and velocity setpoint)
	revolute_axis.vel_error = revolute_axis.command_vel + revolute_axis.velocity_setpoint - revolute_axis.kalman_velocity;

	// Velocity PID control
	revolute_axis.command_pos = PWM_Satuation(
			PID_CONTROLLER_Compute(&revolute_velocity_pid, revolute_axis.vel_error),
			ZGX45RGG_150RPM_Constant.U_max,
			-ZGX45RGG_150RPM_Constant.U_max);

	// Calculate feedforward terms
//	revolute_axis.ffd = revolute_MOTOR_FFD_Compute(&revolute_motor_ffd,
//			revolute_axis.velocity_setpoint / 1000.0f);
//
//	revolute_axis.dfd = revolute_MOTOR_DFD_Compute(&revolute_motor_dfd,
//			revolute_encoder.rads, 0.0f, revolute_encoder.mm / 1000.0f);

	// Add feedforward compensation
//	revolute_axis.command_pos += revolute_axis.ffd + revolute_axis.dfd;

	// Final saturation
	revolute_axis.command_pos = PWM_Satuation(revolute_axis.command_pos,
			ZGX45RGG_150RPM_Constant.U_max, -ZGX45RGG_150RPM_Constant.U_max);

	// Apply command to motor
	MDXX_set_range(&revolute_motor, 2000, revolute_axis.command_pos);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim2) {
		// Update sensor readings
		plotter_update_sensors();

		QEI_get_diff_count(&revolute_encoder);
		QEI_compute_data(&revolute_encoder);
		QEI_get_diff_count(&revolute_encoder);
		QEI_compute_data(&revolute_encoder);

		// Calculate revolute motor input voltage for Kalman filter
		revolute_axis.input_voltage = mapf(revolute_axis.command_pos,
				-65535.0f, 65535.0f, -12.0f, 12.0f);

		// Update Kalman filtered velocity
		revolute_axis.kalman_velocity = SteadyStateKalmanFilter(&revolute_kalman,
				revolute_axis.input_voltage, revolute_encoder.rads);

		if (isnan(revolute_axis.kalman_velocity)) {
			revolute_axis.kalman_velocity = 0.0f;
		}

		// Increment tick counter
		position_control_tick++;

		// Position control at 100Hz (every 10 ticks at 1000Hz)
		if (position_control_tick >= POSITION_CONTROL_DIVIDER) {
			position_control_tick = 0;

			// Generate position setpoint (example: sine wave)
			revolute_axis.position_setpoint += revolute_axis.velocity_setpoint * 0.01;
			revolute_axis.velocity_setpoint = SIGNAL_generate(&sine_sg_revolute, 0.01f);

			// Update position control
			update_revolute_position_control();
		}
		// Velocity control runs at full 1000Hz
		update_revolute_velocity_control();
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
