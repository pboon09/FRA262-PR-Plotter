/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define RCC_OSC32_IN_Pin GPIO_PIN_14
#define RCC_OSC32_IN_GPIO_Port GPIOC
#define RCC_OSC32_OUT_Pin GPIO_PIN_15
#define RCC_OSC32_OUT_GPIO_Port GPIOC
#define RCC_OSC_IN_Pin GPIO_PIN_0
#define RCC_OSC_IN_GPIO_Port GPIOF
#define RCC_OSC_OUT_Pin GPIO_PIN_1
#define RCC_OSC_OUT_GPIO_Port GPIOF
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define UPPER_PHOTO_Pin GPIO_PIN_0
#define UPPER_PHOTO_GPIO_Port GPIOB
#define UPPER_PHOTO_EXTI_IRQn EXTI0_IRQn
#define LOWER_LIM_Pin GPIO_PIN_1
#define LOWER_LIM_GPIO_Port GPIOB
#define UPPER_LIM_Pin GPIO_PIN_2
#define UPPER_LIM_GPIO_Port GPIOB
#define LOWER_PHOTO_Pin GPIO_PIN_12
#define LOWER_PHOTO_GPIO_Port GPIOB
#define LOWER_PHOTO_EXTI_IRQn EXTI15_10_IRQn
#define upperphoto_Pin GPIO_PIN_8
#define upperphoto_GPIO_Port GPIOA
#define upperphoto_EXTI_IRQn EXTI9_5_IRQn
#define PILOT_Pin GPIO_PIN_10
#define PILOT_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define J3_Pin GPIO_PIN_10
#define J3_GPIO_Port GPIOC
#define J3_EXTI_IRQn EXTI15_10_IRQn
#define J4_Pin GPIO_PIN_11
#define J4_GPIO_Port GPIOC
#define J4_EXTI_IRQn EXTI15_10_IRQn
#define J2_Pin GPIO_PIN_12
#define J2_GPIO_Port GPIOC
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define EMER_Pin GPIO_PIN_4
#define EMER_GPIO_Port GPIOB
#define EMER_EXTI_IRQn EXTI4_IRQn
#define prox_Pin GPIO_PIN_5
#define prox_GPIO_Port GPIOB
#define prox_EXTI_IRQn EXTI9_5_IRQn
#define PROX_Pin GPIO_PIN_6
#define PROX_GPIO_Port GPIOB
#define PROX_EXTI_IRQn EXTI9_5_IRQn
#define J1_Pin GPIO_PIN_7
#define J1_GPIO_Port GPIOB
#define J1_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
