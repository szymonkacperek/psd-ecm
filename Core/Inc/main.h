/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f4xx_hal.h"

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
#define OPTO_INPUT2_Pin GPIO_PIN_2
#define OPTO_INPUT2_GPIO_Port GPIOE
#define OPTO_INPUT2_EXTI_IRQn EXTI2_IRQn
#define OPTO_INPUT3_Pin GPIO_PIN_3
#define OPTO_INPUT3_GPIO_Port GPIOE
#define OPTO_INPUT3_EXTI_IRQn EXTI3_IRQn
#define OPTO_INPUT4_Pin GPIO_PIN_4
#define OPTO_INPUT4_GPIO_Port GPIOE
#define OPTO_INPUT4_EXTI_IRQn EXTI4_IRQn
#define OPTO_INPUT5_Pin GPIO_PIN_5
#define OPTO_INPUT5_GPIO_Port GPIOE
#define OPTO_INPUT5_EXTI_IRQn EXTI9_5_IRQn
#define OPTO_INPUT6_Pin GPIO_PIN_6
#define OPTO_INPUT6_GPIO_Port GPIOE
#define OPTO_INPUT6_EXTI_IRQn EXTI9_5_IRQn
#define LED_D4_Pin GPIO_PIN_13
#define LED_D4_GPIO_Port GPIOC
#define LED_D5_Pin GPIO_PIN_14
#define LED_D5_GPIO_Port GPIOC
#define LED_D6_Pin GPIO_PIN_15
#define LED_D6_GPIO_Port GPIOC
#define OPTO_INPUT15_Pin GPIO_PIN_7
#define OPTO_INPUT15_GPIO_Port GPIOE
#define OPTO_INPUT15_EXTI_IRQn EXTI9_5_IRQn
#define OPTO_INPUT14_Pin GPIO_PIN_8
#define OPTO_INPUT14_GPIO_Port GPIOE
#define OPTO_INPUT14_EXTI_IRQn EXTI9_5_IRQn
#define OPTO_INPUT13_Pin GPIO_PIN_9
#define OPTO_INPUT13_GPIO_Port GPIOE
#define OPTO_INPUT13_EXTI_IRQn EXTI9_5_IRQn
#define OPTO_INPUT12_Pin GPIO_PIN_10
#define OPTO_INPUT12_GPIO_Port GPIOE
#define OPTO_INPUT12_EXTI_IRQn EXTI15_10_IRQn
#define OPTO_INPUT11_Pin GPIO_PIN_11
#define OPTO_INPUT11_GPIO_Port GPIOE
#define OPTO_INPUT11_EXTI_IRQn EXTI15_10_IRQn
#define OPTO_INPUT10_Pin GPIO_PIN_12
#define OPTO_INPUT10_GPIO_Port GPIOE
#define OPTO_INPUT10_EXTI_IRQn EXTI15_10_IRQn
#define OPTO_INPUT9_Pin GPIO_PIN_13
#define OPTO_INPUT9_GPIO_Port GPIOE
#define OPTO_INPUT9_EXTI_IRQn EXTI15_10_IRQn
#define OPTO_INPUT8_Pin GPIO_PIN_14
#define OPTO_INPUT8_GPIO_Port GPIOE
#define OPTO_INPUT8_EXTI_IRQn EXTI15_10_IRQn
#define OPTO_INPUT7_Pin GPIO_PIN_15
#define OPTO_INPUT7_GPIO_Port GPIOE
#define OPTO_INPUT7_EXTI_IRQn EXTI15_10_IRQn
#define CAN1_RS_Pin GPIO_PIN_10
#define CAN1_RS_GPIO_Port GPIOA
#define CAN2_RS_Pin GPIO_PIN_7
#define CAN2_RS_GPIO_Port GPIOB
#define OPTO_INPUT1_Pin GPIO_PIN_1
#define OPTO_INPUT1_GPIO_Port GPIOE
#define OPTO_INPUT1_EXTI_IRQn EXTI1_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
