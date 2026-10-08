/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32c0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN1_IC3_Pin GPIO_PIN_15
#define IN1_IC3_GPIO_Port GPIOC
#define D0_Pin GPIO_PIN_0
#define D0_GPIO_Port GPIOA
#define ADC1_IN5_NO1_Pin GPIO_PIN_5
#define ADC1_IN5_NO1_GPIO_Port GPIOA
#define IN1_IC4_Pin GPIO_PIN_6
#define IN1_IC4_GPIO_Port GPIOA
#define ADC1_IN7_NO2_Pin GPIO_PIN_7
#define ADC1_IN7_NO2_GPIO_Port GPIOA
#define ADC1_IN17_NO3_Pin GPIO_PIN_0
#define ADC1_IN17_NO3_GPIO_Port GPIOB
#define ADC1_IN18_NO4_Pin GPIO_PIN_1
#define ADC1_IN18_NO4_GPIO_Port GPIOB
#define IN1_IC2_Pin GPIO_PIN_6
#define IN1_IC2_GPIO_Port GPIOC
#define L_GREEN4_Pin GPIO_PIN_10
#define L_GREEN4_GPIO_Port GPIOA
#define L_RED4_Pin GPIO_PIN_11
#define L_RED4_GPIO_Port GPIOA
#define IN1_IC1_Pin GPIO_PIN_15
#define IN1_IC1_GPIO_Port GPIOA
#define L_GREEN3_Pin GPIO_PIN_3
#define L_GREEN3_GPIO_Port GPIOB
#define L_RED3_Pin GPIO_PIN_4
#define L_RED3_GPIO_Port GPIOB
#define L_GREEN2_Pin GPIO_PIN_5
#define L_GREEN2_GPIO_Port GPIOB
#define L_RED2_Pin GPIO_PIN_6
#define L_RED2_GPIO_Port GPIOB
#define L_GREEN1_Pin GPIO_PIN_7
#define L_GREEN1_GPIO_Port GPIOB
#define L_RED1_Pin GPIO_PIN_8
#define L_RED1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
