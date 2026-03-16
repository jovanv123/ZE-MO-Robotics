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
#include "stm32h7xx_hal.h"

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
#define MOTOR1_SLP_Pin GPIO_PIN_1
#define MOTOR1_SLP_GPIO_Port GPIOF
#define MOTOR2_SLP_Pin GPIO_PIN_2
#define MOTOR2_SLP_GPIO_Port GPIOF
#define MOTOR1_DIR_Pin GPIO_PIN_5
#define MOTOR1_DIR_GPIO_Port GPIOF
#define MOTOR2_DIR_Pin GPIO_PIN_6
#define MOTOR2_DIR_GPIO_Port GPIOF
#define MS1_1_MICROSTEP_Pin GPIO_PIN_7
#define MS1_1_MICROSTEP_GPIO_Port GPIOF
#define STEP1_DIR_Pin GPIO_PIN_8
#define STEP1_DIR_GPIO_Port GPIOF
#define STEPDRIVER1_EN_Pin GPIO_PIN_9
#define STEPDRIVER1_EN_GPIO_Port GPIOF
#define TIM2_CH1_ENKODER2_Pin GPIO_PIN_0
#define TIM2_CH1_ENKODER2_GPIO_Port GPIOA
#define TIM2_CH2_ENKODER2_Pin GPIO_PIN_1
#define TIM2_CH2_ENKODER2_GPIO_Port GPIOA
#define MS1_2_MICROSTEP_Pin GPIO_PIN_11
#define MS1_2_MICROSTEP_GPIO_Port GPIOF
#define STEPDRIVER2_EN_Pin GPIO_PIN_12
#define STEPDRIVER2_EN_GPIO_Port GPIOF
#define STEP2_DIR_Pin GPIO_PIN_13
#define STEP2_DIR_GPIO_Port GPIOF
#define MS2_1_MICROSTEP_Pin GPIO_PIN_14
#define MS2_1_MICROSTEP_GPIO_Port GPIOF
#define MS2_2_MICROSTEP_Pin GPIO_PIN_15
#define MS2_2_MICROSTEP_GPIO_Port GPIOF
#define TIM1_CH1_ENKODER1_Pin GPIO_PIN_9
#define TIM1_CH1_ENKODER1_GPIO_Port GPIOE
#define TIM1_CH2_ENKODER1_Pin GPIO_PIN_11
#define TIM1_CH2_ENKODER1_GPIO_Port GPIOE
#define TIM4_CH1_STEPPER1_Pin GPIO_PIN_12
#define TIM4_CH1_STEPPER1_GPIO_Port GPIOD
#define TIM4_CH2_STEPPER2_Pin GPIO_PIN_13
#define TIM4_CH2_STEPPER2_GPIO_Port GPIOD
#define TIM3_CH1_MOTOR2_Pin GPIO_PIN_6
#define TIM3_CH1_MOTOR2_GPIO_Port GPIOC
#define TIM3_CH2_MOTOR1_Pin GPIO_PIN_7
#define TIM3_CH2_MOTOR1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
