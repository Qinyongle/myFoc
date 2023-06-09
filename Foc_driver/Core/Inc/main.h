/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define TIM_1_8_PERIOD_CLOCKS 3500
#define M0_CS_Pin GPIO_PIN_13
#define M0_CS_GPIO_Port GPIOC
#define M0_SO1_Pin GPIO_PIN_0
#define M0_SO1_GPIO_Port GPIOC
#define M0_SO2_Pin GPIO_PIN_1
#define M0_SO2_GPIO_Port GPIOC
#define EN_GATE_Pin GPIO_PIN_12
#define EN_GATE_GPIO_Port GPIOB
#define M0_AL_Pin GPIO_PIN_13
#define M0_AL_GPIO_Port GPIOB
#define M0_BL_Pin GPIO_PIN_14
#define M0_BL_GPIO_Port GPIOB
#define M0_CL_Pin GPIO_PIN_15
#define M0_CL_GPIO_Port GPIOB
#define M0_AH_Pin GPIO_PIN_8
#define M0_AH_GPIO_Port GPIOA
#define M0_BH_Pin GPIO_PIN_9
#define M0_BH_GPIO_Port GPIOA
#define M0_CH_Pin GPIO_PIN_10
#define M0_CH_GPIO_Port GPIOA
#define M0_SCK_Pin GPIO_PIN_10
#define M0_SCK_GPIO_Port GPIOC
#define M0_MISO_Pin GPIO_PIN_11
#define M0_MISO_GPIO_Port GPIOC
#define M0_MOSI_Pin GPIO_PIN_12
#define M0_MOSI_GPIO_Port GPIOC
#define nFalut_Pin GPIO_PIN_2
#define nFalut_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
#define TIM_TIME_BASE TIM14
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
