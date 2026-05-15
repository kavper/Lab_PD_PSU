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

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOC
#define RESET_Pin GPIO_PIN_10
#define RESET_GPIO_Port GPIOG
#define ADC_VBAT_Pin GPIO_PIN_0
#define ADC_VBAT_GPIO_Port GPIOA
#define ADC_VOUT_Pin GPIO_PIN_2
#define ADC_VOUT_GPIO_Port GPIOA
#define ADC_IOUT_Pin GPIO_PIN_6
#define ADC_IOUT_GPIO_Port GPIOA
#define SD_Pin GPIO_PIN_7
#define SD_GPIO_Port GPIOA
#define FLT_Pin GPIO_PIN_4
#define FLT_GPIO_Port GPIOC
#define STBY_Pin GPIO_PIN_5
#define STBY_GPIO_Port GPIOC
#define BTN_ON_OFF_Pin GPIO_PIN_0
#define BTN_ON_OFF_GPIO_Port GPIOB
#define BTN_AUX1_Pin GPIO_PIN_1
#define BTN_AUX1_GPIO_Port GPIOB
#define BTN_AUX2_Pin GPIO_PIN_2
#define BTN_AUX2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
