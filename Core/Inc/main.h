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
#include "board_rev.h"
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
#define PGOOD_5V_Pin GPIO_PIN_14
#define PGOOD_5V_GPIO_Port GPIOC
#define RESET_Pin GPIO_PIN_10
#define RESET_GPIO_Port GPIOG
#define ADC_VBUS_Pin GPIO_PIN_0
#define ADC_VBUS_GPIO_Port GPIOC
#define ADC_VBAT_Pin GPIO_PIN_0
#define ADC_VBAT_GPIO_Port GPIOA
#define I_IN_BUCK_Pin GPIO_PIN_1
#define I_IN_BUCK_GPIO_Port GPIOA
#define I_OUT_BOOST_Pin GPIO_PIN_3
#define I_OUT_BOOST_GPIO_Port GPIOA
#define STM_OTG_EN_Pin GPIO_PIN_4
#define STM_OTG_EN_GPIO_Port GPIOA
#define FAN_TACH_Pin GPIO_PIN_5
#define FAN_TACH_GPIO_Port GPIOA
#define FAN_PWM_Pin GPIO_PIN_7
#define FAN_PWM_GPIO_Port GPIOA
#define ADC_REMOTE_P_Pin GPIO_PIN_0
#define ADC_REMOTE_P_GPIO_Port GPIOB
#define ADC_REMOTE_N_Pin GPIO_PIN_1
#define ADC_REMOTE_N_GPIO_Port GPIOB
#define ADC_VOUT_Pin GPIO_PIN_2
#define ADC_VOUT_GPIO_Port GPIOB
#define HRTIM_FLT_Pin GPIO_PIN_10
#define HRTIM_FLT_GPIO_Port GPIOB
#define I_L_MEAS_Pin GPIO_PIN_11
#define I_L_MEAS_GPIO_Port GPIOB
#define ADC_LOCAL_VOUT_Pin GPIO_PIN_14
#define ADC_LOCAL_VOUT_GPIO_Port GPIOB
#define I_L_ZERO_Pin GPIO_PIN_15
#define I_L_ZERO_GPIO_Port GPIOB
#define BMS_ALERT_Pin GPIO_PIN_10
#define BMS_ALERT_GPIO_Port GPIOA
#define BOOST_TR_FLT_Pin GPIO_PIN_15
#define BOOST_TR_FLT_GPIO_Port GPIOA
#define BUCK_TR_EN_Pin GPIO_PIN_10
#define BUCK_TR_EN_GPIO_Port GPIOC
#define BUCK_TR_FLT_Pin GPIO_PIN_11
#define BUCK_TR_FLT_GPIO_Port GPIOC
#define BOOST_TR_EN_Pin GPIO_PIN_12
#define BOOST_TR_EN_GPIO_Port GPIOC
#define UART_H7_TX_Pin GPIO_PIN_4
#define UART_H7_TX_GPIO_Port GPIOC
#define UART_H7_RX_Pin GPIO_PIN_5
#define UART_H7_RX_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOD
#define BLEED_ON_Pin GPIO_PIN_5
#define BLEED_ON_GPIO_Port GPIOB
#define REMOTE_ON_Pin GPIO_PIN_6
#define REMOTE_ON_GPIO_Port GPIOB
#define POWER_PERMIT_G4_Pin GPIO_PIN_7
#define POWER_PERMIT_G4_GPIO_Port GPIOB
#define I2C_USBPD_IRQ_Pin GPIO_PIN_9
#define I2C_USBPD_IRQ_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LED1_Pin LED_Pin
#define LED1_GPIO_Port LED_GPIO_Port
#define LED2_Pin LED_Pin
#define LED2_GPIO_Port LED_GPIO_Port
#define OTG_EN_Pin STM_OTG_EN_Pin
#define OTG_EN_GPIO_Port STM_OTG_EN_GPIO_Port
#define FLT_Pin HRTIM_FLT_Pin
#define FLT_GPIO_Port HRTIM_FLT_GPIO_Port
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
