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
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define KEY_ENC1_Pin GPIO_PIN_0
#define KEY_ENC1_GPIO_Port GPIOA
#define KEY_ENC2_Pin GPIO_PIN_1
#define KEY_ENC2_GPIO_Port GPIOA
#define PWM_IRON_Pin GPIO_PIN_2
#define PWM_IRON_GPIO_Port GPIOA
#define TFT_CS_Pin GPIO_PIN_3
#define TFT_CS_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_4
#define FLASH_CS_GPIO_Port GPIOA
#define KEY_ENC3_Pin GPIO_PIN_2
#define KEY_ENC3_GPIO_Port GPIOB
#define RELAY_Pin GPIO_PIN_10
#define RELAY_GPIO_Port GPIOB
#define TFT_RST_Pin GPIO_PIN_12
#define TFT_RST_GPIO_Port GPIOB
#define TFT_DC_Pin GPIO_PIN_13
#define TFT_DC_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOB
#define ENC1_B_Pin GPIO_PIN_8
#define ENC1_B_GPIO_Port GPIOA
#define ENC1_A_Pin GPIO_PIN_9
#define ENC1_A_GPIO_Port GPIOA
#define SW_AIR_Pin GPIO_PIN_10
#define SW_AIR_GPIO_Port GPIOA
#define ENC3_B_Pin GPIO_PIN_15
#define ENC3_B_GPIO_Port GPIOA
#define ENC3_A_Pin GPIO_PIN_3
#define ENC3_A_GPIO_Port GPIOB
#define ENC2_B_Pin GPIO_PIN_4
#define ENC2_B_GPIO_Port GPIOB
#define ENC2_A_Pin GPIO_PIN_5
#define ENC2_A_GPIO_Port GPIOB
#define PWM_AIR_Pin GPIO_PIN_6
#define PWM_AIR_GPIO_Port GPIOB
#define ZERO_CROSS_Pin GPIO_PIN_7
#define ZERO_CROSS_GPIO_Port GPIOB
#define PWM_TEMP_Pin GPIO_PIN_8
#define PWM_TEMP_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
