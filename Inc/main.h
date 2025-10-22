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
#define CS_Pin GPIO_PIN_13
#define CS_GPIO_Port GPIOC
#define INT_IMU_Pin GPIO_PIN_14
#define INT_IMU_GPIO_Port GPIOC
#define TIMEPULSE_Pin GPIO_PIN_15
#define TIMEPULSE_GPIO_Port GPIOC
#define INT_SAM_Pin GPIO_PIN_0
#define INT_SAM_GPIO_Port GPIOA
#define RST_SAM_Pin GPIO_PIN_1
#define RST_SAM_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_4
#define LED3_GPIO_Port GPIOC
#define M1_Pin GPIO_PIN_12
#define M1_GPIO_Port GPIOB
#define M0_Pin GPIO_PIN_13
#define M0_GPIO_Port GPIOB
#define AUX_Pin GPIO_PIN_14
#define AUX_GPIO_Port GPIOB
#define CONFIG_Pin GPIO_PIN_15
#define CONFIG_GPIO_Port GPIOB
#define RST_BT_Pin GPIO_PIN_6
#define RST_BT_GPIO_Port GPIOC
#define LPM_Pin GPIO_PIN_8
#define LPM_GPIO_Port GPIOA
#define INT_ACC_Pin GPIO_PIN_13
#define INT_ACC_GPIO_Port GPIOA
#define INT_BMP_Pin GPIO_PIN_6
#define INT_BMP_GPIO_Port GPIOB
#define INT_MAG_Pin GPIO_PIN_7
#define INT_MAG_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
