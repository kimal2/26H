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
#define KEY_3_Pin GPIO_PIN_2
#define KEY_3_GPIO_Port GPIOE
#define KEY_4_Pin GPIO_PIN_3
#define KEY_4_GPIO_Port GPIOE
#define MPU_SCL_Pin GPIO_PIN_3
#define MPU_SCL_GPIO_Port GPIOA
#define MPU_SDA_Pin GPIO_PIN_4
#define MPU_SDA_GPIO_Port GPIOA
#define GRAY_SCL_Pin GPIO_PIN_4
#define GRAY_SCL_GPIO_Port GPIOC
#define GRAY_SDA_Pin GPIO_PIN_5
#define GRAY_SDA_GPIO_Port GPIOC
#define GRAY_KEY_Pin GPIO_PIN_15
#define GRAY_KEY_GPIO_Port GPIOD
#define LED_2_Pin GPIO_PIN_8
#define LED_2_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_9
#define LED_1_GPIO_Port GPIOC
#define LED_4_Pin GPIO_PIN_4
#define LED_4_GPIO_Port GPIOD
#define LED_3_Pin GPIO_PIN_6
#define LED_3_GPIO_Port GPIOD
#define OLED_SDA_Pin GPIO_PIN_8
#define OLED_SDA_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_9
#define OLED_SCL_GPIO_Port GPIOB
#define KEY_1_Pin GPIO_PIN_0
#define KEY_1_GPIO_Port GPIOE
#define KEY_2_Pin GPIO_PIN_1
#define KEY_2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
