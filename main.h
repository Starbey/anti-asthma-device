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
#define GREEN_LED_Pin GPIO_PIN_0
#define GREEN_LED_GPIO_Port GPIOC
#define RED_LED_Pin GPIO_PIN_1
#define RED_LED_GPIO_Port GPIOC
#define LCD_DB7_Pin GPIO_PIN_5
#define LCD_DB7_GPIO_Port GPIOA
#define LCD_DB6_Pin GPIO_PIN_6
#define LCD_DB6_GPIO_Port GPIOA
#define LCD_DB5_Pin GPIO_PIN_7
#define LCD_DB5_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_10
#define LCD_RS_GPIO_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_8
#define LCD_RW_GPIO_Port GPIOA
#define LCD_E_Pin GPIO_PIN_9
#define LCD_E_GPIO_Port GPIOA
#define DHT22_Pin GPIO_PIN_10
#define DHT22_GPIO_Port GPIOA
#define RED_BUTTON_Pin GPIO_PIN_3
#define RED_BUTTON_GPIO_Port GPIOB
#define RED_BUTTON_EXTI_IRQn EXTI3_IRQn
#define BUZZER_Pin GPIO_PIN_4
#define BUZZER_GPIO_Port GPIOB
#define YELL_BUTTON_Pin GPIO_PIN_5
#define YELL_BUTTON_GPIO_Port GPIOB
#define YELL_BUTTON_EXTI_IRQn EXTI9_5_IRQn
#define LCD_DB4_Pin GPIO_PIN_6
#define LCD_DB4_GPIO_Port GPIOB
#define P_SENSOR_SCL_Pin GPIO_PIN_8
#define P_SENSOR_SCL_GPIO_Port GPIOB
#define P_SENSOR_SDA_Pin GPIO_PIN_9
#define P_SENSOR_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
