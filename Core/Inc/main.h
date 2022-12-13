/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h> 

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
#define adc_USART1_detect_Pin GPIO_PIN_1
#define adc_USART1_detect_GPIO_Port GPIOC
#define adc_USART_aux_detect_Pin GPIO_PIN_3
#define adc_USART_aux_detect_GPIO_Port GPIOC
#define led_TIMER_Pin GPIO_PIN_5
#define led_TIMER_GPIO_Port GPIOA
#define led_GREEN_Pin GPIO_PIN_12
#define led_GREEN_GPIO_Port GPIOB
#define drv_PUL_Pin GPIO_PIN_15
#define drv_PUL_GPIO_Port GPIOA
#define drv_DIR_Pin GPIO_PIN_10
#define drv_DIR_GPIO_Port GPIOC
#define drv_EN_Pin GPIO_PIN_12
#define drv_EN_GPIO_Port GPIOC
#define btn_STOP_Pin GPIO_PIN_4
#define btn_STOP_GPIO_Port GPIOB
#define btn_START_Pin GPIO_PIN_5
#define btn_START_GPIO_Port GPIOB
#define led_BLUE_Pin GPIO_PIN_8
#define led_BLUE_GPIO_Port GPIOB
#define led_RED_Pin GPIO_PIN_9
#define led_RED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
