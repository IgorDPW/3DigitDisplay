/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define DIG1 GPIO_PIN_13
#define DIG2 GPIO_PIN_14
#define DIG3 GPIO_PIN_15
#define DS_pin GPIO_PIN_7
#define SHCP_pin GPIO_PIN_10
#define STCP_pin GPIO_PIN_6
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
#define DIGIT3_Pin GPIO_PIN_13
#define DIGIT3_GPIO_Port GPIOC
#define DIGIT2_Pin GPIO_PIN_14
#define DIGIT2_GPIO_Port GPIOC
#define DIGIT1_Pin GPIO_PIN_15
#define DIGIT1_GPIO_Port GPIOC
#define SHCP_pin_Pin GPIO_PIN_10
#define SHCP_pin_GPIO_Port GPIOB
#define STCP_pin_Pin GPIO_PIN_6
#define STCP_pin_GPIO_Port GPIOB
#define DS_pin_Pin GPIO_PIN_7
#define DS_pin_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
