/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
void TIM3_IRQHandler(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BP_LED_Pin GPIO_PIN_13
#define BP_LED_GPIO_Port GPIOC
#define HALL_C_Pin GPIO_PIN_14
#define HALL_C_GPIO_Port GPIOC
#define POWER_SW_Pin GPIO_PIN_15
#define POWER_SW_GPIO_Port GPIOC
#define HALL_B_Pin GPIO_PIN_1
#define HALL_B_GPIO_Port GPIOD
#define SOLENOID_PWM_Pin GPIO_PIN_0
#define SOLENOID_PWM_GPIO_Port GPIOA
#define TEST_POT_Pin GPIO_PIN_2
#define TEST_POT_GPIO_Port GPIOA
#define POT_SUPPLY_Pin GPIO_PIN_3
#define POT_SUPPLY_GPIO_Port GPIOA
#define V_BAT_Pin GPIO_PIN_4
#define V_BAT_GPIO_Port GPIOA
#define I_BAT_Pin GPIO_PIN_6
#define I_BAT_GPIO_Port GPIOA
#define POWER_LATCH_Pin GPIO_PIN_2
#define POWER_LATCH_GPIO_Port GPIOB
#define BUZZ_Pin GPIO_PIN_10
#define BUZZ_GPIO_Port GPIOB
#define HALL_A_Pin GPIO_PIN_11
#define HALL_A_GPIO_Port GPIOB
#define Y_LOW_Pin GPIO_PIN_13
#define Y_LOW_GPIO_Port GPIOB
#define B_LOW_Pin GPIO_PIN_14
#define B_LOW_GPIO_Port GPIOB
#define G_LOW_Pin GPIO_PIN_15
#define G_LOW_GPIO_Port GPIOB
#define Y_HIGH_Pin GPIO_PIN_8
#define Y_HIGH_GPIO_Port GPIOA
#define B_HIGH_Pin GPIO_PIN_9
#define B_HIGH_GPIO_Port GPIOA
#define G_HIGH_Pin GPIO_PIN_10
#define G_HIGH_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
