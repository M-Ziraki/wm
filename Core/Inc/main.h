/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Motor_Ui1_Pin GPIO_PIN_13
#define Motor_Ui1_GPIO_Port GPIOC
#define pushonkey_Pin GPIO_PIN_1
#define pushonkey_GPIO_Port GPIOA
#define pushonkey_EXTI_IRQn EXTI1_IRQn
#define Motor_Pin GPIO_PIN_2
#define Motor_GPIO_Port GPIOA
#define Freq_in_Pin GPIO_PIN_3
#define Freq_in_GPIO_Port GPIOA
#define Freq_in_EXTI_IRQn EXTI3_IRQn
#define Taco_PLS_Pin GPIO_PIN_4
#define Taco_PLS_GPIO_Port GPIOA
#define Taco_PLS_EXTI_IRQn EXTI4_IRQn
#define Hydro_Pin GPIO_PIN_5
#define Hydro_GPIO_Port GPIOA
#define Hydro_EXTI_IRQn EXTI9_5_IRQn
#define Buzzer_Pin GPIO_PIN_0
#define Buzzer_GPIO_Port GPIOB
#define Door_Open_Pin GPIO_PIN_1
#define Door_Open_GPIO_Port GPIOB
#define Door_Lock_Pin GPIO_PIN_2
#define Door_Lock_GPIO_Port GPIOB
#define Drain_Pomp_Pin GPIO_PIN_10
#define Drain_Pomp_GPIO_Port GPIOB
#define Valve3_Pin GPIO_PIN_11
#define Valve3_GPIO_Port GPIOB
#define Valve2_Pin GPIO_PIN_12
#define Valve2_GPIO_Port GPIOB
#define Valve1_Pin GPIO_PIN_13
#define Valve1_GPIO_Port GPIOB
#define Heater_Pin GPIO_PIN_14
#define Heater_GPIO_Port GPIOB
#define ESPRST_Pin GPIO_PIN_11
#define ESPRST_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_12
#define LED_GPIO_Port GPIOA
#define key1_Pin GPIO_PIN_3
#define key1_GPIO_Port GPIOB
#define MasterRLY_Pin GPIO_PIN_4
#define MasterRLY_GPIO_Port GPIOB
#define Motor_Ui2_Pin GPIO_PIN_5
#define Motor_Ui2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
