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
#include "stm32f0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define mic_Pin GPIO_PIN_6
#define mic_GPIO_Port GPIOA
#define mic_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */
#define Btm_Counter_Pin GPIO_PIN_13
#define Btm_Counter_GPIO_Port GPIOC
#define ADC_GAS_Pin GPIO_PIN_0
#define ADC_GAS_GPIO_Port GPIOC
#define ADC_BATTERY_Pin GPIO_PIN_1
#define ADC_BATTERY_GPIO_Port GPIOC
#define INA_Pin GPIO_PIN_2
#define INA_GPIO_Port GPIOC
#define INB_Pin GPIO_PIN_3
#define INB_GPIO_Port GPIOC
#define Btm_SW_Pin GPIO_PIN_1
#define Btm_SW_GPIO_Port GPIOA
#define STATUS_Pin GPIO_PIN_4
#define STATUS_GPIO_Port GPIOA
#define ADC_IN7_Pin GPIO_PIN_7
#define ADC_IN7_GPIO_Port GPIOA
#define INT_F2_Pin GPIO_PIN_1
#define INT_F2_GPIO_Port GPIOB
#define INT_F2_EXTI_IRQn EXTI0_1_IRQn
#define INT_F1_Pin GPIO_PIN_2
#define INT_F1_GPIO_Port GPIOB
#define INT_F1_EXTI_IRQn EXTI2_3_IRQn
#define Buzzer_Pin GPIO_PIN_10
#define Buzzer_GPIO_Port GPIOB
#define PiazoCouter_Pin GPIO_PIN_12
#define PiazoCouter_GPIO_Port GPIOB
#define PiazoCouter_EXTI_IRQn EXTI4_15_IRQn
#define Btm_Frequemcy_Pin GPIO_PIN_13
#define Btm_Frequemcy_GPIO_Port GPIOB
#define IntPulse_Pin GPIO_PIN_14
#define IntPulse_GPIO_Port GPIOB
#define IntPulse_EXTI_IRQn EXTI4_15_IRQn
#define MOTOR_Pin GPIO_PIN_8
#define MOTOR_GPIO_Port GPIOC
#define I03_Pin GPIO_PIN_8
#define I03_GPIO_Port GPIOA
#define PulseCounter_Pin GPIO_PIN_11
#define PulseCounter_GPIO_Port GPIOA
#define PulseCounter_EXTI_IRQn EXTI4_15_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define IO1_Pin GPIO_PIN_15
#define IO1_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_12
#define CS1_GPIO_Port GPIOC
#define CS2_Pin GPIO_PIN_2
#define CS2_GPIO_Port GPIOD
#define CS3_Pin GPIO_PIN_8
#define CS3_GPIO_Port GPIOB
#define CS4_Pin GPIO_PIN_9
#define CS4_GPIO_Port GPIOB
	float ewma_bat(float new_batt);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
