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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Pil_V__Pin GPIO_PIN_0
#define Pil_V__GPIO_Port GPIOA
#define RF_L1_Pin GPIO_PIN_2
#define RF_L1_GPIO_Port GPIOA
#define RF_L2_Pin GPIO_PIN_3
#define RF_L2_GPIO_Port GPIOA
#define RF_FOOT_Pin GPIO_PIN_4
#define RF_FOOT_GPIO_Port GPIOA
#define LF_FOOT_Pin GPIO_PIN_5
#define LF_FOOT_GPIO_Port GPIOA
#define RB_FOOT_Pin GPIO_PIN_6
#define RB_FOOT_GPIO_Port GPIOA
#define LB_FOOT_Pin GPIO_PIN_7
#define LB_FOOT_GPIO_Port GPIOA
#define RF_L3_Pin GPIO_PIN_0
#define RF_L3_GPIO_Port GPIOB
#define LF_L1_Pin GPIO_PIN_1
#define LF_L1_GPIO_Port GPIOB
#define GPS_TX_Pin GPIO_PIN_10
#define GPS_TX_GPIO_Port GPIOB
#define GPS_RX_Pin GPIO_PIN_11
#define GPS_RX_GPIO_Port GPIOB
#define Pin_CSN_Pin GPIO_PIN_12
#define Pin_CSN_GPIO_Port GPIOB
#define LF_L2_Pin GPIO_PIN_8
#define LF_L2_GPIO_Port GPIOA
#define LF_L3_Pin GPIO_PIN_9
#define LF_L3_GPIO_Port GPIOA
#define RB_L1_Pin GPIO_PIN_10
#define RB_L1_GPIO_Port GPIOA
#define RB_L2_Pin GPIO_PIN_11
#define RB_L2_GPIO_Port GPIOA
#define Pin_CE_Pin GPIO_PIN_12
#define Pin_CE_GPIO_Port GPIOA
#define RB_L3_Pin GPIO_PIN_15
#define RB_L3_GPIO_Port GPIOA
#define LB_L1_Pin GPIO_PIN_3
#define LB_L1_GPIO_Port GPIOB
#define LB_L2_Pin GPIO_PIN_4
#define LB_L2_GPIO_Port GPIOB
#define LB_L3_Pin GPIO_PIN_5
#define LB_L3_GPIO_Port GPIOB
#define Raspberry_TX_Pin GPIO_PIN_6
#define Raspberry_TX_GPIO_Port GPIOB
#define Raspberry_RX_Pin GPIO_PIN_7
#define Raspberry_RX_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_8
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_9
#define MPU_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
