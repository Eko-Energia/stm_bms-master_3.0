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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// including stm32's drivers' drivers



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
#define TEMP_Pin GPIO_PIN_0
#define TEMP_GPIO_Port GPIOC
#define HALL_OUT_Pin GPIO_PIN_1
#define HALL_OUT_GPIO_Port GPIOC
#define VOLTAGE_Pin GPIO_PIN_2
#define VOLTAGE_GPIO_Port GPIOC
#define HVIL_Pin GPIO_PIN_7
#define HVIL_GPIO_Port GPIOA
#define RS_DIR_Pin GPIO_PIN_4
#define RS_DIR_GPIO_Port GPIOC
#define RE_DIR_Pin GPIO_PIN_5
#define RE_DIR_GPIO_Port GPIOC
#define RELAY_CTRL_Pin GPIO_PIN_0
#define RELAY_CTRL_GPIO_Port GPIOB
#define FAN_CONTROL_Pin GPIO_PIN_1
#define FAN_CONTROL_GPIO_Port GPIOB
#define TX_EN_Pin GPIO_PIN_14
#define TX_EN_GPIO_Port GPIOB
#define TRX_CE_Pin GPIO_PIN_15
#define TRX_CE_GPIO_Port GPIOB
#define PWR_UP_Pin GPIO_PIN_6
#define PWR_UP_GPIO_Port GPIOC
#define DR_Pin GPIO_PIN_8
#define DR_GPIO_Port GPIOC
#define AM_Pin GPIO_PIN_9
#define AM_GPIO_Port GPIOC
#define CD_Pin GPIO_PIN_8
#define CD_GPIO_Port GPIOA
#define nCAN2_Stby_Pin GPIO_PIN_10
#define nCAN2_Stby_GPIO_Port GPIOC
#define nCAN1_Stby_Pin GPIO_PIN_11
#define nCAN1_Stby_GPIO_Port GPIOC
#define RED_LD_Pin GPIO_PIN_8
#define RED_LD_GPIO_Port GPIOB
#define GREEN_LD_Pin GPIO_PIN_9
#define GREEN_LD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
