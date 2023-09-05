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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#define CURRENT_C_Pin GPIO_PIN_0
#define CURRENT_C_GPIO_Port GPIOA
#define CURRENT_B_Pin GPIO_PIN_1
#define CURRENT_B_GPIO_Port GPIOA
#define CURRENT_A_Pin GPIO_PIN_2
#define CURRENT_A_GPIO_Port GPIOA
#define CURRENT_FILTER_Pin GPIO_PIN_3
#define CURRENT_FILTER_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOA
#define VOLTAGE_A_Pin GPIO_PIN_6
#define VOLTAGE_A_GPIO_Port GPIOA
#define VOLTAGE_B_Pin GPIO_PIN_7
#define VOLTAGE_B_GPIO_Port GPIOA
#define VOLTAGE_C_Pin GPIO_PIN_4
#define VOLTAGE_C_GPIO_Port GPIOC
#define HALL_A_Pin GPIO_PIN_0
#define HALL_A_GPIO_Port GPIOB
#define HALL_B_Pin GPIO_PIN_1
#define HALL_B_GPIO_Port GPIOB
#define VOLTAGE_BUS_Pin GPIO_PIN_12
#define VOLTAGE_BUS_GPIO_Port GPIOB
#define HALL_C_Pin GPIO_PIN_13
#define HALL_C_GPIO_Port GPIOB
#define TEMPERATURE_Pin GPIO_PIN_14
#define TEMPERATURE_GPIO_Port GPIOB
#define Enable_Pin GPIO_PIN_15
#define Enable_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_6
#define SPI1_CS_GPIO_Port GPIOB
#define Fault_Pin GPIO_PIN_7
#define Fault_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
