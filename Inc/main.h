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
#define BUFFER_EN1_Pin GPIO_PIN_13
#define BUFFER_EN1_GPIO_Port GPIOC
#define MUX_ADD0_Pin GPIO_PIN_0
#define MUX_ADD0_GPIO_Port GPIOC
#define MUX_ADD1_Pin GPIO_PIN_1
#define MUX_ADD1_GPIO_Port GPIOC
#define MUX_ADD2_Pin GPIO_PIN_2
#define MUX_ADD2_GPIO_Port GPIOC
#define MUX_ADD3_Pin GPIO_PIN_3
#define MUX_ADD3_GPIO_Port GPIOC
#define BUFFER_EN0_Pin GPIO_PIN_2
#define BUFFER_EN0_GPIO_Port GPIOA
#define DEBUGE_TO_SLOT_Pin GPIO_PIN_3
#define DEBUGE_TO_SLOT_GPIO_Port GPIOA
#define SLOT_EN_00_Pin GPIO_PIN_4
#define SLOT_EN_00_GPIO_Port GPIOA
#define SLOT_EN_01_Pin GPIO_PIN_5
#define SLOT_EN_01_GPIO_Port GPIOA
#define SLOT_EN_02_Pin GPIO_PIN_6
#define SLOT_EN_02_GPIO_Port GPIOA
#define SLOT_EN_03_Pin GPIO_PIN_7
#define SLOT_EN_03_GPIO_Port GPIOA
#define SLOT_EN_04_Pin GPIO_PIN_4
#define SLOT_EN_04_GPIO_Port GPIOC
#define SLOT_EN_05_Pin GPIO_PIN_5
#define SLOT_EN_05_GPIO_Port GPIOC
#define LED_12_Pin GPIO_PIN_0
#define LED_12_GPIO_Port GPIOB
#define LED_13_Pin GPIO_PIN_1
#define LED_13_GPIO_Port GPIOB
#define LED_14_Pin GPIO_PIN_2
#define LED_14_GPIO_Port GPIOB
#define MUX_EN0_Pin GPIO_PIN_10
#define MUX_EN0_GPIO_Port GPIOB
#define MUX_EN1_Pin GPIO_PIN_11
#define MUX_EN1_GPIO_Port GPIOB
#define LED_01_Pin GPIO_PIN_12
#define LED_01_GPIO_Port GPIOB
#define LED_02_Pin GPIO_PIN_13
#define LED_02_GPIO_Port GPIOB
#define LED_03_Pin GPIO_PIN_14
#define LED_03_GPIO_Port GPIOB
#define LED_04_Pin GPIO_PIN_15
#define LED_04_GPIO_Port GPIOB
#define LED_05_Pin GPIO_PIN_6
#define LED_05_GPIO_Port GPIOC
#define LED_06_Pin GPIO_PIN_7
#define LED_06_GPIO_Port GPIOC
#define LED_07_Pin GPIO_PIN_8
#define LED_07_GPIO_Port GPIOC
#define LED_08_Pin GPIO_PIN_9
#define LED_08_GPIO_Port GPIOC
#define LED_09_Pin GPIO_PIN_8
#define LED_09_GPIO_Port GPIOA
#define LED_10_Pin GPIO_PIN_9
#define LED_10_GPIO_Port GPIOA
#define LED_11_Pin GPIO_PIN_10
#define LED_11_GPIO_Port GPIOA
#define LED_15_Pin GPIO_PIN_10
#define LED_15_GPIO_Port GPIOC
#define LED_16_Pin GPIO_PIN_11
#define LED_16_GPIO_Port GPIOC
#define LED_RELAY_Pin GPIO_PIN_12
#define LED_RELAY_GPIO_Port GPIOC
#define RELAY_SEL_Pin GPIO_PIN_2
#define RELAY_SEL_GPIO_Port GPIOD
#define DEBUGE_TO_MCU_Pin GPIO_PIN_5
#define DEBUGE_TO_MCU_GPIO_Port GPIOB
#define UART1_TX_EN_Pin GPIO_PIN_8
#define UART1_TX_EN_GPIO_Port GPIOB
#define MCU_BLANK_01_Pin GPIO_PIN_9
#define MCU_BLANK_01_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
