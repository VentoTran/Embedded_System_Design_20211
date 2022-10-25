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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"

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
#define GPIO_PUMP_Pin GPIO_PIN_13
#define GPIO_PUMP_GPIO_Port GPIOC
#define ADC_SOIL_Pin GPIO_PIN_0
#define ADC_SOIL_GPIO_Port GPIOA
#define ADC_V_IN_Pin GPIO_PIN_1
#define ADC_V_IN_GPIO_Port GPIOA
#define ADC_V_OUT_Pin GPIO_PIN_3
#define ADC_V_OUT_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_7
#define LCD_RS_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_0
#define LCD_DC_GPIO_Port GPIOB
#define LCD_LED_Pin GPIO_PIN_1
#define LCD_LED_GPIO_Port GPIOB
#define GPIO_B_STATE_Pin GPIO_PIN_12
#define GPIO_B_STATE_GPIO_Port GPIOB
#define T_IRQ_Pin GPIO_PIN_8
#define T_IRQ_GPIO_Port GPIOA
#define T_IRQ_EXTI_IRQn EXTI9_5_IRQn
#define SPI1_SS_TOUCH_Pin GPIO_PIN_11
#define SPI1_SS_TOUCH_GPIO_Port GPIOA
#define SPI1_SS_LCD_Pin GPIO_PIN_12
#define SPI1_SS_LCD_GPIO_Port GPIOA
#define PWM1_Pin GPIO_PIN_15
#define PWM1_GPIO_Port GPIOA
#define GPIO_A_STATE_Pin GPIO_PIN_4
#define GPIO_A_STATE_GPIO_Port GPIOB
#define GPIO_SIM_Pin GPIO_PIN_5
#define GPIO_SIM_GPIO_Port GPIOB
#define GPIO_ADAPTER_Pin GPIO_PIN_8
#define GPIO_ADAPTER_GPIO_Port GPIOB
#define GPIO_SOLAR_Pin GPIO_PIN_9
#define GPIO_SOLAR_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
