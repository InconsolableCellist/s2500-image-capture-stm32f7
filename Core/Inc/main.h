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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t current_adc_mode;
extern uint8_t mode_switch_requested;
extern uint16_t tim7_overflow;
extern uint8_t pulse_fired;
extern uint8_t adc_int;
extern uint8_t usb_transfer_complete;
volatile extern uint8_t rising_or_falling;
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
#define OSC_25M_Pin GPIO_PIN_0
#define OSC_25M_GPIO_Port GPIOH
#define DEBUG_PIN_Pin GPIO_PIN_6
#define DEBUG_PIN_GPIO_Port GPIOC
#define ADC_STATUS_PIN_Pin GPIO_PIN_6
#define ADC_STATUS_PIN_GPIO_Port GPIOF
#define MODE_SWITCH_Pin GPIO_PIN_3
#define MODE_SWITCH_GPIO_Port GPIOJ
#define MODE_SWITCH_EXTI_IRQn EXTI3_IRQn
#define EXT_RST_Pin GPIO_PIN_7
#define EXT_RST_GPIO_Port GPIOH
#define ADC_SIGNAL_Pin GPIO_PIN_6
#define ADC_SIGNAL_GPIO_Port GPIOA
#define XY_PULSE_Pin GPIO_PIN_0
#define XY_PULSE_GPIO_Port GPIOJ
#define XY_PULSE_EXTI_IRQn EXTI0_IRQn
#define USB_STATUS_PIN_Pin GPIO_PIN_1
#define USB_STATUS_PIN_GPIO_Port GPIOJ
/* USER CODE BEGIN Private defines */
#define USB_STATUS_HIGH HAL_GPIO_WritePin(USB_STATUS_PIN_GPIO_Port, USB_STATUS_PIN_Pin, GPIO_PIN_SET);
#define USB_STATUS_LOW  HAL_GPIO_WritePin(USB_STATUS_PIN_GPIO_Port, USB_STATUS_PIN_Pin, GPIO_PIN_RESET);

#define DEBUG_HIGH HAL_GPIO_WritePin(DEBUG_PIN_GPIO_Port, DEBUG_PIN_Pin, GPIO_PIN_SET);
#define DEBUG_LOW  HAL_GPIO_WritePin(DEBUG_PIN_GPIO_Port, DEBUG_PIN_Pin, GPIO_PIN_RESET);

#define ADC_STATUS_HIGH HAL_GPIO_WritePin(ADC_STATUS_PIN_GPIO_Port, ADC_STATUS_PIN_Pin, GPIO_PIN_SET);
#define ADC_STATUS_LOW  HAL_GPIO_WritePin(ADC_STATUS_PIN_GPIO_Port, ADC_STATUS_PIN_Pin, GPIO_PIN_RESET);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
