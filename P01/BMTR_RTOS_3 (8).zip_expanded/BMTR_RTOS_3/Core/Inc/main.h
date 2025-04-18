/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

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
#define USER_BTN_Pin GPIO_PIN_13
#define USER_BTN_GPIO_Port GPIOC
#define USER_BTN_EXTI_IRQn EXTI15_10_IRQn
#define __Pin GPIO_PIN_3
#define __GPIO_Port GPIOF
#define SD_Indication_Pin GPIO_PIN_9
#define SD_Indication_GPIO_Port GPIOF
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define MOTOR2_DIREC_Pin GPIO_PIN_2
#define MOTOR2_DIREC_GPIO_Port GPIOA
#define M_DRIVER2_ENABLE_Pin GPIO_PIN_3
#define M_DRIVER2_ENABLE_GPIO_Port GPIOA
#define Mode_Selection_1_Pin GPIO_PIN_5
#define Mode_Selection_1_GPIO_Port GPIOA
#define Mode_Selection_2_Pin GPIO_PIN_6
#define Mode_Selection_2_GPIO_Port GPIOA
#define Mode_Selection_3_Pin GPIO_PIN_7
#define Mode_Selection_3_GPIO_Port GPIOA
#define Error_Indication_2_Pin GPIO_PIN_5
#define Error_Indication_2_GPIO_Port GPIOC
#define SD_SC_LD1_Pin GPIO_PIN_0
#define SD_SC_LD1_GPIO_Port GPIOB
#define Error_Indication_1_Pin GPIO_PIN_2
#define Error_Indication_1_GPIO_Port GPIOB
#define MST_SLV_SHIFT_Pin GPIO_PIN_12
#define MST_SLV_SHIFT_GPIO_Port GPIOF
#define SD_Writing_Indicator_Pin GPIO_PIN_14
#define SD_Writing_Indicator_GPIO_Port GPIOF
#define SD_WRITING_EN_SWITCH_PIN_Pin GPIO_PIN_15
#define SD_WRITING_EN_SWITCH_PIN_GPIO_Port GPIOF
#define Encoder1_ch1_Pin GPIO_PIN_9
#define Encoder1_ch1_GPIO_Port GPIOE
#define Encoder1_ch2_Pin GPIO_PIN_11
#define Encoder1_ch2_GPIO_Port GPIOE
#define START_STOP_BTN_Pin GPIO_PIN_14
#define START_STOP_BTN_GPIO_Port GPIOE
#define START_STOP_BTN_EXTI_IRQn EXTI15_10_IRQn
#define CHANGE_EXERCISE_Pin GPIO_PIN_10
#define CHANGE_EXERCISE_GPIO_Port GPIOB
#define For_Testing_Pin GPIO_PIN_12
#define For_Testing_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define STLK_RX_Pin GPIO_PIN_8
#define STLK_RX_GPIO_Port GPIOD
#define STLK_TX_Pin GPIO_PIN_9
#define STLK_TX_GPIO_Port GPIOD
#define Encoder2_ch1_Pin GPIO_PIN_12
#define Encoder2_ch1_GPIO_Port GPIOD
#define Encoder2_ch2_Pin GPIO_PIN_13
#define Encoder2_ch2_GPIO_Port GPIOD
#define USB_PowerSwitchOn_Pin GPIO_PIN_6
#define USB_PowerSwitchOn_GPIO_Port GPIOG
#define USB_OverCurrent_Pin GPIO_PIN_7
#define USB_OverCurrent_GPIO_Port GPIOG
#define Limit_SW3_Pin GPIO_PIN_6
#define Limit_SW3_GPIO_Port GPIOC
#define Limit_SW4_Pin GPIO_PIN_8
#define Limit_SW4_GPIO_Port GPIOC
#define DATA_READY_Pin GPIO_PIN_9
#define DATA_READY_GPIO_Port GPIOA
#define Limit_SW1_Pin GPIO_PIN_11
#define Limit_SW1_GPIO_Port GPIOA
#define Limit_SW2_Pin GPIO_PIN_12
#define Limit_SW2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SW0_Pin GPIO_PIN_3
#define SW0_GPIO_Port GPIOB
#define MOTOR1_DIREC_Pin GPIO_PIN_4
#define MOTOR1_DIREC_GPIO_Port GPIOB
#define M_DRIVER1_ENABLE_Pin GPIO_PIN_5
#define M_DRIVER1_ENABLE_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_7
#define LD2_GPIO_Port GPIOB
#define NodeMCU_CS_Pin GPIO_PIN_0
#define NodeMCU_CS_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
