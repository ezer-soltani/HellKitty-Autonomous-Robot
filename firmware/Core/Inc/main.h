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
#define STATUS_SOURIS_LED_Pin GPIO_PIN_14
#define STATUS_SOURIS_LED_GPIO_Port GPIOC
#define STATUS_CHAT_LED_Pin GPIO_PIN_15
#define STATUS_CHAT_LED_GPIO_Port GPIOC
#define BOUTON_RESET_Pin GPIO_PIN_10
#define BOUTON_RESET_GPIO_Port GPIOG
#define ENC1_PH1_Pin GPIO_PIN_0
#define ENC1_PH1_GPIO_Port GPIOA
#define ENC1_PH2_Pin GPIO_PIN_1
#define ENC1_PH2_GPIO_Port GPIOA
#define LIDAR_RX_Pin GPIO_PIN_2
#define LIDAR_RX_GPIO_Port GPIOA
#define PWM_Mot1_CH1_Pin GPIO_PIN_6
#define PWM_Mot1_CH1_GPIO_Port GPIOA
#define PWM_Mot1_CH2_Pin GPIO_PIN_7
#define PWM_Mot1_CH2_GPIO_Port GPIOA
#define PWM_Mot2_CH1_Pin GPIO_PIN_0
#define PWM_Mot2_CH1_GPIO_Port GPIOB
#define PWM_Mot2_CH2_Pin GPIO_PIN_1
#define PWM_Mot2_CH2_GPIO_Port GPIOB
#define Bluetooth_TX_Pin GPIO_PIN_10
#define Bluetooth_TX_GPIO_Port GPIOB
#define Bluetooth_RX_Pin GPIO_PIN_11
#define Bluetooth_RX_GPIO_Port GPIOB
#define ACC_INT2_Pin GPIO_PIN_12
#define ACC_INT2_GPIO_Port GPIOB
#define ACC_INT1_Pin GPIO_PIN_13
#define ACC_INT1_GPIO_Port GPIOB
#define LIDAR_M_CTR_Pin GPIO_PIN_8
#define LIDAR_M_CTR_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_9
#define VCP_TX_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_10
#define VCP_RX_GPIO_Port GPIOA
#define ENC2_PH1_Pin GPIO_PIN_11
#define ENC2_PH1_GPIO_Port GPIOA
#define ENC2_PH2_Pin GPIO_PIN_12
#define ENC2_PH2_GPIO_Port GPIOA
#define TOF1_XSHUT_Pin GPIO_PIN_15
#define TOF1_XSHUT_GPIO_Port GPIOA
#define TOF2_XSHUT_Pin GPIO_PIN_10
#define TOF2_XSHUT_GPIO_Port GPIOC
#define TOF3_XSHUT_Pin GPIO_PIN_11
#define TOF3_XSHUT_GPIO_Port GPIOC
#define TOF4_XSHUT_Pin GPIO_PIN_3
#define TOF4_XSHUT_GPIO_Port GPIOB
#define TOF1_GPIO_Pin GPIO_PIN_4
#define TOF1_GPIO_GPIO_Port GPIOB
#define TOF1_GPIO_EXTI_IRQn EXTI4_IRQn
#define TOF2_GPIO_Pin GPIO_PIN_5
#define TOF2_GPIO_GPIO_Port GPIOB
#define TOF2_GPIO_EXTI_IRQn EXTI9_5_IRQn
#define TOF3_GPIO_Pin GPIO_PIN_6
#define TOF3_GPIO_GPIO_Port GPIOB
#define TOF3_GPIO_EXTI_IRQn EXTI9_5_IRQn
#define TOF4_GPIO_Pin GPIO_PIN_7
#define TOF4_GPIO_GPIO_Port GPIOB
#define TOF4_GPIO_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
