/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define CZUJNIK_1_TRIG_Pin GPIO_PIN_0
#define CZUJNIK_1_TRIG_GPIO_Port GPIOC
#define CZUJNIK_2_TRIG_Pin GPIO_PIN_1
#define CZUJNIK_2_TRIG_GPIO_Port GPIOC
#define CZUJNIK_3_TRIG_Pin GPIO_PIN_2
#define CZUJNIK_3_TRIG_GPIO_Port GPIOC
#define CZUJNIK_4_TRIG_Pin GPIO_PIN_3
#define CZUJNIK_4_TRIG_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define FRONT_LEFT_PHASE_1_Pin GPIO_PIN_6
#define FRONT_LEFT_PHASE_1_GPIO_Port GPIOA
#define FRONT_LEFT_PHASE_2_Pin GPIO_PIN_7
#define FRONT_LEFT_PHASE_2_GPIO_Port GPIOA
#define REAR_PHASE_1_Pin GPIO_PIN_4
#define REAR_PHASE_1_GPIO_Port GPIOC
#define REAR_PHASE_2_Pin GPIO_PIN_5
#define REAR_PHASE_2_GPIO_Port GPIOC
#define FRONT_RIGHT_PHASE_1_Pin GPIO_PIN_1
#define FRONT_RIGHT_PHASE_1_GPIO_Port GPIOB
#define FRONT_RIGHT_PHASE_2_Pin GPIO_PIN_2
#define FRONT_RIGHT_PHASE_2_GPIO_Port GPIOB
#define SERVO_PWM_Pin GPIO_PIN_6
#define SERVO_PWM_GPIO_Port GPIOC
#define REAR_MOTOR_PWM_Pin GPIO_PIN_7
#define REAR_MOTOR_PWM_GPIO_Port GPIOC
#define FRONT_LEFT_PWM_Pin GPIO_PIN_8
#define FRONT_LEFT_PWM_GPIO_Port GPIOC
#define FRONT_RIGHT_PWM_Pin GPIO_PIN_9
#define FRONT_RIGHT_PWM_GPIO_Port GPIOC
#define T1CH1_CZUJNIK_1_ECHO_Pin GPIO_PIN_8
#define T1CH1_CZUJNIK_1_ECHO_GPIO_Port GPIOA
#define T1CH2_CZUJNIK_2_ECHO_Pin GPIO_PIN_9
#define T1CH2_CZUJNIK_2_ECHO_GPIO_Port GPIOA
#define T1CH3_CZUJNIK_3_ECHO_Pin GPIO_PIN_10
#define T1CH3_CZUJNIK_3_ECHO_GPIO_Port GPIOA
#define T1CH4_CZUJNIK_4_ECHO_Pin GPIO_PIN_11
#define T1CH4_CZUJNIK_4_ECHO_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
