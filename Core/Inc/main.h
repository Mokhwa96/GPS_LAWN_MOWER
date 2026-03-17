/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

#define ENC_LEFT_Pin GPIO_PIN_0
#define ENC_LEFT_GPIO_Port GPIOE
#define ENC_RIGHT_Pin GPIO_PIN_1
#define ENC_RIGHT_GPIO_Port GPIOE

#define MOTOR_L_A_Pin GPIO_PIN_2
#define MOTOR_L_A_GPIO_Port GPIOE
#define MOTOR_L_B_Pin GPIO_PIN_3
#define MOTOR_L_B_GPIO_Port GPIOE
#define MOTOR_R_A_Pin GPIO_PIN_4
#define MOTOR_R_A_GPIO_Port GPIOE
#define MOTOR_R_B_Pin GPIO_PIN_5
#define MOTOR_R_B_GPIO_Port GPIOE
#define BT_HEARTBEAT_Pin GPIO_PIN_6
#define BT_HEARTBEAT_GPIO_Port GPIOC

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
