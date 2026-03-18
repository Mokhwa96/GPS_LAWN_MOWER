/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration of the TIM instances.
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
#include "tim.h"

TIM_HandleTypeDef htim1;

void MX_TIM1_Init(void)
{
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.Period = 799;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  __HAL_RCC_TIM1_CLK_ENABLE();
  HAL_TIM_MspPostInit(&htim1);

  TIM1->PSC = htim1.Init.Prescaler;
  TIM1->ARR = htim1.Init.Period;
  TIM1->CCR1 = 0U;
  TIM1->CCR2 = 0U;
  TIM1->CCR3 = 0U;
  TIM1->CCR4 = 0U;
  TIM1->CCMR1 =
      (TIM_CCMR1_OC1PE | (6U << TIM_CCMR1_OC1M_Pos) |
       TIM_CCMR1_OC2PE | (6U << TIM_CCMR1_OC2M_Pos));
  TIM1->CCMR2 =
      (TIM_CCMR2_OC3PE | (6U << TIM_CCMR2_OC3M_Pos) |
       TIM_CCMR2_OC4PE | (6U << TIM_CCMR2_OC4M_Pos));
  TIM1->CCER = 0U;
  TIM1->BDTR = TIM_BDTR_MOE;
  TIM1->CR1 = TIM_CR1_ARPE;
  TIM1->CNT = 0U;
  TIM1->EGR = TIM_EGR_UG;
  TIM1->CR1 |= TIM_CR1_CEN;
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  if (htim->Instance == TIM1)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}
