/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    wheel_drive.c
  * @brief   Wheel motor drive helpers for the module test firmware.
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
#include "wheel_drive.h"

static TIM_HandleTypeDef *g_drive_tim;
static uint32_t g_left_channel;
static uint32_t g_right_channel;
static DriveMode g_current_mode = DRIVE_MODE_STOP;
static volatile uint32_t g_left_encoder_count;
static volatile uint32_t g_right_encoder_count;
static volatile uint16_t g_left_last_duty_permille;
static volatile uint16_t g_right_last_duty_permille;
static volatile uint8_t g_left_last_dir_state;
static volatile uint8_t g_right_last_dir_state;

#define WHEEL_ENCODER_IRQ_PRIORITY 5U

/* 방향 비트를 반전해 정/역회전 전환에 사용한다. */
static uint8_t opposite_dir(uint8_t dir)
{
  return (dir == 0U) ? 1U : 0U;
}

/* 엔코더 입력(PE0, PE1)에 대한 EXTI 라인과 NVIC를 설정한다. */
static void encoder_exti_init(void)
{
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  HAL_NVIC_DisableIRQ(EXTI0_IRQn);
  HAL_NVIC_DisableIRQ(EXTI1_IRQn);
  HAL_NVIC_ClearPendingIRQ(EXTI0_IRQn);
  HAL_NVIC_ClearPendingIRQ(EXTI1_IRQn);

  CLEAR_BIT(EXTI->IMR, EXTI_IMR_IM0 | EXTI_IMR_IM1);
  CLEAR_BIT(EXTI->RTSR, EXTI_RTSR_TR0 | EXTI_RTSR_TR1);
  CLEAR_BIT(EXTI->FTSR, EXTI_FTSR_TR0 | EXTI_FTSR_TR1);

  MODIFY_REG(SYSCFG->EXTICR[0],
             SYSCFG_EXTICR1_EXTI0 | SYSCFG_EXTICR1_EXTI1,
             SYSCFG_EXTICR1_EXTI0_PE | SYSCFG_EXTICR1_EXTI1_PE);

  EXTI->PR = (EXTI_PR_PR0 | EXTI_PR_PR1);
  SET_BIT(EXTI->RTSR, EXTI_RTSR_TR0 | EXTI_RTSR_TR1);
  SET_BIT(EXTI->IMR, EXTI_IMR_IM0 | EXTI_IMR_IM1);

  HAL_NVIC_SetPriority(EXTI0_IRQn, WHEEL_ENCODER_IRQ_PRIORITY, 0U);
  HAL_NVIC_SetPriority(EXTI1_IRQn, WHEEL_ENCODER_IRQ_PRIORITY, 0U);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}

/* 활성 극성에 맞춰 모터 제어 GPIO를 High/Low로 출력한다. */
static void write_logic_pin(GPIO_TypeDef *port, uint16_t pin, uint8_t active, uint8_t active_high)
{
  GPIO_PinState state = GPIO_PIN_RESET;

  if (((active != 0U) && (active_high != 0U)) || ((active == 0U) && (active_high == 0U)))
  {
    state = GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(port, pin, state);
}

static uint8_t logic_level_from_active(uint8_t active, uint8_t active_high)
{
  if (((active != 0U) && (active_high != 0U)) || ((active == 0U) && (active_high == 0U)))
  {
    return 1U;
  }

  return 0U;
}

/* 퍼밀(per-mille) 듀티 값을 현재 타이머 채널의 CCR 값으로 반영한다. */
static void pwm_set_channel(uint32_t channel, uint16_t permille)
{
  uint32_t arr = 0U;
  uint32_t pulse = 0U;

  if ((g_drive_tim == NULL) || (g_drive_tim->Instance == NULL))
  {
    return;
  }

  if (permille > WHEEL_PWM_MAX_PERMILLE)
  {
    permille = WHEEL_PWM_MAX_PERMILLE;
  }

  arr = g_drive_tim->Instance->ARR;
  if (arr == 0U)
  {
    arr = g_drive_tim->Init.Period;
  }

  pulse = (arr * permille) / WHEEL_PWM_MAX_PERMILLE;
  if (pulse > arr)
  {
    pulse = arr;
  }

  switch (channel)
  {
    case TIM_CHANNEL_1:
      g_drive_tim->Instance->CCR1 = pulse;
      break;
    case TIM_CHANNEL_2:
      g_drive_tim->Instance->CCR2 = pulse;
      break;
    default:
      break;
  }
}

/* 지정한 GPIO 핀의 현재 입력 레벨을 0 또는 1로 읽어온다. */
static uint8_t read_input_level(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static uint8_t read_output_level(GPIO_TypeDef *port, uint16_t pin)
{
  return ((port->ODR & pin) != 0U) ? 1U : 0U;
}

static void pwm_set_channel_enabled(uint32_t channel, uint8_t enabled)
{
  if ((g_drive_tim == NULL) || (g_drive_tim->Instance == NULL))
  {
    return;
  }

  switch (channel)
  {
    case TIM_CHANNEL_1:
      if (enabled != 0U)
      {
        SET_BIT(g_drive_tim->Instance->CCER, TIM_CCER_CC1E);
      }
      else
      {
        CLEAR_BIT(g_drive_tim->Instance->CCER, TIM_CCER_CC1E);
      }
      break;
    case TIM_CHANNEL_2:
      if (enabled != 0U)
      {
        SET_BIT(g_drive_tim->Instance->CCER, TIM_CCER_CC2E);
      }
      else
      {
        CLEAR_BIT(g_drive_tim->Instance->CCER, TIM_CCER_CC2E);
      }
      break;
    default:
      break;
  }
}

/* 왼쪽 모터의 방향 핀과 PWM 출력을 함께 적용한다. */
static void apply_left_motor(uint16_t duty, uint8_t dir)
{
  uint8_t enabled = (duty != 0U) ? 1U : 0U;
  uint8_t dir_active = (dir == 0U) ? 1U : 0U;

  g_left_last_duty_permille = duty;
  g_left_last_dir_state = dir;

  pwm_set_channel(g_left_channel, duty);

  pwm_set_channel_enabled(g_left_channel, enabled);
  if (enabled == 0U)
  {
    write_logic_pin(MOTOR_L_B_GPIO_Port, MOTOR_L_B_Pin, 0U, MOTOR_L_DIR_ACTIVE_HIGH);
    write_logic_pin(MOTOR_L_A_GPIO_Port, MOTOR_L_A_Pin, 0U, MOTOR_L_EN_ACTIVE_HIGH);
    return;
  }

  write_logic_pin(MOTOR_L_B_GPIO_Port, MOTOR_L_B_Pin, dir_active, MOTOR_L_DIR_ACTIVE_HIGH);
  write_logic_pin(MOTOR_L_A_GPIO_Port, MOTOR_L_A_Pin, 1U, MOTOR_L_EN_ACTIVE_HIGH);
}

/* 오른쪽 모터의 방향 핀과 PWM 출력을 함께 적용한다. */
static void apply_right_motor(uint16_t duty, uint8_t dir)
{
  uint8_t enabled = (duty != 0U) ? 1U : 0U;
  uint8_t dir_active = (dir == 0U) ? 1U : 0U;

  g_right_last_duty_permille = duty;
  g_right_last_dir_state = dir;

  pwm_set_channel(g_right_channel, duty);

  pwm_set_channel_enabled(g_right_channel, enabled);
  if (enabled == 0U)
  {
    write_logic_pin(MOTOR_R_B_GPIO_Port, MOTOR_R_B_Pin, 0U, MOTOR_R_DIR_ACTIVE_HIGH);
    write_logic_pin(MOTOR_R_A_GPIO_Port, MOTOR_R_A_Pin, 0U, MOTOR_R_EN_ACTIVE_HIGH);
    return;
  }

  write_logic_pin(MOTOR_R_B_GPIO_Port, MOTOR_R_B_Pin, dir_active, MOTOR_R_DIR_ACTIVE_HIGH);
  write_logic_pin(MOTOR_R_A_GPIO_Port, MOTOR_R_A_Pin, 1U, MOTOR_R_EN_ACTIVE_HIGH);
}

/* 좌/우 인덱스에 따라 해당 모터 채널로 제어를 분배한다. */
static void motor_channel_apply(uint8_t index, uint16_t duty, uint8_t dir)
{
  if (index == 0U)
  {
    apply_left_motor(duty, dir);
  }
  else
  {
    apply_right_motor(duty, dir);
  }
}

/* 좌우 모터 듀티와 방향을 직접 지정해 즉시 적용한다. */
void WheelDrive_RunRaw(uint16_t left_duty_permille, uint8_t left_dir,
                       uint16_t right_duty_permille, uint8_t right_dir)
{
  motor_channel_apply(0U, left_duty_permille, left_dir);
  motor_channel_apply(1U, right_duty_permille, right_dir);
}

/* 휠 구동에 사용할 타이머와 채널을 등록하고 엔코더 입력을 초기화한다. */
void WheelDrive_Init(TIM_HandleTypeDef *tim, uint32_t left_channel, uint32_t right_channel)
{
  g_drive_tim = tim;
  g_left_channel = left_channel;
  g_right_channel = right_channel;
  g_current_mode = DRIVE_MODE_STOP;
  g_left_encoder_count = 0U;
  g_right_encoder_count = 0U;
  g_left_last_duty_permille = 0U;
  g_right_last_duty_permille = 0U;
  g_left_last_dir_state = 0U;
  g_right_last_dir_state = 0U;

  if (g_drive_tim == NULL)
  {
    return;
  }

  encoder_exti_init();
  WheelDrive_Stop();
}

/* 지정한 주행 모드에 맞춰 좌우 모터 방향과 듀티를 계산해 적용한다. */
void WheelDrive_ApplyMode(DriveMode mode, uint16_t duty_permille)
{
  g_current_mode = mode;

  if (mode == DRIVE_MODE_STOP)
  {
    motor_channel_apply(0U, 0U, 0U);
    motor_channel_apply(1U, 0U, 0U);
  }
  else if (mode == DRIVE_MODE_FORWARD)
  {
    motor_channel_apply(0U, duty_permille, opposite_dir(MOTOR_L_FORWARD_DIR_STATE));
    motor_channel_apply(1U, duty_permille, MOTOR_R_FORWARD_DIR_STATE);
  }
  else if (mode == DRIVE_MODE_REVERSE)
  {
    motor_channel_apply(0U, duty_permille, opposite_dir(MOTOR_L_FORWARD_DIR_STATE));
    motor_channel_apply(1U, duty_permille, opposite_dir(MOTOR_R_FORWARD_DIR_STATE));
  }
  else if (mode == DRIVE_MODE_RIGHT)
  {
    uint16_t inner_duty = (uint16_t)(duty_permille / 2U);

    if ((duty_permille != 0U) && (inner_duty == 0U))
    {
      inner_duty = 1U;
    }

    motor_channel_apply(0U, duty_permille, opposite_dir(MOTOR_L_FORWARD_DIR_STATE));
    motor_channel_apply(1U, inner_duty, MOTOR_R_FORWARD_DIR_STATE);
  }
  else if (mode == DRIVE_MODE_LEFT)
  {
    uint16_t inner_duty = (uint16_t)(duty_permille / 2U);

    if ((duty_permille != 0U) && (inner_duty == 0U))
    {
      inner_duty = 1U;
    }

    motor_channel_apply(0U, inner_duty, opposite_dir(MOTOR_L_FORWARD_DIR_STATE));
    motor_channel_apply(1U, duty_permille, MOTOR_R_FORWARD_DIR_STATE);
  }
  else if (mode == DRIVE_MODE_CIRCLE)
  {
    uint16_t inner_duty = (uint16_t)(duty_permille / 2U);

    if ((duty_permille != 0U) && (inner_duty == 0U))
    {
      inner_duty = 1U;
    }

    motor_channel_apply(0U, duty_permille, MOTOR_L_FORWARD_DIR_STATE);
    motor_channel_apply(1U, inner_duty, MOTOR_R_FORWARD_DIR_STATE);
  }
}

/* 현재 주행 상태를 정지로 전환한다. */
void WheelDrive_Stop(void)
{
  WheelDrive_ApplyMode(DRIVE_MODE_STOP, 0U);
}

/* 마지막으로 적용된 주행 모드를 반환한다. */
DriveMode WheelDrive_GetMode(void)
{
  return g_current_mode;
}

/* 디버그 출력을 위해 모터/엔코더/타이머 상태를 스냅샷으로 모은다. */
void WheelDrive_GetSnapshot(WheelDriveSnapshot *snapshot)
{
  uint8_t left_enabled;
  uint8_t right_enabled;
  uint8_t left_dir_active;
  uint8_t right_dir_active;

  if (snapshot == NULL)
  {
    return;
  }

  snapshot->mode = g_current_mode;
  snapshot->arr = 0U;
  snapshot->left_ccr = 0U;
  snapshot->right_ccr = 0U;
  snapshot->ccer = 0U;
  snapshot->bdtr = 0U;
  snapshot->gpioa_idr = GPIOA->IDR;
  snapshot->gpioa_odr = GPIOA->ODR;
  snapshot->gpioe_idr = GPIOE->IDR;
  snapshot->gpioe_odr = GPIOE->ODR;
  snapshot->left_encoder_count = g_left_encoder_count;
  snapshot->right_encoder_count = g_right_encoder_count;
  snapshot->left_encoder_level = read_input_level(ENC_LEFT_GPIO_Port, ENC_LEFT_Pin);
  snapshot->right_encoder_level = read_input_level(ENC_RIGHT_GPIO_Port, ENC_RIGHT_Pin);
  snapshot->left_a_level = read_input_level(MOTOR_L_A_GPIO_Port, MOTOR_L_A_Pin);
  snapshot->left_b_level = read_input_level(MOTOR_L_B_GPIO_Port, MOTOR_L_B_Pin);
  snapshot->right_a_level = read_input_level(MOTOR_R_A_GPIO_Port, MOTOR_R_A_Pin);
  snapshot->right_b_level = read_input_level(MOTOR_R_B_GPIO_Port, MOTOR_R_B_Pin);
  snapshot->left_a_odr_level = read_output_level(MOTOR_L_A_GPIO_Port, MOTOR_L_A_Pin);
  snapshot->left_b_odr_level = read_output_level(MOTOR_L_B_GPIO_Port, MOTOR_L_B_Pin);
  snapshot->right_a_odr_level = read_output_level(MOTOR_R_A_GPIO_Port, MOTOR_R_A_Pin);
  snapshot->right_b_odr_level = read_output_level(MOTOR_R_B_GPIO_Port, MOTOR_R_B_Pin);

  left_enabled = (g_left_last_duty_permille != 0U) ? 1U : 0U;
  right_enabled = (g_right_last_duty_permille != 0U) ? 1U : 0U;
  left_dir_active = (g_left_last_dir_state == 0U) ? 1U : 0U;
  right_dir_active = (g_right_last_dir_state == 0U) ? 1U : 0U;

  if (left_enabled == 0U)
  {
    snapshot->left_a_expected_level = logic_level_from_active(0U, MOTOR_L_EN_ACTIVE_HIGH);
    snapshot->left_b_expected_level = logic_level_from_active(0U, MOTOR_L_DIR_ACTIVE_HIGH);
  }
  else
  {
    snapshot->left_a_expected_level = logic_level_from_active(1U, MOTOR_L_EN_ACTIVE_HIGH);
    snapshot->left_b_expected_level = logic_level_from_active(left_dir_active, MOTOR_L_DIR_ACTIVE_HIGH);
  }

  if (right_enabled == 0U)
  {
    snapshot->right_a_expected_level = logic_level_from_active(0U, MOTOR_R_EN_ACTIVE_HIGH);
    snapshot->right_b_expected_level = logic_level_from_active(0U, MOTOR_R_DIR_ACTIVE_HIGH);
  }
  else
  {
    snapshot->right_a_expected_level = logic_level_from_active(1U, MOTOR_R_EN_ACTIVE_HIGH);
    snapshot->right_b_expected_level = logic_level_from_active(right_dir_active, MOTOR_R_DIR_ACTIVE_HIGH);
  }

  snapshot->left_a_mismatch = (snapshot->left_a_level != snapshot->left_a_expected_level) ? 1U : 0U;
  snapshot->left_b_mismatch = (snapshot->left_b_level != snapshot->left_b_expected_level) ? 1U : 0U;
  snapshot->right_a_mismatch = (snapshot->right_a_level != snapshot->right_a_expected_level) ? 1U : 0U;
  snapshot->right_b_mismatch = (snapshot->right_b_level != snapshot->right_b_expected_level) ? 1U : 0U;

  if ((g_drive_tim != NULL) && (g_drive_tim->Instance != NULL))
  {
    snapshot->arr = g_drive_tim->Instance->ARR;
    snapshot->left_ccr = g_drive_tim->Instance->CCR1;
    snapshot->right_ccr = g_drive_tim->Instance->CCR2;
    snapshot->ccer = g_drive_tim->Instance->CCER;
    snapshot->bdtr = g_drive_tim->Instance->BDTR;
  }
}

/* 엔코더 EXTI 인터럽트를 처리하고 좌우 펄스 카운트를 증가시킨다. */
void WheelDrive_EXTI_IRQHandler(uint16_t pin)
{
  if ((pin == ENC_LEFT_Pin) && ((EXTI->PR & EXTI_PR_PR0) != 0U))
  {
    EXTI->PR = EXTI_PR_PR0;
    g_left_encoder_count++;
  }
  else if ((pin == ENC_RIGHT_Pin) && ((EXTI->PR & EXTI_PR_PR1) != 0U))
  {
    EXTI->PR = EXTI_PR_PR1;
    g_right_encoder_count++;
  }
}

/* 누적된 엔코더 카운트를 0으로 초기화한다. */
void WheelDrive_ResetEncoderCounts(void)
{
  g_left_encoder_count = 0U;
  g_right_encoder_count = 0U;
}

/* 주행 모드 enum 값을 사람이 읽기 쉬운 문자열로 변환한다. */
const char *WheelDrive_ModeName(DriveMode mode)
{
  switch (mode)
  {
    case DRIVE_MODE_STOP: return "STOP";
    case DRIVE_MODE_FORWARD: return "FORWARD";
    case DRIVE_MODE_REVERSE: return "REVERSE";
    case DRIVE_MODE_LEFT: return "LEFT";
    case DRIVE_MODE_RIGHT: return "RIGHT";
    case DRIVE_MODE_CIRCLE: return "CIRCLE";
    default: return "UNKNOWN";
  }
}
