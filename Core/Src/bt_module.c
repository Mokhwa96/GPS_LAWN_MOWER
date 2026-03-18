/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bt_module.c
  * @brief   Bluetooth command processing for the module test firmware.
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
#include "bt_module.h"

#include "wheel_drive.h"

#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef struct
{
  volatile uint16_t tail;
  volatile uint16_t head;
  uint8_t data[256];
} RingBuffer;

/* 블루투스 명령 파서가 사용하는 내부 명령 ID 정의 */
enum
{
  CMD_ID_STATUS = 0L,
  CMD_ID_STOP = 10L,
  CMD_ID_FORWARD = 11L,
  CMD_ID_REVERSE = 12L,
  CMD_ID_LEFT = 13L,
  CMD_ID_RIGHT = 14L,
  CMD_ID_PINTEST = 90L,
  CMD_ID_MAPTEST = 91L
};

static UART_HandleTypeDef *g_debug_uart;
static UART_HandleTypeDef *g_bt_uart;
static RingBuffer g_bt_rx;
static uint8_t g_bt_rx_byte;
static DriveMode g_current_mode = DRIVE_MODE_STOP;
static uint32_t g_last_heartbeat_ms;
static uint32_t g_last_stats_ms;
static uint32_t g_uart2_irq_count;
static uint32_t g_uart2_drop_count;

/* Forward declarations used before function definitions. */
static void send_text_response(const char *text);

typedef struct
{
  uint16_t pin;
  const char *name;
} GpioPinProbe;

/* 256바이트 링버퍼에서 다음 인덱스를 계산한다. */
static uint16_t rb_next(uint16_t idx)
{
  return (uint16_t)((idx + 1U) & 0x00FFU);
}

/* 수신한 바이트를 링버퍼에 적재한다. 버퍼가 가득 차면 false를 반환한다. */
static bool rb_push(RingBuffer *rb, uint8_t value)
{
  uint16_t next = rb_next(rb->head);

  if (next == rb->tail)
  {
    return false;
  }

  rb->data[next] = value;
  rb->head = next;
  return true;
}

/* 링버퍼에서 바이트 하나를 꺼낸다. 비어 있으면 false를 반환한다. */
static bool rb_pop(RingBuffer *rb, uint8_t *value)
{
  if (rb->tail == rb->head)
  {
    return false;
  }

  rb->tail = rb_next(rb->tail);
  *value = rb->data[rb->tail];
  return true;
}

/* 지정한 UART로 바이트 배열을 전송한다. */
static void uart_send_data(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len)
{
  if ((huart == NULL) || (data == NULL) || (len == 0U))
  {
    return;
  }

  (void)HAL_UART_Transmit(huart, (uint8_t *)data, len, 100U);
}

/* 문자열을 UART로 전송하고 줄바꿈은 CRLF 형태로 맞춘다. */
static void uart_send_text(UART_HandleTypeDef *huart, const char *text)
{
  uint8_t ch = 0U;

  if ((huart == NULL) || (text == NULL))
  {
    return;
  }

  while (*text != '\0')
  {
    if (*text == '\n')
    {
      ch = '\r';
      uart_send_data(huart, &ch, 1U);
    }

    ch = (uint8_t)*text;
    uart_send_data(huart, &ch, 1U);
    text++;
  }
}

/* 디버그 UART로 로그 문자열을 출력한다. */
static void debug_log(const char *text)
{
  uart_send_text(g_debug_uart, text);
}

/* 현재 주행 상태와 GPIO/TIM 상태를 사람이 읽기 쉬운 로그로 남긴다. */
static void log_drive_output(const char *tag)
{
  char out[640];
  WheelDriveSnapshot snapshot;

  memset(&snapshot, 0, sizeof(snapshot));
  WheelDrive_GetSnapshot(&snapshot);

  (void)snprintf(out, sizeof(out),
                 "[DRV] %s mode=%s arr=%lu ccr1=%lu ccr2=%lu ccer=0x%lX bdtr=0x%lX "
                 "encL=%lu encR=%lu pe0=%lu pe1=%lu "
                 "L(A i/o/e/m=%lu/%lu/%lu/%lu B i/o/e/m=%lu/%lu/%lu/%lu) "
                 "R(A i/o/e/m=%lu/%lu/%lu/%lu B i/o/e/m=%lu/%lu/%lu/%lu) "
                 "gpioe_idr=0x%04lX gpioe_odr=0x%04lX gpioa_idr=0x%04lX gpioa_odr=0x%04lX\n",
                 tag,
                 WheelDrive_ModeName(snapshot.mode),
                 (unsigned long)snapshot.arr,
                 (unsigned long)snapshot.left_ccr,
                 (unsigned long)snapshot.right_ccr,
                 (unsigned long)snapshot.ccer,
                 (unsigned long)snapshot.bdtr,
                 (unsigned long)snapshot.left_encoder_count,
                 (unsigned long)snapshot.right_encoder_count,
                 (unsigned long)snapshot.left_encoder_level,
                 (unsigned long)snapshot.right_encoder_level,
                 (unsigned long)snapshot.left_a_level,
                 (unsigned long)snapshot.left_a_odr_level,
                 (unsigned long)snapshot.left_a_expected_level,
                 (unsigned long)snapshot.left_a_mismatch,
                 (unsigned long)snapshot.left_b_level,
                 (unsigned long)snapshot.left_b_odr_level,
                 (unsigned long)snapshot.left_b_expected_level,
                 (unsigned long)snapshot.left_b_mismatch,
                 (unsigned long)snapshot.right_a_level,
                 (unsigned long)snapshot.right_a_odr_level,
                 (unsigned long)snapshot.right_a_expected_level,
                 (unsigned long)snapshot.right_a_mismatch,
                 (unsigned long)snapshot.right_b_level,
                 (unsigned long)snapshot.right_b_odr_level,
                 (unsigned long)snapshot.right_b_expected_level,
                 (unsigned long)snapshot.right_b_mismatch,
                 (unsigned long)snapshot.gpioe_idr,
                 (unsigned long)snapshot.gpioe_odr,
                 (unsigned long)snapshot.gpioa_idr,
                 (unsigned long)snapshot.gpioa_odr);
  debug_log(out);
}

static void log_direction_alert(const WheelDriveSnapshot *snapshot)
{
  char out[480];
  uint8_t expected_left_run_dir = (MOTOR_L_FORWARD_DIR_STATE == 0U) ? 1U : 0U;
  uint8_t expected_right_run_dir = MOTOR_R_FORWARD_DIR_STATE;

  if (snapshot == NULL)
  {
    return;
  }

  (void)snprintf(out, sizeof(out),
                 "[DIR-ALERT] mode=%s tgtDir(L/R)=%lu/%lu lastDir(L/R)=%lu/%lu "
                 "tgtDuty(L/R)=%lu/%lu ccr(L/R)=%lu/%lu "
                 "LB(i/o/e/m)=%lu/%lu/%lu/%lu RB(i/o/e/m)=%lu/%lu/%lu/%lu "
                 "fixedDirExpected(L/R)=%lu/%lu gpioe_idr=0x%04lX gpioe_odr=0x%04lX\n",
                 WheelDrive_ModeName(snapshot->mode),
                 (unsigned long)snapshot->left_target_dir_state,
                 (unsigned long)snapshot->right_target_dir_state,
                 (unsigned long)snapshot->left_last_dir_state,
                 (unsigned long)snapshot->right_last_dir_state,
                 (unsigned long)snapshot->left_target_duty_permille,
                 (unsigned long)snapshot->right_target_duty_permille,
                 (unsigned long)snapshot->left_ccr,
                 (unsigned long)snapshot->right_ccr,
                 (unsigned long)snapshot->left_b_level,
                 (unsigned long)snapshot->left_b_odr_level,
                 (unsigned long)snapshot->left_b_expected_level,
                 (unsigned long)snapshot->left_b_mismatch,
                 (unsigned long)snapshot->right_b_level,
                 (unsigned long)snapshot->right_b_odr_level,
                 (unsigned long)snapshot->right_b_expected_level,
                 (unsigned long)snapshot->right_b_mismatch,
                 (unsigned long)expected_left_run_dir,
                 (unsigned long)expected_right_run_dir,
                 (unsigned long)snapshot->gpioe_idr,
                 (unsigned long)snapshot->gpioe_odr);
  debug_log(out);
}

static uint8_t pin_level_from_reg(uint32_t reg, uint16_t pin)
{
  return ((reg & (uint32_t)pin) != 0U) ? 1U : 0U;
}

static void set_motor_pin_pattern(uint8_t la, uint8_t lb, uint8_t ra, uint8_t rb)
{
  HAL_GPIO_WritePin(MOTOR_L_A_GPIO_Port, MOTOR_L_A_Pin, (la != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_L_B_GPIO_Port, MOTOR_L_B_Pin, (lb != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_R_A_GPIO_Port, MOTOR_R_A_Pin, (ra != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_R_B_GPIO_Port, MOTOR_R_B_Pin, (rb != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void log_motor_pin_probe(const char *tag)
{
  char out[256];
  uint32_t idr = GPIOE->IDR;
  uint32_t odr = GPIOE->ODR;

  (void)snprintf(out, sizeof(out),
                 "[PINTEST] %s L_A(i/o)=%lu/%lu L_B(i/o)=%lu/%lu R_A(i/o)=%lu/%lu R_B(i/o)=%lu/%lu idr=0x%04lX odr=0x%04lX\n",
                 tag,
                 (unsigned long)pin_level_from_reg(idr, MOTOR_L_A_Pin),
                 (unsigned long)pin_level_from_reg(odr, MOTOR_L_A_Pin),
                 (unsigned long)pin_level_from_reg(idr, MOTOR_L_B_Pin),
                 (unsigned long)pin_level_from_reg(odr, MOTOR_L_B_Pin),
                 (unsigned long)pin_level_from_reg(idr, MOTOR_R_A_Pin),
                 (unsigned long)pin_level_from_reg(odr, MOTOR_R_A_Pin),
                 (unsigned long)pin_level_from_reg(idr, MOTOR_R_B_Pin),
                 (unsigned long)pin_level_from_reg(odr, MOTOR_R_B_Pin),
                 (unsigned long)idr,
                 (unsigned long)odr);
  debug_log(out);
}

static void run_motor_pin_probe(void)
{
  send_text_response("OK,PINTEST,START");

  WheelDrive_RunRaw(0U, 0U, 0U, 0U);
  g_current_mode = DRIVE_MODE_STOP;
  HAL_Delay(20U);
  log_motor_pin_probe("STOP_BASE");

  set_motor_pin_pattern(0U, 0U, 0U, 0U);
  HAL_Delay(20U);
  log_motor_pin_probe("ALL_LOW");

  set_motor_pin_pattern(1U, 1U, 1U, 1U);
  HAL_Delay(20U);
  log_motor_pin_probe("ALL_HIGH");

  set_motor_pin_pattern(1U, 0U, 0U, 0U);
  HAL_Delay(20U);
  log_motor_pin_probe("L_A_HIGH");

  set_motor_pin_pattern(0U, 1U, 0U, 0U);
  HAL_Delay(20U);
  log_motor_pin_probe("L_B_HIGH");

  set_motor_pin_pattern(0U, 0U, 1U, 0U);
  HAL_Delay(20U);
  log_motor_pin_probe("R_A_HIGH");

  set_motor_pin_pattern(0U, 0U, 0U, 1U);
  HAL_Delay(20U);
  log_motor_pin_probe("R_B_HIGH");

  WheelDrive_RunRaw(0U, 0U, 0U, 0U);
  g_current_mode = DRIVE_MODE_STOP;
  HAL_Delay(20U);
  log_motor_pin_probe("RESTORE_STOP");

  send_text_response("OK,PINTEST,DONE");
}

static void run_motor_pin_map_probe(void)
{
  static const GpioPinProbe probes[] =
  {
    { GPIO_PIN_2,  "PE2"  },
    { GPIO_PIN_3,  "PE3"  },
    { GPIO_PIN_4,  "PE4"  },
    { GPIO_PIN_5,  "PE5"  },
    { GPIO_PIN_8,  "PE8"  },
    { GPIO_PIN_9,  "PE9"  },
    { GPIO_PIN_11, "PE11" },
    { GPIO_PIN_12, "PE12" }
  };
  uint32_t save_moder;
  uint32_t save_otyper;
  uint32_t save_ospeedr;
  uint32_t save_pupdr;
  uint32_t save_afrl;
  uint32_t save_afrh;
  uint32_t save_odr;
  uint16_t mask = 0U;
  char out[160];
  uint32_t i;

  send_text_response("OK,MAPTEST,START");

  WheelDrive_RunRaw(0U, 0U, 0U, 0U);
  g_current_mode = DRIVE_MODE_STOP;
  HAL_Delay(20U);

  for (i = 0U; i < (sizeof(probes) / sizeof(probes[0])); i++)
  {
    mask = (uint16_t)(mask | probes[i].pin);
  }

  save_moder = GPIOE->MODER;
  save_otyper = GPIOE->OTYPER;
  save_ospeedr = GPIOE->OSPEEDR;
  save_pupdr = GPIOE->PUPDR;
  save_afrl = GPIOE->AFR[0];
  save_afrh = GPIOE->AFR[1];
  save_odr = GPIOE->ODR;

  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = mask;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
  }

  for (i = 0U; i < (sizeof(probes) / sizeof(probes[0])); i++)
  {
    uint8_t low_i;
    uint8_t low_o;
    uint8_t high_i;
    uint8_t high_o;
    uint8_t stuck;
    uint8_t inverted;

    HAL_GPIO_WritePin(GPIOE, mask, GPIO_PIN_RESET);
    HAL_Delay(5U);
    low_i = pin_level_from_reg(GPIOE->IDR, probes[i].pin);
    low_o = pin_level_from_reg(GPIOE->ODR, probes[i].pin);

    HAL_GPIO_WritePin(GPIOE, probes[i].pin, GPIO_PIN_SET);
    HAL_Delay(5U);
    high_i = pin_level_from_reg(GPIOE->IDR, probes[i].pin);
    high_o = pin_level_from_reg(GPIOE->ODR, probes[i].pin);

    stuck = (low_i == high_i) ? 1U : 0U;
    inverted = ((low_i == 1U) && (high_i == 0U)) ? 1U : 0U;

    (void)snprintf(out, sizeof(out),
                   "[MAPTEST] %s low(i/o)=%lu/%lu high(i/o)=%lu/%lu stuck=%lu inv=%lu idr=0x%04lX odr=0x%04lX\n",
                   probes[i].name,
                   (unsigned long)low_i,
                   (unsigned long)low_o,
                   (unsigned long)high_i,
                   (unsigned long)high_o,
                   (unsigned long)stuck,
                   (unsigned long)inverted,
                   (unsigned long)GPIOE->IDR,
                   (unsigned long)GPIOE->ODR);
    debug_log(out);
  }

  GPIOE->ODR = save_odr;
  GPIOE->AFR[0] = save_afrl;
  GPIOE->AFR[1] = save_afrh;
  GPIOE->MODER = save_moder;
  GPIOE->OTYPER = save_otyper;
  GPIOE->OSPEEDR = save_ospeedr;
  GPIOE->PUPDR = save_pupdr;

  HAL_Delay(10U);
  log_motor_pin_probe("MAP_RESTORE_MOTOR");
  send_text_response("OK,MAPTEST,DONE");
}

/* UART2 인터럽트로 들어온 원시 수신 바이트를 디버그용으로 출력한다. */
static void debug_log_uart2_rx(uint8_t value)
{
  char out[40];

  if ((value >= 32U) && (value <= 126U))
  {
    (void)snprintf(out, sizeof(out), "[BT-RAW] IRQ 0x%02X '%c' OK\n", value, (char)value);
  }
  else
  {
    (void)snprintf(out, sizeof(out), "[BT-RAW] IRQ 0x%02X '.' OK\n", value);
  }

  debug_log(out);
}

/* 블루투스 쪽으로 한 줄 응답 문자열을 전송한다. */
static void send_text_response(const char *text)
{
  uart_send_text(g_bt_uart, text);
  uart_send_text(g_bt_uart, "\n");
}

/* 현재 모드와 상태값을 블루투스로 응답한다. */
static void send_status_response(void)
{
  char out[64];

  (void)snprintf(out, sizeof(out), "OK,STATUS,MODE=%s,MV=%u.\n",
                 WheelDrive_ModeName(g_current_mode),
                 (unsigned int)BT_STATUS_PLACEHOLDER_MV);
  uart_send_text(g_bt_uart, out);
}

/* 명령 처리 후 현재 모드를 요약한 상태 응답을 전송한다. */
static void send_state_response(const char *tag)
{
  char out[64];

  (void)snprintf(out, sizeof(out), "OK,%s,MODE=%s.\n",
                 tag,
                 WheelDrive_ModeName(g_current_mode));
  uart_send_text(g_bt_uart, out);
}

/* 지원하는 제어 명령 목록을 블루투스로 안내한다. */
static void send_help_response(void)
{
  send_text_response("HELP: F FORWARD B BACK L LEFT R RIGHT 0 STOP X STATUS T PINTEST C MAPTEST PING HELP");
}

/* 하트비트 LED와 주기 통계를 갱신해 시스템 상태를 보여준다. */
static void heartbeat_update(void)
{
  uint32_t now = HAL_GetTick();
  char out[480];
  WheelDriveSnapshot snapshot;
  static uint8_t prev_dir_mismatch_l;
  static uint8_t prev_dir_mismatch_r;

  if ((uint32_t)(now - g_last_heartbeat_ms) >= BT_HEARTBEAT_MS)
  {
    HAL_GPIO_TogglePin(BT_HEARTBEAT_GPIO_Port, BT_HEARTBEAT_Pin);
    g_last_heartbeat_ms = now;
  }

  if ((uint32_t)(now - g_last_stats_ms) >= BT_STATS_MS)
  {
    memset(&snapshot, 0, sizeof(snapshot));
    WheelDrive_GetSnapshot(&snapshot);

    (void)snprintf(out, sizeof(out),
                   "[BT-STATS] irq=%lu drop=%lu mode=%s encL=%lu encR=%lu pe0=%lu pe1=%lu "
                   "mismatch(LA/LB/RA/RB)=%lu/%lu/%lu/%lu "
                   "dir(L tgt/last i/o/e)=%lu/%lu %lu/%lu/%lu "
                   "dir(R tgt/last i/o/e)=%lu/%lu %lu/%lu/%lu "
                   "gpioe_idr=0x%04lX gpioe_odr=0x%04lX gpioa_idr=0x%04lX\n",
                   (unsigned long)g_uart2_irq_count,
                   (unsigned long)g_uart2_drop_count,
                   WheelDrive_ModeName(snapshot.mode),
                   (unsigned long)snapshot.left_encoder_count,
                   (unsigned long)snapshot.right_encoder_count,
                   (unsigned long)snapshot.left_encoder_level,
                   (unsigned long)snapshot.right_encoder_level,
                   (unsigned long)snapshot.left_a_mismatch,
                   (unsigned long)snapshot.left_b_mismatch,
                   (unsigned long)snapshot.right_a_mismatch,
                   (unsigned long)snapshot.right_b_mismatch,
                   (unsigned long)snapshot.left_target_dir_state,
                   (unsigned long)snapshot.left_last_dir_state,
                   (unsigned long)snapshot.left_b_level,
                   (unsigned long)snapshot.left_b_odr_level,
                   (unsigned long)snapshot.left_b_expected_level,
                   (unsigned long)snapshot.right_target_dir_state,
                   (unsigned long)snapshot.right_last_dir_state,
                   (unsigned long)snapshot.right_b_level,
                   (unsigned long)snapshot.right_b_odr_level,
                   (unsigned long)snapshot.right_b_expected_level,
                   (unsigned long)snapshot.gpioe_idr,
                   (unsigned long)snapshot.gpioe_odr,
                   (unsigned long)snapshot.gpioa_idr);
    debug_log(out);

    if ((snapshot.mode != DRIVE_MODE_STOP) &&
        ((snapshot.left_b_mismatch != 0U) || (snapshot.right_b_mismatch != 0U) ||
         (snapshot.left_b_mismatch != prev_dir_mismatch_l) ||
         (snapshot.right_b_mismatch != prev_dir_mismatch_r)))
    {
      log_direction_alert(&snapshot);
    }

    prev_dir_mismatch_l = snapshot.left_b_mismatch;
    prev_dir_mismatch_r = snapshot.right_b_mismatch;
    g_last_stats_ms = now;
  }
}

/* 대소문자를 무시하고 두 ASCII 문자열이 같은지 비교한다. */
static bool ascii_iequals(const char *lhs, const char *rhs)
{
  while ((*lhs != '\0') && (*rhs != '\0'))
  {
    if (tolower((unsigned char)*lhs) != tolower((unsigned char)*rhs))
    {
      return false;
    }

    lhs++;
    rhs++;
  }

  return (*lhs == '\0') && (*rhs == '\0');
}

/* 한 글자만으로 즉시 실행할 수 있는 단문 명령인지 확인한다. */
static bool is_single_char_command(char c)
{
  switch ((char)toupper((unsigned char)c))
  {
    case 'F':
    case 'B':
    case 'L':
    case 'R':
    case 'T':
    case 'C':
    case 'X':
      return true;
    default:
      return false;
  }
}

/* 수신 문자열을 내부 명령 ID로 변환한다. */
static bool parse_uart2_command_id(const char *token, long *cmd_out)
{
  if ((token == NULL) || (cmd_out == NULL))
  {
    return false;
  }

  if (ascii_iequals(token, "Q") || ascii_iequals(token, "STATUS"))
  {
    *cmd_out = CMD_ID_STATUS;
    return true;
  }

  if (ascii_iequals(token, "STOP") || ascii_iequals(token, "X"))
  {
    *cmd_out = CMD_ID_STOP;
    return true;
  }

  if (strcmp(token, "0") == 0)
  {
    *cmd_out = CMD_ID_STOP;
    return true;
  }

  if (ascii_iequals(token, "F") || ascii_iequals(token, "FORWARD"))
  {
    *cmd_out = CMD_ID_FORWARD;
    return true;
  }

  if (ascii_iequals(token, "B") || ascii_iequals(token, "BACK") ||
      ascii_iequals(token, "BACKWARD") || ascii_iequals(token, "REVERSE"))
  {
    *cmd_out = CMD_ID_REVERSE;
    return true;
  }

  if (ascii_iequals(token, "L") || ascii_iequals(token, "LEFT"))
  {
    *cmd_out = CMD_ID_LEFT;
    return true;
  }

  if (ascii_iequals(token, "R") || ascii_iequals(token, "RIGHT"))
  {
    *cmd_out = CMD_ID_RIGHT;
    return true;
  }

  if (ascii_iequals(token, "T") || ascii_iequals(token, "PINTEST") || ascii_iequals(token, "GPIOTEST"))
  {
    *cmd_out = CMD_ID_PINTEST;
    return true;
  }

  if (ascii_iequals(token, "C") || ascii_iequals(token, "MAPTEST") || ascii_iequals(token, "PINMAP") || ascii_iequals(token, "M"))
  {
    *cmd_out = CMD_ID_MAPTEST;
    return true;
  }

  return false;
}

/* 주행 모드를 적용하고 관련 상태와 로그를 함께 갱신한다. */
static void apply_mode(DriveMode mode)
{
  bool changed = (g_current_mode != mode);

  g_current_mode = mode;
  WheelDrive_ResetEncoderCounts();
  WheelDrive_ApplyMode(mode, WHEEL_DRIVE_DUTY_PERMILLE);
  log_drive_output("APPLY");

  if (changed)
  {
    send_state_response("MOVE");
  }
}

/* 파싱된 명령과 원본 문자열을 디버그 로그로 남긴다. */
static void log_command(const char *line, long cmd)
{
  char dbg[96];

  (void)snprintf(dbg, sizeof(dbg), "[BT] cmd=%ld line=%s\n", cmd, line);
  debug_log(dbg);
}

/* 한 줄 단위 명령 문자열을 해석해 실제 동작으로 연결한다. */
static void process_uart2_command(const char *line)
{
  char copy[256];
  char dbg[96];
  char *token;
  char *tail;
  long cmd = -1L;

  strncpy(copy, line, sizeof(copy) - 1U);
  copy[sizeof(copy) - 1U] = '\0';

  token = copy;
  while (isspace((unsigned char)*token))
  {
    token++;
  }

  tail = token + strlen(token);
  while ((tail > token) && isspace((unsigned char)tail[-1]))
  {
    tail--;
    *tail = '\0';
  }

  if (*token == '\0')
  {
    debug_log("[BT] empty line\n");
    return;
  }

  (void)snprintf(dbg, sizeof(dbg), "[BT-LINE] len=%u line=%.60s\n",
                 (unsigned int)strlen(token), token);
  debug_log(dbg);

  if (ascii_iequals(token, "PING"))
  {
    send_text_response("PONG");
    return;
  }

  if (ascii_iequals(token, "HELP") || ascii_iequals(token, "?"))
  {
    send_help_response();
    return;
  }

  if ((*token == '$') && (strchr(token + 1, ',') != NULL))
  {
    return;
  }

  if (!parse_uart2_command_id(token, &cmd))
  {
    (void)snprintf(dbg, sizeof(dbg), "[BT] ignored line=%.56s\n", token);
    debug_log(dbg);
    return;
  }

  log_command(token, cmd);

  if (cmd == CMD_ID_STATUS)
  {
    send_status_response();
  }
  else if (cmd == CMD_ID_STOP)
  {
    apply_mode(DRIVE_MODE_STOP);
  }
  else if (cmd == CMD_ID_FORWARD)
  {
    apply_mode(DRIVE_MODE_FORWARD);
  }
  else if (cmd == CMD_ID_REVERSE)
  {
    apply_mode(DRIVE_MODE_REVERSE);
  }
  else if (cmd == CMD_ID_LEFT)
  {
    apply_mode(DRIVE_MODE_LEFT);
  }
  else if (cmd == CMD_ID_RIGHT)
  {
    apply_mode(DRIVE_MODE_RIGHT);
  }
  else if (cmd == CMD_ID_PINTEST)
  {
    run_motor_pin_probe();
  }
  else if (cmd == CMD_ID_MAPTEST)
  {
    run_motor_pin_map_probe();
  }
}

/* 누적된 문자열 버퍼를 완성된 한 줄 명령으로 처리한다. */
static void flush_uart2_line(char *line, uint16_t *idx)
{
  if ((line == NULL) || (idx == NULL) || (*idx == 0U))
  {
    return;
  }

  line[*idx] = '\0';
  process_uart2_command(line);
  *idx = 0U;
}

/* 링버퍼에 쌓인 UART2 수신 데이터를 단문 명령 또는 줄 단위 명령으로 처리한다. */
static void process_uart2_stream(void)
{
  static char line[256];
  static uint16_t idx;
  static uint32_t single_char_started_ms;
  uint8_t c = 0U;
  char quick_cmd[2] = {0};

  while (rb_pop(&g_bt_rx, &c))
  {
    debug_log_uart2_rx(c);

    if (is_single_char_command((char)c))
    {
      if (idx != 0U)
      {
        flush_uart2_line(line, &idx);
      }

      quick_cmd[0] = (char)c;
      quick_cmd[1] = '\0';
      process_uart2_command(quick_cmd);
      single_char_started_ms = 0U;
      continue;
    }

    if ((idx == 0U) && (c == '0'))
    {
      quick_cmd[0] = '0';
      quick_cmd[1] = '\0';
      process_uart2_command(quick_cmd);
      single_char_started_ms = 0U;
      continue;
    }

    if ((c == '\r') || (c == '\n'))
    {
      flush_uart2_line(line, &idx);
      single_char_started_ms = 0U;
      continue;
    }

    if (idx < (sizeof(line) - 1U))
    {
      line[idx++] = (char)c;

      if ((idx == 1U) && is_single_char_command(line[0]))
      {
        single_char_started_ms = HAL_GetTick();
      }
      else
      {
        single_char_started_ms = 0U;
      }
    }
    else
    {
      idx = 0U;
      single_char_started_ms = 0U;
      send_text_response("ERR,LINE_TOO_LONG");
    }
  }

  if ((idx == 1U) &&
      is_single_char_command(line[0]) &&
      ((uint32_t)(HAL_GetTick() - single_char_started_ms) >= BT_SINGLE_CHAR_TIMEOUT_MS))
  {
    flush_uart2_line(line, &idx);
    single_char_started_ms = 0U;
  }
}

/* 블루투스/디버그 UART와 내부 상태를 초기화하고 수신 인터럽트를 시작한다. */
void BT_Module_Init(UART_HandleTypeDef *debug_uart, UART_HandleTypeDef *bt_uart)
{
  memset(&g_bt_rx, 0, sizeof(g_bt_rx));
  g_debug_uart = debug_uart;
  g_bt_uart = bt_uart;
  g_bt_rx_byte = 0U;
  g_current_mode = DRIVE_MODE_STOP;
  g_last_heartbeat_ms = 0U;
  g_last_stats_ms = 0U;
  g_uart2_irq_count = 0U;
  g_uart2_drop_count = 0U;

  WheelDrive_ResetEncoderCounts();
  WheelDrive_Stop();

  if ((g_bt_uart != NULL) && (HAL_UART_Receive_IT(g_bt_uart, &g_bt_rx_byte, 1U) != HAL_OK))
  {
    Error_Handler();
  }

  debug_log("\n[ModuleTest] Bluetooth forward-only test ready.\n");
  debug_log("[ModuleTest] USART1 debug: 115200 8N1\n");
  debug_log("[ModuleTest] USART2 bluetooth: 9600 8N1\n");
  debug_log("[ModuleTest] Wheel drive command: F=forward, B=back, L=left, R=right, 0=stop, T=pintest, C=maptest.\n");
  log_drive_output("INIT");
  send_text_response("BT_READY");
}

/* 메인 루프에서 호출되어 수신 명령 처리와 상태 갱신을 수행한다. */
void BT_Module_RunStep(void)
{
  process_uart2_stream();
  WheelDrive_Update();
  heartbeat_update();
}

/* UART 수신 완료 시 바이트를 링버퍼에 넣고 다음 수신을 다시 건다. */
void BT_Module_OnUartRxComplete(UART_HandleTypeDef *huart)
{
  bool pushed = false;

  if ((huart == NULL) || (huart != g_bt_uart))
  {
    return;
  }

  pushed = rb_push(&g_bt_rx, g_bt_rx_byte);
  g_uart2_irq_count++;

  if (!pushed)
  {
    g_uart2_drop_count++;
  }

  (void)HAL_UART_Receive_IT(g_bt_uart, &g_bt_rx_byte, 1U);
}

/* UART 에러 발생 시 플래그를 정리하고 다음 수신을 재시작한다. */
void BT_Module_OnUartError(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart != g_bt_uart))
  {
    return;
  }

  __HAL_UART_CLEAR_PEFLAG(huart);
  (void)HAL_UART_Receive_IT(g_bt_uart, &g_bt_rx_byte, 1U);
}

/* HAL UART 수신 완료 콜백을 모듈 내부 처리 함수로 연결한다. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BT_Module_OnUartRxComplete(huart);
}

/* HAL UART 에러 콜백을 모듈 내부 에러 처리 함수로 연결한다. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  BT_Module_OnUartError(huart);
}
