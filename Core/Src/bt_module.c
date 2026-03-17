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

enum
{
  CMD_ID_STATUS = 0L,
  CMD_ID_STOP = 10L,
  CMD_ID_FORWARD = 11L
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

static uint16_t rb_next(uint16_t idx)
{
  return (uint16_t)((idx + 1U) & 0x00FFU);
}

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

static void uart_send_data(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len)
{
  if ((huart == NULL) || (data == NULL) || (len == 0U))
  {
    return;
  }

  (void)HAL_UART_Transmit(huart, (uint8_t *)data, len, 100U);
}

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

static void debug_log(const char *text)
{
  uart_send_text(g_debug_uart, text);
}

static void log_drive_output(const char *tag)
{
  char out[320];
  WheelDriveSnapshot snapshot;

  memset(&snapshot, 0, sizeof(snapshot));
  WheelDrive_GetSnapshot(&snapshot);

  (void)snprintf(out, sizeof(out),
                 "[DRV] %s mode=%s arr=%lu ccr1=%lu ccr2=%lu ccer=0x%lX bdtr=0x%lX encL=%lu encR=%lu pe0=%lu pe1=%lu pe2=%lu pe3=%lu pe4=%lu pe5=%lu gpioe_idr=0x%04lX gpioe_odr=0x%04lX gpioa_idr=0x%04lX gpioa_odr=0x%04lX\n",
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
                 (unsigned long)snapshot.left_b_level,
                 (unsigned long)snapshot.right_a_level,
                 (unsigned long)snapshot.right_b_level,
                 (unsigned long)snapshot.gpioe_idr,
                 (unsigned long)snapshot.gpioe_odr,
                 (unsigned long)snapshot.gpioa_idr,
                 (unsigned long)snapshot.gpioa_odr);
  debug_log(out);
}

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

static void send_text_response(const char *text)
{
  uart_send_text(g_bt_uart, text);
  uart_send_text(g_bt_uart, "\n");
}

static void send_status_response(void)
{
  char out[64];

  (void)snprintf(out, sizeof(out), "OK,STATUS,MODE=%s,MV=%u.\n",
                 WheelDrive_ModeName(g_current_mode),
                 (unsigned int)BT_STATUS_PLACEHOLDER_MV);
  uart_send_text(g_bt_uart, out);
}

static void send_state_response(const char *tag)
{
  char out[64];

  (void)snprintf(out, sizeof(out), "OK,%s,MODE=%s.\n",
                 tag,
                 WheelDrive_ModeName(g_current_mode));
  uart_send_text(g_bt_uart, out);
}

static void send_help_response(void)
{
  send_text_response("HELP: F FORWARD 0 STOP X STATUS PING HELP");
}

static void heartbeat_update(void)
{
  uint32_t now = HAL_GetTick();
  char out[224];
  WheelDriveSnapshot snapshot;

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
                   "[BT-STATS] irq=%lu drop=%lu mode=%s encL=%lu encR=%lu pe0=%lu pe1=%lu gpioe_idr=0x%04lX gpioe_odr=0x%04lX gpioa_idr=0x%04lX\n",
                   (unsigned long)g_uart2_irq_count,
                   (unsigned long)g_uart2_drop_count,
                   WheelDrive_ModeName(snapshot.mode),
                   (unsigned long)snapshot.left_encoder_count,
                   (unsigned long)snapshot.right_encoder_count,
                   (unsigned long)snapshot.left_encoder_level,
                   (unsigned long)snapshot.right_encoder_level,
                   (unsigned long)snapshot.gpioe_idr,
                   (unsigned long)snapshot.gpioe_odr,
                   (unsigned long)snapshot.gpioa_idr);
    debug_log(out);
    g_last_stats_ms = now;
  }
}

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

static bool is_single_char_command(char c)
{
  switch ((char)toupper((unsigned char)c))
  {
    case 'F':
    case 'X':
      return true;
    default:
      return false;
  }
}

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

  return false;
}

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

static void log_command(const char *line, long cmd)
{
  char dbg[96];

  (void)snprintf(dbg, sizeof(dbg), "[BT] cmd=%ld line=%s\n", cmd, line);
  debug_log(dbg);
}

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
}

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
  debug_log("[ModuleTest] Wheel drive command: F=forward, 0=stop.\n");
  log_drive_output("INIT");
  send_text_response("BT_READY");
}

void BT_Module_RunStep(void)
{
  process_uart2_stream();
  heartbeat_update();
}

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

void BT_Module_OnUartError(UART_HandleTypeDef *huart)
{
  if ((huart == NULL) || (huart != g_bt_uart))
  {
    return;
  }

  __HAL_UART_CLEAR_PEFLAG(huart);
  (void)HAL_UART_Receive_IT(g_bt_uart, &g_bt_rx_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BT_Module_OnUartRxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  BT_Module_OnUartError(huart);
}
