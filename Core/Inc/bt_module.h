#ifndef __BT_MODULE_H
#define __BT_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

#define BT_SINGLE_CHAR_TIMEOUT_MS      120U
#define BT_HEARTBEAT_MS               500U
#define BT_STATS_MS                  1000U
#define BT_STATUS_PLACEHOLDER_MV     3700U

void BT_Module_Init(UART_HandleTypeDef *debug_uart, UART_HandleTypeDef *bt_uart);
void BT_Module_RunStep(void);
void BT_Module_OnUartRxComplete(UART_HandleTypeDef *huart);
void BT_Module_OnUartError(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __BT_MODULE_H */
