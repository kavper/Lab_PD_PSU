#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "main.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * USART1 is the H7 / host telemetry port (T/TB/TC).
 * Debug_Printf is OFF by default so it does not corrupt machine frames.
 * Enable with host command VERBOSE 1 (or Debug_SetEnabled(true)).
 */
#ifndef DEBUG_UART_DEFAULT_ENABLED
#define DEBUG_UART_DEFAULT_ENABLED       0U
#endif

void Debug_Init(UART_HandleTypeDef *huart);
void Debug_SetEnabled(bool enabled);
bool Debug_IsEnabled(void);
void Debug_Write(const char *text);
void Debug_Printf(const char *fmt, ...);
void Debug_BlankLine(void);
uint32_t Debug_GetTxBufferUsed(void);
uint32_t Debug_GetDroppedCount(void);
bool Debug_IsTxBusy(void);
void Debug_UART_DMA_IRQHandler(void);
void Debug_UART_IRQHandler(void);

#endif
