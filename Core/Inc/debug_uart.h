#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "main.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

/* 0: during BQ bring-up only messages whose format starts with "[BQ" pass. */
#ifndef DEBUG_LOG_NON_BQ
#define DEBUG_LOG_NON_BQ 0U
#endif

void Debug_Init(UART_HandleTypeDef *huart);
void Debug_Write(const char *text);
void Debug_Printf(const char *fmt, ...);
void Debug_BlankLine(void);
uint32_t Debug_GetTxBufferUsed(void);
uint32_t Debug_GetDroppedCount(void);
bool Debug_IsTxBusy(void);
void Debug_UART_DMA_IRQHandler(void);
void Debug_UART_IRQHandler(void);

#endif
