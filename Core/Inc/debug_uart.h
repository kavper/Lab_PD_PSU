#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "main.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>

void Debug_Init(UART_HandleTypeDef *huart);
void Debug_Write(const char *text);
void Debug_Printf(const char *fmt, ...);
uint32_t Debug_GetTxBufferUsed(void);
uint32_t Debug_GetDroppedCount(void);
bool Debug_IsTxBusy(void);
void Debug_UART_DMA_IRQHandler(void);
void Debug_UART_IRQHandler(void);

#endif
