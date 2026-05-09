#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include "main.h"
#include <stdarg.h>

void Debug_Init(UART_HandleTypeDef *huart);
void Debug_Write(const char *text);
void Debug_Printf(const char *fmt, ...);

#endif