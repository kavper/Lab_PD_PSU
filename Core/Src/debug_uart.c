#include "debug_uart.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#define DEBUG_UART_BUFFER_SIZE      1024U
#define DEBUG_UART_TIMEOUT_MS       2U

static UART_HandleTypeDef *debug_uart = NULL;

void Debug_Init(UART_HandleTypeDef *huart)
{
    debug_uart = huart;
}

void Debug_Write(const char *text)
{
    size_t length;

    if ((debug_uart == NULL) || (text == NULL)) {
        return;
    }

    length = strlen(text);

    if (length == 0U) {
        return;
    }

    HAL_UART_Transmit(debug_uart,
                      (uint8_t *)text,
                      (uint16_t)length,
                      DEBUG_UART_TIMEOUT_MS);
}

void Debug_Printf(const char *fmt, ...)
{
    char buffer[DEBUG_UART_BUFFER_SIZE];
    va_list args;
    int length;

    if ((debug_uart == NULL) || (fmt == NULL)) {
        return;
    }

    va_start(args, fmt);
    length = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (length <= 0) {
        return;
    }

    if ((uint32_t)length >= sizeof(buffer)) {
        length = (int)(sizeof(buffer) - 3U);
        buffer[length++] = '\r';
        buffer[length++] = '\n';
        buffer[length] = '\0';
    } else if ((length < 1) || (buffer[length - 1] != '\n')) {
        if ((uint32_t)length <= (sizeof(buffer) - 3U)) {
            buffer[length++] = '\r';
            buffer[length++] = '\n';
            buffer[length] = '\0';
        }
    }

    HAL_UART_Transmit(debug_uart,
                      (uint8_t *)buffer,
                      (uint16_t)length,
                      DEBUG_UART_TIMEOUT_MS);
}
