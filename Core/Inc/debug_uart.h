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

/* Runtime debug verbosity (host command "DBG 0|1|2").
 *   MIN     : bare minimum - state changes, concise [PD] status, faults, errors.
 *   NORMAL  : + PD policy decisions, capability lists, bring-up prints.
 *   VERBOSE : + [MON] table, [PD-PDO] dumps, forwarded G0 "TLM" telemetry.
 * Default is MIN so the USB-C/PD behaviour is easy to read on the bench. */
#define DEBUG_LEVEL_MIN     0U
#define DEBUG_LEVEL_NORMAL  1U
#define DEBUG_LEVEL_VERBOSE 2U

#ifndef DEBUG_LEVEL_DEFAULT
#define DEBUG_LEVEL_DEFAULT DEBUG_LEVEL_MIN
#endif

void Debug_SetLevel(uint8_t level);
uint8_t Debug_GetLevel(void);

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
