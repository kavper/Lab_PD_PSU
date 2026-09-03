#ifndef HOST_LINK_H
#define HOST_LINK_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * USART1 PC4 TX / PC5 RX — H7 / PC parser only (115200 8N1).
 *
 * Default: clean machine telemetry. Debug spam is OFF (VERBOSE 0).
 * See docs/HOST_TELEMETRY.md for the full field list and parser notes.
 *
 * Frame (every TEL period or on ? / STATUS):
 *   T  … PSU / G0 / PD / pre-reg
 *   TB … BMS (cells, pack/stack V, pack I, FETs, safety)
 *   TC … BQ25731 + TPS path
 *
 * Commands: HELP, ON, OFF, SET, ILIM, PERMIT, REMOTE, TEL, ?, STATUS,
 *           BMS, VERBOSE, G0DIAG, G0SWAP, CLR
 */

void HostLink_Init(UART_HandleTypeDef *huart);
/* Optional: print RCC CSR reset cause on the host banner (call before clear). */
void HostLink_SetBootResetFlags(uint32_t rcc_csr);
void HostLink_Task(void);
void HostLink_RxCplt(UART_HandleTypeDef *huart);
void HostLink_OnUartError(UART_HandleTypeDef *huart);
void HostLink_ForwardLine(const char *line);

#endif /* HOST_LINK_H */
