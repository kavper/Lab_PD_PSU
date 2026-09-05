#ifndef HOST_LINK_H
#define HOST_LINK_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * USART1 PC4 TX / PC5 RX — H7 / PC parser only (115200 8N1).
 *
 * Default: clean machine telemetry. Debug spam is OFF (VERBOSE 0).
 * G0 TLM/ACK stay on USART2; they are not mirrored here.
 * See docs/HOST_TELEMETRY.md for the field list, H7 parser notes, and G0 FW.
 *
 * Frame (every TEL period or on ? / STATUS):
 *   T  … PSU / G0 / PD / pre-reg  (follows TEL; back-pressured if TX busy)
 *   TB … BMS (cells, pack/stack V, pack I, FETs, safety)
 *   TC … BQ25731 + TPS path
 *   TEL < 200 ms → T at TEL, TB/TC at 200 ms (115200 cannot carry all three
 *   at 50–100 Hz). Cell voltages are never dropped; they are just not 10 Hz.
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
/* G0 USART2 lines: TLM/ACK dropped unless VERBOSE 1; NACK always forwarded. */
void HostLink_ForwardG0Line(const char *line);

#endif /* HOST_LINK_H */
