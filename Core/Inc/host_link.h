#ifndef HOST_LINK_H
#define HOST_LINK_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * USART1 PC4 TX / PC5 RX — H7 production link (460800 8N1, DMA).
 * Binary frames are defined in h7_link_proto.h. METER is 5 ms, BMS/PD 200 ms.
 * Diagnostic ASCII is carried in TEXT frames (type 0x20), lowest TX priority.
 * TEL is ignored. G0 traffic stays on USART2.
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
