#ifndef HOST_LINK_H
#define HOST_LINK_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * USART1 PC4 TX / PC5 RX — link to H7 GUI (now: PC/minicom).
 *
 * Physical: 115200 8N1, no flow control.
 * Framing: one ASCII line per message, terminated by \n (\r ignored).
 *
 * Telemetry (G4 -> host), space-separated key=value, integers only:
 *
 *   T vin_mv=... vout_mv=... iout_ma=... set_mv=... ilim_ma=... duty_ppm=...
 *     run=0|1 mode=IDLE|CV|CC fault=... pd=0|1 pd_mv=... pd_ma=... pd_mw=...
 *     permit=0|1 bms=0|1 alert=0|1 alarm=0x.... c1_mv=... c2_mv=... c3_mv=...
 *     c4_mv=... c5_mv=... pack_mv=... i_cc2_ma=...
 *
 *   duty_ppm is PWM duty A in parts-per-million (0.512 duty => 512000).
 *
 * Commands (host -> G4), case-insensitive, optional argument:
 *
 *   ON                 start CV (same as old GUI PSU_Start)
 *   OFF                stop / idle
 *   SET <volts>        e.g. SET 12.5
 *   ILIM <amps>        e.g. ILIM 3
 *   USB AUTO|SINK|SOURCE
 *   PERMIT 0|1         1 = allow power-stage LDO, 0 = kill LDO (safe)
 *   TEL [period_ms]    0 = stop periodic T lines; default 200
 *   ?                  one immediate T line
 *
 * Replies: OK, ERR <reason>, or a T line.
 */

void HostLink_Init(UART_HandleTypeDef *huart);
void HostLink_Task(void);
void HostLink_OnUartError(UART_HandleTypeDef *huart);

#endif /* HOST_LINK_H */
