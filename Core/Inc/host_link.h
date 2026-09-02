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
 *   HELP / H           command list
 *   STATUS / ST        human summary + hints
 *   ON                 start G4 DCDC + G0 SET/OUT ON sequencer
 *   OFF                G0 OUT OFF + PERMIT kill + DCDC stop
 *   SET <volts>        G0 LDO voltage (also local DCDC if G0 idle)
 *   ILIM <amps>        G0 current limit
 *   USB AUTO|SINK|SOURCE
 *   PERMIT 0|1         1 = ena (PB6 HIGH, clears G0 POWER_KILL), 0 = LDO zabity (PB6 LOW) + OUT OFF
 *   REMOTE ON|1        enable remote sense path (PB5 REMOTE_ON high)
 *   REMOTE OFF|0       local sense (default; REMOTE_ON low)
 *   TEL [period_ms]    0 = stop periodic T lines; default 500
 *   ?                  one immediate T line
 *
 * Telemetry also includes rem_sense=, g0_want=, g0_ctrl=, g0_kill=, g0_outoff=.
 * Replies: OK..., ERR <reason>, STATUS/HELP text, or a T line.
 */

void HostLink_Init(UART_HandleTypeDef *huart);
void HostLink_Task(void);
void HostLink_RxCplt(UART_HandleTypeDef *huart);
void HostLink_OnUartError(UART_HandleTypeDef *huart);
void HostLink_ForwardLine(const char *line);

#endif /* HOST_LINK_H */
