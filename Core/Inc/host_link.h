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
 * =============================================================================
 * Machine telemetry (default, HUMAN 0) — H7-friendly
 * =============================================================================
 * Three space-separated key=value lines per frame (integers / hex only):
 *
 *   T  … PSU / G0 / PD / pre-reg core
 *   TB … full BMS (cells, pack/stack V, pack I, FETs, safety)
 *   TC … BQ25731 charger + TPS path
 *
 * Stable field names (H7 may ignore unknown keys):
 *   T:  vin_mv vout_mv iout_ma set_mv ilim_ma duty_ppm run mode fault
 *       pd pd_mv pd_ma pd_mw permit rem_sense
 *       g0 g0_out g0_want g0_ctrl g0_kill g0_outoff g0_vout_mv
 *       vpre_req_mv vpre_cmd_mv reg_ok stage_en ps_en flt hold_ms ps_err
 *       g0_rx g0_tlm g0_age_ms g0_err g0_uart pm_st fmt(=0)
 *   TB: bms cfg st fault alert alarm sa sb sc fet manuf chg dsg fets series
 *       c1_mv..c5_mv min_mv max_mv dV_mv sum_mv pack_mv stack_mv
 *       i_pack_ma i_cc2_ma sample alerts i2c_err
 *       (c4_mv=-1 expected on 4S skip-VC4)
 *   TC: bq_ok bq_vbat_mv bq_vsys_mv bq_ibat_ma bq_ichg_ma bq_idchg_ma
 *       bq_vbus_mv bq_iin_ma bq_vreg_mv bq_ichg_set_ma bq_iin_set_ma
 *       bq_st bq_fault bq_in bq_pre bq_fast bq_otg bq_iindpm bq_vindpm
 *       tps_vbus_mv cc1 cc2 role conn pd_role pd_mv pd_ma
 *
 * duty_ppm: PWM duty A in ppm (0.512 => 512000).
 *
 * =============================================================================
 * Human mode (HUMAN 1) — multi-line labeled dump for terminal / bring-up
 * =============================================================================
 *
 * Commands (host -> G4), case-insensitive:
 *
 *   HELP / H           command list
 *   HUMAN 0|1          machine (H7) vs human dump; default 0
 *   MACHINE / H7       alias HUMAN 0
 *   STATUS / ST        one human snapshot (even if HUMAN 0)
 *   ON / OFF           G0 sequencer + DCDC
 *   SET <volts>        G0 LDO voltage
 *   ILIM <amps>        G0 current limit
 *   USB AUTO|SINK|SOURCE
 *   PERMIT 0|1         PB7 kill / allow
 *   REMOTE ON|OFF      sense path
 *   TEL [period_ms]    0 = stop periodic frames; default 500
 *   ?                  one telemetry frame now
 *   G0DIAG / G0SWAP
 *   CLR / CLEAR
 *
 * Replies: OK..., ERR <reason>, HELP text, STATUS dump, or T/TB/TC / HUMAN block.
 */

void HostLink_Init(UART_HandleTypeDef *huart);
void HostLink_Task(void);
void HostLink_RxCplt(UART_HandleTypeDef *huart);
void HostLink_OnUartError(UART_HandleTypeDef *huart);
void HostLink_ForwardLine(const char *line);

#endif /* HOST_LINK_H */
