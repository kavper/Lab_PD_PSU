#include "host_link.h"

#include "app.h"
#include "board_rev.h"
#include "bms_board.h"
#include "bq76922.h"
#include "ldo_link.h"
#include "ldo_prereg.h"
#include "power_manager.h"
#include "power_stage.h"
#include "psu_gui_api.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#define HOST_LINK_RX_LINE_MAX        96U
#define HOST_LINK_TX_MAX             768U
#define HOST_LINK_TEL_DEFAULT_MS     500U

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_rx_byte;
static char s_rx_line[HOST_LINK_RX_LINE_MAX];
static volatile uint8_t s_rx_len;
static volatile bool s_line_ready;
static volatile bool s_rx_overflow;
static uint32_t s_tel_period_ms = HOST_LINK_TEL_DEFAULT_MS;
static uint32_t s_last_tel_ms;
/* 0 = machine T/TB/TC for H7, 1 = multi-line human dump */
static uint8_t s_human_mode = 0U;

static void HostLink_ArmRx(void)
{
    if (s_huart != NULL) {
        (void)HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1U);
    }
}

static void HostLink_Tx(const char *text)
{
    size_t len;

    if ((s_huart == NULL) || (text == NULL)) {
        return;
    }
    len = strlen(text);
    if (len == 0U) {
        return;
    }
    (void)HAL_UART_Transmit(s_huart, (uint8_t *)text, (uint16_t)len, 80U);
}

static int32_t HostLink_Mv(float volts)
{
    return (int32_t)((volts * 1000.0f) + ((volts >= 0.0f) ? 0.5f : -0.5f));
}

static int32_t HostLink_Ma(float amps)
{
    return (int32_t)((amps * 1000.0f) + ((amps >= 0.0f) ? 0.5f : -0.5f));
}

static const char *HostLink_ModeName(void)
{
    switch (App_GetRequestedMode()) {
        case MODE_CC:
            return "CC";
        case MODE_CV:
            return "CV";
        default:
            return "IDLE";
    }
}

static const char *HostLink_BmsStateName(BQ76922_State_t st)
{
    switch (st) {
        case BQ76922_STATE_ABSENT: return "ABSENT";
        case BQ76922_STATE_INIT: return "INIT";
        case BQ76922_STATE_READY: return "READY";
        case BQ76922_STATE_DISCHARGING: return "DSG";
        case BQ76922_STATE_CHARGING: return "CHG";
        case BQ76922_STATE_WARN: return "WARN";
        case BQ76922_STATE_FAULT: return "FAULT";
        default: return "?";
    }
}

static const char *HostLink_PdRoleName(PowerManager_PdPowerRole_t role)
{
    switch (role) {
        case POWER_MANAGER_PD_ROLE_SINK: return "SINK";
        case POWER_MANAGER_PD_ROLE_SOURCE: return "SOURCE";
        default: return "NONE";
    }
}

static void HostLink_FmtCell(char *out, size_t out_sz, int16_t mv, uint8_t idx)
{
    if (!BMS_CELL_USED(idx)) {
        (void)snprintf(out, out_sz, "skip");
    } else {
        (void)snprintf(out, out_sz, "%d", (int)mv);
    }
}

static void HostLink_SendMachineTelemetry(void)
{
    char line[HOST_LINK_TX_MAX];
    BQ76922_Snapshot_t bms;
    LdoPrereg_Status_t prereg;
    LdoLink_Status_t ldo;
    PowerManager_Status_t pm;
    float pd_v = 0.0f;
    float pd_a = 0.0f;
    float pd_w = 0.0f;
    uint8_t pd_ok;
    uint32_t now_ms = HAL_GetTick();
    uint32_t g0_age_ms = 0U;
    int n;
    int16_t dV;

    BQ76922_GetSnapshot(&g_bq76922, &bms);
    pd_ok = PSU_GuiGetPdContract(&pd_v, &pd_a, &pd_w, NULL);
    LdoPrereg_GetStatus(&prereg);
    LdoLink_GetStatus(&ldo);
    PowerManager_GetStatus(&pm);

    if (ldo.last_rx_ms != 0U) {
        g0_age_ms = now_ms - ldo.last_rx_ms;
    } else {
        g0_age_ms = 0xFFFFFFFFu;
    }

    dV = (int16_t)(bms.max_cell_mv - bms.min_cell_mv);

    /* T — PSU / G0 / PD core (stable keys for H7 parser) */
    n = snprintf(line, sizeof(line),
                 "T vin_mv=%ld vout_mv=%ld iout_ma=%ld set_mv=%ld ilim_ma=%ld "
                 "duty_ppm=%lu run=%u mode=%s fault=%lu "
                 "pd=%u pd_mv=%ld pd_ma=%ld pd_mw=%ld "
                 "permit=%u rem_sense=%u "
                 "g0=%u g0_out=%u g0_want=%u g0_ctrl=%u g0_kill=%u g0_outoff=%u "
                 "g0_vout_mv=%lu vpre_req_mv=%ld vpre_cmd_mv=%ld reg_ok=%u "
                 "stage_en=%u ps_en=%u flt=%u hold_ms=%lu ps_err=%u "
                 "g0_rx=%lu g0_tlm=%lu g0_age_ms=%lu g0_err=%lu g0_uart=0x%lX "
                 "pm_st=%u fmt=0\r\n",
                 (long)HostLink_Mv(App_GetInputVoltage()),
                 (long)HostLink_Mv(App_GetOutputVoltage()),
                 (long)HostLink_Ma(App_GetOutputCurrent()),
                 (long)HostLink_Mv(LdoLink_IsOutputWanted() || LdoPrereg_IsG0Active()
                                   ? LdoLink_GetG0Voltage()
                                   : App_GetCvSetpoint()),
                 (long)HostLink_Ma(LdoLink_GetG0Current()),
                 (unsigned long)(PowerStage_GetDutyA() * 1000000.0f + 0.5f),
                 (unsigned int)PSU_IsRunning(),
                 HostLink_ModeName(),
                 (unsigned long)App_GetFaultFlags(),
                 (unsigned int)pd_ok,
                 (long)HostLink_Mv(pd_v),
                 (long)HostLink_Ma(pd_a),
                 (long)HostLink_Ma(pd_w),
                 (unsigned int)LdoPrereg_IsPermitGranted(),
                 (unsigned int)(LdoLink_IsRemoteSenseEnabled() ? 1U : 0U),
                 (unsigned int)(prereg.g0_active ? 1U : 0U),
                 (unsigned int)(ldo.output_on ? 1U : 0U),
                 (unsigned int)(LdoLink_IsOutputWanted() ? 1U : 0U),
                 (unsigned int)LdoLink_GetCtrlState(),
                 (unsigned int)ldo.kill_reported,
                 (unsigned int)ldo.outoff_reported,
                 (unsigned long)ldo.vout_mv,
                 (long)(prereg.vpre_request_v * 1000.0f),
                 (long)(prereg.vpre_command_v * 1000.0f),
                 (unsigned int)(prereg.regulation_ok ? 1U : 0U),
                 (unsigned int)App_IsStageEnabled(),
                 (unsigned int)PowerStage_IsEnabled(),
                 (unsigned int)PowerStage_IsFaultActive(),
                 (unsigned long)App_GetStartupHoldRemainingMs(),
                 (unsigned int)PowerStage_GetLastError(),
                 (unsigned long)ldo.rx_bytes,
                 (unsigned long)ldo.tlm_count,
                 (unsigned long)g0_age_ms,
                 (unsigned long)ldo.rx_errors,
                 (unsigned long)ldo.last_error_code,
                 (unsigned int)pm.state);
    if (n > 0) {
        HostLink_Tx(line);
    }

    /* TB — full BMS snapshot (cells, pack/stack V, pack I, FETs, safety) */
    n = snprintf(line, sizeof(line),
                 "TB bms=%u cfg=%u st=%u fault=0x%08lX alert=%u alarm=0x%04X "
                 "sa=0x%02X sb=0x%02X sc=0x%02X fet=0x%02X manuf=0x%04X "
                 "chg=%u dsg=%u fets=%u series=%u "
                 "c1_mv=%d c2_mv=%d c3_mv=%d c4_mv=%d c5_mv=%d "
                 "min_mv=%d max_mv=%d dV_mv=%d sum_mv=%d "
                 "pack_mv=%d stack_mv=%d i_pack_ma=%d i_cc2_ma=%d "
                 "sample=%u alerts=%lu i2c_err=%lu\r\n",
                 (unsigned int)(BQ76922_IsEnabled() && bms.present ? 1U : 0U),
                 (unsigned int)(BQ76922_IsEnabled() && bms.configured ? 1U : 0U),
                 (unsigned int)(BQ76922_IsEnabled() ? bms.state : 0U),
                 (unsigned long)(BQ76922_IsEnabled() ? bms.fault_flags : 0UL),
                 (unsigned int)(BQ76922_IsEnabled() &&
                                (bms.alert_latched || bms.alert_pin) ? 1U : 0U),
                 (unsigned int)bms.alarm_status,
                 (unsigned int)bms.safety_status_a,
                 (unsigned int)bms.safety_status_b,
                 (unsigned int)bms.safety_status_c,
                 (unsigned int)bms.fet_status,
                 (unsigned int)bms.manuf_status,
                 (unsigned int)(bms.chg_fet_on ? 1U : 0U),
                 (unsigned int)(bms.dsg_fet_on ? 1U : 0U),
                 (unsigned int)(bms.fets_enabled ? 1U : 0U),
                 (unsigned int)BMS_SERIES_COUNT,
                 (int)bms.cell_mv[0],
                 (int)bms.cell_mv[1],
                 (int)bms.cell_mv[2],
                 (int)bms.cell_mv[3],
                 (int)bms.cell_mv[4],
                 (int)bms.min_cell_mv,
                 (int)bms.max_cell_mv,
                 (int)dV,
                 (int)bms.cell_sum_mv,
                 (int)bms.pack_mv,
                 (int)bms.stack_mv,
                 (int)bms.cc2_ma,
                 (int)bms.cc2_ma,
                 (unsigned int)(bms.sample_valid ? 1U : 0U),
                 (unsigned long)bms.alert_count,
                 (unsigned long)bms.i2c_error_count);
    if (n > 0) {
        HostLink_Tx(line);
    }

    /* TC — BQ25731 charger + TPS path (everything already decoded) */
    n = snprintf(line, sizeof(line),
                 "TC bq_ok=%u bq_vbat_mv=%lu bq_vsys_mv=%lu bq_ibat_ma=%ld "
                 "bq_ichg_ma=%lu bq_idchg_ma=%lu bq_vbus_mv=%lu bq_iin_ma=%lu "
                 "bq_vreg_mv=%lu bq_ichg_set_ma=%lu bq_iin_set_ma=%lu "
                 "bq_st=0x%04X bq_fault=0x%02X "
                 "bq_in=%u bq_pre=%u bq_fast=%u bq_otg=%u bq_iindpm=%u bq_vindpm=%u "
                 "tps_vbus_mv=%lu cc1=%u cc2=%u role=%u conn=%u "
                 "pd_role=%u pd_mv=%lu pd_ma=%lu\r\n",
                 (unsigned int)(pm.bq.online && pm.bq.adc_sample_valid ? 1U : 0U),
                 (unsigned long)pm.bq.adc_vbat_mv,
                 (unsigned long)pm.bq.adc_vsys_mv,
                 (long)pm.bq.battery_current_ma,
                 (unsigned long)pm.bq.adc_ichg_ma,
                 (unsigned long)pm.bq.adc_idchg_ma,
                 (unsigned long)pm.bq.adc_vbus_mv,
                 (unsigned long)pm.bq.adc_iin_ma,
                 (unsigned long)pm.bq.charge_voltage_mv,
                 (unsigned long)pm.bq.charge_current_ma,
                 (unsigned long)pm.bq.input_current_ma,
                 (unsigned int)pm.bq.charger_status,
                 (unsigned int)pm.bq.fault_flags,
                 (unsigned int)(pm.bq.input_present ? 1U : 0U),
                 (unsigned int)(pm.bq.in_precharge ? 1U : 0U),
                 (unsigned int)(pm.bq.in_fast_charge ? 1U : 0U),
                 (unsigned int)(pm.bq.in_otg ? 1U : 0U),
                 (unsigned int)(pm.bq.in_iin_dpm ? 1U : 0U),
                 (unsigned int)(pm.bq.in_vindpm ? 1U : 0U),
                 (unsigned long)pm.tps.vbus_mv,
                 (unsigned int)pm.tps.cc1_state,
                 (unsigned int)pm.tps.cc2_state,
                 (unsigned int)pm.tps.role,
                 (unsigned int)pm.tps.connection_state,
                 (unsigned int)pm.pd_snapshot.power_role,
                 (unsigned long)pm.pd_snapshot.contract_voltage_mv,
                 (unsigned long)pm.pd_snapshot.contract_current_ma);
    if (n > 0) {
        HostLink_Tx(line);
    }
}

static void HostLink_PrintMv(char *out, size_t out_sz, int32_t mv)
{
    int32_t abs_mv = (mv < 0) ? -mv : mv;
    (void)snprintf(out, out_sz, "%s%ld.%03ld",
                   (mv < 0) ? "-" : "",
                   (long)(abs_mv / 1000),
                   (long)(abs_mv % 1000));
}

static void HostLink_SendHumanTelemetry(void)
{
    char line[HOST_LINK_TX_MAX];
    char c1[12], c2[12], c3[12], c4[12], c5[12];
    char s_vin[16], s_vout[16], s_iout[16], s_set[16], s_ilim[16];
    char s_pdv[16], s_pda[16], s_pdw[16];
    BQ76922_Snapshot_t bms;
    LdoPrereg_Status_t prereg;
    LdoLink_Status_t ldo;
    PowerManager_Status_t pm;
    float pd_v = 0.0f;
    float pd_a = 0.0f;
    float pd_w = 0.0f;
    uint8_t pd_ok;
    int16_t dV;
    const char *bq_phase;

    BQ76922_GetSnapshot(&g_bq76922, &bms);
    pd_ok = PSU_GuiGetPdContract(&pd_v, &pd_a, &pd_w, NULL);
    LdoPrereg_GetStatus(&prereg);
    LdoLink_GetStatus(&ldo);
    PowerManager_GetStatus(&pm);
    dV = (int16_t)(bms.max_cell_mv - bms.min_cell_mv);

    HostLink_FmtCell(c1, sizeof(c1), bms.cell_mv[0], 0U);
    HostLink_FmtCell(c2, sizeof(c2), bms.cell_mv[1], 1U);
    HostLink_FmtCell(c3, sizeof(c3), bms.cell_mv[2], 2U);
    HostLink_FmtCell(c4, sizeof(c4), bms.cell_mv[3], 3U);
    HostLink_FmtCell(c5, sizeof(c5), bms.cell_mv[4], 4U);

    HostLink_PrintMv(s_vin, sizeof(s_vin), HostLink_Mv(App_GetInputVoltage()));
    HostLink_PrintMv(s_vout, sizeof(s_vout), HostLink_Mv(App_GetOutputVoltage()));
    HostLink_PrintMv(s_iout, sizeof(s_iout), HostLink_Ma(App_GetOutputCurrent()));
    HostLink_PrintMv(s_set, sizeof(s_set), HostLink_Mv(LdoLink_GetG0Voltage()));
    HostLink_PrintMv(s_ilim, sizeof(s_ilim), HostLink_Ma(LdoLink_GetG0Current()));
    HostLink_PrintMv(s_pdv, sizeof(s_pdv), HostLink_Mv(pd_v));
    HostLink_PrintMv(s_pda, sizeof(s_pda), HostLink_Ma(pd_a));
    HostLink_PrintMv(s_pdw, sizeof(s_pdw), HostLink_Ma(pd_w));

    if (pm.bq.in_fast_charge) {
        bq_phase = "FAST";
    } else if (pm.bq.in_precharge) {
        bq_phase = "PRE";
    } else if (pm.bq.in_otg) {
        bq_phase = "OTG";
    } else if (pm.bq.input_present) {
        bq_phase = "IDLE_IN";
    } else {
        bq_phase = "NO_IN";
    }

    (void)snprintf(line, sizeof(line),
                   "\r\n======== TEL t=%lu ms (HUMAN) ========\r\n",
                   (unsigned long)HAL_GetTick());
    HostLink_Tx(line);

    (void)snprintf(line, sizeof(line),
                   "PSU   VIN=%s V  VOUT=%s V  IOUT=%s A\r\n"
                   "      SET=%s V  ILIM=%s A  MODE=%s  RUN=%u  FAULT=0x%lX\r\n",
                   s_vin, s_vout, s_iout, s_set, s_ilim,
                   HostLink_ModeName(),
                   (unsigned int)PSU_IsRunning(),
                   (unsigned long)App_GetFaultFlags());
    HostLink_Tx(line);

    (void)snprintf(line, sizeof(line),
                   "PD    %s  contract=%s V / %s A / %s W  ok=%u\r\n"
                   "      TPS_VBUS=%lu mV  CC1=%u CC2=%u  conn=%u\r\n",
                   HostLink_PdRoleName(pm.pd_snapshot.power_role),
                   s_pdv, s_pda, s_pdw,
                   (unsigned int)pd_ok,
                   (unsigned long)pm.tps.vbus_mv,
                   (unsigned int)pm.tps.cc1_state,
                   (unsigned int)pm.tps.cc2_state,
                   (unsigned int)pm.tps.connection_state);
    HostLink_Tx(line);

    (void)snprintf(line, sizeof(line),
                   "BMS   %uS %s  FETs CHG=%s DSG=%s  alert=%u\r\n"
                   "      pack=%d mV  stack=%d mV  I_pack=%d mA  sum_cells=%d mV\r\n"
                   "      C1=%s C2=%s C3=%s C4=%s C5=%s mV\r\n"
                   "      min=%d max=%d dV=%d  SA=0x%02X SB=0x%02X SC=0x%02X "
                   "alarm=0x%04X fault=0x%08lX\r\n",
                   (unsigned int)BMS_SERIES_COUNT,
                   HostLink_BmsStateName(bms.state),
                   bms.chg_fet_on ? "ON" : "off",
                   bms.dsg_fet_on ? "ON" : "off",
                   (unsigned int)(bms.alert_latched || bms.alert_pin),
                   (int)bms.pack_mv,
                   (int)bms.stack_mv,
                   (int)bms.cc2_ma,
                   (int)bms.cell_sum_mv,
                   c1, c2, c3, c4, c5,
                   (int)bms.min_cell_mv,
                   (int)bms.max_cell_mv,
                   (int)dV,
                   (unsigned int)bms.safety_status_a,
                   (unsigned int)bms.safety_status_b,
                   (unsigned int)bms.safety_status_c,
                   (unsigned int)bms.alarm_status,
                   (unsigned long)bms.fault_flags);
    HostLink_Tx(line);

    (void)snprintf(line, sizeof(line),
                   "BQ    VBAT=%lu mV  IBAT=%ld mA  VSYS=%lu mV  phase=%s\r\n"
                   "      VBUS=%lu mV  IIN=%lu mA  VREG=%lu mV  ICHG_SET=%lu mA\r\n"
                   "      status=0x%04X fault=0x%02X  IINDPM=%u VINDPM=%u\r\n",
                   (unsigned long)pm.bq.adc_vbat_mv,
                   (long)pm.bq.battery_current_ma,
                   (unsigned long)pm.bq.adc_vsys_mv,
                   bq_phase,
                   (unsigned long)pm.bq.adc_vbus_mv,
                   (unsigned long)pm.bq.adc_iin_ma,
                   (unsigned long)pm.bq.charge_voltage_mv,
                   (unsigned long)pm.bq.charge_current_ma,
                   (unsigned int)pm.bq.charger_status,
                   (unsigned int)pm.bq.fault_flags,
                   (unsigned int)(pm.bq.in_iin_dpm ? 1U : 0U),
                   (unsigned int)(pm.bq.in_vindpm ? 1U : 0U));
    HostLink_Tx(line);

    (void)snprintf(line, sizeof(line),
                   "G0    link=%u out=%u want=%u ctrl=%u kill=%u outoff=%u "
                   "vout=%lu mV\r\n"
                   "PRE   req=%ld mV cmd=%ld mV permit=%u reg_ok=%u\r\n"
                   "========================================\r\n",
                   (unsigned int)(prereg.g0_active ? 1U : 0U),
                   (unsigned int)(ldo.output_on ? 1U : 0U),
                   (unsigned int)(LdoLink_IsOutputWanted() ? 1U : 0U),
                   (unsigned int)LdoLink_GetCtrlState(),
                   (unsigned int)ldo.kill_reported,
                   (unsigned int)ldo.outoff_reported,
                   (unsigned long)ldo.vout_mv,
                   (long)(prereg.vpre_request_v * 1000.0f),
                   (long)(prereg.vpre_command_v * 1000.0f),
                   (unsigned int)LdoPrereg_IsPermitGranted(),
                   (unsigned int)(prereg.regulation_ok ? 1U : 0U));
    HostLink_Tx(line);
}

static void HostLink_SendTelemetry(void)
{
    if (s_human_mode != 0U) {
        HostLink_SendHumanTelemetry();
    } else {
        HostLink_SendMachineTelemetry();
    }
}

static bool HostLink_EqToken(const char *s, const char *token)
{
    while ((*s != '\0') && (*token != '\0')) {
        if (toupper((unsigned char)*s) != toupper((unsigned char)*token)) {
            return false;
        }
        s++;
        token++;
    }
    return (*token == '\0') && ((*s == '\0') || isspace((unsigned char)*s));
}

static const char *HostLink_SkipToken(const char *s)
{
    while ((*s != '\0') && !isspace((unsigned char)*s)) {
        s++;
    }
    while (isspace((unsigned char)*s)) {
        s++;
    }
    return s;
}

static bool HostLink_ParseFloat(const char *s, float *out)
{
    float value = 0.0f;
    float frac = 0.1f;
    bool neg = false;
    bool seen = false;
    bool after_dot = false;

    if ((s == NULL) || (out == NULL)) {
        return false;
    }
    while (isspace((unsigned char)*s)) {
        s++;
    }
    if (*s == '-') {
        neg = true;
        s++;
    } else if (*s == '+') {
        s++;
    }
    while (*s != '\0') {
        if ((*s >= '0') && (*s <= '9')) {
            seen = true;
            if (!after_dot) {
                value = (value * 10.0f) + (float)(*s - '0');
            } else {
                value += (float)(*s - '0') * frac;
                frac *= 0.1f;
            }
        } else if ((*s == '.') && !after_dot) {
            after_dot = true;
        } else if (isspace((unsigned char)*s)) {
            break;
        } else {
            return false;
        }
        s++;
    }
    if (!seen) {
        return false;
    }
    *out = neg ? -value : value;
    return true;
}

static bool HostLink_ParseU32(const char *s, uint32_t *out)
{
    uint32_t value = 0U;
    bool seen = false;

    if ((s == NULL) || (out == NULL)) {
        return false;
    }
    while (isspace((unsigned char)*s)) {
        s++;
    }
    while ((*s >= '0') && (*s <= '9')) {
        seen = true;
        value = (value * 10U) + (uint32_t)(*s - '0');
        s++;
    }
    if (!seen) {
        return false;
    }
    *out = value;
    return true;
}

static void HostLink_SendHelp(void)
{
    HostLink_Tx(
        "HELP G4 host USART1 115200\r\n"
        "  ON / OFF        start/stop DCDC + G0 LDO\r\n"
        "  SET <V>         G0 voltage e.g. SET 5.0\r\n"
        "  ILIM <A>        G0 current e.g. ILIM 0.1\r\n"
        "  PERMIT 0|1      hard kill / allow PB7\r\n"
        "  REMOTE ON|OFF   sense path\r\n"
        "  HUMAN 0|1       0=machine T/TB/TC (H7), 1=human dump\r\n"
        "  MACHINE         alias HUMAN 0\r\n"
        "  STATUS          one human snapshot (even in MACHINE)\r\n"
        "  TEL [ms]        periodic telemetry (0=off, default 500)\r\n"
        "  ?               one telemetry frame now\r\n"
        "  G0DIAG / G0SWAP USART2 diag / TX-RX swap\r\n"
        "  CLR             clear fault latch\r\n"
        "H7: parse lines T (PSU), TB (BMS), TC (BQ/TPS); key=value ints.\r\n");
}

static void HostLink_SendStatus(void)
{
    /* Always human-readable, regardless of HUMAN mode. */
    HostLink_SendHumanTelemetry();
}

static void HostLink_HandleLine(char *line)
{
    const char *arg;
    float value;
    uint32_t u32;
    char reply[80];
    uint32_t v_mv;
    uint32_t i_ma;

    while ((*line != '\0') && isspace((unsigned char)*line)) {
        line++;
    }
    if (*line == '\0') {
        return;
    }

    arg = HostLink_SkipToken(line);

    if (HostLink_EqToken(line, "HELP") || HostLink_EqToken(line, "H")) {
        HostLink_SendHelp();
        return;
    }
    if (HostLink_EqToken(line, "STATUS") || HostLink_EqToken(line, "ST")) {
        HostLink_SendStatus();
        return;
    }
    if (HostLink_EqToken(line, "G0DIAG") || HostLink_EqToken(line, "G0")) {
        LdoLink_DumpDiag();
        return;
    }
    if (HostLink_EqToken(line, "G0SWAP")) {
        if (!HostLink_ParseU32(arg, &u32) || (u32 > 1U)) {
            HostLink_Tx("ERR G0SWAP use: G0SWAP 0|1\r\n");
            return;
        }
        LdoLink_SetUartPinSwap(u32 != 0U);
        (void)snprintf(reply, sizeof(reply), "OK G0SWAP %lu\r\n", (unsigned long)u32);
        HostLink_Tx(reply);
        LdoLink_DumpDiag();
        return;
    }
    if (HostLink_EqToken(line, "ON")) {
        LdoPrereg_SetForceDisable(false);
        LdoPrereg_SetPermitOverrideOff(false);
        PSU_Start();
        LdoLink_RequestOutput(true);
        v_mv = (uint32_t)((LdoLink_GetG0Voltage() * 1000.0f) + 0.5f);
        i_ma = (uint32_t)((LdoLink_GetG0Current() * 1000.0f) + 0.5f);
        (void)snprintf(reply, sizeof(reply),
                       "OK ON set=%lu mV ilim=%lu mA (wait g0_out=1)\r\n",
                       (unsigned long)v_mv, (unsigned long)i_ma);
        HostLink_Tx(reply);
        if (!LdoPrereg_IsG0Active()) {
            HostLink_Tx("WARN g0_rx=0 — DCDC may start; LDO needs G0 TLM\r\n");
        }
        return;
    }
    if (HostLink_EqToken(line, "OFF")) {
        LdoLink_RequestOutput(false);
        LdoPrereg_SetForceDisable(true);
        LdoPrereg_SetPermitOverrideOff(true);
        PSU_Stop();
        HostLink_Tx("OK OFF\r\n");
        return;
    }
    if (HostLink_EqToken(line, "CLR") || HostLink_EqToken(line, "CLEAR")) {
        App_ClearFaults();
        HostLink_Tx("OK CLR\r\n");
        return;
    }
    if (HostLink_EqToken(line, "SET")) {
        if (!HostLink_ParseFloat(arg, &value)) {
            HostLink_Tx("ERR SET use: SET 5.0\r\n");
            return;
        }
        LdoLink_SetG0Voltage(value);
        if (!LdoPrereg_IsG0Active() && !LdoLink_IsOutputWanted()) {
            PSU_GuiSetTargetVoltage(value + 3.0f);
        }
        v_mv = (uint32_t)((LdoLink_GetG0Voltage() * 1000.0f) + 0.5f);
        (void)snprintf(reply, sizeof(reply), "OK SET %lu mV\r\n", (unsigned long)v_mv);
        HostLink_Tx(reply);
        return;
    }
    if (HostLink_EqToken(line, "ILIM")) {
        if (!HostLink_ParseFloat(arg, &value)) {
            HostLink_Tx("ERR ILIM use: ILIM 0.1\r\n");
            return;
        }
        LdoLink_SetG0Current(value);
        PSU_GuiSetTargetCurrent(value);
        i_ma = (uint32_t)((LdoLink_GetG0Current() * 1000.0f) + 0.5f);
        (void)snprintf(reply, sizeof(reply), "OK ILIM %lu mA\r\n", (unsigned long)i_ma);
        HostLink_Tx(reply);
        return;
    }
    if (HostLink_EqToken(line, "USB")) {
        if (HostLink_EqToken(arg, "AUTO")) {
            (void)PSU_GuiSetUsbMode(PSU_GUI_USB_MODE_AUTO);
        } else if (HostLink_EqToken(arg, "SINK")) {
            (void)PSU_GuiSetUsbMode(PSU_GUI_USB_MODE_SINK_ONLY);
        } else if (HostLink_EqToken(arg, "SOURCE")) {
            (void)PSU_GuiSetUsbMode(PSU_GUI_USB_MODE_SOURCE_ONLY);
        } else {
            HostLink_Tx("ERR USB\r\n");
            return;
        }
        HostLink_Tx("OK USB\r\n");
        return;
    }
    if (HostLink_EqToken(line, "PERMIT")) {
        if (!HostLink_ParseU32(arg, &u32)) {
            HostLink_Tx("ERR PERMIT\r\n");
            return;
        }
        if (u32 == 0U) {
            LdoLink_RequestOutput(false);
            LdoPrereg_SetPermitOverrideOff(true);
            LdoPrereg_SetForceDisable(true);
            PSU_Stop();
            HostLink_Tx("OK PERMIT 0 (LDO zabity)\r\n");
        } else {
            LdoPrereg_SetPermitOverrideOff(false);
            LdoPrereg_SetForceDisable(false);
            HostLink_Tx("OK PERMIT 1\r\n");
        }
        return;
    }
    if (HostLink_EqToken(line, "REMOTE")) {
        if (HostLink_EqToken(arg, "ON") || HostLink_EqToken(arg, "1")) {
            LdoLink_SetRemoteSense(true);
            HostLink_Tx("OK REMOTE\r\n");
            return;
        }
        if (HostLink_EqToken(arg, "OFF") || HostLink_EqToken(arg, "0")) {
            LdoLink_SetRemoteSense(false);
            HostLink_Tx("OK LOCAL\r\n");
            return;
        }
        HostLink_Tx("ERR REMOTE\r\n");
        return;
    }
    if (HostLink_EqToken(line, "HUMAN") || HostLink_EqToken(line, "FMT")) {
        if ((*arg == '\0') || HostLink_EqToken(arg, "ON") || HostLink_EqToken(arg, "1") ||
            HostLink_EqToken(arg, "HUMAN")) {
            s_human_mode = 1U;
            HostLink_Tx("OK HUMAN 1 (readable dump; STATUS/?/TEL use human)\r\n");
            HostLink_SendHumanTelemetry();
            return;
        }
        if (HostLink_EqToken(arg, "OFF") || HostLink_EqToken(arg, "0") ||
            HostLink_EqToken(arg, "MACHINE") || HostLink_EqToken(arg, "H7")) {
            s_human_mode = 0U;
            HostLink_Tx("OK HUMAN 0 (machine T/TB/TC for H7)\r\n");
            return;
        }
        HostLink_Tx("ERR HUMAN use: HUMAN 0|1  (or MACHINE)\r\n");
        return;
    }
    if (HostLink_EqToken(line, "MACHINE") || HostLink_EqToken(line, "H7")) {
        s_human_mode = 0U;
        HostLink_Tx("OK HUMAN 0 (machine T/TB/TC for H7)\r\n");
        return;
    }
    if (HostLink_EqToken(line, "TEL")) {
        if (*arg == '\0') {
            s_tel_period_ms = HOST_LINK_TEL_DEFAULT_MS;
        } else if (!HostLink_ParseU32(arg, &s_tel_period_ms)) {
            HostLink_Tx("ERR TEL\r\n");
            return;
        }
        (void)snprintf(reply, sizeof(reply), "OK TEL %lu ms human=%u\r\n",
                       (unsigned long)s_tel_period_ms,
                       (unsigned int)s_human_mode);
        HostLink_Tx(reply);
        return;
    }
    if (HostLink_EqToken(line, "?")) {
        HostLink_SendTelemetry();
        return;
    }

    HostLink_Tx("ERR CMD — send HELP\r\n");
}

void HostLink_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_rx_len = 0U;
    s_line_ready = false;
    s_rx_overflow = false;
    s_human_mode = 0U;
    s_tel_period_ms = HOST_LINK_TEL_DEFAULT_MS;
    s_last_tel_ms = HAL_GetTick();
    HostLink_ArmRx();
    HostLink_Tx("\r\n=== Lab_PD_PSU G4 host ready (USART1 115200) ===\r\n");
    HostLink_Tx("Hint: HUMAN 1 for readable telemetry; MACHINE for H7 T/TB/TC\r\n");
    HostLink_SendHelp();
}

void HostLink_Task(void)
{
    uint32_t now_ms;
    char line[HOST_LINK_RX_LINE_MAX];

    if (s_huart == NULL) {
        return;
    }

    if (s_line_ready) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        memcpy(line, s_rx_line, sizeof(line));
        s_line_ready = false;
        s_rx_len = 0U;
        if (primask == 0U) {
            __enable_irq();
        }
        HostLink_HandleLine(line);
    } else if (s_rx_overflow) {
        s_rx_overflow = false;
        HostLink_Tx("ERR LINE\r\n");
    }

    now_ms = HAL_GetTick();
    if ((s_tel_period_ms > 0U) &&
        ((uint32_t)(now_ms - s_last_tel_ms) >= s_tel_period_ms)) {
        s_last_tel_ms = now_ms;
        HostLink_SendTelemetry();
    }
}

void HostLink_ForwardLine(const char *line)
{
    if ((line == NULL) || (s_huart == NULL)) {
        return;
    }
    HostLink_Tx(line);
    HostLink_Tx("\r\n");
}

void HostLink_OnUartError(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart == s_huart)) {
        HostLink_ArmRx();
    }
}

void HostLink_RxCplt(UART_HandleTypeDef *huart)
{
    uint8_t ch;

    if ((huart == NULL) || (huart != s_huart)) {
        return;
    }

    ch = s_rx_byte;
    if ((ch == (uint8_t)'\n') || (ch == (uint8_t)'\r')) {
        if (s_rx_len > 0U) {
            s_rx_line[s_rx_len] = '\0';
            s_line_ready = true;
            s_rx_len = 0U;
        }
    } else if (!s_line_ready) {
        if (s_rx_len < (HOST_LINK_RX_LINE_MAX - 1U)) {
            s_rx_line[s_rx_len++] = (char)ch;
        } else {
            s_rx_overflow = true;
            s_rx_len = 0U;
        }
    }

    HostLink_ArmRx();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    HostLink_RxCplt(huart);
    LdoLink_RxCplt(huart);
}
