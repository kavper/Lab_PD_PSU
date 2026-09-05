#include "host_link.h"

#include "app.h"
#include "board_rev.h"
#include "bms_board.h"
#include "bq76922.h"
#include "debug_uart.h"
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
static uint32_t s_boot_reset_flags;
static uint8_t s_rx_byte;
static char s_rx_line[HOST_LINK_RX_LINE_MAX];
static volatile uint8_t s_rx_len;
static volatile bool s_line_ready;
static volatile bool s_rx_overflow;
static uint32_t s_tel_period_ms = HOST_LINK_TEL_DEFAULT_MS;
static uint32_t s_last_tel_ms;

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
                 "T vin_mv=%ld vout_mv=%ld iout_ma=%ld i_buck_ma=%ld i_boost_ma=%ld "
                 "set_mv=%ld ilim_ma=%ld "
                 "duty_ppm=%lu ucc_a=%u ucc_c=%u run=%u mode=%s fault=%lu "
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
                 (long)HostLink_Ma(App_GetHsBuckCurrent()),
                 (long)HostLink_Ma(App_GetHsBoostCurrent()),
                 (long)HostLink_Mv(LdoLink_IsOutputWanted() || LdoPrereg_IsG0Active()
                                   ? LdoLink_GetG0Voltage()
                                   : App_GetCvSetpoint()),
                 (long)HostLink_Ma(LdoLink_GetG0Current()),
                 (unsigned long)(PowerStage_GetDutyA() * 1000000.0f + 0.5f),
                 (unsigned int)(PowerStage_IsBuckTrEnActive() ? 1U : 0U),
                 (unsigned int)(PowerStage_IsBoostTrEnActive() ? 1U : 0U),
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
                 "init_step=%u vcell_rb=0x%04X batt=0x%04X cfg_fail=%u "
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
                 (unsigned int)bms.init_step,
                 (unsigned int)bms.vcell_mode_rb,
                 (unsigned int)bms.battery_status,
                 (unsigned int)bms.cfg_fail_count,
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
                 "plug=%u typec=0x%02X rst=%lu rst_busy=%u "
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
                 (unsigned int)(pm.tps.attached ? 1U : 0U),
                 (unsigned int)pm.tps.typec_port_state,
                 (unsigned long)pm.pd_reset_seq,
                 (unsigned int)(pm.pd_reset_busy ? 1U : 0U),
                 (unsigned int)pm.pd_snapshot.power_role,
                 (unsigned long)pm.pd_snapshot.contract_voltage_mv,
                 (unsigned long)pm.pd_snapshot.contract_current_ma);
    if (n > 0) {
        HostLink_Tx(line);
    }
}

static void HostLink_SendTelemetry(void)
{
    HostLink_SendMachineTelemetry();
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
        "HELP G4 USART1 115200 — machine T/TB/TC for H7/parser\r\n"
        "  ON / OFF        start/stop DCDC + G0 LDO\r\n"
        "  SET <V> / ILIM <A>\r\n"
        "  PERMIT 0|1      PB7 kill / allow\r\n"
        "  REMOTE ON|OFF   sense path\r\n"
        "  TEL [ms]        periodic T/TB/TC (0=off, default 500)\r\n"
        "  ? / STATUS      one T/TB/TC frame now\r\n"
        "  BMS             soft: skip CFGUPDATE if already healthy\r\n"
        "  BMS FORCE       full BQ76922 CFGUPDATE + ALL_FETS_ON (may reboot)\r\n"
        "  BMS SHUTDOWN    enter AFE SHUTDOWN; wake = TS2 button only (auto FETs)\r\n"
        "  BMS OTP STATUS  read-only OTP/RAM snapshot (safe, no CFGUPDATE)\r\n"
        "  BMS OTP CHECK   OTP_WR_CHECK (BAT 10-12V; briefly drops FETs + reinit)\r\n"
        "  BMS OTP BURN I-UNDERSTAND-OTP   one-shot OTP program (lab only)\r\n"
        "  VERBOSE 0|1     debug spam on USART1 (default 0 — keep clean)\r\n"
        "  G0DIAG / G0SWAP / CLR\r\n"
        "Parse: lines starting T / TB / TC, key=value ints. See docs/HOST_TELEMETRY.md\r\n");
}

static void HostLink_SendStatus(void)
{
    HostLink_SendTelemetry();
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
    if (HostLink_EqToken(line, "VERBOSE")) {
        if ((*arg == '\0') || HostLink_EqToken(arg, "1") || HostLink_EqToken(arg, "ON")) {
            Debug_SetEnabled(true);
            HostLink_Tx("OK VERBOSE 1 (debug logs on USART1 — may break parsers)\r\n");
            return;
        }
        if (HostLink_EqToken(arg, "0") || HostLink_EqToken(arg, "OFF")) {
            Debug_SetEnabled(false);
            HostLink_Tx("OK VERBOSE 0\r\n");
            return;
        }
        HostLink_Tx("ERR VERBOSE use: VERBOSE 0|1\r\n");
        return;
    }
    if (HostLink_EqToken(line, "BMS") || HostLink_EqToken(line, "BMSREINIT")) {
        bool force = HostLink_EqToken(line, "BMSREINIT") ||
                     HostLink_EqToken(arg, "FORCE") ||
                     HostLink_EqToken(arg, "1") ||
                     HostLink_EqToken(arg, "REINIT");
        bool shutdown = HostLink_EqToken(arg, "SHUTDOWN") ||
                        HostLink_EqToken(arg, "SHUT") ||
                        HostLink_EqToken(arg, "OFF");
        bool otp = HostLink_EqToken(arg, "OTP");

        if (otp) {
            const char *otp_arg = HostLink_SkipToken(arg);
            char msg[220];

            if (HostLink_EqToken(otp_arg, "STATUS") || (*otp_arg == '\0')) {
                BQ76922_OtpReport_t rep;
                BQ76922_Status_t st = BQ76922_OtpGetReport(&g_bq76922, &rep);

                if (st != BQ76922_OK) {
                    HostLink_Tx("ERR BMS OTP STATUS I2C failed\r\n");
                    return;
                }
                (void)snprintf(msg, sizeof(msg),
                    "OK BMS OTP STATUS batt=0x%04X mfg_init=0x%04X "
                    "vcell=0x%04X fet_opt=0x%02X want=0x%02X match=%u "
                    "full=%u otpb_now=%u burned=%u "
                    "(ro; OTPB is normally 1 outside CFGUPDATE; CHECK tests it)\r\n",
                    (unsigned)rep.battery_status,
                    (unsigned)rep.mfg_status_init,
                    (unsigned)rep.vcell_mode,
                    (unsigned)rep.fet_options,
                    (unsigned)rep.fet_options_want,
                    (rep.fet_options == rep.fet_options_want) ? 1U : 0U,
                    rep.fullaccess ? 1U : 0U,
                    rep.otpb_blocked ? 1U : 0U,
                    rep.session_already_burned ? 1U : 0U);
                HostLink_Tx(msg);
                return;
            }

            if (HostLink_EqToken(otp_arg, "CHECK")) {
                uint8_t check = 0U;
                uint16_t fail = 0U;
                BQ76922_Status_t st =
                    BQ76922_OtpRunWrCheck(&g_bq76922, &check, &fail);

                (void)snprintf(msg, sizeof(msg),
                    "%s BMS OTP CHECK check=0x%02X fail_addr=0x%04X "
                    "nodata=%u nosig=%u "
                    "(0x80=golden readback verified + can burn; FETs reinited)\r\n",
                    (st == BQ76922_OK) ? "OK" : "ERR",
                    (unsigned)check,
                    (unsigned)fail,
                    ((check & 0x08U) != 0U) ? 1U : 0U,
                    ((check & 0x10U) != 0U) ? 1U : 0U);
                HostLink_Tx(msg);
                return;
            }

            if (HostLink_EqToken(otp_arg, "BURN")) {
                const char *token = HostLink_SkipToken(otp_arg);
                uint8_t check = 0U;
                uint8_t result = 0U;
                uint16_t fail = 0U;
                BQ76922_Status_t st;

                if (!HostLink_EqToken(token, "I-UNDERSTAND-OTP")) {
                    HostLink_Tx(
                        "ERR BMS OTP BURN requires exact token:\r\n"
                        "  BMS OTP BURN I-UNDERSTAND-OTP\r\n"
                        "OTP is one-way. Boot never burns. Run BMS OTP CHECK first.\r\n");
                    return;
                }

                HostLink_Tx(
                    "OK BMS OTP BURN starting — golden (fet_opt want 0x1D) + "
                    "OTP_WRITE (BAT 10-12V required)...\r\n");
                HAL_Delay(20U);
                st = BQ76922_OtpBurn(&g_bq76922, "I-UNDERSTAND-OTP",
                                     &check, &result, &fail);
                if (st != BQ76922_OK) {
                    (void)snprintf(msg, sizeof(msg),
                        "ERR BMS OTP BURN failed st=%d check=0x%02X "
                        "write=0x%02X fail_addr=0x%04X nodata=%u nosig=%u "
                        "(FULLACCESS+OTPB=0+BAT10-12V; 0x9308=PDSG bit may be exhausted)\r\n",
                        (int)st, (unsigned)check, (unsigned)result,
                        (unsigned)fail,
                        ((check & 0x08U) != 0U) ? 1U : 0U,
                        ((check & 0x10U) != 0U) ? 1U : 0U);
                    HostLink_Tx(msg);
                    return;
                }
                (void)snprintf(msg, sizeof(msg),
                    "OK BMS OTP BURNED check=0x%02X write=0x%02X — "
                    "cold-reset golden verified; then "
                    "BMS SHUTDOWN + short TS2.\r\n",
                    (unsigned)check, (unsigned)result);
                HostLink_Tx(msg);
                return;
            }

            HostLink_Tx(
                "ERR BMS OTP use: STATUS | CHECK | BURN I-UNDERSTAND-OTP\r\n");
            return;
        }

        if (shutdown) {
            BQ76922_Status_t st;

            /* Kill PSU path first so PACK load does not fight FET-off. */
            LdoLink_RequestOutput(false);
            LdoPrereg_SetForceDisable(true);
            LdoPrereg_SetPermitOverrideOff(true);
            PSU_Stop();

            HostLink_Tx(
                "OK BMS SHUTDOWN — balance cable on; release TS2 (no hold). "
                "Wake = short TS2 press only; FETs auto, no command.\r\n");
            /* Flush UART before AFE drops. */
            HAL_Delay(20U);
            st = BQ76922_EnterShutdown(&g_bq76922);
            if (st != BQ76922_OK) {
                HostLink_Tx("ERR BMS SHUTDOWN I2C failed (AFE may still be awake)\r\n");
            }
            return;
        }

        /* GUI "BMS config" historically sent plain BMS while FETs were already
         * on → full CFGUPDATE, I2C bus-hold (bq_ok=0), ALL_FETS_ON inrush,
         * VIN dip, PIN/POR reboot. Soft path skips that when healthy. */
        if (!force && BQ76922_IsConfiguredHealthy(&g_bq76922)) {
            BQ76922_ClearShutdownRequest(&g_bq76922);
            App_ClearFaults();
            HostLink_Tx(
                "OK BMS already cfg=1 FETs on — skipped CFGUPDATE "
                "(BMS FORCE = full reinit; may reboot)\r\n");
            return;
        }

        BQ76922_RequestReinit(&g_bq76922);
        BQ76922_ClearShutdownRequest(&g_bq76922);
        App_ClearFaults();
        HostLink_Tx(
            "OK BMS reinit (4S CFGUPDATE + ALL_FETS_ON, no OTP; "
            "bus-hold + FET inrush may reboot)\r\n");
        return;
    }
    if (HostLink_EqToken(line, "TEL")) {
        if (*arg == '\0') {
            s_tel_period_ms = HOST_LINK_TEL_DEFAULT_MS;
        } else if (!HostLink_ParseU32(arg, &s_tel_period_ms)) {
            HostLink_Tx("ERR TEL\r\n");
            return;
        }
        (void)snprintf(reply, sizeof(reply), "OK TEL %lu ms\r\n",
                       (unsigned long)s_tel_period_ms);
        HostLink_Tx(reply);
        return;
    }
    if (HostLink_EqToken(line, "?")) {
        HostLink_SendTelemetry();
        return;
    }

    HostLink_Tx("ERR CMD — send HELP\r\n");
}

void HostLink_SetBootResetFlags(uint32_t rcc_csr)
{
    s_boot_reset_flags = rcc_csr;
}

void HostLink_Init(UART_HandleTypeDef *huart)
{
    char line[96];
    int n;

    s_huart = huart;
    s_rx_len = 0U;
    s_line_ready = false;
    s_rx_overflow = false;
    s_tel_period_ms = HOST_LINK_TEL_DEFAULT_MS;
    s_last_tel_ms = HAL_GetTick();
    HostLink_ArmRx();
    HostLink_Tx("\r\n=== Lab_PD_PSU G4 host ready (USART1 115200) ===\r\n");
    n = snprintf(line, sizeof(line),
                 "boot rcc_csr=0x%08lX (PIN=%u POR=%u SFT=%u IWDG=%u WWDG=%u LPWR=%u)\r\n",
                 (unsigned long)s_boot_reset_flags,
                 (s_boot_reset_flags & RCC_CSR_PINRSTF) ? 1U : 0U,
                 (s_boot_reset_flags & RCC_CSR_BORRSTF) ? 1U : 0U,
                 (s_boot_reset_flags & RCC_CSR_SFTRSTF) ? 1U : 0U,
                 (s_boot_reset_flags & RCC_CSR_IWDGRSTF) ? 1U : 0U,
                 (s_boot_reset_flags & RCC_CSR_WWDGRSTF) ? 1U : 0U,
                 (s_boot_reset_flags & RCC_CSR_LPWRRSTF) ? 1U : 0U);
    if (n > 0) {
        HostLink_Tx(line);
    }
    HostLink_Tx("USART1 = T/TB/TC telemetry only (VERBOSE 0). Send HELP.\r\n");
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
