#include "host_link.h"

#include "app.h"
#include "board_rev.h"
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
    (void)HAL_UART_Transmit(s_huart, (uint8_t *)text, (uint16_t)len, 40U);
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

static void HostLink_SendTelemetry(void)
{
    char line[HOST_LINK_TX_MAX];
    BQ76922_Snapshot_t bms;
    float pd_v = 0.0f;
    float pd_a = 0.0f;
    float pd_w = 0.0f;
    uint8_t pd_ok;
    int n;

    BQ76922_GetSnapshot(&g_bq76922, &bms);
    pd_ok = PSU_GuiGetPdContract(&pd_v, &pd_a, &pd_w, NULL);

    LdoPrereg_Status_t prereg;
    LdoLink_Status_t ldo;
    PowerManager_Status_t pm;
    uint32_t now_ms = HAL_GetTick();
    uint32_t g0_age_ms = 0U;

    LdoPrereg_GetStatus(&prereg);
    LdoLink_GetStatus(&ldo);
    PowerManager_GetStatus(&pm);

    if (ldo.last_rx_ms != 0U) {
        g0_age_ms = now_ms - ldo.last_rx_ms;
    } else {
        g0_age_ms = 0xFFFFFFFFu;
    }

    n = snprintf(line, sizeof(line),
                 "T vin_mv=%ld vout_mv=%ld iout_ma=%ld set_mv=%ld ilim_ma=%ld "
                 "duty_ppm=%lu run=%u mode=%s fault=%lu pd=%u pd_mv=%ld pd_ma=%ld pd_mw=%ld "
                 "permit=%u rem_sense=%u bms=%u bms_cfg=%u bms_st=%u bms_fault=0x%08lX alert=%u alarm=0x%04X "
                 "c1_mv=%d c2_mv=%d c3_mv=%d c4_mv=%d c5_mv=%d min_mv=%d max_mv=%d pack_mv=%d "
                 "i_cc2_ma=%d g0=%u g0_out=%u g0_want=%u g0_ctrl=%u g0_kill=%u g0_outoff=%u "
                 "g0_vout_mv=%lu vpre_req_mv=%ld vpre_cmd_mv=%ld reg_ok=%u "
                 "stage_en=%u ps_en=%u flt=%u hold_ms=%lu ps_err=%u "
                 "g0_rx=%lu g0_tlm=%lu g0_age_ms=%lu g0_err=%lu g0_uart=0x%lX "
                 "i2c_tps=%u i2c_bq=%u i2c_bms=%u pm_st=%u\r\n",
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
                 (unsigned int)(BQ76922_IsEnabled() && bms.present ? 1U : 0U),
                 (unsigned int)(BQ76922_IsEnabled() && bms.configured ? 1U : 0U),
                 (unsigned int)(BQ76922_IsEnabled() ? bms.state : 0U),
                 (unsigned long)(BQ76922_IsEnabled() ? bms.fault_flags : 0UL),
                 (unsigned int)(BQ76922_IsEnabled() &&
                                (bms.alert_latched || bms.alert_pin) ? 1U : 0U),
                 (unsigned int)bms.alarm_status,
                 (int)bms.cell_mv[0],
                 (int)bms.cell_mv[1],
                 (int)bms.cell_mv[2],
                 (int)bms.cell_mv[3],
                 (int)bms.cell_mv[4],
                 (int)bms.min_cell_mv,
                 (int)bms.max_cell_mv,
                 (int)bms.pack_mv,
                 (int)bms.cc2_ma,
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
                 (unsigned int)((pm.state != POWER_MANAGER_INIT) &&
                                (pm.state != POWER_MANAGER_FAULT) ? 1U : 0U),
                 (unsigned int)((pm.state == POWER_MANAGER_RUN) ||
                                (pm.state == POWER_MANAGER_BQ_PROBE) ||
                                (pm.state == POWER_MANAGER_BQ_ADC_SETUP) ||
                                (pm.state == POWER_MANAGER_DEGRADED) ? 1U : 0U),
                 (unsigned int)(BQ76922_IsEnabled() && bms.present ? 1U : 0U),
                 (unsigned int)pm.state);
    if (n > 0) {
        HostLink_Tx(line);
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
        "  ON              start DCDC + G0 LDO (SET/OUT sequencer)\r\n"
        "  OFF             stop LDO + DCDC (PB6 kill)\r\n"
        "  SET <V>         G0 voltage e.g. SET 5.0\r\n"
        "  ILIM <A>        G0 current e.g. ILIM 0.1\r\n"
        "  PERMIT 0|1      hard kill / allow PB6\r\n"
        "  REMOTE ON|OFF   sense path\r\n"
        "  STATUS          human summary\r\n"
        "  G0DIAG          USART2 ISR/GPIO dump (PA2 TX / PB4 RX)\r\n"
        "  G0SWAP 0|1      runtime TX/RX swap if flywires crossed\r\n"
        "  TEL [ms]        periodic T lines (0=off, default 500)\r\n"
        "  ?               one T line\r\n"
        "  CLR             clear fault latch\r\n"
        "Need G0 TLM on USART2 (g0_rx rising). LDO out = G0, not DCDC rail.\r\n");
}

static void HostLink_SendStatus(void)
{
    char line[256];
    LdoLink_Status_t ldo;
    LdoPrereg_Status_t prereg;
    uint32_t v_mv;
    uint32_t i_ma;

    LdoLink_GetStatus(&ldo);
    LdoPrereg_GetStatus(&prereg);
    v_mv = (uint32_t)((LdoLink_GetG0Voltage() * 1000.0f) + 0.5f);
    i_ma = (uint32_t)((LdoLink_GetG0Current() * 1000.0f) + 0.5f);

    (void)snprintf(line, sizeof(line),
                   "STATUS run=%u mode=%s dcdc_mv=%ld g0_link=%u g0_rx=%lu g0_tlm=%lu "
                   "g0_want=%u g0_ctrl=%u g0_out=%u g0_kill=%u g0_outoff=%u "
                   "permit=%u set=%lu mV ilim=%lu mA g0_vout=%lu mV\r\n",
                   (unsigned int)PSU_IsRunning(),
                   HostLink_ModeName(),
                   (long)HostLink_Mv(App_GetOutputVoltage()),
                   (unsigned int)(prereg.g0_active ? 1U : 0U),
                   (unsigned long)ldo.rx_bytes,
                   (unsigned long)ldo.tlm_count,
                   (unsigned int)(LdoLink_IsOutputWanted() ? 1U : 0U),
                   (unsigned int)LdoLink_GetCtrlState(),
                   (unsigned int)(ldo.output_on ? 1U : 0U),
                   (unsigned int)ldo.kill_reported,
                   (unsigned int)ldo.outoff_reported,
                   (unsigned int)LdoPrereg_IsPermitGranted(),
                   (unsigned long)v_mv,
                   (unsigned long)i_ma,
                   (unsigned long)ldo.vout_mv);
    HostLink_Tx(line);

    if (ldo.rx_bytes == 0U) {
        HostLink_Tx("HINT no G0 UART into MCU — J6 TLM can be G0-side of isolator.\r\n"
                    "HINT probe PB4 RX (or PA2 if swapped). Flywire: not PB14/15.\r\n");
    } else if (LdoLink_IsOutputWanted() && (!ldo.output_on)) {
        HostLink_Tx("HINT G0 sequencer running — wait g0_ctrl=9 and g0_out=1\r\n");
    } else if (ldo.output_on) {
        HostLink_Tx("HINT LDO should be live — meter G0 output (not G4 DCDC/PB2)\r\n");
    } else {
        HostLink_Tx("HINT send: SET 5.0  then  ILIM 0.1  then  ON\r\n");
    }
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

void HostLink_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_rx_len = 0U;
    s_line_ready = false;
    s_rx_overflow = false;
    s_tel_period_ms = HOST_LINK_TEL_DEFAULT_MS;
    s_last_tel_ms = HAL_GetTick();
    HostLink_ArmRx();
    HostLink_Tx("\r\n=== Lab_PD_PSU G4 host ready (USART1 115200) ===\r\n");
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
