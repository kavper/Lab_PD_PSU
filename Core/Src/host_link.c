#include "host_link.h"

#include "app.h"
#include "board_rev.h"
#include "bms_board.h"
#include "bq76922.h"
#include "debug_uart.h"
#include "h7_link_proto.h"
#include "ldo_link.h"
#include "ldo_prereg.h"
#include "link_uart.h"
#include "power_manager.h"
#include "power_stage.h"
#include "psu_gui_api.h"
#include "fan_tach.h"
#include "remote_sense.h"
#include "host_link_policy.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *s_huart = NULL;
static uint32_t s_boot_reset_flags;
static LinkUart s_host_uart;
static H7LinkParser s_host_parser;
static uint32_t s_meter_ms;
static uint32_t s_slow_ms;
static uint8_t s_host_tx_seq;
static bool s_on_wait;
static uint8_t s_on_seq;
static uint32_t s_on_since_ms;
static uint8_t s_on_q[4];
static uint8_t s_on_qn;
static bool s_replay_valid[8];
static uint8_t s_replay_seq[8];
static uint8_t s_replay_ack[8];
static uint8_t s_replay_reason[8];

static int HostLink_ReplayIndex(uint8_t type)
{
    switch (type) {
    case H7_LINK_ON: return 0;
    case H7_LINK_OFF: return 1;
    case H7_LINK_CLEAR: return 2;
    case H7_LINK_PING: return 3;
    case H7_LINK_PERMIT: return 4;
    case H7_LINK_REMOTE: return 5;
    case H7_LINK_BMS: return 6;
    case H7_LINK_USB: return 7;
    default: return -1;
    }
}

static bool HostLink_QueueFrame(uint8_t type, uint8_t seq, const uint8_t *payload,
                                uint8_t payload_len, LinkUartPri pri,
                                uint8_t slow_index)
{
    uint8_t frame[H7_LINK_MAX_FRAME];
    uint16_t length;

    length = H7Link_Build(frame, sizeof(frame), type, seq, payload, payload_len);
    if (length == 0U) {
        return false;
    }
    return LinkUart_Submit(&s_host_uart, pri, slow_index, frame, length);
}

static void HostLink_QueueAck(uint8_t seq, uint8_t type)
{
    (void)HostLink_QueueFrame(H7_LINK_ACK, seq, &type, 1U, LINK_UART_PRI_ACK, 0U);
}

static void HostLink_QueueNack(uint8_t seq, uint8_t type, uint8_t reason)
{
    uint8_t payload[2];

    payload[0] = type;
    payload[1] = reason;
    (void)HostLink_QueueFrame(H7_LINK_NACK, seq, payload, 2U, LINK_UART_PRI_ACK, 0U);
}

static void HostLink_Remember(uint8_t type, uint8_t seq, bool ack, uint8_t reason)
{
    int slot = HostLink_ReplayIndex(type);

    if (slot < 0) {
        return;
    }
    s_replay_valid[slot] = true;
    s_replay_seq[slot] = seq;
    s_replay_ack[slot] = ack ? 1U : 0U;
    s_replay_reason[slot] = reason;
}

static void HostLink_Tx(const char *text)
{
    size_t length;
    size_t offset;

    if ((s_huart == NULL) || (text == NULL)) {
        return;
    }
    length = strlen(text);
    for (offset = 0U; offset < length;) {
        size_t chunk = length - offset;

        if (chunk > 96U) {
            chunk = 96U;
        }
        (void)HostLink_QueueFrame(H7_LINK_TEXT, s_host_tx_seq++,
                                  (const uint8_t *)(text + offset),
                                  (uint8_t)chunk, LINK_UART_PRI_TEXT, 0U);
        offset += chunk;
    }
}

static int32_t HostLink_Mv(float volts)
{
    return (int32_t)((volts * 1000.0f) + ((volts >= 0.0f) ? 0.5f : -0.5f));
}

static int32_t HostLink_Ma(float amps)
{
    return (int32_t)((amps * 1000.0f) + ((amps >= 0.0f) ? 0.5f : -0.5f));
}

/* 0..10000 duty → 0..1000, meaning 0.0% .. 100.0% (divide by 10 to get percent). */
static int32_t HostLink_DutyX10(uint32_t duty_10k)
{
    if (duty_10k > 10000U) {
        duty_10k = 10000U;
    }

    return (int32_t)((duty_10k + 5U) / 10U);
}


static void HostLink_SendTelemetry(void)
{
    HostLink_Tx("OK STATUS binary METER 5 ms, BMS/PD 200 ms. TEL is ignored.\r\n");
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

static void HostLink_FinishOn(bool ack, uint8_t reason);

static void HostLink_SendHelp(void)
{
    HostLink_Tx(
        "HELP G4 USART1 460800 binary. METER 5 ms, BMS/PD 200 ms.\r\n"
        "  TEXT_CMD 0x21 carries these lines. TEL is ignored.\r\n"
        "  ON / OFF / SET V= I= / PERMIT / REMOTE / BMS / USB\r\n"
        "  VERBOSE       ignored on the production link\r\n"
        "  G0DIAG / G0SWAP / CLR / STATUS\r\n");
}

static void HostLink_ApplyOff(void)
{
    if (s_on_wait) {
        HostLink_FinishOn(false, H7_LINK_NACK_UNSAFE);
    }
    LdoLink_RequestOutput(false);
    LdoPrereg_SetForceDisable(true);
    LdoPrereg_SetPermitOverrideOff(true);
    PSU_Stop();
}

static void HostLink_ApplyOn(void)
{
    LdoPrereg_SetForceDisable(false);
    LdoPrereg_SetPermitOverrideOff(false);
    PSU_Start();
    LdoLink_RequestOutput(true);
}

static void HostLink_SendStatus(void)
{
    HostLink_Tx("OK STATUS binary METER 5 ms, BMS/PD 200 ms. TEL is ignored.\r\n");
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
    if (!HostLink_LooksLikeHostCommand(line)) {
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
        HostLink_ApplyOn();
        HostLink_Tx("OK ON accepted (binary ACK is the production reply)\r\n");
        return;
    }
    if (HostLink_EqToken(line, "OFF")) {
        HostLink_ApplyOff();
        HostLink_Tx("OK OFF accepted\r\n");
        return;
    }
    if (HostLink_EqToken(line, "CLR") || HostLink_EqToken(line, "CLEAR")) {
        App_ClearFaults();
        HostLink_Tx("OK CLR\r\n");
        return;
    }
    if (HostLink_EqToken(line, "SET")) {
        const char *current_arg;
        float current_value;

        if ((toupper((unsigned char)arg[0]) == 'V') && (arg[1] == '=')) {
            current_arg = HostLink_SkipToken(arg);
            if (!HostLink_ParseFloat(&arg[2], &value) ||
                (toupper((unsigned char)current_arg[0]) != 'I') ||
                (current_arg[1] != '=') ||
                !HostLink_ParseFloat(&current_arg[2], &current_value) ||
                (*HostLink_SkipToken(current_arg) != '\0') ||
                (value < 0.0f) || (value > 27.0f) ||
                (current_value < 0.0f) || (current_value > 5.0f)) {
                HostLink_Tx("ERR SET use: SET V=0.000..27.000 I=0.000..5.000\r\n");
                return;
            }
            v_mv = (uint32_t)((value * 1000.0f) + 0.5f);
            i_ma = (uint32_t)((current_value * 1000.0f) + 0.5f);
            if (LdoLink_SubmitHostSet(s_host_tx_seq, v_mv, i_ma, NULL) ==
                LDO_HOST_SET_REPLAY_NACK) {
                HostLink_Tx("ERR SET range\r\n");
                return;
            }
            s_host_tx_seq++;
            if (!LdoPrereg_IsG0Active() && !LdoLink_IsOutputWanted()) {
                PSU_GuiSetTargetVoltage(value + BOARD_VPRE_MARGIN_V);
            }
            HostLink_Tx("OK SET queued until G0 ACK\r\n");
            return;
        }
        if (!HostLink_ParseFloat(arg, &value) || (value < 0.0f) ||
            (value > 27.0f)) {
            HostLink_Tx("ERR SET range: 0.000..27.000 V\r\n");
            return;
        }
        LdoLink_SetG0Voltage(value);
        if (!LdoPrereg_IsG0Active() && !LdoLink_IsOutputWanted()) {
            PSU_GuiSetTargetVoltage(value + BOARD_VPRE_MARGIN_V);
        }
        v_mv = (uint32_t)((LdoLink_GetG0Voltage() * 1000.0f) + 0.5f);
        (void)snprintf(reply, sizeof(reply), "OK SET %lu mV\r\n", (unsigned long)v_mv);
        HostLink_Tx(reply);
        return;
    }
    if (HostLink_EqToken(line, "ILIM")) {
        if (!HostLink_ParseFloat(arg, &value) || (value < 0.0f) ||
            (value > 5.0f)) {
            HostLink_Tx("ERR ILIM range: 0.000..5.000 A\r\n");
            return;
        }
        LdoLink_SetG0Current(value);
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
        (void)arg;
        HostLink_Tx("OK VERBOSE ignored (production link, TEXT only)\r\n");
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
        (void)arg;
        HostLink_Tx("ERR TEL ignored; METER is fixed at 5 ms\r\n");
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

static void HostLink_QueueMeter(void)
{
    uint8_t payload[H7_LINK_METER_BYTES];
    LdoPrereg_Status_t prereg;
    LdoLink_Status_t ldo;
    uint32_t now_ms = HAL_GetTick();
    uint32_t age = 0xFFFFU;
    uint8_t flags0 = 0U;
    uint8_t flags1 = 0U;
    uint8_t result_seq = 0U;
    uint8_t result_ack = 0U;
    uint8_t result_reason = 0U;

    memset(payload, 0, sizeof(payload));
    LdoPrereg_GetStatus(&prereg);
    LdoLink_GetStatus(&ldo);
    if (ldo.telemetry_valid && (ldo.last_tlm_ms != 0U)) {
        uint32_t raw_age = now_ms - ldo.last_tlm_ms;
        age = (raw_age > 0xFFFEU) ? 0xFFFEU : raw_age;
    }
    H7Link_PutU32(&payload[H7_METER_VIN_MV], (uint32_t)HostLink_Mv(App_GetInputVoltage()));
    H7Link_PutU32(&payload[H7_METER_VOUT_MV], (uint32_t)HostLink_Mv(App_GetOutputVoltage()));
    H7Link_PutI32(&payload[H7_METER_I_BUCK_MA], HostLink_Ma(App_GetHsBuckCurrent()));
    H7Link_PutI32(&payload[H7_METER_I_BOOST_MA], HostLink_Ma(App_GetHsBoostCurrent()));
    H7Link_PutU32(&payload[H7_METER_SET_MV],
                  (uint32_t)HostLink_Mv(LdoLink_GetG0Voltage()));
    H7Link_PutU32(&payload[H7_METER_ILIM_MA],
                  (uint32_t)HostLink_Ma(LdoLink_GetG0Current()));
    H7Link_PutU32(&payload[H7_METER_G0_VOUT_MV], ldo.vout_mv);
    H7Link_PutU32(&payload[H7_METER_G0_IOUT_MA], ldo.iout_ma);
    H7Link_PutU32(&payload[H7_METER_G0_VIN_MV], ldo.vin_mv);
    H7Link_PutU32(&payload[H7_METER_G0_VSET_MV], ldo.vset_mv);
    H7Link_PutU32(&payload[H7_METER_G0_ISET_MA], ldo.iset_ma);
    H7Link_PutU32(&payload[H7_METER_VPRE_REQ_MV],
                  (uint32_t)((prereg.vpre_request_v * 1000.0f) + 0.5f));
    H7Link_PutU32(&payload[H7_METER_VPRE_CMD_MV],
                  (uint32_t)((prereg.vpre_command_v * 1000.0f) + 0.5f));
    H7Link_PutU16(&payload[H7_METER_G0_AGE_MS], (uint16_t)age);
    H7Link_PutU16(&payload[H7_METER_DUTY_A_X10],
                  (uint16_t)HostLink_DutyX10(PowerStage_GetDutyA10k()));
    H7Link_PutU16(&payload[H7_METER_DUTY_C_X10],
                  (uint16_t)HostLink_DutyX10(PowerStage_GetDutyCPhys10k()));
    H7Link_PutU32(&payload[H7_METER_G0_FAULT], ldo.fault_flags);
    H7Link_PutU32(&payload[H7_METER_PSU_FAULT], App_GetFaultFlags());
    if (ldo.output_on) flags0 |= H7_F0_G0_OUT;
    if (LdoLink_IsOutputWanted()) flags0 |= H7_F0_G0_WANT;
    if (ldo.kill_reported) flags0 |= H7_F0_G0_KILL;
    if (ldo.outoff_reported) flags0 |= H7_F0_G0_OUTOFF;
    if (ldo.cc_cv) flags0 |= H7_F0_G0_CC;
    if (LdoPrereg_IsPermitGranted()) flags0 |= H7_F0_PERMIT;
    if (PSU_IsRunning()) flags0 |= H7_F0_RUN;
    if (prereg.regulation_ok) flags0 |= H7_F0_REG_OK;
    if (App_IsStageEnabled()) flags1 |= H7_F1_STAGE_EN;
    if (PowerStage_IsEnabled()) flags1 |= H7_F1_PS_EN;
    if (PowerStage_IsFaultActive()) flags1 |= H7_F1_PS_FAULT;
    if (LdoLink_IsRemoteSenseEnabled()) flags1 |= H7_F1_REM_SENSE;
    if (PowerStage_IsBuckTrEnActive()) flags1 |= H7_F1_UCC_A;
    if (PowerStage_IsBoostTrEnActive()) flags1 |= H7_F1_UCC_C;
    if (ldo.telemetry_valid) flags1 |= H7_F1_G0_VALID;
    if (LdoLink_GetCtrlState() == LDO_G0_CTRL_FAULT) flags1 |= H7_F1_FAULT_LATCH;
    payload[H7_METER_FLAGS0] = flags0;
    payload[H7_METER_FLAGS1] = flags1;
    payload[H7_METER_G0_CTRL] = (uint8_t)LdoLink_GetCtrlState();
    payload[H7_METER_G0_MODE] = ldo.mode;
    payload[H7_METER_SET_PHASE] = LdoLink_HostSetPhase();
    payload[H7_METER_G0_STALE] =
        (!ldo.telemetry_valid || (age > H7_LINK_G0_STALE_MS)) ? 1U : 0U;
    (void)HostLink_QueueFrame(H7_LINK_METER, s_host_tx_seq++, payload,
                              H7_LINK_METER_BYTES, LINK_UART_PRI_FAST, 0U);
    if (LdoLink_TakeHostSetResult(&result_seq, &result_ack, &result_reason)) {
        if (result_ack != 0U) {
            HostLink_QueueAck(result_seq, H7_LINK_SET);
        } else {
            HostLink_QueueNack(result_seq, H7_LINK_SET, result_reason);
        }
    }
}

static void HostLink_QueueSlow(void)
{
    uint8_t bms_payload[H7_LINK_BMS_BYTES];
    uint8_t pd_payload[H7_LINK_PD_BYTES];
    BQ76922_Snapshot_t bms;
    PowerManager_Status_t pm;
    int16_t dV;
    uint8_t flags = 0U;
    float pd_v = 0.0f;
    float pd_a = 0.0f;

    memset(bms_payload, 0, sizeof(bms_payload));
    memset(pd_payload, 0, sizeof(pd_payload));
    BQ76922_GetSnapshot(&g_bq76922, &bms);
    PowerManager_GetStatus(&pm);
    (void)PSU_GuiGetPdContract(&pd_v, &pd_a, NULL, NULL);
    dV = (int16_t)(bms.max_cell_mv - bms.min_cell_mv);
    bms_payload[0] = (BQ76922_IsEnabled() && bms.present) ? 1U : 0U;
    bms_payload[1] = (BQ76922_IsEnabled() && bms.configured) ? 1U : 0U;
    bms_payload[2] = BQ76922_IsEnabled() ? (uint8_t)bms.state : 0U;
    bms_payload[3] = (bms.alert_latched || bms.alert_pin) ? 1U : 0U;
    H7Link_PutU32(&bms_payload[4], bms.fault_flags);
    H7Link_PutU16(&bms_payload[8], bms.alarm_status);
    bms_payload[10] = bms.safety_status_a;
    bms_payload[11] = bms.safety_status_b;
    bms_payload[12] = bms.safety_status_c;
    bms_payload[13] = bms.fet_status;
    H7Link_PutU16(&bms_payload[14], bms.manuf_status);
    bms_payload[16] = bms.init_step;
    bms_payload[17] = bms.cfg_fail_count;
    H7Link_PutU16(&bms_payload[18], bms.vcell_mode_rb);
    H7Link_PutU16(&bms_payload[20], bms.battery_status);
    bms_payload[22] = bms.chg_fet_on ? 1U : 0U;
    bms_payload[23] = bms.dsg_fet_on ? 1U : 0U;
    bms_payload[24] = bms.fets_enabled ? 1U : 0U;
    bms_payload[25] = (uint8_t)BMS_SERIES_COUNT;
    H7Link_PutU16(&bms_payload[26], (uint16_t)bms.cell_mv[0]);
    H7Link_PutU16(&bms_payload[28], (uint16_t)bms.cell_mv[1]);
    H7Link_PutU16(&bms_payload[30], (uint16_t)bms.cell_mv[2]);
    H7Link_PutU16(&bms_payload[32], (uint16_t)bms.cell_mv[3]);
    H7Link_PutU16(&bms_payload[34], (uint16_t)bms.cell_mv[4]);
    H7Link_PutU16(&bms_payload[36], (uint16_t)bms.min_cell_mv);
    H7Link_PutU16(&bms_payload[38], (uint16_t)bms.max_cell_mv);
    H7Link_PutU16(&bms_payload[40], (uint16_t)dV);
    H7Link_PutU16(&bms_payload[42], (uint16_t)bms.cell_sum_mv);
    H7Link_PutU16(&bms_payload[44], (uint16_t)bms.pack_mv);
    H7Link_PutU16(&bms_payload[46], (uint16_t)bms.stack_mv);
    H7Link_PutI32(&bms_payload[48], bms.cc2_ma);
    bms_payload[52] = bms.sample_valid ? 1U : 0U;
    H7Link_PutI32(&bms_payload[H7_BMS_PASSQ_MAH], bms.passq_mah);
    H7Link_PutI32(&bms_payload[H7_BMS_SESSION_MAH], bms.session_mah);
    H7Link_PutU16(&bms_payload[H7_BMS_SOC_PERMILLE], bms.soc_permille);
    H7Link_PutU16(&bms_payload[H7_BMS_CC1_MA], (uint16_t)bms.cc1_ma);
    H7Link_PutU16(&bms_payload[H7_BMS_INT_TEMP_DK], (uint16_t)bms.int_temp_dk);
    bms_payload[H7_BMS_BALANCE] = bms.balance_mask;
    bms_payload[H7_BMS_SOC_FLAGS] = bms.soc_flags;

    if (pm.bq.input_present) flags |= 0x01U;
    if (pm.bq.in_precharge) flags |= 0x02U;
    if (pm.bq.in_fast_charge) flags |= 0x04U;
    if (pm.bq.in_otg) flags |= 0x08U;
    if (pm.bq.in_iin_dpm) flags |= 0x10U;
    if (pm.bq.in_vindpm) flags |= 0x20U;
    if (pm.pd_reset_busy) flags |= 0x40U;
    if (pm.tps.attached) flags |= 0x80U;
    pd_payload[0] = (pm.bq.online && pm.bq.adc_sample_valid) ? 1U : 0U;
    pd_payload[1] = flags;
    H7Link_PutU16(&pd_payload[2], pm.bq.charger_status);
    pd_payload[4] = pm.bq.fault_flags;
    pd_payload[5] = pm.tps.cc1_state;
    pd_payload[6] = pm.tps.cc2_state;
    pd_payload[7] = pm.tps.role;
    pd_payload[8] = pm.tps.connection_state;
    pd_payload[9] = pm.tps.typec_port_state;
    pd_payload[10] = pm.pd_snapshot.power_role;
    H7Link_PutU32(&pd_payload[12], pm.bq.adc_vbat_mv);
    H7Link_PutU32(&pd_payload[16], pm.bq.adc_vsys_mv);
    H7Link_PutI32(&pd_payload[20], pm.bq.battery_current_ma);
    H7Link_PutU32(&pd_payload[24], pm.bq.adc_ichg_ma);
    H7Link_PutU32(&pd_payload[28], pm.bq.adc_idchg_ma);
    H7Link_PutU32(&pd_payload[32], pm.bq.adc_vbus_mv);
    H7Link_PutU32(&pd_payload[36], pm.bq.adc_iin_ma);
    H7Link_PutU32(&pd_payload[40], pm.bq.charge_voltage_mv);
    H7Link_PutU32(&pd_payload[44], pm.bq.charge_current_ma);
    H7Link_PutU32(&pd_payload[48], pm.bq.input_current_ma);
    H7Link_PutU32(&pd_payload[52], pm.tps.vbus_mv);
    H7Link_PutU32(&pd_payload[56], (uint32_t)HostLink_Mv(pd_v));
    H7Link_PutU32(&pd_payload[60], (uint32_t)HostLink_Ma(pd_a));
    (void)HostLink_QueueFrame(H7_LINK_BMS_TLM, s_host_tx_seq++, bms_payload,
                              H7_LINK_BMS_BYTES, LINK_UART_PRI_SLOW, 0U);
    (void)HostLink_QueueFrame(H7_LINK_PD_TLM, s_host_tx_seq++, pd_payload,
                              H7_LINK_PD_BYTES, LINK_UART_PRI_SLOW, 1U);
    {
        uint8_t aux_payload[H7_LINK_AUX_BYTES];
        LdoLink_Status_t ldo;
        uint8_t i;

        memset(aux_payload, 0, sizeof(aux_payload));
        LdoLink_GetStatus(&ldo);
        H7Link_PutU32(&aux_payload[H7_AUX_DAC_CV_MV], ldo.dac_cv_mv);
        H7Link_PutU32(&aux_payload[H7_AUX_DAC_CC_MV], ldo.dac_cc_mv);
        for (i = 0U; i < 4U; i++) {
            int16_t deci_c = ldo.telemetry_valid ? ldo.temp_centi_c[i]
                                                 : (int16_t)INT16_MIN;

            /* G0 already scaled to °C×10. Copy the int16; do not rescale. */
            H7Link_PutU16(&aux_payload[H7_AUX_T1_CC + (uint8_t)(2U * i)],
                          (uint16_t)deci_c);
        }
        aux_payload[H7_AUX_FAN] = ldo.fan_percent;
        aux_payload[H7_AUX_PGOOD] = ldo.pgood;
        aux_payload[H7_AUX_BLEED] = ldo.bleed_request;
        aux_payload[H7_AUX_VALID] = ldo.telemetry_valid ? 1U : 0U;
        H7Link_PutU16(&aux_payload[H7_AUX_LOCAL_MV], RemoteSense_LocalMv());
        H7Link_PutU16(&aux_payload[H7_AUX_REMOTE_P_MV], RemoteSense_RemotePMv());
        H7Link_PutU16(&aux_payload[H7_AUX_REMOTE_N_MV], RemoteSense_RemoteNMv());
        aux_payload[H7_AUX_SENSE_CODE] = RemoteSense_Code();
        aux_payload[H7_AUX_SENSE_FLAGS] =
            (uint8_t)((RemoteSense_IsClosed() ? 0x01U : 0U) |
                      (RemoteSense_IsWanted() ? 0x02U : 0U));
        H7Link_PutU16(&aux_payload[H7_AUX_FAN_RPM], FanTach_Rpm());
        (void)HostLink_QueueFrame(H7_LINK_AUX_TLM, s_host_tx_seq++, aux_payload,
                                  H7_LINK_AUX_BYTES, LINK_UART_PRI_SLOW, 2U);
    }
}

static void HostLink_FinishOn(bool ack, uint8_t reason)
{
    uint8_t i;

    if (!s_on_wait) {
        return;
    }
    if (ack) {
        HostLink_QueueAck(s_on_seq, H7_LINK_ON);
    } else {
        HostLink_QueueNack(s_on_seq, H7_LINK_ON, reason);
    }
    HostLink_Remember(H7_LINK_ON, s_on_seq, ack, reason);
    for (i = 0U; i < s_on_qn; i++) {
        if (ack) {
            HostLink_QueueAck(s_on_q[i], H7_LINK_ON);
        } else {
            HostLink_QueueNack(s_on_q[i], H7_LINK_ON, reason);
        }
        HostLink_Remember(H7_LINK_ON, s_on_q[i], ack, reason);
    }
    s_on_qn = 0U;
    s_on_wait = false;
}

static void HostLink_Command(uint8_t type, uint8_t seq, const uint8_t *payload,
                             uint8_t payload_len)
{
    int slot = HostLink_ReplayIndex(type);
    char text[97];

    if ((slot >= 0) && s_replay_valid[slot] && (s_replay_seq[slot] == seq)) {
        if (s_replay_ack[slot] != 0U) {
            HostLink_QueueAck(seq, type);
        } else {
            HostLink_QueueNack(seq, type, s_replay_reason[slot]);
        }
        return;
    }

    switch (type) {
    case H7_LINK_SET: {
        uint8_t reason = 0U;
        int result;

        if (payload_len != 8U) {
            HostLink_QueueNack(seq, type, H7_LINK_NACK_BAD_PAYLOAD);
            return;
        }
        result = LdoLink_SubmitHostSet(seq, H7Link_GetU32(&payload[0]),
                                       H7Link_GetU32(&payload[4]), &reason);
        if (result == LDO_HOST_SET_REPLAY_ACK) {
            HostLink_QueueAck(seq, type);
        } else if (result == LDO_HOST_SET_REPLAY_NACK) {
            HostLink_QueueNack(seq, type, reason);
        }
        break;
    }
    case H7_LINK_ON:
        if (payload_len != 0U) {
            HostLink_QueueNack(seq, type, H7_LINK_NACK_BAD_PAYLOAD);
            return;
        }
        if ((LdoLink_GetCtrlState() == LDO_G0_CTRL_RUNNING) && !s_on_wait) {
            HostLink_QueueAck(seq, type);
            HostLink_Remember(type, seq, true, 0U);
            return;
        }
        if (s_on_wait) {
            if (s_on_qn < 4U) {
                s_on_q[s_on_qn++] = seq;
            } else {
                HostLink_QueueNack(seq, type, H7_LINK_NACK_BUSY);
            }
            return;
        }
        HostLink_ApplyOn();
        s_on_wait = true;
        s_on_seq = seq;
        s_on_since_ms = HAL_GetTick();
        break;
    case H7_LINK_OFF:
        if (payload_len != 0U) {
            HostLink_QueueNack(seq, type, H7_LINK_NACK_BAD_PAYLOAD);
            return;
        }
        HostLink_ApplyOff();
        HostLink_QueueAck(seq, type);
        HostLink_Remember(type, seq, true, 0U);
        break;
    case H7_LINK_CLEAR:
        App_ClearFaults();
        HostLink_QueueAck(seq, type);
        HostLink_Remember(type, seq, true, 0U);
        break;
    case H7_LINK_PING:
        HostLink_QueueAck(seq, type);
        HostLink_Remember(type, seq, true, 0U);
        break;
    case H7_LINK_DIAG:
        HostLink_Tx("DIAG METER=5ms BMS=200ms baud=460800\r\n");
        break;
    case H7_LINK_PERMIT:
        if (payload_len != 1U) {
            HostLink_QueueNack(seq, type, H7_LINK_NACK_BAD_PAYLOAD);
            return;
        }
        if (payload[0] == 0U) {
            HostLink_ApplyOff();
        } else {
            LdoPrereg_SetPermitOverrideOff(false);
            LdoPrereg_SetForceDisable(false);
        }
        HostLink_QueueAck(seq, type);
        HostLink_Remember(type, seq, true, 0U);
        break;
    case H7_LINK_REMOTE:
        if ((payload_len != 1U) || (payload[0] > 1U)) {
            HostLink_QueueNack(seq, type, H7_LINK_NACK_BAD_PAYLOAD);
            return;
        }
        LdoLink_SetRemoteSense(payload[0] != 0U);
        HostLink_QueueAck(seq, type);
        HostLink_Remember(type, seq, true, 0U);
        break;
    case H7_LINK_BMS:
        if (payload_len != 1U) {
            HostLink_QueueNack(seq, type, H7_LINK_NACK_BAD_PAYLOAD);
            return;
        }
        if ((payload[0] == 0U) && BQ76922_IsConfiguredHealthy(&g_bq76922)) {
            BQ76922_ClearShutdownRequest(&g_bq76922);
            App_ClearFaults();
        } else {
            BQ76922_RequestReinit(&g_bq76922);
            BQ76922_ClearShutdownRequest(&g_bq76922);
            App_ClearFaults();
        }
        HostLink_QueueAck(seq, type);
        HostLink_Remember(type, seq, true, 0U);
        break;
    case H7_LINK_USB:
        if ((payload_len != 1U) || (payload[0] > 2U)) {
            HostLink_QueueNack(seq, type, H7_LINK_NACK_BAD_PAYLOAD);
            return;
        }
        if (payload[0] == 1U) {
            (void)PSU_GuiSetUsbMode(PSU_GUI_USB_MODE_SINK_ONLY);
        } else if (payload[0] == 2U) {
            (void)PSU_GuiSetUsbMode(PSU_GUI_USB_MODE_SOURCE_ONLY);
        } else {
            (void)PSU_GuiSetUsbMode(PSU_GUI_USB_MODE_AUTO);
        }
        HostLink_QueueAck(seq, type);
        HostLink_Remember(type, seq, true, 0U);
        break;
    case H7_LINK_TEXT_CMD:
        if ((payload_len == 0U) || (payload_len > 96U)) {
            HostLink_QueueNack(seq, type, H7_LINK_NACK_BAD_PAYLOAD);
            return;
        }
        memcpy(text, payload, payload_len);
        text[payload_len] = '\0';
        HostLink_HandleLine(text);
        HostLink_QueueAck(seq, type);
        break;
    default:
        HostLink_QueueNack(seq, type, H7_LINK_NACK_UNKNOWN);
        break;
    }
}

static void HostLink_OnByte(uint8_t byte, void *ctx)
{
    (void)ctx;
    if (H7Link_ParserByte(&s_host_parser, byte) != 0) {
        HostLink_Command(s_host_parser.type, s_host_parser.seq,
                         &s_host_parser.body[2], s_host_parser.payload_len);
    }
}

void HostLink_Init(UART_HandleTypeDef *huart)
{
    char line[96];
    int n;

    s_huart = huart;
    s_on_wait = false;
    s_on_qn = 0U;
    memset(s_replay_valid, 0, sizeof(s_replay_valid));
    H7Link_ParserInit(&s_host_parser);
    s_meter_ms = HAL_GetTick();
    s_slow_ms = s_meter_ms;
    LinkUart_Init(&s_host_uart, huart);
    HostLink_Tx("G4 host binary 460800. METER 5 ms. TEL ignored.\r\n");
    n = snprintf(line, sizeof(line),
                 "boot rcc_csr=0x%08lX\r\n",
                 (unsigned long)s_boot_reset_flags);
    if (n > 0) {
        HostLink_Tx(line);
    }
}

void HostLink_Task(void)
{
    uint32_t now_ms;
    LdoLink_CtrlState_t ctrl;

    if (s_huart == NULL) {
        return;
    }

    LinkUart_Poll(&s_host_uart, HostLink_OnByte, NULL);

    if (s_on_wait) {
        ctrl = LdoLink_GetCtrlState();
        if (ctrl == LDO_G0_CTRL_RUNNING) {
            HostLink_FinishOn(true, 0U);
        } else if (ctrl == LDO_G0_CTRL_FAULT) {
            HostLink_FinishOn(false, H7_LINK_NACK_UNSAFE);
        } else if ((uint32_t)(HAL_GetTick() - s_on_since_ms) >= H7_LINK_ON_TIMEOUT_MS) {
            HostLink_FinishOn(false, H7_LINK_NACK_TIMEOUT);
            LdoLink_RequestOutput(false);
            LdoPrereg_SetForceDisable(true);
            LdoPrereg_SetPermitOverrideOff(true);
            PSU_Stop();
        }
    }

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - s_meter_ms) >= H7_LINK_METER_PERIOD_MS) {
        s_meter_ms = now_ms;
        HostLink_QueueMeter();
    }
    if ((uint32_t)(now_ms - s_slow_ms) >= H7_LINK_SLOW_PERIOD_MS) {
        s_slow_ms = now_ms;
        HostLink_QueueSlow();
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

void HostLink_ForwardG0Line(const char *line)
{
    if (!HostLink_ShouldForwardG0Line(line, Debug_IsEnabled())) {
        return;
    }
    HostLink_ForwardLine(line);
}

void HostLink_OnUartError(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart == s_huart)) {
        LinkUart_OnError(huart);
    }
}

void HostLink_RxCplt(UART_HandleTypeDef *huart)
{
    (void)huart;
}
