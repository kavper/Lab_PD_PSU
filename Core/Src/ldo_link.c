#include "ldo_link.h"

#include "board_rev.h"
#include "debug_uart.h"
#include "fan_pwm.h"
#include "host_link.h"
#include "ldo_prereg.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LDO_RX_LINE_MAX              256U
#define LDO_TLM_STALE_MS             500U
#define LDO_FAN_FAILSAFE_PERCENT     40U
#define LDO_LINK_HEALTH_MS           2000U
#define LDO_RX_RECOVER_MS            1000U
#define LDO_CMD_TIMEOUT_MS           400U
#define LDO_CMD_RETRY_MAX            8U
#define LDO_VIN_MIN_MV               4500U
#define LDO_VOUT_ZERO_MV             250U
#define LDO_V_MIN                    0.0f
#define LDO_V_MAX                    27.0f
#define LDO_I_MIN                    0.0f
#define LDO_I_MAX                    5.0f
#define LDO_DEFAULT_V                5.0f
#define LDO_DEFAULT_I                0.1f

typedef enum {
    LDO_PENDING_NONE = 0,
    LDO_PENDING_SET,
    LDO_PENDING_OUT_ON,
    LDO_PENDING_OUT_OFF
} LdoPendingCmd_t;

static UART_HandleTypeDef *s_huart_g0 = NULL;
static uint8_t s_rx_byte;
static char s_rx_line[LDO_RX_LINE_MAX];
static volatile uint16_t s_rx_len;
static volatile bool s_line_ready;
static volatile bool s_rx_overflow;

static LdoLink_Status_t s_status;
static bool s_dcdc_permit_request;
static bool s_remote_sense;
static uint8_t s_applied_bleed;
static uint8_t s_applied_fan;
static bool s_applied_permit;
static uint32_t s_last_health_ms;
static uint32_t s_last_recover_ms;

static bool s_output_wanted;
static float s_g0_volts = LDO_DEFAULT_V;
static float s_g0_amps = LDO_DEFAULT_I;
static bool s_setpoint_dirty;
static LdoLink_CtrlState_t s_ctrl;
static LdoPendingCmd_t s_pending;
static uint32_t s_pending_since_ms;
static uint8_t s_retry_count;
static uint32_t s_state_since_ms;
static bool s_ack_set_ok;
static bool s_ack_out_on_ok;
static bool s_ack_out_off_ok;
static bool s_nack_seen;
static char s_nack_line[48];

static float LdoLink_Clampf(float value, float min_v, float max_v)
{
    if (value < min_v) {
        return min_v;
    }
    if (value > max_v) {
        return max_v;
    }
    return value;
}

static void LdoLink_ArmRx(void)
{
    HAL_StatusTypeDef st;

    if (s_huart_g0 == NULL) {
        return;
    }
    st = HAL_UART_Receive_IT(s_huart_g0, &s_rx_byte, 1U);
    if ((st != HAL_OK) && (st != HAL_BUSY)) {
        Debug_Printf("[LDO] ArmRx fail st=%d err=0x%lX state=%u\r\n",
                     (int)st,
                     (unsigned long)s_huart_g0->ErrorCode,
                     (unsigned int)s_huart_g0->RxState);
    }
}

static void LdoLink_FeedRxByte(uint8_t ch)
{
    s_status.rx_bytes++;
    s_status.last_rx_ms = HAL_GetTick();
    if (s_status.first_rx_len < sizeof(s_status.first_rx)) {
        s_status.first_rx[s_status.first_rx_len++] = ch;
    }
    if ((ch == (uint8_t)'\n') || (ch == (uint8_t)'\r')) {
        if (s_rx_len > 0U) {
            s_rx_line[s_rx_len] = '\0';
            s_line_ready = true;
            s_rx_len = 0U;
        }
    } else if (!s_line_ready) {
        if (s_rx_len < (LDO_RX_LINE_MAX - 1U)) {
            s_rx_line[s_rx_len++] = (char)ch;
        } else {
            s_rx_overflow = true;
            s_rx_len = 0U;
        }
    }
}

/* Polling fallback: if NVIC/IT path is dead, still assemble lines. */
static void LdoLink_PollRx(void)
{
    uint32_t guard = 0U;

    if (s_huart_g0 == NULL) {
        return;
    }

    while (__HAL_UART_GET_FLAG(s_huart_g0, UART_FLAG_RXNE) && (guard < 64U)) {
        uint8_t ch = (uint8_t)(s_huart_g0->Instance->RDR & 0xFFU);
        LdoLink_FeedRxByte(ch);
        guard++;
    }

    if (__HAL_UART_GET_FLAG(s_huart_g0, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(s_huart_g0, UART_FLAG_FE) ||
        __HAL_UART_GET_FLAG(s_huart_g0, UART_FLAG_NE) ||
        __HAL_UART_GET_FLAG(s_huart_g0, UART_FLAG_PE)) {
        s_status.rx_errors++;
        s_status.last_error_code |= s_huart_g0->ErrorCode;
        if (s_status.last_error_code == 0U) {
            s_status.last_error_code = 1U;
        }
        __HAL_UART_CLEAR_OREFLAG(s_huart_g0);
        __HAL_UART_CLEAR_NEFLAG(s_huart_g0);
        __HAL_UART_CLEAR_FEFLAG(s_huart_g0);
        __HAL_UART_CLEAR_PEFLAG(s_huart_g0);
    }
}

static void LdoLink_DumpFirstRx(void)
{
    char hex[64];
    size_t n = 0U;
    uint8_t i;

    if (s_status.first_rx_dumped || (s_status.first_rx_len == 0U)) {
        return;
    }

    for (i = 0U; (i < s_status.first_rx_len) && (n + 4U < sizeof(hex)); i++) {
        int wrote = snprintf(&hex[n], sizeof(hex) - n, "%02X ",
                             (unsigned int)s_status.first_rx[i]);
        if (wrote <= 0) {
            break;
        }
        n += (size_t)wrote;
    }
    if ((n > 0U) && (hex[n - 1U] == ' ')) {
        hex[n - 1U] = '\0';
    }
    Debug_Printf("[LDO] G0 first RX (%u B): %s\r\n",
                 (unsigned int)s_status.first_rx_len, hex);
    s_status.first_rx_dumped = true;
}

static void LdoLink_RecoverRx(uint32_t now_ms)
{
    if (s_huart_g0 == NULL) {
        return;
    }
    if ((uint32_t)(now_ms - s_last_recover_ms) < LDO_RX_RECOVER_MS) {
        return;
    }
    s_last_recover_ms = now_ms;

    (void)HAL_UART_AbortReceive(s_huart_g0);
    __HAL_UART_CLEAR_OREFLAG(s_huart_g0);
    __HAL_UART_CLEAR_NEFLAG(s_huart_g0);
    __HAL_UART_CLEAR_FEFLAG(s_huart_g0);
    __HAL_UART_CLEAR_PEFLAG(s_huart_g0);
    s_rx_len = 0U;
    s_line_ready = false;
    LdoLink_ArmRx();
}

static void LdoLink_SetBleed(bool on)
{
    HAL_GPIO_WritePin(BLEED_ON_GPIO_Port, BLEED_ON_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    s_applied_bleed = on ? 1U : 0U;
}

static void LdoLink_SetPermitPin(bool permit)
{
#if (BOARD_POWER_PERMIT_ACTIVE_HIGH != 0U)
    HAL_GPIO_WritePin(POWER_PERMIT_G4_GPIO_Port, POWER_PERMIT_G4_Pin,
                      permit ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(POWER_PERMIT_G4_GPIO_Port, POWER_PERMIT_G4_Pin,
                      permit ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
    s_applied_permit = permit;
}

static bool LdoLink_ParseU8Token(const char *line, const char *key, uint8_t *out)
{
    const char *p;
    char *end;
    unsigned long value;

    if ((line == NULL) || (key == NULL) || (out == NULL)) {
        return false;
    }

    p = strstr(line, key);
    if (p == NULL) {
        return false;
    }

    value = strtoul(p + strlen(key), &end, 10);
    if (end == (p + strlen(key))) {
        return false;
    }

    *out = (uint8_t)value;
    return true;
}

static bool LdoLink_ParseU32Token(const char *line, const char *key, uint32_t *out)
{
    const char *p;
    char *end;
    unsigned long value;

    if ((line == NULL) || (key == NULL) || (out == NULL)) {
        return false;
    }

    p = strstr(line, key);
    if (p == NULL) {
        return false;
    }

    value = strtoul(p + strlen(key), &end, 10);
    if (end == (p + strlen(key))) {
        return false;
    }

    *out = (uint32_t)value;
    return true;
}

static void LdoLink_ParseFaultToken(const char *line)
{
    const char *p;
    const char *end;
    size_t len;

    memset(s_status.fault, 0, sizeof(s_status.fault));
    p = strstr(line, "fault=");
    if (p == NULL) {
        return;
    }

    p += 6U;
    end = p;
    while ((*end != '\0') && (*end != ' ') && (*end != '\r') && (*end != '\n')) {
        end++;
    }

    len = (size_t)(end - p);
    if (len >= sizeof(s_status.fault)) {
        len = sizeof(s_status.fault) - 1U;
    }
    memcpy(s_status.fault, p, len);
}

static bool LdoLink_TxLine(const char *line)
{
    size_t len;

    if ((s_huart_g0 == NULL) || (line == NULL)) {
        return false;
    }

    len = strlen(line);
    if (len == 0U) {
        return false;
    }

    if (HAL_UART_Transmit(s_huart_g0, (uint8_t *)line, (uint16_t)len, 20U) != HAL_OK) {
        Debug_Printf("[LDO] TX fail: %s", line);
        return false;
    }

    Debug_Printf("[LDO] TX %s", line);
    return true;
}

static void LdoLink_ClearPendingAcks(void)
{
    s_ack_set_ok = false;
    s_ack_out_on_ok = false;
    s_ack_out_off_ok = false;
    s_nack_seen = false;
    s_nack_line[0] = '\0';
    s_pending = LDO_PENDING_NONE;
}

static void LdoLink_EnterState(LdoLink_CtrlState_t next, uint32_t now_ms)
{
    if (s_ctrl != next) {
        Debug_Printf("[LDO] ctrl %u -> %u\r\n",
                     (unsigned int)s_ctrl, (unsigned int)next);
        s_ctrl = next;
        s_state_since_ms = now_ms;
        s_status.ctrl_state = next;
    }
}

static void LdoLink_HardKillFromFault(const char *why)
{
    Debug_Printf("[LDO] HARD KILL PB7 (%s)\r\n", (why != NULL) ? why : "?");
    LdoLink_SetPermitPin(false);
    s_dcdc_permit_request = false;
    s_output_wanted = false;
    s_status.output_wanted = false;
    LdoPrereg_SetPermitOverrideOff(true);
    LdoPrereg_SetForceDisable(true);
    LdoLink_ClearPendingAcks();
    LdoLink_EnterState(LDO_G0_CTRL_FAULT, HAL_GetTick());
}

static void LdoLink_FormatSetCmd(char *cmd, size_t cmd_size)
{
    uint32_t v_mv;
    uint32_t i_ma;

    /* nano.specs has no %f — build SET with integer millivolts/milliamps. */
    v_mv = (uint32_t)((s_g0_volts * 1000.0f) + 0.5f);
    i_ma = (uint32_t)((s_g0_amps * 1000.0f) + 0.5f);
    if (v_mv > 27000U) {
        v_mv = 27000U;
    }
    if (i_ma > 5000U) {
        i_ma = 5000U;
    }

    (void)snprintf(cmd, cmd_size, "SET V=%lu.%03lu I=%lu.%03lu\r\n",
                   (unsigned long)(v_mv / 1000U),
                   (unsigned long)(v_mv % 1000U),
                   (unsigned long)(i_ma / 1000U),
                   (unsigned long)(i_ma % 1000U));
}

static bool LdoLink_SendSet(uint32_t now_ms)
{
    char cmd[48];

    LdoLink_FormatSetCmd(cmd, sizeof(cmd));

    LdoLink_ClearPendingAcks();
    if (!LdoLink_TxLine(cmd)) {
        return false;
    }

    s_pending = LDO_PENDING_SET;
    s_pending_since_ms = now_ms;
    s_setpoint_dirty = false;
    return true;
}

static bool LdoLink_SendOutOn(uint32_t now_ms)
{
    LdoLink_ClearPendingAcks();
    if (!LdoLink_TxLine("OUT ON\r\n")) {
        return false;
    }
    s_pending = LDO_PENDING_OUT_ON;
    s_pending_since_ms = now_ms;
    return true;
}

static bool LdoLink_SendOutOff(uint32_t now_ms)
{
    LdoLink_ClearPendingAcks();
    if (!LdoLink_TxLine("OUT OFF\r\n")) {
        return false;
    }
    s_pending = LDO_PENDING_OUT_OFF;
    s_pending_since_ms = now_ms;
    return true;
}

static void LdoLink_HandleAckLine(const char *line)
{
    /* USART1 is host+debug — forward once via host link only. */
    HostLink_ForwardLine(line);
    s_status.ack_ok_count++;

    if (strstr(line, "ACK SET") != NULL) {
        s_ack_set_ok = true;
        if (s_pending == LDO_PENDING_SET) {
            s_pending = LDO_PENDING_NONE;
        }
    } else if (strstr(line, "ACK OUT ON") != NULL) {
        s_ack_out_on_ok = true;
        if (s_pending == LDO_PENDING_OUT_ON) {
            s_pending = LDO_PENDING_NONE;
        }
    } else if (strstr(line, "ACK OUT OFF") != NULL) {
        s_ack_out_off_ok = true;
        if (s_pending == LDO_PENDING_OUT_OFF) {
            s_pending = LDO_PENDING_NONE;
        }
    }
}

static void LdoLink_HandleNackLine(const char *line)
{
    size_t len;

    HostLink_ForwardLine(line);

    s_status.nack_count++;
    s_nack_seen = true;

    len = strlen(line);
    if (len >= sizeof(s_nack_line)) {
        len = sizeof(s_nack_line) - 1U;
    }
    memcpy(s_nack_line, line, len);
    s_nack_line[len] = '\0';

    len = strlen(line);
    if (len >= sizeof(s_status.last_nack)) {
        len = sizeof(s_status.last_nack) - 1U;
    }
    memcpy(s_status.last_nack, line, len);
    s_status.last_nack[len] = '\0';

    if (strstr(line, "NACK FAULT=") != NULL) {
        LdoLink_HardKillFromFault(line);
        return;
    }

    if ((strstr(line, "REASON=POWER_KILL") != NULL) ||
        (strstr(line, "FAULT=POWER_KILL") != NULL)) {
        LdoPrereg_SetPermitOverrideOff(false);
        s_dcdc_permit_request = true;
        LdoLink_SetPermitPin(true);
    }
}

static void LdoLink_HandleTlmLine(const char *line)
{
    uint8_t value;

    if ((line == NULL) || (strncmp(line, "TLM ", 4U) != 0)) {
        return;
    }

    s_status.telemetry_valid = true;
    s_status.last_tlm_ms = HAL_GetTick();
    s_status.tlm_count++;

    if (LdoLink_ParseU8Token(line, "out=", &value)) {
        s_status.output_on = (value != 0U);
    }
    if (LdoLink_ParseU8Token(line, "mode=", &value)) {
        s_status.mode = value;
    }
    if (LdoLink_ParseU8Token(line, "bleed=", &value)) {
        s_status.bleed_request = (value != 0U) ? 1U : 0U;
    }
    if (LdoLink_ParseU8Token(line, "fan=", &value)) {
        s_status.fan_percent = value;
    }
    if (LdoLink_ParseU8Token(line, "pgood=", &value)) {
        s_status.pgood = (value != 0U) ? 1U : 0U;
    }
    if (LdoLink_ParseU8Token(line, "kill=", &value)) {
        s_status.kill_reported = (value != 0U) ? 1U : 0U;
    }
    if (LdoLink_ParseU8Token(line, "outoff=", &value)) {
        s_status.outoff_reported = (value != 0U) ? 1U : 0U;
    }
    if (LdoLink_ParseU8Token(line, "cccv=", &value)) {
        s_status.cc_cv = (value != 0U) ? 1U : 0U;
    }
    if (LdoLink_ParseU32Token(line, "vset=", &s_status.vset_mv)) {
        /* keep */
    }
    if (LdoLink_ParseU32Token(line, "vout=", &s_status.vout_mv)) {
        /* keep */
    }
    if (LdoLink_ParseU32Token(line, "vin=", &s_status.vin_mv)) {
        /* keep */
    }
    if (LdoLink_ParseU32Token(line, "iset=", &s_status.iset_ma)) {
        /* keep */
    }
    if (LdoLink_ParseU32Token(line, "iout=", &s_status.iout_ma)) {
        /* keep */
    }
    if (LdoLink_ParseU32Token(line, "vpre=", &s_status.vpre_mv)) {
        s_status.vpre_present = true;
    }
    LdoLink_ParseFaultToken(line);

    /* Forwarded G0 "TLM" lines flood the host UART and are unrelated to
     * USB-C/PD debugging; emit them only at VERBOSE. State parsing above and
     * the machine-readable "T" line (host_link) keep the G0 fields regardless. */
    if (Debug_GetLevel() >= DEBUG_LEVEL_VERBOSE) {
        HostLink_ForwardLine(line);
    }
}

static void LdoLink_HandleRxLine(const char *line)
{
    s_status.rx_lines++;

    if (strncmp(line, "TLM ", 4U) == 0) {
        LdoLink_HandleTlmLine(line);
    } else if (strncmp(line, "ACK ", 4U) == 0) {
        LdoLink_HandleAckLine(line);
    } else if (strncmp(line, "NACK ", 5U) == 0) {
        LdoLink_HandleNackLine(line);
    } else {
        Debug_Printf("[LDO] G0 ignore: %s\r\n", line);
    }
}

static void LdoLink_ApplyActuators(uint32_t now_ms)
{
    uint8_t fan = s_status.fan_percent;
    bool permit;

    if (!s_status.telemetry_valid ||
        ((uint32_t)(now_ms - s_status.last_tlm_ms) > LDO_TLM_STALE_MS)) {
        fan = LDO_FAN_FAILSAFE_PERCENT;
    }

    if (fan > 100U) {
        fan = 100U;
    }

    LdoLink_SetBleed(s_status.bleed_request != 0U);
    FanPwm_SetPercent(fan);
    s_applied_fan = fan;

    permit = s_dcdc_permit_request;
    if (s_applied_permit != permit) {
        LdoLink_SetPermitPin(permit);
    }
}

static bool LdoLink_TlmFresh(uint32_t now_ms)
{
    return s_status.telemetry_valid &&
           ((uint32_t)(now_ms - s_status.last_tlm_ms) <= LDO_TLM_STALE_MS);
}

static bool LdoLink_PendingTimedOut(uint32_t now_ms)
{
    return (s_pending != LDO_PENDING_NONE) &&
           ((uint32_t)(now_ms - s_pending_since_ms) >= LDO_CMD_TIMEOUT_MS);
}

static void LdoLink_CtrlTask(uint32_t now_ms)
{
    s_status.output_wanted = s_output_wanted;
    s_status.ctrl_state = s_ctrl;

    if (!s_output_wanted) {
        if ((s_ctrl != LDO_G0_CTRL_IDLE) &&
            (s_ctrl != LDO_G0_CTRL_SEND_OUT_OFF) &&
            (s_ctrl != LDO_G0_CTRL_WAIT_OUT_OFF_ACK) &&
            (s_ctrl != LDO_G0_CTRL_FAULT)) {
            s_retry_count = 0U;
            LdoLink_EnterState(LDO_G0_CTRL_SEND_OUT_OFF, now_ms);
        }
    }

    switch (s_ctrl) {
    case LDO_G0_CTRL_IDLE:
        if (s_output_wanted) {
            s_retry_count = 0U;
            LdoLink_EnterState(LDO_G0_CTRL_WAIT_LINK, now_ms);
        }
        break;

    case LDO_G0_CTRL_WAIT_LINK:
        if (!s_output_wanted) {
            LdoLink_EnterState(LDO_G0_CTRL_IDLE, now_ms);
            break;
        }
        if (LdoLink_TlmFresh(now_ms)) {
            s_retry_count = 0U;
            LdoLink_EnterState(LDO_G0_CTRL_WAIT_PERMIT, now_ms);
        }
        break;

    case LDO_G0_CTRL_WAIT_PERMIT:
        if (!s_output_wanted) {
            break;
        }
        /* Ensure opto permit while waiting for kill=0. */
        LdoPrereg_SetPermitOverrideOff(false);
        s_dcdc_permit_request = true;
        if (!s_applied_permit) {
            LdoLink_SetPermitPin(true);
        }

        if (LdoLink_TlmFresh(now_ms) &&
            (s_status.kill_reported == 0U) &&
            LdoLink_IsPowerPermitted()) {
            s_retry_count = 0U;
            LdoLink_EnterState(LDO_G0_CTRL_WAIT_VIN, now_ms);
        } else if ((uint32_t)(now_ms - s_state_since_ms) > 2000U) {
            Debug_Printf("[LDO] wait permit: kill=%u pb6=%u pgood=%u\r\n",
                         (unsigned int)s_status.kill_reported,
                         LdoLink_IsPowerPermitted() ? 1U : 0U,
                         (unsigned int)s_status.pgood);
            s_state_since_ms = now_ms;
        }
        break;

    case LDO_G0_CTRL_WAIT_VIN:
        if (!s_output_wanted) {
            break;
        }
        if (LdoLink_TlmFresh(now_ms) &&
            (s_status.pgood != 0U) &&
            (s_status.vin_mv >= LDO_VIN_MIN_MV) &&
            (s_status.kill_reported == 0U)) {
            s_retry_count = 0U;
            LdoLink_EnterState(LDO_G0_CTRL_SEND_SET, now_ms);
        } else if ((uint32_t)(now_ms - s_state_since_ms) > 2000U) {
            Debug_Printf("[LDO] wait VIN: pgood=%u vin=%lu kill=%u\r\n",
                         (unsigned int)s_status.pgood,
                         (unsigned long)s_status.vin_mv,
                         (unsigned int)s_status.kill_reported);
            s_state_since_ms = now_ms;
            if (s_status.kill_reported != 0U) {
                LdoLink_EnterState(LDO_G0_CTRL_WAIT_PERMIT, now_ms);
            }
        }
        break;

    case LDO_G0_CTRL_SEND_SET:
        if (!s_output_wanted) {
            break;
        }
        if (LdoLink_SendSet(now_ms)) {
            LdoLink_EnterState(LDO_G0_CTRL_WAIT_SET_ACK, now_ms);
        } else if (++s_retry_count >= LDO_CMD_RETRY_MAX) {
            LdoLink_EnterState(LDO_G0_CTRL_FAULT, now_ms);
        }
        break;

    case LDO_G0_CTRL_WAIT_SET_ACK:
        if (s_ack_set_ok) {
            s_retry_count = 0U;
            if (s_status.vout_mv > LDO_VOUT_ZERO_MV) {
                LdoLink_EnterState(LDO_G0_CTRL_WAIT_VOUT_ZERO, now_ms);
            } else {
                LdoLink_EnterState(LDO_G0_CTRL_SEND_OUT_ON, now_ms);
            }
        } else if (s_nack_seen) {
            s_nack_seen = false;
            if (++s_retry_count >= LDO_CMD_RETRY_MAX) {
                LdoLink_EnterState(LDO_G0_CTRL_FAULT, now_ms);
            } else {
                LdoLink_EnterState(LDO_G0_CTRL_SEND_SET, now_ms);
            }
        } else if (LdoLink_PendingTimedOut(now_ms)) {
            s_status.cmd_timeout_count++;
            if (++s_retry_count >= LDO_CMD_RETRY_MAX) {
                LdoLink_EnterState(LDO_G0_CTRL_FAULT, now_ms);
            } else {
                LdoLink_EnterState(LDO_G0_CTRL_SEND_SET, now_ms);
            }
        }
        break;

    case LDO_G0_CTRL_WAIT_VOUT_ZERO:
        if (!s_output_wanted) {
            break;
        }
        if (!LdoLink_TlmFresh(now_ms)) {
            break;
        }
        if (s_status.vout_mv <= LDO_VOUT_ZERO_MV) {
            s_retry_count = 0U;
            LdoLink_EnterState(LDO_G0_CTRL_SEND_OUT_ON, now_ms);
        } else if ((uint32_t)(now_ms - s_state_since_ms) > 3000U) {
            /* Ask G0 to discharge, then retry zero wait. */
            (void)LdoLink_SendOutOff(now_ms);
            s_state_since_ms = now_ms;
        }
        break;

    case LDO_G0_CTRL_SEND_OUT_ON:
        if (!s_output_wanted) {
            break;
        }
        if (s_status.kill_reported != 0U) {
            LdoLink_EnterState(LDO_G0_CTRL_WAIT_PERMIT, now_ms);
            break;
        }
        if (LdoLink_SendOutOn(now_ms)) {
            LdoLink_EnterState(LDO_G0_CTRL_WAIT_OUT_ON_ACK, now_ms);
        } else if (++s_retry_count >= LDO_CMD_RETRY_MAX) {
            LdoLink_EnterState(LDO_G0_CTRL_FAULT, now_ms);
        }
        break;

    case LDO_G0_CTRL_WAIT_OUT_ON_ACK:
        if (s_ack_out_on_ok ||
            (LdoLink_TlmFresh(now_ms) && s_status.output_on &&
             (s_status.kill_reported == 0U))) {
            s_retry_count = 0U;
            LdoLink_EnterState(LDO_G0_CTRL_RUNNING, now_ms);
            Debug_Printf("[LDO] G0 OUT ON ok (vset=%lu mV iset=%lu mA)\r\n",
                         (unsigned long)((s_g0_volts * 1000.0f) + 0.5f),
                         (unsigned long)((s_g0_amps * 1000.0f) + 0.5f));
        } else if (s_nack_seen) {
            s_nack_seen = false;
            if (strstr(s_nack_line, "POWER_KILL") != NULL) {
                LdoLink_EnterState(LDO_G0_CTRL_WAIT_PERMIT, now_ms);
            } else if (strstr(s_nack_line, "VIN_LOW") != NULL) {
                LdoLink_EnterState(LDO_G0_CTRL_WAIT_VIN, now_ms);
            } else if (strstr(s_nack_line, "VOUT_NOT_ZERO") != NULL) {
                LdoLink_EnterState(LDO_G0_CTRL_WAIT_VOUT_ZERO, now_ms);
            } else if (++s_retry_count >= LDO_CMD_RETRY_MAX) {
                LdoLink_EnterState(LDO_G0_CTRL_FAULT, now_ms);
            } else {
                LdoLink_EnterState(LDO_G0_CTRL_SEND_OUT_ON, now_ms);
            }
        } else if (LdoLink_PendingTimedOut(now_ms)) {
            s_status.cmd_timeout_count++;
            if (++s_retry_count >= LDO_CMD_RETRY_MAX) {
                LdoLink_EnterState(LDO_G0_CTRL_FAULT, now_ms);
            } else {
                LdoLink_EnterState(LDO_G0_CTRL_SEND_OUT_ON, now_ms);
            }
        }
        break;

    case LDO_G0_CTRL_RUNNING:
        if (!s_output_wanted) {
            break;
        }
        if (LdoLink_TlmFresh(now_ms) && (s_status.kill_reported != 0U)) {
            Debug_Printf("[LDO] kill asserted while running — hard kill\r\n");
            LdoLink_HardKillFromFault("TLM kill=1");
            break;
        }
        if (s_setpoint_dirty) {
            s_retry_count = 0U;
            LdoLink_EnterState(LDO_G0_CTRL_SEND_SET, now_ms);
        }
        break;

    case LDO_G0_CTRL_SEND_OUT_OFF:
        if (LdoLink_SendOutOff(now_ms)) {
            LdoLink_EnterState(LDO_G0_CTRL_WAIT_OUT_OFF_ACK, now_ms);
        } else {
            LdoLink_EnterState(LDO_G0_CTRL_IDLE, now_ms);
        }
        break;

    case LDO_G0_CTRL_WAIT_OUT_OFF_ACK:
        if (s_ack_out_off_ok ||
            (LdoLink_TlmFresh(now_ms) && (!s_status.output_on)) ||
            LdoLink_PendingTimedOut(now_ms)) {
            LdoLink_ClearPendingAcks();
            LdoLink_EnterState(LDO_G0_CTRL_IDLE, now_ms);
        }
        break;

    case LDO_G0_CTRL_FAULT:
        if (s_output_wanted) {
            /* Host cleared force-disable / re-requested ON. */
            if (!LdoPrereg_IsPermitGranted() && s_dcdc_permit_request) {
                /* wait */
            }
            s_retry_count = 0U;
            LdoLink_EnterState(LDO_G0_CTRL_WAIT_LINK, now_ms);
        }
        break;

    default:
        LdoLink_EnterState(LDO_G0_CTRL_IDLE, now_ms);
        break;
    }
}

void LdoLink_Init(UART_HandleTypeDef *huart_g0)
{
    memset(&s_status, 0, sizeof(s_status));
    strncpy(s_status.fault, "NONE", sizeof(s_status.fault) - 1U);

    s_huart_g0 = huart_g0;
    s_rx_len = 0U;
    s_line_ready = false;
    s_rx_overflow = false;
    s_dcdc_permit_request = false;
    s_remote_sense = false;
    s_last_health_ms = 0U;
    s_last_recover_ms = 0U;
    s_output_wanted = false;
    s_g0_volts = LDO_DEFAULT_V;
    s_g0_amps = LDO_DEFAULT_I;
    s_setpoint_dirty = false;
    s_ctrl = LDO_G0_CTRL_IDLE;
    s_retry_count = 0U;
    s_state_since_ms = HAL_GetTick();
    LdoLink_ClearPendingAcks();

    FanPwm_Init();
    LdoLink_SetBleed(false);
#if (BOARD_BRINGUP_PERMIT_EARLY != 0U)
    LdoLink_SetPermitPin(true);
    s_dcdc_permit_request = true;
#else
    LdoLink_SetPermitPin(false);
#endif
    HAL_GPIO_WritePin(REMOTE_ON_GPIO_Port, REMOTE_ON_Pin, GPIO_PIN_RESET);

    LdoLink_ArmRx();
    Debug_Printf("[LDO] USART2 G0 link ready (TLM forward + SET/OUT control)\r\n");
#if (BOARD_USART2_G0_PIN_SWAP != 0U)
    Debug_Printf("[LDO] USART2 TX/RX SWAP=1 at boot (G0SWAP 0|1 to change)\r\n");
#else
    Debug_Printf("[LDO] USART2 TX/RX SWAP=0 at boot (G0SWAP 0|1 to change)\r\n");
#endif
    Debug_Printf("[LDO] Sense: LOCAL; default G0 SET V=%lu.%03lu I=%lu.%03lu\r\n",
                 (unsigned long)((uint32_t)((s_g0_volts * 1000.0f) + 0.5f) / 1000U),
                 (unsigned long)((uint32_t)((s_g0_volts * 1000.0f) + 0.5f) % 1000U),
                 (unsigned long)((uint32_t)((s_g0_amps * 1000.0f) + 0.5f) / 1000U),
                 (unsigned long)((uint32_t)((s_g0_amps * 1000.0f) + 0.5f) % 1000U));
#if (BOARD_BRINGUP_PERMIT_EARLY != 0U)
    Debug_Printf("[LDO] Bring-up: POWER_PERMIT early HIGH/ena (clear G0 POWER_KILL)\r\n");
#endif
    LdoLink_DumpDiag();
}

void LdoLink_SetDcdcPermitRequest(bool permit)
{
    s_dcdc_permit_request = permit;
}

bool LdoLink_IsDcdcPermitRequested(void)
{
    return s_dcdc_permit_request;
}

void LdoLink_RequestOutput(bool on)
{
    s_output_wanted = on;
    s_status.output_wanted = on;
    if (on) {
        LdoPrereg_SetForceDisable(false);
        LdoPrereg_SetPermitOverrideOff(false);
        if (s_ctrl == LDO_G0_CTRL_FAULT) {
            LdoLink_EnterState(LDO_G0_CTRL_WAIT_LINK, HAL_GetTick());
        }
        Debug_Printf("[LDO] output WANT ON (V=%lu mV I=%lu mA)\r\n",
                     (unsigned long)((s_g0_volts * 1000.0f) + 0.5f),
                     (unsigned long)((s_g0_amps * 1000.0f) + 0.5f));
    } else {
        Debug_Printf("[LDO] output WANT OFF\r\n");
    }
}

bool LdoLink_IsOutputWanted(void)
{
    return s_output_wanted;
}

void LdoLink_SetG0Setpoint(float volts, float amps)
{
    LdoLink_SetG0Voltage(volts);
    LdoLink_SetG0Current(amps);
}

void LdoLink_SetG0Voltage(float volts)
{
    float v = LdoLink_Clampf(volts, LDO_V_MIN, LDO_V_MAX);
    if (v != s_g0_volts) {
        s_g0_volts = v;
        s_setpoint_dirty = true;
    }
}

void LdoLink_SetG0Current(float amps)
{
    float a = LdoLink_Clampf(amps, LDO_I_MIN, LDO_I_MAX);
    if (a != s_g0_amps) {
        s_g0_amps = a;
        s_setpoint_dirty = true;
    }
}

float LdoLink_GetG0Voltage(void)
{
    return s_g0_volts;
}

float LdoLink_GetG0Current(void)
{
    return s_g0_amps;
}

LdoLink_CtrlState_t LdoLink_GetCtrlState(void)
{
    return s_ctrl;
}

void LdoLink_DumpDiag(void)
{
    uint32_t isr = 0U;
    uint32_t cr1 = 0U;
    uint32_t cr2 = 0U;
    uint32_t swap = 0U;

    if (s_huart_g0 == NULL) {
        Debug_Printf("[LDO] DIAG no huart\r\n");
        return;
    }

    isr = s_huart_g0->Instance->ISR;
    cr1 = s_huart_g0->Instance->CR1;
    cr2 = s_huart_g0->Instance->CR2;
    swap = (cr2 & USART_CR2_SWAP) ? 1U : 0U;

    Debug_Printf("[LDO] DIAG swap=%lu RxSt=%u err=0x%lX ISR=0x%08lX CR1=0x%08lX "
                 "PB3=%u PB4=%u rx=%lu tlm=%lu RE=%u UE=%u RXNE=%u ORE=%u FE=%u\r\n",
                 (unsigned long)swap,
                 (unsigned int)s_huart_g0->RxState,
                 (unsigned long)s_status.last_error_code,
                 (unsigned long)isr,
                 (unsigned long)cr1,
                 (GPIOB->IDR & GPIO_PIN_3) ? 1U : 0U,
                 (GPIOB->IDR & GPIO_PIN_4) ? 1U : 0U,
                 (unsigned long)s_status.rx_bytes,
                 (unsigned long)s_status.tlm_count,
                 (cr1 & USART_CR1_RE) ? 1U : 0U,
                 (cr1 & USART_CR1_UE) ? 1U : 0U,
                 (isr & USART_ISR_RXNE_RXFNE) ? 1U : 0U,
                 (isr & USART_ISR_ORE) ? 1U : 0U,
                 (isr & USART_ISR_FE) ? 1U : 0U);
}

void LdoLink_SetUartPinSwap(bool enable)
{
    if (s_huart_g0 == NULL) {
        return;
    }

    CLEAR_BIT(s_huart_g0->Instance->CR1, USART_CR1_UE);
    if (enable) {
        SET_BIT(s_huart_g0->Instance->CR2, USART_CR2_SWAP);
    } else {
        CLEAR_BIT(s_huart_g0->Instance->CR2, USART_CR2_SWAP);
    }
    SET_BIT(s_huart_g0->Instance->CR1, USART_CR1_UE);

    s_status.first_rx_len = 0U;
    s_status.first_rx_dumped = false;
    s_rx_len = 0U;
    s_line_ready = false;
    (void)HAL_UART_AbortReceive(s_huart_g0);
    LdoLink_ArmRx();

    Debug_Printf("[LDO] USART2 SWAP now %u\r\n", enable ? 1U : 0U);
}

bool LdoLink_IsUartPinSwapEnabled(void)
{
    if (s_huart_g0 == NULL) {
        return false;
    }
    return (s_huart_g0->Instance->CR2 & USART_CR2_SWAP) != 0U;
}

void LdoLink_SetRemoteSense(bool enable)
{
    s_remote_sense = enable;
    HAL_GPIO_WritePin(REMOTE_ON_GPIO_Port, REMOTE_ON_Pin,
                       enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    Debug_Printf("[LDO] Sense: %s (REMOTE_ON=%u)\r\n",
                 enable ? "REMOTE" : "LOCAL",
                 enable ? 1U : 0U);
}

bool LdoLink_IsRemoteSenseEnabled(void)
{
    return s_remote_sense;
}

void LdoLink_GetStatus(LdoLink_Status_t *out)
{
    if (out != NULL) {
        *out = s_status;
        out->output_wanted = s_output_wanted;
        out->ctrl_state = s_ctrl;
    }
}

void LdoLink_Task(void)
{
    char line[LDO_RX_LINE_MAX];
    uint32_t now_ms = HAL_GetTick();
    uint32_t age_ms;

    /* Always poll RXNE — recovers if IT/NVIC path is stuck. */
    LdoLink_PollRx();

    if (s_line_ready) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        memcpy(line, s_rx_line, sizeof(line));
        s_line_ready = false;
        s_rx_len = 0U;
        if (primask == 0U) {
            __enable_irq();
        }
        LdoLink_DumpFirstRx();
        LdoLink_HandleRxLine(line);
    } else if (s_rx_overflow) {
        s_rx_overflow = false;
        Debug_Printf("[LDO] WARN: G0 RX line overflow\r\n");
    }

    if (s_status.last_rx_ms != 0U) {
        age_ms = now_ms - s_status.last_rx_ms;
    } else {
        age_ms = 0xFFFFFFFFu;
    }

    if (!s_status.telemetry_valid ||
        ((s_status.last_tlm_ms != 0U) &&
         ((uint32_t)(now_ms - s_status.last_tlm_ms) > LDO_TLM_STALE_MS))) {
        if ((age_ms > LDO_RX_RECOVER_MS) || (s_status.rx_errors > 0U)) {
            LdoLink_RecoverRx(now_ms);
        }
        if ((uint32_t)(now_ms - s_last_health_ms) >= LDO_LINK_HEALTH_MS) {
            s_last_health_ms = now_ms;
            LdoLink_DumpFirstRx();
            Debug_Printf("[LDO] G0 link weak: rx=%lu lines=%lu tlm=%lu err=%lu "
                         "age_ms=%lu ctrl=%u want=%u swap=%u\r\n",
                         (unsigned long)s_status.rx_bytes,
                         (unsigned long)s_status.rx_lines,
                         (unsigned long)s_status.tlm_count,
                         (unsigned long)s_status.rx_errors,
                         (unsigned long)age_ms,
                         (unsigned int)s_ctrl,
                         s_output_wanted ? 1U : 0U,
                         LdoLink_IsUartPinSwapEnabled() ? 1U : 0U);
            LdoLink_DumpDiag();
        }
    }

    LdoLink_CtrlTask(now_ms);
    LdoLink_ApplyActuators(now_ms);
}

void LdoLink_RxCplt(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart != s_huart_g0)) {
        return;
    }

    LdoLink_FeedRxByte(s_rx_byte);
    LdoLink_ArmRx();
}

void LdoLink_OnUartError(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart == s_huart_g0)) {
        s_status.rx_errors++;
        s_status.last_error_code |= huart->ErrorCode;
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        (void)HAL_UART_AbortReceive(huart);
        LdoLink_ArmRx();
    }
}

bool LdoLink_IsPowerPermitted(void)
{
#if (BOARD_POWER_PERMIT_ACTIVE_HIGH != 0U)
    return HAL_GPIO_ReadPin(POWER_PERMIT_G4_GPIO_Port, POWER_PERMIT_G4_Pin) ==
           GPIO_PIN_SET;
#else
    return HAL_GPIO_ReadPin(POWER_PERMIT_G4_GPIO_Port, POWER_PERMIT_G4_Pin) ==
           GPIO_PIN_RESET;
#endif
}
