#include "ldo_link.h"

#include "board_rev.h"
#include "debug_uart.h"
#include "fan_pwm.h"
#include "ldo_ctrl_policy.h"
#include "ldo_prereg.h"

#include <stdio.h>
#include <string.h>

#define LDO_PROTO_SOF1               0xA5U
#define LDO_PROTO_SOF2               0x5AU
#define LDO_PROTO_MIN_LENGTH         2U
#define LDO_PROTO_MAX_PAYLOAD        80U
#define LDO_PROTO_MAX_LENGTH         (LDO_PROTO_MIN_LENGTH + LDO_PROTO_MAX_PAYLOAD)
#define LDO_PROTO_MAX_FRAME          (2U + 1U + LDO_PROTO_MAX_LENGTH + 2U)
#define LDO_PROTO_SET_OUTPUT         0x03U
#define LDO_PROTO_PING               0x04U
#define LDO_PROTO_SETPOINT           0x05U
#define LDO_PROTO_TELEMETRY          0x80U
#define LDO_PROTO_ACK                0x81U
#define LDO_PROTO_NACK               0x82U
#define LDO_TLM_PAYLOAD_LENGTH       68U
#define LDO_FAULT_VIN_LOW            (1UL << 3)
#define LDO_RX_RING_SIZE             256U
#define LDO_TLM_STALE_MS             500U
#define LDO_FAN_FAILSAFE_PERCENT     40U
#define LDO_LINK_HEALTH_MS           2000U
#define LDO_RX_RECOVER_MS            1000U
#define LDO_CMD_TIMEOUT_MS           150U
#define LDO_CMD_RETRY_MAX            4U
#define LDO_VIN_MIN_MV               4500U
#define LDO_VOUT_ZERO_MV             250U
#define LDO_LIVE_SET_MIN_MS          80U
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

typedef enum {
    LDO_RX_WAIT_SOF1 = 0,
    LDO_RX_WAIT_SOF2,
    LDO_RX_WAIT_LENGTH,
    LDO_RX_WAIT_BODY,
    LDO_RX_WAIT_CRC_LOW,
    LDO_RX_WAIT_CRC_HIGH
} LdoRxState_t;

static UART_HandleTypeDef *s_huart_g0 = NULL;
static uint8_t s_rx_byte;
static uint8_t s_rx_ring[LDO_RX_RING_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile bool s_rx_overflow;
static LdoRxState_t s_rx_state;
static uint8_t s_rx_length;
static uint8_t s_rx_body[LDO_PROTO_MAX_LENGTH];
static uint8_t s_rx_body_index;
static uint16_t s_rx_crc;
static uint16_t s_rx_received_crc;

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
static uint32_t s_last_live_set_ms;
static LdoLink_CtrlState_t s_ctrl;
static LdoPendingCmd_t s_pending;
static uint8_t s_pending_type;
static uint8_t s_pending_seq;
static uint8_t s_tx_sequence;
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
    uint16_t next_head;

    s_status.rx_bytes++;
    s_status.last_rx_ms = HAL_GetTick();
    if (s_status.first_rx_len < sizeof(s_status.first_rx)) {
        s_status.first_rx[s_status.first_rx_len++] = ch;
    }

    next_head = (uint16_t)((s_rx_head + 1U) % LDO_RX_RING_SIZE);
    if (next_head != s_rx_tail) {
        s_rx_ring[s_rx_head] = ch;
        s_rx_head = next_head;
    } else {
        s_rx_overflow = true;
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
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_state = LDO_RX_WAIT_SOF1;
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

static uint16_t LdoLink_Crc16Update(uint16_t crc, uint8_t byte)
{
    uint8_t bit;

    crc ^= (uint16_t)byte << 8;
    for (bit = 0U; bit < 8U; bit++) {
        crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                              : (uint16_t)(crc << 1);
    }
    return crc;
}

static uint32_t LdoLink_GetU32Le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void LdoLink_PutU32Le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static bool LdoLink_TxFrame(uint8_t type, uint8_t sequence,
                            const uint8_t *payload, uint8_t payload_length)
{
    uint8_t frame[LDO_PROTO_MAX_FRAME];
    uint8_t length;
    uint16_t index = 0U;
    uint16_t crc = 0xFFFFU;
    uint8_t i;

    if ((s_huart_g0 == NULL) || (payload_length > LDO_PROTO_MAX_PAYLOAD)) {
        return false;
    }

    length = (uint8_t)(payload_length + LDO_PROTO_MIN_LENGTH);
    frame[index++] = LDO_PROTO_SOF1;
    frame[index++] = LDO_PROTO_SOF2;
    frame[index++] = length;
    frame[index++] = type;
    frame[index++] = sequence;
    crc = LdoLink_Crc16Update(crc, length);
    crc = LdoLink_Crc16Update(crc, type);
    crc = LdoLink_Crc16Update(crc, sequence);
    for (i = 0U; i < payload_length; i++) {
        frame[index++] = payload[i];
        crc = LdoLink_Crc16Update(crc, payload[i]);
    }
    frame[index++] = (uint8_t)crc;
    frame[index++] = (uint8_t)(crc >> 8);

    if (HAL_UART_Transmit(s_huart_g0, frame, index, 20U) != HAL_OK) {
        Debug_Printf("[LDO] binary TX fail type=0x%02X seq=%u\r\n",
                     (unsigned int)type, (unsigned int)sequence);
        return false;
    }
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

static bool LdoLink_SendSet(uint32_t now_ms)
{
    uint8_t payload[8];
    uint32_t v_mv = (uint32_t)((s_g0_volts * 1000.0f) + 0.5f);
    uint32_t i_ma = (uint32_t)((s_g0_amps * 1000.0f) + 0.5f);

    LdoLink_ClearPendingAcks();
    s_pending_seq = s_tx_sequence++;
    s_pending_type = LDO_PROTO_SETPOINT;
    LdoLink_PutU32Le(&payload[0], v_mv);
    LdoLink_PutU32Le(&payload[4], i_ma);
    if (!LdoLink_TxFrame(s_pending_type, s_pending_seq, payload, sizeof(payload))) {
        return false;
    }

    s_pending = LDO_PENDING_SET;
    s_pending_since_ms = now_ms;
    s_setpoint_dirty = false;
    return true;
}

static bool LdoLink_SendOutOn(uint32_t now_ms)
{
    const uint8_t enabled = 1U;

    LdoLink_ClearPendingAcks();
    s_pending_seq = s_tx_sequence++;
    s_pending_type = LDO_PROTO_SET_OUTPUT;
    if (!LdoLink_TxFrame(s_pending_type, s_pending_seq, &enabled, 1U)) {
        return false;
    }
    s_pending = LDO_PENDING_OUT_ON;
    s_pending_since_ms = now_ms;
    return true;
}

static bool LdoLink_SendOutOff(uint32_t now_ms)
{
    const uint8_t enabled = 0U;

    LdoLink_ClearPendingAcks();
    s_pending_seq = s_tx_sequence++;
    s_pending_type = LDO_PROTO_SET_OUTPUT;
    if (!LdoLink_TxFrame(s_pending_type, s_pending_seq, &enabled, 1U)) {
        return false;
    }
    s_pending = LDO_PENDING_OUT_OFF;
    s_pending_since_ms = now_ms;
    return true;
}

static void LdoLink_HandleAck(uint8_t sequence, const uint8_t *payload,
                              uint8_t payload_length)
{
    if ((payload_length != 1U) || (s_pending == LDO_PENDING_NONE) ||
        (sequence != s_pending_seq) || (payload[0] != s_pending_type)) {
        return;
    }

    s_status.ack_ok_count++;
    if ((s_pending == LDO_PENDING_SET) && (payload[0] == LDO_PROTO_SETPOINT)) {
        s_ack_set_ok = true;
    } else if (s_pending == LDO_PENDING_OUT_ON) {
        s_ack_out_on_ok = true;
    } else if (s_pending == LDO_PENDING_OUT_OFF) {
        s_ack_out_off_ok = true;
    }
    s_pending = LDO_PENDING_NONE;
}

static void LdoLink_HandleNack(uint8_t sequence, const uint8_t *payload,
                               uint8_t payload_length)
{
    if ((payload_length != 2U) || (s_pending == LDO_PENDING_NONE) ||
        (sequence != s_pending_seq) || (payload[0] != s_pending_type)) {
        return;
    }

    s_status.nack_count++;
    s_nack_seen = true;
    (void)snprintf(s_nack_line, sizeof(s_nack_line),
                   "NACK type=%02X reason=%02X",
                   (unsigned int)payload[0], (unsigned int)payload[1]);
    (void)strncpy(s_status.last_nack, s_nack_line,
                  sizeof(s_status.last_nack) - 1U);
    s_status.last_nack[sizeof(s_status.last_nack) - 1U] = '\0';
}

static void LdoLink_HandleTelemetry(const uint8_t *payload,
                                    uint8_t payload_length)
{
    if (payload_length != LDO_TLM_PAYLOAD_LENGTH) {
        s_status.rx_errors++;
        return;
    }

    s_status.telemetry_valid = true;
    s_status.last_tlm_ms = HAL_GetTick();
    s_status.tlm_count++;
    s_status.vout_mv = LdoLink_GetU32Le(&payload[0]);
    s_status.iout_ma = LdoLink_GetU32Le(&payload[4]);
    s_status.vin_mv = LdoLink_GetU32Le(&payload[8]);
    s_status.vset_mv = LdoLink_GetU32Le(&payload[20]);
    s_status.iset_ma = LdoLink_GetU32Le(&payload[24]);
    s_status.vpre_mv = LdoLink_GetU32Le(&payload[28]);
    s_status.vpre_present = true;
    s_status.mode = payload[32];
    s_status.output_on = payload[33] != 0U;
    s_status.bleed_request = payload[34] != 0U;
    s_status.pgood = payload[35] != 0U;
    s_status.fault_flags = LdoLink_GetU32Le(&payload[36]);
    if (s_status.fault_flags == 0U) {
        (void)strncpy(s_status.fault, "NONE", sizeof(s_status.fault) - 1U);
    } else {
        (void)strncpy(s_status.fault, "G0_FLAGS", sizeof(s_status.fault) - 1U);
    }
    s_status.fault[sizeof(s_status.fault) - 1U] = '\0';
    s_status.fan_percent = payload[64];
    s_status.kill_reported = payload[65] != 0U;
    s_status.cc_cv = payload[66] != 0U;
    s_status.outoff_reported = payload[67] != 0U;
}

static void LdoLink_DispatchFrame(void)
{
    uint8_t type = s_rx_body[0];
    uint8_t sequence = s_rx_body[1];
    const uint8_t *payload = &s_rx_body[2];
    uint8_t payload_length = (uint8_t)(s_rx_length - LDO_PROTO_MIN_LENGTH);

    s_status.rx_lines++;
    if (type == LDO_PROTO_TELEMETRY) {
        LdoLink_HandleTelemetry(payload, payload_length);
    } else if (type == LDO_PROTO_ACK) {
        LdoLink_HandleAck(sequence, payload, payload_length);
    } else if (type == LDO_PROTO_NACK) {
        LdoLink_HandleNack(sequence, payload, payload_length);
    }
}

static void LdoLink_ResetParser(void)
{
    s_rx_state = LDO_RX_WAIT_SOF1;
    s_rx_length = 0U;
    s_rx_body_index = 0U;
    s_rx_crc = 0xFFFFU;
    s_rx_received_crc = 0U;
}

static void LdoLink_ParseByte(uint8_t byte)
{
    switch (s_rx_state) {
    case LDO_RX_WAIT_SOF1:
        if (byte == LDO_PROTO_SOF1) {
            s_rx_state = LDO_RX_WAIT_SOF2;
        }
        break;
    case LDO_RX_WAIT_SOF2:
        if (byte == LDO_PROTO_SOF2) {
            s_rx_state = LDO_RX_WAIT_LENGTH;
        } else if (byte != LDO_PROTO_SOF1) {
            s_rx_state = LDO_RX_WAIT_SOF1;
        }
        break;
    case LDO_RX_WAIT_LENGTH:
        if ((byte < LDO_PROTO_MIN_LENGTH) || (byte > LDO_PROTO_MAX_LENGTH)) {
            s_status.rx_errors++;
            LdoLink_ResetParser();
            break;
        }
        s_rx_length = byte;
        s_rx_body_index = 0U;
        s_rx_crc = LdoLink_Crc16Update(0xFFFFU, byte);
        s_rx_state = LDO_RX_WAIT_BODY;
        break;
    case LDO_RX_WAIT_BODY:
        s_rx_body[s_rx_body_index++] = byte;
        s_rx_crc = LdoLink_Crc16Update(s_rx_crc, byte);
        if (s_rx_body_index >= s_rx_length) {
            s_rx_state = LDO_RX_WAIT_CRC_LOW;
        }
        break;
    case LDO_RX_WAIT_CRC_LOW:
        s_rx_received_crc = byte;
        s_rx_state = LDO_RX_WAIT_CRC_HIGH;
        break;
    case LDO_RX_WAIT_CRC_HIGH:
        s_rx_received_crc |= (uint16_t)byte << 8;
        if (s_rx_received_crc == s_rx_crc) {
            LdoLink_DispatchFrame();
        } else {
            s_status.rx_errors++;
        }
        LdoLink_ResetParser();
        break;
    default:
        LdoLink_ResetParser();
        break;
    }
}

static void LdoLink_ParseRxRing(void)
{
    while (s_rx_tail != s_rx_head) {
        uint8_t byte = s_rx_ring[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % LDO_RX_RING_SIZE);
        LdoLink_ParseByte(byte);
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
            if (Ldo_SetAckRequiresVoutZero(false) &&
                (s_status.vout_mv > LDO_VOUT_ZERO_MV)) {
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
        if (LdoLink_TlmFresh(now_ms) && (s_status.fault_flags != 0U)) {
            if (s_status.fault_flags == LDO_FAULT_VIN_LOW) {
                Debug_Printf("[LDO] G0 VIN_LOW; keep pre-reg alive and recover\r\n");
                LdoLink_ClearPendingAcks();
                LdoLink_EnterState(LDO_G0_CTRL_WAIT_VIN, now_ms);
            } else {
                LdoLink_HardKillFromFault("G0 fault flags");
            }
            break;
        }
        if (s_nack_seen && (s_pending == LDO_PENDING_SET)) {
            s_nack_seen = false;
            s_pending = LDO_PENDING_NONE;
            s_setpoint_dirty = true;
        }
        if ((s_pending == LDO_PENDING_SET) && LdoLink_PendingTimedOut(now_ms)) {
            s_status.cmd_timeout_count++;
            s_pending = LDO_PENDING_NONE;
            s_setpoint_dirty = true;
        }
        if (s_setpoint_dirty &&
            (s_pending == LDO_PENDING_NONE) &&
            Ldo_LiveSetIntervalElapsed(now_ms, s_last_live_set_ms,
                                       LDO_LIVE_SET_MIN_MS)) {
            if (LdoLink_SendSet(now_ms)) {
                s_last_live_set_ms = now_ms;
                s_retry_count = 0U;
            }
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
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_overflow = false;
    LdoLink_ResetParser();
    s_dcdc_permit_request = false;
    s_remote_sense = false;
    s_last_health_ms = 0U;
    s_last_recover_ms = 0U;
    s_output_wanted = false;
    s_g0_volts = LDO_DEFAULT_V;
    s_g0_amps = LDO_DEFAULT_I;
    s_setpoint_dirty = false;
    s_last_live_set_ms = 0U;
    s_ctrl = LDO_G0_CTRL_IDLE;
    s_retry_count = 0U;
    s_tx_sequence = 0U;
    s_pending_seq = 0U;
    s_pending_type = 0U;
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
    Debug_Printf("[LDO] USART2 G0 binary link ready (CRC16 + sequence + ACK)\r\n");
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
    s_rx_head = 0U;
    s_rx_tail = 0U;
    LdoLink_ResetParser();
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
    uint32_t now_ms = HAL_GetTick();
    uint32_t age_ms;

    /* Always poll RXNE — recovers if IT/NVIC path is stuck. */
    LdoLink_PollRx();

    LdoLink_ParseRxRing();
    LdoLink_DumpFirstRx();
    if (s_rx_overflow) {
        s_rx_overflow = false;
        s_status.rx_errors++;
        Debug_Printf("[LDO] WARN: G0 RX ring overflow\r\n");
    }

    if (s_status.last_rx_ms != 0U) {
        age_ms = now_ms - s_status.last_rx_ms;
    } else {
        age_ms = 0xFFFFFFFFu;
    }

    if (!s_status.telemetry_valid ||
        ((s_status.last_tlm_ms != 0U) &&
         ((uint32_t)(now_ms - s_status.last_tlm_ms) > LDO_TLM_STALE_MS))) {
        if (age_ms > LDO_RX_RECOVER_MS) {
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
        /* HAL already aborted the IT RX on ORE. Do not AbortReceive again —
         * that drops FIFO contents and retriggers overrun. */
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
