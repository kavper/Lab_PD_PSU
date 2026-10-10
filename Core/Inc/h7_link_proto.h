#ifndef H7_LINK_PROTO_H
#define H7_LINK_PROTO_H

/*
 * H7 UART7 <-> G4 USART1 production frame. Little-endian.
 * A5 5A LEN TYPE SEQ PAYLOAD CRC16_LE
 * LEN counts TYPE + SEQ + PAYLOAD. CRC-16/CCITT-FALSE (poly 0x1021,
 * init 0xFFFF) covers LEN through the last payload byte.
 * Total size is LEN + 5 and must be <= 120.
 * H7 does not set the METER period. G4's own 5 ms timer does.
 */

#include <stddef.h>
#include <stdint.h>

#define H7_LINK_SOF1                 0xA5U
#define H7_LINK_SOF2                 0x5AU
#define H7_LINK_BAUD                 460800U
#define H7_LINK_MAX_FRAME            120U
#define H7_LINK_MAX_LEN              (H7_LINK_MAX_FRAME - 5U)
#define H7_LINK_MAX_PAYLOAD          (H7_LINK_MAX_LEN - 2U)

#define H7_LINK_METER_PERIOD_MS      5U
#define H7_LINK_SLOW_PERIOD_MS       200U
#define H7_LINK_METER_STALE_MS       50U
#define H7_LINK_SLOW_STALE_MS        1000U
#define H7_LINK_G0_STALE_MS          500U
#define H7_LINK_SET_ATTEMPTS         4U
#define H7_LINK_SET_ATTEMPT_MS       150U
#define H7_LINK_HOST_SET_TIMEOUT_MS  800U
#define H7_LINK_ON_TIMEOUT_MS        8000U
#define H7_LINK_UI_REFRESH_MS        33U

/* H7 -> G4 */
#define H7_LINK_SET                  0x01U /* u32 mV, u32 mA */
#define H7_LINK_ON                   0x02U
#define H7_LINK_OFF                  0x03U
#define H7_LINK_CLEAR                0x04U
#define H7_LINK_PING                 0x05U
#define H7_LINK_DIAG                 0x07U /* empty; reply is TEXT */
#define H7_LINK_PERMIT               0x08U /* u8 0/1 */
#define H7_LINK_REMOTE               0x09U /* u8 0 local, 1 remote */
#define H7_LINK_BMS                  0x0AU /* u8 0 soft, 1 force */
#define H7_LINK_USB                  0x0BU /* u8 0 auto, 1 sink, 2 source */
#define H7_LINK_TEXT_CMD             0x21U /* diagnostic ASCII, no CR/LF required */

/* G4 -> H7 */
#define H7_LINK_METER                0x10U
#define H7_LINK_BMS_TLM              0x11U
#define H7_LINK_PD_TLM               0x12U
#define H7_LINK_AUX_TLM              0x13U /* G0 DAC, NTC, fan, sense self-test */
#define H7_LINK_TEXT                 0x20U
#define H7_LINK_ACK                  0x81U /* u8 acknowledged type */
#define H7_LINK_NACK                 0x82U /* u8 rejected type, u8 reason */

#define H7_LINK_NACK_UNKNOWN         1U
#define H7_LINK_NACK_BAD_PAYLOAD     2U
#define H7_LINK_NACK_RANGE           3U
#define H7_LINK_NACK_UNSAFE          4U
#define H7_LINK_NACK_BUSY            5U
#define H7_LINK_NACK_TIMEOUT         6U
#define H7_LINK_NACK_LINK            7U

#define H7_LINK_METER_BYTES          72U
#define H7_LINK_BMS_BYTES            72U
#define H7_LINK_PD_BYTES             64U
#define H7_LINK_AUX_BYTES            32U

/* BMS bytes 0..52 match the previous 56-byte payload (53..55 stay zero). */
#define H7_BMS_PASSQ_MAH             56U /* i32, DASTATUS6 mAh, signed */
#define H7_BMS_SESSION_MAH           60U /* i32, zero; not a G4 coulomb counter */
#define H7_BMS_SOC_PERMILLE          64U /* u16, from passQ after a voltage seed; 0xFFFF unknown */
#define H7_BMS_CC1_MA                66U /* i16 */
#define H7_BMS_INT_TEMP_DK           68U /* i16, 0.1 K; 0 = unread */
#define H7_BMS_BALANCE               70U /* u8, bit0 = cell 1 */
#define H7_BMS_SOC_FLAGS             71U /* u8, BMS_SOC_FLAG_* */

#define H7_AUX_DAC_CV_MV             0U  /* u32, G0 CV DAC readback */
#define H7_AUX_DAC_CC_MV             4U  /* u32, G0 CC DAC readback */
#define H7_AUX_T1_CC                 8U  /* i16 °C×10, MOSFET; 253 = 25.3 °C; INT16_MIN invalid */
#define H7_AUX_T2_CC                 10U /* i16 °C×10, ambient */
#define H7_AUX_T3_CC                 12U /* i16 °C×10, bleeder */
#define H7_AUX_T4_CC                 14U /* i16 °C×10, 3.3 V LDO / 15→5 V area */
#define H7_AUX_FAN                   16U /* u8 percent actually applied to the fan */
#define H7_AUX_PGOOD                 17U /* u8 */
#define H7_AUX_BLEED                 18U /* u8 */
#define H7_AUX_VALID                 19U /* u8, 1 only while G0 telemetry is younger than 500 ms */
#define H7_AUX_LOCAL_MV              20U /* u16, ADC_LOCAL_VOUT after the 12:1 divider; 0xFFFF = no sample */
#define H7_AUX_REMOTE_P_MV           22U /* u16, REMOTE_P, same scale */
#define H7_AUX_REMOTE_N_MV           24U /* u16, REMOTE_N, same scale */
#define H7_AUX_SENSE_CODE            26U /* u8, SENSE_* from sense_check.h */
#define H7_AUX_SENSE_FLAGS           27U /* u8, bit0 relay closed, bit1 remote requested, bit2 latched off */
#define H7_AUX_STOP_REASON           30U /* u8 LDO_STOP_*, durable until CLEAR/new ON */
#define H7_AUX_FAN_RPM               28U /* u16, 2 pulses/rev; 0 stopped; 0xFFFF no sample yet */
/* Bytes 30..31 stay zero. */

#define H7_METER_VIN_MV              0U
#define H7_METER_VOUT_MV             4U
#define H7_METER_I_BUCK_MA           8U
#define H7_METER_I_BOOST_MA          12U
#define H7_METER_SET_MV              16U
#define H7_METER_ILIM_MA             20U
#define H7_METER_G0_VOUT_MV          24U
#define H7_METER_G0_IOUT_MA          28U
#define H7_METER_G0_VIN_MV           32U
#define H7_METER_G0_VSET_MV          36U
#define H7_METER_G0_ISET_MA          40U
#define H7_METER_VPRE_REQ_MV         44U
#define H7_METER_VPRE_CMD_MV         48U
#define H7_METER_G0_AGE_MS           52U
#define H7_METER_DUTY_A_X10          54U
#define H7_METER_DUTY_C_X10          56U
#define H7_METER_G0_FAULT            58U
#define H7_METER_PSU_FAULT           62U
#define H7_METER_FLAGS0              66U
#define H7_METER_FLAGS1              67U
#define H7_METER_G0_CTRL             68U
#define H7_METER_G0_MODE             69U
#define H7_METER_SET_PHASE           70U
#define H7_METER_G0_STALE            71U

#define H7_F0_G0_OUT                 0x01U
#define H7_F0_G0_WANT                0x02U
#define H7_F0_G0_KILL                0x04U
#define H7_F0_G0_OUTOFF              0x08U
#define H7_F0_G0_CC                  0x10U
#define H7_F0_PERMIT                 0x20U
#define H7_F0_RUN                    0x40U
#define H7_F0_REG_OK                 0x80U

#define H7_F1_STAGE_EN               0x01U
#define H7_F1_PS_EN                  0x02U
#define H7_F1_PS_FAULT               0x04U
#define H7_F1_REM_SENSE              0x08U
#define H7_F1_UCC_A                  0x10U
#define H7_F1_UCC_C                  0x20U
#define H7_F1_G0_VALID               0x40U
#define H7_F1_FAULT_LATCH            0x80U

#define H7_SET_PHASE_IDLE            0U
#define H7_SET_PHASE_INFLIGHT        1U
#define H7_SET_PHASE_PENDING         2U

static inline uint16_t H7Link_Crc16Update(uint16_t crc, uint8_t byte)
{
    uint8_t bit;

    crc ^= (uint16_t)byte << 8;
    for (bit = 0U; bit < 8U; ++bit) {
        crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                              : (uint16_t)(crc << 1);
    }
    return crc;
}

static inline uint16_t H7Link_Crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    size_t i;

    for (i = 0U; i < length; ++i) {
        crc = H7Link_Crc16Update(crc, data[i]);
    }
    return crc;
}

static inline void H7Link_PutU16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
}

static inline void H7Link_PutU32(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)value;
    dst[1] = (uint8_t)(value >> 8);
    dst[2] = (uint8_t)(value >> 16);
    dst[3] = (uint8_t)(value >> 24);
}

static inline void H7Link_PutI32(uint8_t *dst, int32_t value)
{
    H7Link_PutU32(dst, (uint32_t)value);
}

static inline uint16_t H7Link_GetU16(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static inline uint32_t H7Link_GetU32(const uint8_t *src)
{
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

static inline int32_t H7Link_GetI32(const uint8_t *src)
{
    return (int32_t)H7Link_GetU32(src);
}

/* Returns frame length, or 0 if it does not fit in 120 bytes. */
static inline uint16_t H7Link_Build(uint8_t *dst, size_t capacity,
                                    uint8_t type, uint8_t seq,
                                    const uint8_t *payload, uint8_t payload_len)
{
    uint8_t length;
    uint16_t crc;
    uint16_t index = 0U;
    uint8_t i;

    if ((dst == NULL) || (payload_len > H7_LINK_MAX_PAYLOAD)) {
        return 0U;
    }
    length = (uint8_t)(payload_len + 2U);
    if ((size_t)length + 5U > capacity) {
        return 0U;
    }
    dst[index++] = H7_LINK_SOF1;
    dst[index++] = H7_LINK_SOF2;
    dst[index++] = length;
    dst[index++] = type;
    dst[index++] = seq;
    for (i = 0U; i < payload_len; ++i) {
        dst[index++] = payload[i];
    }
    crc = H7Link_Crc16(&dst[2], (size_t)length + 1U);
    H7Link_PutU16(&dst[index], crc);
    index = (uint16_t)(index + 2U);
    return index;
}

typedef enum {
    H7_PARSE_SOF1 = 0,
    H7_PARSE_SOF2,
    H7_PARSE_LEN,
    H7_PARSE_BODY,
    H7_PARSE_CRC0,
    H7_PARSE_CRC1
} H7ParseState;

typedef struct {
    uint8_t state;
    uint8_t length;
    uint8_t body[H7_LINK_MAX_LEN];
    uint8_t index;
    uint16_t crc;
    uint16_t rx_crc;
    uint8_t type;
    uint8_t seq;
    uint8_t payload_len;
} H7LinkParser;

static inline void H7Link_ParserInit(H7LinkParser *parser)
{
    parser->state = H7_PARSE_SOF1;
    parser->length = 0U;
    parser->index = 0U;
    parser->crc = 0xFFFFU;
    parser->rx_crc = 0U;
    parser->type = 0U;
    parser->seq = 0U;
    parser->payload_len = 0U;
}

/* 1 = parser->body holds a valid frame. Payload starts at body[2]. */
static inline int H7Link_ParserByte(H7LinkParser *parser, uint8_t byte)
{
    switch (parser->state) {
    case H7_PARSE_SOF1:
        if (byte == H7_LINK_SOF1) {
            parser->state = H7_PARSE_SOF2;
        }
        break;
    case H7_PARSE_SOF2:
        if (byte == H7_LINK_SOF2) {
            parser->state = H7_PARSE_LEN;
        } else if (byte != H7_LINK_SOF1) {
            parser->state = H7_PARSE_SOF1;
        }
        break;
    case H7_PARSE_LEN:
        if ((byte < 2U) || (byte > H7_LINK_MAX_LEN)) {
            H7Link_ParserInit(parser);
            if (byte == H7_LINK_SOF1) {
                parser->state = H7_PARSE_SOF2;
            }
            break;
        }
        parser->length = byte;
        parser->index = 0U;
        parser->crc = H7Link_Crc16Update(0xFFFFU, byte);
        parser->state = H7_PARSE_BODY;
        break;
    case H7_PARSE_BODY:
        parser->body[parser->index++] = byte;
        parser->crc = H7Link_Crc16Update(parser->crc, byte);
        if (parser->index >= parser->length) {
            parser->state = H7_PARSE_CRC0;
        }
        break;
    case H7_PARSE_CRC0:
        parser->rx_crc = byte;
        parser->state = H7_PARSE_CRC1;
        break;
    case H7_PARSE_CRC1:
        parser->rx_crc |= (uint16_t)byte << 8;
        if (parser->rx_crc == parser->crc) {
            parser->type = parser->body[0];
            parser->seq = parser->body[1];
            parser->payload_len = (uint8_t)(parser->length - 2U);
            parser->state = H7_PARSE_SOF1;
            return 1;
        }
        H7Link_ParserInit(parser);
        break;
    default:
        H7Link_ParserInit(parser);
        break;
    }
    return 0;
}

#endif /* H7_LINK_PROTO_H */
