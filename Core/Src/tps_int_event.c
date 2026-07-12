#include "tps_int_event.h"
#include <string.h>

static uint32_t ReadLe32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

bool TPS_IntEventAny(const uint8_t event[TPS_INT_EVENT_BYTES])
{
    uint8_t i;
    for (i = 0U; i < TPS_INT_EVENT_BYTES; ++i)
        if (event[i] != 0U) return true;
    return false;
}

bool TPS_IntEventBit(const uint8_t event[TPS_INT_EVENT_BYTES], uint8_t bit)
{
    return (bit < 88U) && ((event[bit / 8U] & (uint8_t)(1U << (bit % 8U))) != 0U);
}

TPS25751_Status_t TPS_ReadIntEvent(TPS25751_Device_t *dev,
                                   uint8_t event[TPS_INT_EVENT_BYTES])
{
    uint8_t len = 0U;
    TPS25751_Status_t st = TPS25751_ReadPayload(dev, TPS_REG_INT_EVENT,
                                                event, TPS_INT_EVENT_BYTES, &len);
    if (st != TPS25751_OK) return st;
    return (len == TPS_INT_EVENT_BYTES) ? TPS25751_OK : TPS25751_BAD_LENGTH;
}

TPS25751_Status_t TPS_ReadEventSnapshot(TPS25751_Device_t *dev,
                                        TPS_IntEventSnapshot_t *s)
{
    uint8_t p[12], len;
    TPS25751_Status_t st;
    if ((dev == NULL) || (s == NULL)) return TPS25751_INVALID_ARG;
    st = TPS_ReadIntEvent(dev, s->int_event);
    if (st != TPS25751_OK) return st;
    st = TPS25751_ReadPayload(dev, TPS_REG_ACTIVE_PDO, p, 6U, &len);
    if ((st != TPS25751_OK) || (len < 4U)) return (st != TPS25751_OK) ? st : TPS25751_BAD_LENGTH;
    s->active_pdo = ReadLe32(p);
    st = TPS25751_ReadPayload(dev, TPS_REG_ACTIVE_RDO, p, 12U, &len);
    if ((st != TPS25751_OK) || (len < 4U)) return (st != TPS25751_OK) ? st : TPS25751_BAD_LENGTH;
    s->active_rdo = ReadLe32(p);
    st = TPS25751_ReadPayload(dev, TPS_REG_POWER_PATH_STATUS, p, 8U, &len);
    if ((st != TPS25751_OK) || (len < 1U)) return (st != TPS25751_OK) ? st : TPS25751_BAD_LENGTH;
    s->power_path = 0U;
    memcpy(&s->power_path, p, len > 8U ? 8U : len);
    st = TPS25751_ReadPayload(dev, TPS_REG_PD_STATUS, p, 4U, &len);
    if ((st != TPS25751_OK) || (len < 1U)) return (st != TPS25751_OK) ? st : TPS25751_BAD_LENGTH;
    s->pd_status = 0U;
    memcpy(&s->pd_status, p, len > 4U ? 4U : len);
    return TPS25751_OK;
}

TPS25751_Status_t TPS_ClearIntEvent(TPS25751_Device_t *dev,
                                    const uint8_t event[TPS_INT_EVENT_BYTES])
{
    return TPS25751_WritePayload(dev, TPS_REG_INT_CLEAR, event, TPS_INT_EVENT_BYTES);
}

TPS25751_Status_t TPS_EnablePolledEvents(TPS25751_Device_t *dev)
{
    uint8_t mask[TPS_INT_EVENT_BYTES], len;
    TPS25751_Status_t st = TPS25751_ReadPayload(dev, TPS_REG_INT_MASK,
                                                mask, sizeof(mask), &len);
    if (st != TPS25751_OK) return st;
    if (len != sizeof(mask)) return TPS25751_BAD_LENGTH;
    /* hard reset, plug, power swap, provider contract, source caps,
     * sink transition complete and unable-to-source */
    mask[0] |= (1U << 1) | (1U << 3) | (1U << 4);
    mask[1] |= (1U << 5) | (1U << 6);
    mask[5] |= (1U << 2) | (1U << 6);
    return TPS25751_WritePayload(dev, TPS_REG_INT_MASK, mask, sizeof(mask));
}

TPS25751_Status_t TPS_LimitSourceCapsTo5V(TPS25751_Device_t *dev)
{
    /* Only the 31-byte useful prefix is needed for seven PDOs and routing.
     * Do not write the full 63-byte register during startup. */
    uint8_t caps[31U], len;
    uint8_t verify[31U], verify_len;
    uint32_t pdo;
    TPS25751_Status_t st = TPS25751_ReadPayload(dev, 0x32U, caps,
                                                sizeof(caps), &len);
    if (st != TPS25751_OK) return st;
    if (len < 7U) return TPS25751_BAD_LENGTH;
    /* TX Source Capabilities uses a three-byte register header. Refuse to
     * advertise anything if PDO1 itself is not a fixed 5 V PDO. */
    pdo = ReadLe32(&caps[3]);
    if (((pdo >> 30) != 0U) || ((((pdo >> 10) & 0x3FFU) * 50U) != 5000U))
        return TPS25751_ERROR;
    /* Fixed PDO current is in 10 mA units: advertise the same 3 A limit
     * configured in BQ25731 OTG mode. */
    pdo = (pdo & ~0x3FFUL) | 300UL;
    caps[3] = (uint8_t)pdo;
    caps[4] = (uint8_t)(pdo >> 8);
    caps[5] = (uint8_t)(pdo >> 16);
    caps[6] = (uint8_t)(pdo >> 24);
    caps[0] = (uint8_t)((caps[0] & 0xF8U) | 1U);
    /* TX_SOURCE_CAPS bits 9:8: 2 selects PP3 = PPHV for PDO1.
     * Bit 8 is byte 1 bit 0 because payload byte 0 contains bits 7:0. */
    caps[1] = (uint8_t)((caps[1] & ~0x03U) | 0x02U);

    st = TPS25751_WritePayload(dev, 0x32U, caps, len);
    if (st != TPS25751_OK) return st;
    st = TPS25751_ReadPayload(dev, 0x32U, verify, sizeof(verify), &verify_len);
    if (st != TPS25751_OK) return st;
    if ((verify_len != len) || ((verify[0] & 0x07U) != 1U) ||
        ((verify[1] & 0x03U) != 0x02U) ||
        ((ReadLe32(&verify[3]) & 0x3FFU) != 300U)) return TPS25751_ERROR;
    /* Do not issue SSrC during TPS/BQ startup. The next Source attach reads
     * this verified TX_SOURCE_CAPS value. Renegotiation commands are only
     * appropriate once the policy engine is already running. */
    return TPS25751_OK;
}
