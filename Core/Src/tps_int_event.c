#include "tps_int_event.h"

#include <string.h>

bool TPS_IntEventAny(const uint8_t event[TPS_INT_EVENT_BYTES])
{
    uint8_t i;

    if (event == NULL) {
        return false;
    }
    for (i = 0U; i < TPS_INT_EVENT_BYTES; ++i) {
        if (event[i] != 0U) {
            return true;
        }
    }
    return false;
}

bool TPS_IntEventBit(const uint8_t event[TPS_INT_EVENT_BYTES], uint8_t bit)
{
    return (event != NULL) && (bit < 88U) &&
           ((event[bit >> 3] & (uint8_t)(1U << (bit & 7U))) != 0U);
}

void TPS_IntEventDecode(TPS_IntEvent_t *decoded,
                        const uint8_t event[TPS_INT_EVENT_BYTES])
{
    if ((decoded == NULL) || (event == NULL)) {
        return;
    }

    memset(decoded, 0, sizeof(*decoded));
    memcpy(decoded->raw, event, TPS_INT_EVENT_BYTES);
    decoded->any = TPS_IntEventAny(event);
    decoded->hard_reset = TPS_IntEventBit(event, 1U);
    decoded->plug_changed = TPS_IntEventBit(event, 3U);
    decoded->power_swap_complete = TPS_IntEventBit(event, 4U);
    decoded->overcurrent = TPS_IntEventBit(event, 9U);
    decoded->new_contract_consumer = TPS_IntEventBit(event, 12U);
    decoded->new_contract_provider = TPS_IntEventBit(event, 13U);
    decoded->source_caps_received = TPS_IntEventBit(event, 14U);
    decoded->power_swap_requested = TPS_IntEventBit(event, 17U);
    decoded->power_path_changed = TPS_IntEventBit(event, 23U);
    decoded->status_updated = TPS_IntEventBit(event, 26U);
    decoded->pd_status_updated = TPS_IntEventBit(event, 27U);
    decoded->power_event_error = TPS_IntEventBit(event, 35U);
    decoded->sink_transition_complete = TPS_IntEventBit(event, 42U);
    decoded->unable_to_source = TPS_IntEventBit(event, 46U);
    decoded->ext_source_safe_state = TPS_IntEventBit(event, 57U);
    decoded->i2c_controller_nack = TPS_IntEventBit(event, 82U);
}

static void TPS_IntEventSetBit(uint8_t data[TPS_INT_EVENT_BYTES], uint8_t bit)
{
    data[bit >> 3] |= (uint8_t)(1U << (bit & 7U));
}

void TPS_IntEventEnableRequiredBits(uint8_t mask[TPS_INT_EVENT_BYTES])
{
    if (mask == NULL) {
        return;
    }

    TPS_IntEventSetBit(mask, 1U);  /* hard reset */
    TPS_IntEventSetBit(mask, 3U);  /* attach/detach */
    TPS_IntEventSetBit(mask, 4U);  /* power-role swap complete */
    TPS_IntEventSetBit(mask, 9U);  /* overcurrent */
    TPS_IntEventSetBit(mask, 12U); /* contract as consumer */
    TPS_IntEventSetBit(mask, 13U); /* contract as provider */
    TPS_IntEventSetBit(mask, 14U); /* partner Source_Capabilities received */
    TPS_IntEventSetBit(mask, 17U); /* partner requested a power-role swap */
    TPS_IntEventSetBit(mask, 23U); /* power path changed */
    TPS_IntEventSetBit(mask, 26U); /* status changed */
    TPS_IntEventSetBit(mask, 27U); /* PD status changed */
    TPS_IntEventSetBit(mask, 35U); /* VBUS power event */
    TPS_IntEventSetBit(mask, 42U); /* source transition */
    TPS_IntEventSetBit(mask, 46U); /* unable to source */
    TPS_IntEventSetBit(mask, 57U); /* source safe state */
    TPS_IntEventSetBit(mask, 82U); /* I2Cc NACK */
}

TPS25751_Status_t TPS_IntEventStartRead(TPS25751_Device_t *dev)
{
    return TPS25751_StartReadRegister(dev, TPS25751_REG_INT_EVENT,
                                      TPS_INT_EVENT_BYTES);
}

TPS25751_Status_t TPS_IntEventStartClear(TPS25751_Device_t *dev,
                                         const uint8_t event[TPS_INT_EVENT_BYTES])
{
    if (event == NULL) {
        return TPS25751_INVALID_ARG;
    }
    return TPS25751_StartWriteRegister(dev, TPS25751_REG_INT_CLEAR,
                                       event, TPS_INT_EVENT_BYTES);
}
