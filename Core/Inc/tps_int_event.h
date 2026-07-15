#ifndef TPS_INT_EVENT_H
#define TPS_INT_EVENT_H

#include "tps25751.h"

#include <stdbool.h>
#include <stdint.h>

#define TPS_INT_EVENT_BYTES 11U

typedef struct {
    uint8_t raw[TPS_INT_EVENT_BYTES];
    bool any;
    bool hard_reset;
    bool plug_changed;
    bool power_swap_complete;
    bool overcurrent;
    bool new_contract_consumer;
    bool new_contract_provider;
    bool source_caps_received;
    bool power_swap_requested;
    bool power_path_changed;
    bool status_updated;
    bool pd_status_updated;
    bool power_event_error;
    bool sink_transition_complete;
    bool unable_to_source;
    bool ext_source_safe_state;
    bool i2c_controller_nack;
} TPS_IntEvent_t;

bool TPS_IntEventAny(const uint8_t event[TPS_INT_EVENT_BYTES]);
bool TPS_IntEventBit(const uint8_t event[TPS_INT_EVENT_BYTES], uint8_t bit);
void TPS_IntEventDecode(TPS_IntEvent_t *decoded,
                        const uint8_t event[TPS_INT_EVENT_BYTES]);
void TPS_IntEventEnableRequiredBits(uint8_t mask[TPS_INT_EVENT_BYTES]);

TPS25751_Status_t TPS_IntEventStartRead(TPS25751_Device_t *dev);
TPS25751_Status_t TPS_IntEventStartClear(TPS25751_Device_t *dev,
                                         const uint8_t event[TPS_INT_EVENT_BYTES]);

#endif
