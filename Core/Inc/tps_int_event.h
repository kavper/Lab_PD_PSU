#ifndef TPS_INT_EVENT_H
#define TPS_INT_EVENT_H

#include "tps25751.h"
#include <stdbool.h>
#include <stdint.h>

#define TPS_REG_INT_EVENT           0x14U
#define TPS_REG_INT_MASK            0x16U
#define TPS_REG_INT_CLEAR           0x18U
#define TPS_REG_ACTIVE_PDO          0x34U
#define TPS_REG_ACTIVE_RDO          0x35U
#define TPS_REG_POWER_PATH_STATUS   0x26U
#define TPS_REG_PD_STATUS           0x40U
#define TPS_INT_EVENT_BYTES         11U
#define TPS_INT_EVENT_POLL_MS       5U

/* Keep this at zero until the complete high-voltage transition sequence is
 * validated.  Zero means that firmware may only produce USB-C default 5 V. */
#define PD_SOURCE_HIGH_VOLTAGE_ENABLE 0U

typedef struct {
    uint8_t int_event[TPS_INT_EVENT_BYTES];
    uint32_t active_pdo;
    uint32_t active_rdo;
    uint64_t power_path;
    uint32_t pd_status;
} TPS_IntEventSnapshot_t;

bool TPS_IntEventAny(const uint8_t event[TPS_INT_EVENT_BYTES]);
bool TPS_IntEventBit(const uint8_t event[TPS_INT_EVENT_BYTES], uint8_t bit);
TPS25751_Status_t TPS_ReadIntEvent(TPS25751_Device_t *dev,
                                   uint8_t event[TPS_INT_EVENT_BYTES]);
TPS25751_Status_t TPS_ReadEventSnapshot(TPS25751_Device_t *dev,
                                        TPS_IntEventSnapshot_t *snapshot);
TPS25751_Status_t TPS_ClearIntEvent(TPS25751_Device_t *dev,
                                    const uint8_t event[TPS_INT_EVENT_BYTES]);
TPS25751_Status_t TPS_EnablePolledEvents(TPS25751_Device_t *dev);
TPS25751_Status_t TPS_LimitSourceCapsTo5V(TPS25751_Device_t *dev);

#endif
