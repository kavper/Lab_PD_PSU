#ifndef BMS_FET_GUARD_H
#define BMS_FET_GUARD_H
#include <stdbool.h>
#include <stdint.h>
/* Hardware timeout is 500 ms. This independent software ceiling includes
 * status-scan latency; it is not a resistor energy rating. */
#define BMS_PDSG_HOST_LIMIT_MS 1000U
typedef struct {
    bool start_attempted;
    bool pdsg_seen;
    bool latched;
    uint32_t pdsg_since_ms;
} BmsFetGuard;
static inline bool BmsFetGuard_AllowStart(BmsFetGuard *g) {
    if (g->latched || g->start_attempted) { g->latched=true; return false; }
    return true;
}
static inline void BmsFetGuard_Observe(BmsFetGuard *g, uint8_t fet, uint32_t now) {
    if (fet & 8U) {
        if (!g->pdsg_seen) { g->pdsg_seen=true; g->pdsg_since_ms=now; }
    } else { g->pdsg_seen=false; }
}
static inline bool BmsFetGuard_Expired(BmsFetGuard *g, uint32_t now) {
    if (g->pdsg_seen && (uint32_t)(now-g->pdsg_since_ms)>=BMS_PDSG_HOST_LIMIT_MS)
        g->latched=true;
    return g->latched;
}
#endif
