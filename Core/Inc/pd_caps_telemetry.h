#ifndef PD_CAPS_TELEMETRY_H
#define PD_CAPS_TELEMETRY_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#define PD_CAPS_PAYLOAD_BYTES 44U
typedef struct {
    bool sent;
    uint32_t sent_ms;
    uint8_t payload[PD_CAPS_PAYLOAD_BYTES];
} PdCapsTelemetry;
/* Detach is sent once; attached sessions retain a 1 s heartbeat so a
 * restarted H7 recovers and can enforce its existing freshness deadline. */
static inline bool PdCapsTelemetry_Due(const PdCapsTelemetry *s,
    const uint8_t payload[PD_CAPS_PAYLOAD_BYTES], uint32_t now)
{
    return !s->sent || memcmp(s->payload,payload,PD_CAPS_PAYLOAD_BYTES)!=0 ||
        ((payload[1]&1U)!=0U && (uint32_t)(now-s->sent_ms)>=1000U);
}
static inline void PdCapsTelemetry_Sent(PdCapsTelemetry *s,
    const uint8_t payload[PD_CAPS_PAYLOAD_BYTES], uint32_t now)
{
    memcpy(s->payload,payload,PD_CAPS_PAYLOAD_BYTES);
    s->sent_ms=now; s->sent=true;
}
#endif
