#ifndef HOST_HEARTBEAT_H
#define HOST_HEARTBEAT_H
#include <stdbool.h>
#include <stdint.h>
#define HOST_HEARTBEAT_TIMEOUT_MS 1000U
static inline bool HostHeartbeat_Expired(bool output_wanted, uint32_t now,
                                         uint32_t last_frame)
{
    return output_wanted && (uint32_t)(now - last_frame) > HOST_HEARTBEAT_TIMEOUT_MS;
}
#endif
