#ifndef LDO_CTRL_POLICY_H
#define LDO_CTRL_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/*
 * First-start SET may wait for Vout≈0 (G0 historically NACK OUT ON
 * with VOUT_NOT_ZERO). A live SET while already RUNNING must not drop
 * the rail with OUT OFF — the H7 slider sends SET on every tick.
 */
static inline bool Ldo_SetAckRequiresVoutZero(bool already_running)
{
    return !already_running;
}

/* Coalesce slider spam on USART2; 80 ms is one G0 TLM + ACK window. */
static inline bool Ldo_LiveSetIntervalElapsed(uint32_t now_ms,
                                              uint32_t last_set_ms,
                                              uint32_t min_ms)
{
    if (last_set_ms == 0U) {
        return true;
    }
    return ((uint32_t)(now_ms - last_set_ms) >= min_ms);
}

#endif /* LDO_CTRL_POLICY_H */
