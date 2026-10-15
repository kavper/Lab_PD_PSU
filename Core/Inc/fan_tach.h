#ifndef FAN_TACH_H
#define FAN_TACH_H

#include <stdint.h>

/* Open-collector tach, two falling edges per revolution. */
#define FAN_TACH_PULSES_PER_REV      2U
#define FAN_TACH_WINDOW_MS           1000U
#define FAN_TACH_RPM_NONE            0xFFFFU

void FanTach_Init(void);
void FanTach_Task(void);
uint16_t FanTach_Rpm(void);

/*
 * rpm = edges * 60 / pulses / window_s = edges * 30000 / window_ms.
 * 0xFFFF is reserved for "the first window has not closed yet".
 */
static inline uint16_t FanTach_RpmFromEdges(uint32_t edges, uint32_t window_ms)
{
    uint32_t rpm;

    if (window_ms == 0U) {
        return 0U;
    }
    rpm = (uint32_t)(((uint64_t)edges * (60ULL * 1000ULL / FAN_TACH_PULSES_PER_REV))
                     / window_ms);
    if (rpm >= FAN_TACH_RPM_NONE) {
        rpm = FAN_TACH_RPM_NONE - 1U;
    }
    return (uint16_t)rpm;
}

#endif /* FAN_TACH_H */
