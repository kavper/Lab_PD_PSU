#ifndef FAN_PWM_H
#define FAN_PWM_H

#include <stdint.h>

void FanPwm_Init(void);
void FanPwm_SetPercent(uint8_t percent);

/*
 * Q9 (MMBT3904) sits between PA7 and J7 pin 4, so the MCU pin is inverted
 * onto the fan. A 4-wire fan runs while its PWM pin is high. Compare ticks
 * are therefore the MCU high time for (100 - percent): 0% holds PA7 high
 * and 100% holds it low.
 */
static inline uint32_t FanPwm_CompareTicks(uint32_t period, uint8_t percent)
{
    uint32_t mcu_high;

    if (percent > 100U) {
        percent = 100U;
    }
    mcu_high = 100U - (uint32_t)percent;
    return (period * mcu_high + 50U) / 100U;
}

#endif /* FAN_PWM_H */
