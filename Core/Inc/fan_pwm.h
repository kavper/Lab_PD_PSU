#ifndef FAN_PWM_H
#define FAN_PWM_H

#include <stdint.h>

void FanPwm_Init(void);
void FanPwm_SetPercent(uint8_t percent);

#endif /* FAN_PWM_H */
