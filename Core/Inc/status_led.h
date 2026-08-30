#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Single status LED (PD2). Pattern priority: fault > BMS > startup > link > run > idle.
 */

typedef enum {
    STATUS_LED_PATTERN_OFF = 0,
    STATUS_LED_PATTERN_SOLID_ON,
    STATUS_LED_PATTERN_SLOW_BLINK,
    STATUS_LED_PATTERN_FAST_BLINK,
    STATUS_LED_PATTERN_DOUBLE_PULSE,
    STATUS_LED_PATTERN_SOS,
} StatusLed_Pattern_t;

void StatusLed_Init(void);
void StatusLed_Task(void);
void StatusLed_SetPattern(StatusLed_Pattern_t pattern);
StatusLed_Pattern_t StatusLed_GetPattern(void);

#endif /* STATUS_LED_H */
