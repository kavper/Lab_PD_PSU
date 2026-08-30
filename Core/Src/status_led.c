#include "status_led.h"

#include "main.h"

typedef struct {
    uint32_t phase_ms;
    uint8_t step;
} StatusLed_Runtime_t;

static StatusLed_Pattern_t s_pattern = STATUS_LED_PATTERN_OFF;
static StatusLed_Runtime_t s_rt;
static uint32_t s_last_ms;

static bool StatusLed_OutputLevel(uint8_t step, StatusLed_Pattern_t pattern)
{
    switch (pattern) {
        case STATUS_LED_PATTERN_SOLID_ON:
            return true;

        case STATUS_LED_PATTERN_SLOW_BLINK:
            return (step & 0x01U) == 0U;

        case STATUS_LED_PATTERN_FAST_BLINK:
            return (step & 0x01U) == 0U;

        case STATUS_LED_PATTERN_DOUBLE_PULSE:
            return (step == 0U) || (step == 2U);

        case STATUS_LED_PATTERN_SOS:
            /* ... --- ... : 3 short, 3 long, 3 short */
            switch (step) {
                case 0U:
                case 2U:
                case 4U:
                case 12U:
                case 16U:
                case 20U:
                    return true;
                default:
                    return false;
            }

        case STATUS_LED_PATTERN_OFF:
        default:
            return false;
    }
}

static uint32_t StatusLed_StepPeriodMs(StatusLed_Pattern_t pattern, uint8_t step)
{
    switch (pattern) {
        case STATUS_LED_PATTERN_SLOW_BLINK:
            return 500U;

        case STATUS_LED_PATTERN_FAST_BLINK:
            return 125U;

        case STATUS_LED_PATTERN_DOUBLE_PULSE:
            if ((step == 1U) || (step == 3U)) {
                return 700U;
            }
            return 120U;

        case STATUS_LED_PATTERN_SOS:
            if ((step >= 6U) && (step <= 11U)) {
                return 250U;
            }
            if ((step >= 13U) && (step <= 15U)) {
                return 250U;
            }
            if ((step == 5U) || (step == 11U) || (step == 21U)) {
                return 400U;
            }
            return 120U;

        default:
            return 1000U;
    }
}

static uint8_t StatusLed_StepCount(StatusLed_Pattern_t pattern)
{
    switch (pattern) {
        case STATUS_LED_PATTERN_DOUBLE_PULSE:
            return 4U;

        case STATUS_LED_PATTERN_SOS:
            return 22U;

        case STATUS_LED_PATTERN_SLOW_BLINK:
        case STATUS_LED_PATTERN_FAST_BLINK:
            return 2U;

        default:
            return 1U;
    }
}

void StatusLed_Init(void)
{
    s_pattern = STATUS_LED_PATTERN_OFF;
    s_rt.phase_ms = 0U;
    s_rt.step = 0U;
    s_last_ms = HAL_GetTick();
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

void StatusLed_SetPattern(StatusLed_Pattern_t pattern)
{
    if (pattern != s_pattern) {
        s_pattern = pattern;
        s_rt.phase_ms = 0U;
        s_rt.step = 0U;
    }
}

StatusLed_Pattern_t StatusLed_GetPattern(void)
{
    return s_pattern;
}

void StatusLed_Task(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed;
    bool level;

    elapsed = now_ms - s_last_ms;
    s_last_ms = now_ms;
    s_rt.phase_ms += elapsed;

    while (s_rt.phase_ms >= StatusLed_StepPeriodMs(s_pattern, s_rt.step)) {
        s_rt.phase_ms -= StatusLed_StepPeriodMs(s_pattern, s_rt.step);
        s_rt.step++;
        if (s_rt.step >= StatusLed_StepCount(s_pattern)) {
            s_rt.step = 0U;
        }
    }

    level = StatusLed_OutputLevel(s_rt.step, s_pattern);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
