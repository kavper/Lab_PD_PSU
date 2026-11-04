#include <assert.h>
#include <stdio.h>
#include "bms_lowbatt_policy.h"

static BmsLowBatt_Config_t c;
static BmsLowBatt_State_t s;

/* Feed samples every 250 ms from t0 to t1; return the time of the shutdown or 0. */
static uint32_t run(uint32_t t0, uint32_t t1, int16_t mv, int32_t ma)
{
    for (uint32_t t = t0; t <= t1; t += 250U) {
        if (BmsLowBatt_Step(&c, &s, t, mv, ma)) return t;
    }
    return 0U;
}

int main(void)
{
    BmsLowBatt_DefaultConfig(&c);

    /* Healthy pack: nothing. */
    s = (BmsLowBatt_State_t){0};
    assert(!run(1000, 60000, 3300, -300) && !BmsLowBatt_Flags(&s));

    /* Low warning with hysteresis. */
    assert(!run(60250, 61000, 2990, -300) && BmsLowBatt_Flags(&s) == BMS_LOWBATT_FLAG_LOW);
    assert(!run(61250, 62000, 3030, -300) && (BmsLowBatt_Flags(&s) & BMS_LOWBATT_FLAG_LOW));
    assert(!run(62250, 63000, 3060, -300) && !BmsLowBatt_Flags(&s));

    /* Short dip below cut-off (load step) is ignored. */
    s = (BmsLowBatt_State_t){0};
    assert(!run(0, 20000, 3200, -300));
    assert(!run(20250, 25000, 2780, -300));          /* 4.75 s below */
    assert(!run(25250, 40000, 2850, -300));          /* recovered */
    assert(!(BmsLowBatt_Flags(&s) & BMS_LOWBATT_FLAG_EMPTY));

    /* Sustained cut-off: EMPTY shown, shutdown 10 s + 3 s later, exactly once. */
    uint32_t t = run(40250, 80000, 2780, -300);
    assert(t >= 40250U + 10000U + 3000U && t <= 40250U + 10000U + 3000U + 500U);
    assert(BmsLowBatt_Flags(&s) & BMS_LOWBATT_FLAG_EMPTY);
    assert(!run(t + 250U, t + 30000U, 2780, -300));

    /* Host survived SHUTDOWN (external power, not charging): no AFE cycling. */
    BmsLowBatt_PackLost(&s);
    assert(!run(200000, 260000, 2900, -50));
    /* Charging resumes normal operation. */
    assert(!run(260250, 262000, 2900, 1500) && !BmsLowBatt_Flags(&s));

    /* Charging during the notice cancels the shutdown. */
    s = (BmsLowBatt_State_t){0};
    assert(!run(0, 16000, 3200, -300));
    assert(!run(16250, 26500, 2780, -300) && (BmsLowBatt_Flags(&s) & BMS_LOWBATT_FLAG_EMPTY));
    assert(!run(26750, 27000, 2800, 2000) && !BmsLowBatt_Flags(&s));
    assert(!run(27250, 60000, 2900, 2000));

    /* Wake on a flat pack (rebounded to 2.92 V) without charger: refuse. */
    s = (BmsLowBatt_State_t){0};
    t = run(500, 30000, 2920, -300);
    assert(t >= 500U + 15000U + 3000U && t <= 500U + 15000U + 3000U + 500U);

    /* Same wake but a charger starts within the boot window: allowed. */
    s = (BmsLowBatt_State_t){0};
    assert(!run(500, 4000, 2920, -300));
    assert(!run(4250, 6000, 2920, 3000));
    assert(!run(6250, 60000, 2950, -300) && !(BmsLowBatt_Flags(&s) & BMS_LOWBATT_FLAG_EMPTY));

    /* Wake with enough rebound (3.05 V): allowed, then normal cut-off later. */
    s = (BmsLowBatt_State_t){0};
    assert(!run(500, 30000, 3050, -300));
    assert(run(30250, 50000, 2790, -300));

    /* Fresh boot after a battery-only shutdown starts clean. */
    s = (BmsLowBatt_State_t){0};
    assert(!BmsLowBatt_Flags(&s));

    puts("PASS: low-battery warning with hysteresis, 2.80 V cut-off after 10 s + 3 s notice, "
         "boot refusal below 2.95 V without charger, charging cancels, no AFE cycling on external power");
    return 0;
}
