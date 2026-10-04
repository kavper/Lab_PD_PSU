#include "dcdc_permit.h"

#include <stdio.h>

static int g_failures;

static void Expect(int cond, const char *msg)
{
    if (!cond) {
        g_failures++;
        printf("FAIL: %s\n", msg);
    }
}

int main(void)
{
    DcdcPermitRail rail;
    const float grant_v = 6.0f;
    const float hold_v = 4.5f;
    const uint32_t hold_ms = 40U;

    /* ON, stage still off, rail not ready: start the DCDC, keep permit off. */
    Expect(Dcdc_StageStartAllowed(true, true, false, false),
           "shipping ON starts the pre-regulator without permit");
    Expect(!Dcdc_PermitAllowed(true, false, false, false),
           "permit stays off until the stage is on and the rail is ready");

    Expect(Dcdc_PermitAllowed(true, true, true, false),
           "permit follows a running rail that is already up");
    Expect(!Dcdc_PermitAllowed(true, true, false, false),
           "a running stage below the floor does not get permit");
    Expect(!Dcdc_PermitAllowed(true, true, true, true),
           "the off override holds permit down");
    Expect(!Dcdc_PermitAllowed(false, true, true, false),
           "an idle request does not assert permit");

    Dcdc_PermitRailInit(&rail);
    Expect(!Dcdc_PermitRailUpdate(&rail, false, 12.0f, grant_v, hold_v, 0U, hold_ms),
           "a stopped stage clears permit");
    Expect(!Dcdc_PermitRailUpdate(&rail, true, 5.9f, grant_v, hold_v, 10U, hold_ms),
           "5.9 V is still under the 6 V floor");
    Expect(!Dcdc_PermitRailUpdate(&rail, true, 6.0f, grant_v, hold_v, 100U, hold_ms),
           "the first sample at 6 V does not release POWER_KILL");
    Expect(!Dcdc_PermitRailUpdate(&rail, true, 6.2f, grant_v, hold_v, 139U, hold_ms),
           "39 ms at the floor is not yet a grant");
    Expect(Dcdc_PermitRailUpdate(&rail, true, 6.0f, grant_v, hold_v, 140U, hold_ms),
           "40 ms at or above 6 V releases POWER_KILL");
    Expect(Dcdc_PermitRailUpdate(&rail, true, 5.0f, grant_v, hold_v, 200U, hold_ms),
           "a sag to 5 V keeps the pin released");
    Expect(!Dcdc_PermitRailUpdate(&rail, true, 4.49f, grant_v, hold_v, 210U, hold_ms),
           "under 4.5 V kills the LDO again");
    Expect(!Dcdc_PermitRailUpdate(&rail, true, 6.0f, grant_v, hold_v, 300U, hold_ms),
           "returning to 6 V starts the hold again");
    Expect(Dcdc_PermitRailUpdate(&rail, true, 6.5f, grant_v, hold_v, 340U, hold_ms),
           "the second climb grants after another 40 ms");
    Expect(!Dcdc_PermitRailUpdate(&rail, false, 6.5f, grant_v, hold_v, 400U, hold_ms),
           "disabling the stage drops permit");

    Expect(!Dcdc_StageStartAllowed(true, true, true, false),
           "bench bring-up still waits for its own permit");
    Expect(Dcdc_StageStartAllowed(true, true, true, true),
           "bench bring-up starts once that permit is forced");
    Expect(!Dcdc_StageStartAllowed(true, false, false, false),
           "the startup hold still blocks the stage");
    Expect(!Dcdc_StageStartAllowed(false, true, false, true),
           "idle does not start the stage");

    if (g_failures != 0) {
        printf("%d failure(s)\n", g_failures);
        return 1;
    }
    printf("test_dcdc_permit: PASS\n");
    return 0;
}
