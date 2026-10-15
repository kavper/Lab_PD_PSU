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
    /* Explicit ON starts the DCDC and clears G0's hardware kill. */
    Expect(Dcdc_StageStartAllowed(true, true, false, false),
           "shipping ON starts the pre-regulator without permit");
    Expect(Dcdc_PermitAllowed(true, false),
           "ON clears POWER_KILL without waiting for measured voltage");
    Expect(!Dcdc_PermitAllowed(true, true),
           "the off override holds permit down");
    Expect(!Dcdc_PermitAllowed(false, false),
           "an idle request does not assert permit");

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
