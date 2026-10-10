#include "prereg_request.h"

#include <math.h>
#include <stdio.h>

static int g_failures;

static void ExpectNear(float got, float want, const char *msg)
{
    if (fabsf(got - want) > 0.001f) {
        g_failures++;
        printf("FAIL: %s (got %.3f want %.3f)\n", msg, (double)got, (double)want);
    }
}

int main(void)
{
    const float margin = 1.5f;
    const float vin_floor = 1.5f;
    const float vmin = 1.5f;
    const float vmax = 36.0f;
    const float cv_floor = 13.5f; /* 12 V set + 1.5 V */

    ExpectNear(Prereg_SelectRequestV(false, false, false, true, 8.0f, 0.0f,
                                     cv_floor, margin, vin_floor, vmin, vmax),
               vmin, "idle output off goes to the minimum");

    ExpectNear(Prereg_SelectRequestV(false, true, false, true, 8.0f, 0.0f,
                                     cv_floor, margin, vin_floor, vmin, vmax),
               cv_floor, "output wanted but still off holds the CV floor");

    ExpectNear(Prereg_SelectRequestV(true, true, false, true, 10.0f, 12.0f,
                                     cv_floor, margin, vin_floor, vmin, vmax),
               cv_floor, "CV lifts a low vpre back to Vset + dropout");

    ExpectNear(Prereg_SelectRequestV(true, true, true, true, 9.5f, 8.0f,
                                     cv_floor, margin, vin_floor, vmin, vmax),
               9.5f, "CC keeps G0's Vout + dropout request");

    ExpectNear(Prereg_SelectRequestV(true, true, true, true, 2.5f, 1.0f,
                                     cv_floor, margin, vin_floor, vmin, vmax),
               2.5f, "1 V CC preserves G0 request of 2.5 V");

    ExpectNear(Prereg_SelectRequestV(true, true, true, true, 20.0f, 8.0f,
                                     cv_floor, margin, vin_floor, vmin, vmax),
               cv_floor, "CC does not climb above the CV headroom");

    ExpectNear(Prereg_SelectRequestV(true, true, true, false, 0.0f, 8.0f,
                                     cv_floor, margin, vin_floor, vmin, vmax),
               9.5f, "CC without vpre uses measured Vout + dropout");

    ExpectNear(Prereg_SelectRequestV(true, true, true, false, 0.0f, 1.0f,
                                     cv_floor, margin, vin_floor, vmin, vmax),
               2.5f, "1 V CC without vpre requests 2.5 V");

    ExpectNear(Prereg_SelectRequestV(true, true, false, false, 0.0f, 2.0f,
                                     cv_floor, margin, vin_floor, vmin, vmax),
               cv_floor, "CV without vpre stays on the CV floor");

    ExpectNear(Prereg_SelectRequestV(true, true, false, true, 4.0f, 2.0f,
                                     2.5f, margin, vin_floor, vmin, vmax),
               4.0f, "CV honors the received G0 request during transition");

    ExpectNear(Prereg_SelectRequestV(false, true, false, false, 0.0f, 0.0f,
                                     2.5f, margin, vin_floor, vmin, vmax),
               2.5f, "1 V startup requests 2.5 V");
    ExpectNear(Prereg_SelectRequestV(true, true, false, true, 2.5f, 1.0f,
                                     2.5f, margin, vin_floor, vmin, vmax),
               2.5f, "1 V CV runs at 2.5 V");
    ExpectNear(Prereg_SelectRequestV(true, true, true, true, 1.5f, 0.0f,
                                     cv_floor, margin, vin_floor, vmin, vmax),
               1.5f, "short-circuit CC runs at 1.5 V");

    if (g_failures != 0) {
        printf("%d failure(s)\n", g_failures);
        return 1;
    }
    printf("test_prereg_request: PASS\n");
    return 0;
}
