#include "fan_pwm.h"
#include "fan_tach.h"

#include <stdio.h>

static int g_failures;

static void ExpectEq(unsigned got, unsigned want, const char *msg)
{
    if (got != want) {
        g_failures++;
        printf("FAIL: %s (got %u want %u)\n", msg, got, want);
    }
}

int main(void)
{
    /* Q9 inverts: 0% command holds the MCU pin high for the whole period. */
    ExpectEq(FanPwm_CompareTicks(6800U, 0U), 6800U,
             "0 percent keeps the transistor on and the fan pin low");
    ExpectEq(FanPwm_CompareTicks(6800U, 100U), 0U,
             "100 percent releases the transistor so the fan pin idles high");
    ExpectEq(FanPwm_CompareTicks(6800U, 50U), 3400U,
             "50 percent is half the MCU high time");
    ExpectEq(FanPwm_CompareTicks(6800U, 200U), 0U,
             "a request above 100 percent is full speed");

    ExpectEq(FanTach_RpmFromEdges(0U, 1000U), 0U, "no edges is a stopped fan");
    ExpectEq(FanTach_RpmFromEdges(2U, 1000U), 60U, "one revolution per second is 60 rpm");
    ExpectEq(FanTach_RpmFromEdges(100U, 1000U), 3000U, "100 Hz is 3000 rpm");
    ExpectEq(FanTach_RpmFromEdges(200U, 2000U), 3000U,
             "the same rate over two seconds is still 3000 rpm");
    ExpectEq(FanTach_RpmFromEdges(3000U, 1000U), 65534U,
             "an overflowing count stays below the no-sample code");
    ExpectEq(FAN_TACH_RPM_NONE, 0xFFFFU, "no sample is 0xFFFF");

    if (g_failures != 0) {
        printf("%d failure(s)\n", g_failures);
        return 1;
    }
    printf("test_fan_drive: PASS\n");
    return 0;
}
