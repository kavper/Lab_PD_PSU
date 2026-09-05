#include "dcdc_hs_policy.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static int g_failures;

static void ExpectTrue(int cond, const char *msg)
{
    if (!cond) {
        g_failures++;
        printf("FAIL: %s\n", msg);
    }
}

static void ExpectNear(float got, float want, float eps, const char *msg)
{
    if (fabsf(got - want) > eps) {
        g_failures++;
        printf("FAIL: %s (got %.6f want %.6f)\n", msg, (double)got, (double)want);
    }
}

int main(void)
{
    uint32_t hits;
    uint16_t mid;
    uint16_t four_amp;
    float i;

    /* UCC: off until 97%, then stay on until 95%. */
    ExpectTrue(!Dcdc_UccNeededForHsDuty(9699U, false), "96.99% must not enable UCC");
    ExpectTrue(Dcdc_UccNeededForHsDuty(9700U, false), "97.00% must enable UCC");
    ExpectTrue(Dcdc_UccNeededForHsDuty(10000U, false), "100% HS must enable UCC");
    ExpectTrue(Dcdc_UccNeededForHsDuty(9500U, true), "hysteresis keeps UCC at 95%");
    ExpectTrue(!Dcdc_UccNeededForHsDuty(9499U, true), "below 95% drops UCC");
    ExpectTrue(!Dcdc_UccNeededForHsDuty(5000U, false), "50% PWM must not enable UCC");
    ExpectTrue(Dcdc_UccDutyOff10k() == 9500U, "off threshold is 95.00%");

    /* INA296A path: 3.0 V Vref, gain 100, 1 mOhm → 10 A/V, 0 A at 1.5 V. */
    mid = (uint16_t)((1.5f / 3.0f) * 4095.0f + 0.5f);
    i = Dcdc_Ina296CountsToAmpere(mid, 3.0f, 100.0f, 0.001f);
    ExpectNear(i, 0.0f, 0.05f, "mid-scale is ~0 A");

    /* 4 A → 1.5 V + 4 A * 0.001 * 100 = 1.9 V. */
    four_amp = (uint16_t)((1.9f / 3.0f) * 4095.0f + 0.5f);
    i = Dcdc_Ina296CountsToAmpere(four_amp, 3.0f, 100.0f, 0.001f);
    ExpectNear(i, 4.0f, 0.05f, "1.9 V is ~4 A");

    hits = 0U;
    hits = Dcdc_OcpUpdateHits(hits, true, DCDC_HS_OCP_HIT_LIMIT);
    ExpectTrue(hits == 1U, "first over-limit hit");
    ExpectTrue(!Dcdc_OcpTripped(hits, DCDC_HS_OCP_HIT_LIMIT), "not yet tripped");
    while (!Dcdc_OcpTripped(hits, DCDC_HS_OCP_HIT_LIMIT)) {
        hits = Dcdc_OcpUpdateHits(hits, true, DCDC_HS_OCP_HIT_LIMIT);
    }
    ExpectTrue(hits == DCDC_HS_OCP_HIT_LIMIT, "trips after consecutive hits");
    hits = Dcdc_OcpUpdateHits(hits, false, DCDC_HS_OCP_HIT_LIMIT);
    ExpectTrue(hits == (DCDC_HS_OCP_HIT_LIMIT - 1U), "under-limit decays");

    /* ADC trigger sits in the middle of the overlapping HS-ON windows. */
    ExpectNear(Dcdc_AdcTriggerInHsOn(0.50f, 1.00f), 0.25f, 0.001f,
               "buck 50% / boost pass-through → 25%");
    ExpectNear(Dcdc_AdcTriggerInHsOn(1.00f, 0.60f), 0.30f, 0.001f,
               "boost LS 40% (HS 60%) / buck pass-through → 30%");
    ExpectNear(Dcdc_AdcTriggerInHsOn(0.80f, 0.70f), 0.35f, 0.001f,
               "buck-boost overlap ends at 70% → 35%");
    ExpectNear(Dcdc_AdcTriggerInHsOn(1.00f, 1.00f), 0.50f, 0.001f,
               "both HS static 100% → 50%");
    ExpectNear(Dcdc_AdcTriggerInHsOn(0.12f, 1.00f), 0.05f, 0.001f,
               "short HS pulse keeps trigger inside the window");
    ExpectNear(Dcdc_AdcTriggerInHsOn(0.18f, 1.00f), 0.08f, 0.001f,
               "10% trailing margin clips mid of a short-ish pulse");
    ExpectNear(Dcdc_AdcTriggerInHsOn(0.03f, 1.00f), 0.05f, 0.001f,
               "tiny HS window falls back to 5%");

    if (g_failures != 0) {
        printf("%d failure(s)\n", g_failures);
        return EXIT_FAILURE;
    }

    printf("test_dcdc_hs_policy: PASS\n");
    return EXIT_SUCCESS;
}
