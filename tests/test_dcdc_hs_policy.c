#include "dcdc_hs_policy.h"
#include "dcdc_mode_policy.h"
#include "host_link_policy.h"
#include "ldo_ctrl_policy.h"
#include "ldo_tlm_parse.h"

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

    /* UCC: off until 98%, then stay on until 96%. */
    ExpectTrue(!Dcdc_UccNeededForHsDuty(9799U, false), "97.99% must not enable UCC");
    ExpectTrue(Dcdc_UccNeededForHsDuty(9800U, false), "98.00% must enable UCC");
    ExpectTrue(Dcdc_UccNeededForHsDuty(10000U, false), "100% HS must enable UCC");
    ExpectTrue(Dcdc_UccNeededForHsDuty(9600U, true), "hysteresis keeps UCC at 96%");
    ExpectTrue(!Dcdc_UccNeededForHsDuty(9599U, true), "below 96% drops UCC");
    ExpectTrue(!Dcdc_UccNeededForHsDuty(5000U, false), "50% PWM must not enable UCC");
    ExpectTrue(Dcdc_UccDutyOff10k() == 9600U, "off threshold is 96.00%");

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
    ExpectTrue(!Dcdc_HsEmergencyOvercurrent(9.99f, 9.99f),
               "G4 half-bridge currents below 10 A do not emergency-trip");
    ExpectTrue(Dcdc_HsEmergencyOvercurrent(10.00f, 0.0f),
               "G4 emergency OCP trips from buck sensor at 10 A");
    ExpectTrue(Dcdc_HsEmergencyOvercurrent(0.0f, 10.00f),
               "G4 emergency OCP trips from boost sensor at 10 A");

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

    {
        Dcdc_Duties_t d;
        Dcdc_Duties_t d_lo;
        Dcdc_Duties_t d_hi;
        float m_hi = Dcdc_MixedMHi();
        float prev_da;
        float prev_dc;
        int i;
        float max_dda = 0.0f;
        float max_ddc = 0.0f;

        d = Dcdc_FeedForwardDuties(15.0f, 7.5f);
        ExpectNear(d.da, 0.50f, 0.002f, "pure buck Da = M");
        ExpectNear(d.dc, 0.0f, 0.001f, "pure buck Dc = 0");

        d_lo = Dcdc_FeedForwardDuties(15.0f, 15.0f * DCDC_MIXED_DA_LO);
        ExpectNear(d_lo.da, DCDC_MIXED_DA_LO, 0.002f, "buck/mixed boundary Da");
        ExpectNear(d_lo.dc, 0.0f, 0.002f, "buck/mixed boundary Dc = 0");

        d_hi = Dcdc_FeedForwardDuties(15.0f, 15.0f * m_hi);
        ExpectNear(d_hi.da, 1.0f, 0.002f, "mixed/boost boundary Da = 1");
        ExpectNear(d_hi.dc, DCDC_MIXED_DC_MIN, 0.003f, "mixed/boost boundary Dc");

        d = Dcdc_FeedForwardDuties(15.0f, 20.0f);
        ExpectNear(d.da, 1.0f, 0.001f, "pure boost Da = 1");
        ExpectNear(d.dc, 1.0f - (15.0f / 20.0f), 0.003f, "pure boost Dc = 1-Vin/Vout");
        ExpectNear(Dcdc_ImpliedM(d.da, d.dc), 20.0f / 15.0f, 0.01f,
                   "boost implied M tracks Vset/Vin");

        d = Dcdc_FeedForwardDuties(15.0f, 15.0f);
        ExpectNear(Dcdc_ImpliedM(d.da, d.dc), 1.0f, 0.02f,
                   "mixed at VIN keeps implied M ≈ 1");

        prev_da = 0.0f;
        prev_dc = 0.0f;
        for (i = 40; i <= 180; i++) {
            float vset = 15.0f * ((float)i / 100.0f);
            Dcdc_Duties_t step = Dcdc_FeedForwardDuties(15.0f, vset);
            if (i > 40) {
                float dda = fabsf(step.da - prev_da);
                float ddc = fabsf(step.dc - prev_dc);
                if (dda > max_dda) {
                    max_dda = dda;
                }
                if (ddc > max_ddc) {
                    max_ddc = ddc;
                }
            }
            prev_da = step.da;
            prev_dc = step.dc;
        }
        ExpectTrue(max_dda < 0.03f, "Da is continuous across 0.40..1.80 M");
        ExpectTrue(max_ddc < 0.03f, "Dc is continuous across 0.40..1.80 M");

        ExpectTrue(Dcdc_SelectRegion(DCDC_REGION_BUCK, 15.0f, 20.0f) ==
                   DCDC_REGION_BUCK_BOOST,
                   "buck must not skip into boost");
        ExpectTrue(Dcdc_SelectRegion(DCDC_REGION_BOOST, 15.0f, 10.0f) ==
                   DCDC_REGION_BUCK_BOOST,
                   "boost must not skip into buck");
        ExpectTrue(Dcdc_SelectRegion(DCDC_REGION_BUCK_BOOST, 15.0f, 20.0f) ==
                   DCDC_REGION_BOOST,
                   "mixed enters boost above VIN+1.5 V");
        ExpectTrue(Dcdc_SelectRegion(DCDC_REGION_BUCK_BOOST, 15.0f, 10.0f) ==
                   DCDC_REGION_BUCK,
                   "mixed enters buck below VIN-1.5 V");
        ExpectTrue(Dcdc_SelectRegion(DCDC_REGION_BOOST, 15.0f, 20.0f) ==
                   DCDC_REGION_BOOST,
                   "boost stays boost well above VIN");

        ExpectTrue(Dcdc_TopologyFromDuties(0.50f, 0.0f) == DCDC_REGION_BUCK,
                   "Dc=0 is buck topology");
        ExpectTrue(Dcdc_TopologyFromDuties(1.00f, 0.20f) == DCDC_REGION_BOOST,
                   "Da=1 and Dc>0 is boost topology");
        ExpectTrue(Dcdc_TopologyFromDuties(0.92f, 0.04f) == DCDC_REGION_BUCK_BOOST,
                   "both legs PWM is mixed topology");
    }

    ExpectTrue(Ldo_TlmLooksComplete(
                   "TLM out=0 mode=0 vset=5000 vout=37 iset=100 iout=1 vin=9256 "
                   "pgood=1 kill=0 outoff=1 cccv=0 fault=VIN_LOW"),
               "well-formed TLM is accepted");
    ExpectTrue(!Ldo_TlmLooksComplete(
                   "TLM ut0 od=0 vset=5000 vou=3 pgood=1 kill=0 fault=VIN_OW"),
               "shredded TLM prefix is rejected");
    ExpectTrue(!Ldo_TlmLooksComplete(
                   "TLM out=0 mode=0 vset=5000 vout=37 pgood=1 fault=VIN_LOW"),
               "TLM missing kill= is rejected");

    ExpectTrue(!HostLink_ShouldForwardG0Line(
                   "TLM out=1 mode=1 vset=10100 vout=10271 pgood=1 kill=0 fault=NONE",
                   false),
               "G0 TLM must not hit USART1 when VERBOSE=0");
    ExpectTrue(HostLink_ShouldForwardG0Line(
                   "TLM out=1 mode=1 vset=10100 vout=10271 pgood=1 kill=0 fault=NONE",
                   true),
               "VERBOSE=1 still forwards TLM for bench debug");
    ExpectTrue(!HostLink_ShouldForwardG0Line(
                   "ACK SET V=10.100 I=1.650 OUT=ON", false),
               "G0 ACK SET must not hit USART1");
    ExpectTrue(HostLink_ShouldForwardG0Line(
                   "NACK FAULT=VIN_LOW", false),
               "G0 NACK stays visible on the host console");
    ExpectTrue(HostLink_LooksLikeHostCommand("SET 17.300"),
               "SET at start of line is a command");
    ExpectTrue(!HostLink_LooksLikeHostCommand("7.300"),
               "shredded SET fragment is not a command");
    ExpectTrue(HostLink_BmsPeriodMs(500U) == 500U,
               "slow TEL keeps TB/TC in lockstep");
    ExpectTrue(HostLink_BmsPeriodMs(100U) == 200U,
               "fast TEL still sends cells at 200 ms");
    ExpectTrue(HostLink_BmsPeriodMs(0U) == 0U,
               "TEL 0 stops BMS frames too");

    ExpectTrue(Ldo_SetAckRequiresVoutZero(false),
               "first-start SET may wait for Vout≈0");
    ExpectTrue(!Ldo_SetAckRequiresVoutZero(true),
               "live SET while RUNNING must not drop the rail");
    ExpectTrue(Ldo_LiveSetIntervalElapsed(80U, 0U, 80U),
               "first live SET is immediate");
    ExpectTrue(!Ldo_LiveSetIntervalElapsed(79U, 1U, 80U),
               "slider SET is coalesced inside 80 ms");
    ExpectTrue(Ldo_LiveSetIntervalElapsed(81U, 1U, 80U),
               "live SET goes out after the coalesce window");

    {
        static const char *host_second[] = {
            "TLM out=1 mode=1 vset=10100 vout=10271 iset=1650 iout=1 vin=36592 t1=3214 t2=4237 t3=3813 t4=3244 bleed=0 fan=75 pgood=1 kill=0 outoff=0 cccv=0 fault=NONE",
            "TLM out=1 mode=1 vset=10100 vout=10271 iset=1650 iout=1 vin=36591 t1=3214 t2=4237 t3=3816 t4=3247 bleed=0 fan=75 pgood=1 kill=0 outoff=0 cccv=0 fault=NONE",
            "TLM out=1 mode=1 vset=10100 vout=10271 iset=1650 iout=1 vin=36590 t1=3214 t2=4237 t3=3816 t4=3253 bleed=0 fan=75 pgood=1 kill=0 outoff=0 cccv=0 fault=NONE",
            "T vin_mv=15251 vout_mv=16451 iout_ma=-5 set_mv=10100 ilim_ma=1650 g0_vout_mv=10271",
            "TB c1_mv=3862 c2_mv=3848 c3_mv=3857 c4_mv=-1 c5_mv=3862 dV_mv=14 sum_mv=15429",
            "TC bq_vbus_mv=19296 pd_mv=20000 pd_ma=5000",
            "ACK SET V=10.100 I=1.650 OUT=ON",
            "ACK OUT ON"
        };
        size_t i;
        size_t raw = 0U;
        size_t fwd = 0U;

        for (i = 0U; i < (sizeof(host_second) / sizeof(host_second[0])); i++) {
            size_t n = strlen(host_second[i]) + 2U;
            raw += n;
            if (HostLink_IsG0TlmLine(host_second[i]) ||
                HostLink_IsG0AckLine(host_second[i])) {
                continue;
            }
            fwd += n;
        }
        ExpectTrue(raw > fwd, "dropping TLM+ACK shrinks USART1 payload");
        ExpectTrue(fwd < (raw / 2U),
                   "one TLM-heavy second should lose more than half the bytes");
        (void)printf("host UART sample: raw %u B -> forwarded %u B\n",
                     (unsigned)raw, (unsigned)fwd);
    }

    if (g_failures != 0) {
        printf("%d failure(s)\n", g_failures);
        return EXIT_FAILURE;
    }

    printf("test_dcdc_hs_policy: PASS\n");
    return EXIT_SUCCESS;
}
