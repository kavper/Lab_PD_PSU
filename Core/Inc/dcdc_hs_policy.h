#ifndef DCDC_HS_POLICY_H
#define DCDC_HS_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/*
 * High-side half-bridge helpers (HAL-free, host-testable).
 *
 * UCC33420 (BUCK_TR_EN / BOOST_TR_EN) is extra bootstrap support for a leg
 * whose high-side FET stays on so long that the LS pulse cannot refresh C_BST.
 * Enable per-leg only; never blanket both converters.
 *
 * High-side INA296 shunts (I_IN_BUCK, I_OUT_BOOST) are the current sensors
 * used for averaging and OCP. The ACS37100 series inductor sensor is ignored.
 */

#ifndef DCDC_UCC_HS_DUTY_ON_10K
#define DCDC_UCC_HS_DUTY_ON_10K          9700U  /* 97.00% HS duty */
#endif

#ifndef DCDC_UCC_HS_DUTY_HYST_10K
#define DCDC_UCC_HS_DUTY_HYST_10K        200U   /* drop EN below 95.00% */
#endif

#ifndef DCDC_HS_OCP_HIT_LIMIT
#define DCDC_HS_OCP_HIT_LIMIT            8U     /* consecutive 4 kHz control cycles */
#endif

#ifndef DCDC_ADC_FULL_SCALE
#define DCDC_ADC_FULL_SCALE              4095.0f
#endif

static inline uint32_t Dcdc_ClampDuty10k(uint32_t duty_10k)
{
    return (duty_10k > 10000U) ? 10000U : duty_10k;
}

static inline uint32_t Dcdc_UccDutyOff10k(void)
{
    if (DCDC_UCC_HS_DUTY_ON_10K > DCDC_UCC_HS_DUTY_HYST_10K) {
        return DCDC_UCC_HS_DUTY_ON_10K - DCDC_UCC_HS_DUTY_HYST_10K;
    }
    return 0U;
}

/* True when this leg's high-side duty needs UCC EN (or LS refresh fallback). */
static inline bool Dcdc_UccNeededForHsDuty(uint32_t hs_duty_10k, bool currently_enabled)
{
    uint32_t duty = Dcdc_ClampDuty10k(hs_duty_10k);

    if (currently_enabled) {
        return duty >= Dcdc_UccDutyOff10k();
    }

    return duty >= DCDC_UCC_HS_DUTY_ON_10K;
}

static inline float Dcdc_Ina296CountsToAmpere(uint16_t raw,
                                              float vref_v,
                                              float gain,
                                              float rshunt_ohm)
{
    float volts;
    float a_per_v;

    if ((gain <= 0.0f) || (rshunt_ohm <= 0.0f) || (vref_v <= 0.0f)) {
        return 0.0f;
    }

    volts = ((float)raw * vref_v) / DCDC_ADC_FULL_SCALE;
    a_per_v = 1.0f / (gain * rshunt_ohm);
    return (volts - (vref_v * 0.5f)) * a_per_v;
}

static inline uint32_t Dcdc_OcpUpdateHits(uint32_t hits, bool over_limit, uint32_t limit)
{
    if (over_limit) {
        if (hits < limit) {
            hits++;
        }
    } else if (hits > 0U) {
        hits--;
    }

    return hits;
}

static inline bool Dcdc_OcpTripped(uint32_t hits, uint32_t limit)
{
    return hits >= limit;
}

/*
 * ADC trigger as a fraction of the PWM period, in the overlap of both
 * high-side ON windows. HS is ON from PER (0) until this end.
 * S/H for INA296 is ~0.15 µs (~8% of 2 µs), so keep a 10% margin before HS off.
 */
#ifndef DCDC_ADC_HS_SAMPLE_MARGIN
#define DCDC_ADC_HS_SAMPLE_MARGIN            0.10f
#endif

static inline float Dcdc_MinFloat(float a, float b)
{
    return (a < b) ? a : b;
}

static inline float Dcdc_Clamp01(float value)
{
    if (!(value == value)) {
        return 0.0f;
    }
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 1.0f) {
        return 1.0f;
    }
    return value;
}

static inline float Dcdc_AdcTriggerInHsOn(float buck_hs_end, float boost_hs_end)
{
    float hs_end;
    float mid;
    float max_trig;

    buck_hs_end = Dcdc_Clamp01(buck_hs_end);
    boost_hs_end = Dcdc_Clamp01(boost_hs_end);
    hs_end = Dcdc_MinFloat(buck_hs_end, boost_hs_end);

    if (hs_end < 0.04f) {
        return 0.05f;
    }

    mid = 0.5f * hs_end;
    max_trig = hs_end - DCDC_ADC_HS_SAMPLE_MARGIN;
    if (max_trig < 0.05f) {
        /* Pulse too short for a 10% trailing margin; stay at 5% of the period. */
        return 0.05f;
    }
    if (mid > max_trig) {
        mid = max_trig;
    }
    if (mid < 0.05f) {
        mid = 0.05f;
    }
    if (mid > 0.95f) {
        mid = 0.95f;
    }

    return mid;
}

#endif /* DCDC_HS_POLICY_H */
