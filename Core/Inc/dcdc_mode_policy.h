#ifndef DCDC_MODE_POLICY_H
#define DCDC_MODE_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/*
 * 4-switch buck-boost (leg A = buck HS TA1, leg C = boost LS TC2).
 * Conversion: Vout / Vin = Da / (1 - Dc), boost HS = 1 - Dc.
 *
 *   BUCK        Dc = 0, Da = M
 *   BOOST       Da = 1, Dc = 1 - 1/M
 *   BUCK-BOOST  Da glides 0.90 → 1.00 as M goes 0.90 → 1/(1-0.05)
 *               Dc = 1 - Da/M
 *
 * Duties are continuous at the boundaries so the HRTIM topology
 * (PWM vs StaticHigh) only flips when a leg is already at 0% or ~100% HS.
 */

#ifndef DCDC_MIXED_DA_LO
#define DCDC_MIXED_DA_LO                 0.90f
#endif

#ifndef DCDC_MIXED_DC_MIN
#define DCDC_MIXED_DC_MIN                0.05f
#endif

#ifndef DCDC_DC_MAX
#define DCDC_DC_MAX                      0.80f
#endif

#ifndef DCDC_DA_BOOST_TOPOLOGY
#define DCDC_DA_BOOST_TOPOLOGY           0.995f
#endif

#ifndef DCDC_DC_BUCK_TOPOLOGY
#define DCDC_DC_BUCK_TOPOLOGY            0.020f
#endif

#ifndef DCDC_BUCK_ENTER_MARGIN_V
#define DCDC_BUCK_ENTER_MARGIN_V         1.50f
#endif

#ifndef DCDC_BUCK_EXIT_MARGIN_V
#define DCDC_BUCK_EXIT_MARGIN_V          0.80f
#endif

#ifndef DCDC_BOOST_ENTER_MARGIN_V
#define DCDC_BOOST_ENTER_MARGIN_V        1.50f
#endif

#ifndef DCDC_BOOST_EXIT_MARGIN_V
#define DCDC_BOOST_EXIT_MARGIN_V         0.80f
#endif

#ifndef DCDC_LOW_SETPOINT_V
#define DCDC_LOW_SETPOINT_V              2.00f
#endif

enum {
    DCDC_REGION_BUCK = 0,
    DCDC_REGION_BOOST = 1,
    DCDC_REGION_BUCK_BOOST = 2
};

typedef struct {
    float da;
    float dc;
} Dcdc_Duties_t;

static inline float Dcdc_ModeMinFloat(float a, float b)
{
    return (a < b) ? a : b;
}

static inline float Dcdc_ModeMaxFloat(float a, float b)
{
    return (a > b) ? a : b;
}

static inline float Dcdc_ModeClamp(float value, float lo, float hi)
{
    if (!(value == value)) {
        return lo;
    }
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static inline float Dcdc_MixedMHi(void)
{
    return 1.0f / (1.0f - DCDC_MIXED_DC_MIN);
}

static inline float Dcdc_ConversionRatio(float vin, float vset)
{
    if (vin < 0.10f) {
        vin = 0.10f;
    }
    if (vset < 0.0f) {
        vset = 0.0f;
    }
    return vset / vin;
}

/* Open-loop duty map. Continuous in M — region hysteresis is separate. */
static inline Dcdc_Duties_t Dcdc_FeedForwardDuties(float vin, float vset)
{
    Dcdc_Duties_t d;
    float m;
    float m_hi;
    float t;
    float da;
    float dc;

    if (vset <= 0.02f) {
        d.da = 0.0f;
        d.dc = 0.0f;
        return d;
    }

    m = Dcdc_ConversionRatio(vin, vset);
    m_hi = Dcdc_MixedMHi();

    if (m <= DCDC_MIXED_DA_LO) {
        d.da = Dcdc_ModeClamp(m, 0.0f, 1.0f);
        d.dc = 0.0f;
        return d;
    }

    if (m >= m_hi) {
        d.da = 1.0f;
        dc = 1.0f - (1.0f / m);
        d.dc = Dcdc_ModeClamp(dc, DCDC_MIXED_DC_MIN, DCDC_DC_MAX);
        return d;
    }

    t = (m - DCDC_MIXED_DA_LO) / (m_hi - DCDC_MIXED_DA_LO);
    da = DCDC_MIXED_DA_LO + (t * (1.0f - DCDC_MIXED_DA_LO));
    dc = 1.0f - (da / m);
    d.da = Dcdc_ModeClamp(da, DCDC_MIXED_DA_LO, 1.0f);
    d.dc = Dcdc_ModeClamp(dc, 0.0f, DCDC_DC_MAX);
    return d;
}

static inline float Dcdc_ImpliedM(float da, float dc)
{
    float den = 1.0f - dc;

    if (den < 0.02f) {
        den = 0.02f;
    }
    return da / den;
}

/*
 * Control-region hysteresis on Vset vs Vin. Never skips mixed.
 * 0 = buck, 1 = boost, 2 = buck-boost.
 */
static inline int Dcdc_SelectRegion(int current, float vin, float vset)
{
    float buck_enter = vin - DCDC_BUCK_ENTER_MARGIN_V;
    float buck_exit = vin - DCDC_BUCK_EXIT_MARGIN_V;
    float boost_enter = vin + DCDC_BOOST_ENTER_MARGIN_V;
    float boost_exit = vin + DCDC_BOOST_EXIT_MARGIN_V;
    int candidate;

    if (vset < DCDC_LOW_SETPOINT_V) {
        return DCDC_REGION_BUCK;
    }

    switch (current) {
        case DCDC_REGION_BUCK:
            candidate = (vset >= buck_exit) ? DCDC_REGION_BUCK_BOOST : DCDC_REGION_BUCK;
            break;
        case DCDC_REGION_BOOST:
            candidate = (vset <= boost_exit) ? DCDC_REGION_BUCK_BOOST : DCDC_REGION_BOOST;
            break;
        default:
            if (vset <= buck_enter) {
                candidate = DCDC_REGION_BUCK;
            } else if (vset >= boost_enter) {
                candidate = DCDC_REGION_BOOST;
            } else {
                candidate = DCDC_REGION_BUCK_BOOST;
            }
            break;
    }

    if ((current == DCDC_REGION_BUCK) && (candidate == DCDC_REGION_BOOST)) {
        candidate = DCDC_REGION_BUCK_BOOST;
    }
    if ((current == DCDC_REGION_BOOST) && (candidate == DCDC_REGION_BUCK)) {
        candidate = DCDC_REGION_BUCK_BOOST;
    }

    return candidate;
}

/* PWM vs StaticHigh from the duties that are actually on the FET. */
static inline int Dcdc_TopologyFromDuties(float da, float dc)
{
    if (dc <= DCDC_DC_BUCK_TOPOLOGY) {
        return DCDC_REGION_BUCK;
    }
    if (da >= DCDC_DA_BOOST_TOPOLOGY) {
        return DCDC_REGION_BOOST;
    }
    return DCDC_REGION_BUCK_BOOST;
}

#endif /* DCDC_MODE_POLICY_H */
