#ifndef SENSE_CHECK_H
#define SENSE_CHECK_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/*
 * Kelvin sense on the wire side of K1 (G6K-2G-DC5).
 *
 * PB14 ADC_LOCAL_VOUT, PB0 ADC_REMOTE_P and PB1 ADC_REMOTE_N each see a
 * 220 kΩ / 20 kΩ divider, then 1 kΩ into the pin and 10 nF to ground.
 * The 1 kΩ does not change the DC ratio. Vadc = Vin / 12. Clamps are BAT54
 * to 3.3 V and ground. The ADC reference is 3.0 V, so full scale is 36 V.
 *
 * PB0 and PB1 stay on REMOTE_P and REMOTE_N. K1 only moves the LDO sense
 * amp: coil off, COM (V_SNS_P / V_SNS_N) is local VOUT and local GND; coil
 * on, COM is the remote pair. The wire check therefore runs while regulation
 * is still local.
 *
 * R112 and R115 (4.7 kΩ) sit on those common contacts, from local VOUT to
 * V_SNS_P and from V_SNS_N to ground. They reach the remote wires only while
 * K1 is energized. An open positive lead then rests at 240/244.7 of Vout
 * (98.1%), and an open negative lead rests at ground. That pair is inside a
 * normal drop and is reported as OK. It is not a detected break. Before K1
 * closes, the same resistors do not pull the remote wires: an open lead falls
 * toward ground on its own divider and blocks the close. A break that must
 * be caught while K1 is closed needs a different hardware diagnostic.
 *
 * Each channel's uncalibrated error is separate from the cable-drop limits.
 * No DMM trim is stored. SENSE_GAIN_PPM and SENSE_OFFSET_MV are the budget:
 *   divider 220 kΩ / 20 kΩ, 1% resistors, worst-case ratio
 *   (222.2+19.8)/19.8 = 12.222 against 12.000 → 1.85% → 19000 ppm.
 *   The shared Vref cancels in a difference; this ppm is the mismatch
 *   that does not. Offset and INL are about 4 LSB. 36 V / 4095 ≈ 8.8 mV,
 *   so the budget uses ±40 mV at the divider input.
 * SENSE_DROP_WIRE_MV (500) and SENSE_DROP_SUM_MV (1000) are the first
 * numbers to check on the bench. They are not a locked product spec.
 *
 * The divider Thevenin is 220 kΩ || 20 kΩ ≈ 18.3 kΩ into 10 nF, so
 * τ ≈ 0.183 ms. Five time constants are under 1 ms. The task period is
 * 100 ms, so a sample is settled before it is used. Agreement between
 * samples is the output and the wires holding still.
 *
 * DP = local_vout − remote_p, DN = remote_n, VD = remote_p − remote_n.
 * The gate checks DP, DN, the sum of the drops, and the top of the local
 * reading. Before K1 closes it also requires SENSE_OK_STREAK samples that
 * agree within the channel budget. After close it keeps watching the drops
 * and a missed conversion. In CV it also waits for VD to meet the setpoint;
 * in CC that comparison stays off, and a descent of the output to zero is
 * the low-voltage state rather than a wiring fault.
 *
 * Below Sense_MinLocalMv the sample is NOT_READY. That is separate from a
 * remote failure: with K1 already closed the relay stays shut and PERMIT
 * stays up, including a CC load short that falls to 0 V. Host OFF opens the
 * relay and does not latch. A real fault asks the caller to drop PERMIT
 * first and only then to open K1, and it latches remote off.
 */

#define SENSE_DIVIDER_NUM            12U
#define SENSE_ADC_FULL_SCALE         4095U
#define SENSE_FULL_SCALE_MV          36000U
#define SENSE_LOCAL_MAX_MV           (SENSE_FULL_SCALE_MV - 1U)

#define SENSE_GAIN_PPM               19000U
#define SENSE_OFFSET_MV              40U

#define SENSE_DROP_WIRE_MV           500U
#define SENSE_DROP_SUM_MV            1000U

#define SENSE_R_FALLBACK_OHM         4700U
#define SENSE_DIVIDER_LOAD_OHM       240000U
#define SENSE_OPEN_PLUS_NUM          2400U
#define SENSE_OPEN_PLUS_DEN          2447U

#define SENSE_FILTER_TAU_US          183U
#define SENSE_PERIOD_MS              100U
#define SENSE_OK_STREAK              3U
#define SENSE_MV_MISSING             0xFFFFU

#define SENSE_FLAG_CLOSED            0x01U
#define SENSE_FLAG_WANTED            0x02U
#define SENSE_FLAG_LATCH             0x04U

enum {
    SENSE_OK = 0,         /* DP, DN and the sum sit inside limit + error */
    SENSE_NOT_READY = 1,  /* local Vout is below the budget threshold; not a fault */
    SENSE_DROP_P = 2,     /* positive drop outside the wire limit and its error */
    SENSE_DROP_SUM = 3,   /* each wire passes, the sum does not */
    SENSE_REVERSED = 4,   /* negative lead is up at the output, positive is not */
    SENSE_N_ON_POUT = 5,  /* both leads sit on the positive output */
    SENSE_SHORT = 6,      /* the two sense leads read the same mid voltage */
    SENSE_DROP_N = 7,     /* negative drop outside the wire limit and its error */
    SENSE_NO_SAMPLE = 8,  /* the injected conversion did not finish */
    SENSE_LOCAL_HIGH = 9, /* local reading is saturated or above the measurable top */
    SENSE_CV_VD = 10      /* relay already closed, CV, VD missed the setpoint */
};

typedef struct {
    uint32_t wire_mv;
    uint32_t sum_mv;
    uint32_t local_max_mv;
} SenseLimits;

typedef struct {
    uint32_t local_mv;
    uint32_t remote_p_mv;
    uint32_t remote_n_mv;
    uint32_t setpoint_mv;
    bool have_sample;
    bool compare_setpoint;
    bool wanted;
    bool output_on; /* host output request; OFF releases K1 without a latch */
} SenseInput;

typedef struct {
    bool wanted;
    bool closed;
    bool latched;
    bool have_prev;
    uint8_t streak;
    uint8_t cv_streak;
    uint8_t code;
    uint32_t prev_local_mv;
    uint32_t prev_p_mv;
    uint32_t prev_n_mv;
} SenseGate;

typedef struct {
    uint8_t code;
    bool closed;
    bool latched;
    bool drop_permit;
} SenseStep;

static inline uint32_t Sense_CountsToMv(uint16_t counts, uint32_t vref_mv)
{
    if (counts > SENSE_ADC_FULL_SCALE) {
        counts = (uint16_t)SENSE_ADC_FULL_SCALE;
    }
    return (uint32_t)(((uint64_t)counts * (uint64_t)vref_mv *
                       (uint64_t)SENSE_DIVIDER_NUM +
                       (SENSE_ADC_FULL_SCALE / 2U)) /
                      SENSE_ADC_FULL_SCALE);
}

static inline uint16_t Sense_MvToU16(uint32_t mv)
{
    if (mv >= SENSE_MV_MISSING) {
        return (uint16_t)(SENSE_MV_MISSING - 1U);
    }
    return (uint16_t)mv;
}

static inline uint32_t Sense_ChannelErrorMv(uint32_t reading_mv)
{
    return ((reading_mv * SENSE_GAIN_PPM) / 1000000U) + SENSE_OFFSET_MV;
}

static inline uint32_t Sense_DpErrorMv(uint32_t local_mv, uint32_t remote_p_mv)
{
    return Sense_ChannelErrorMv(local_mv) + Sense_ChannelErrorMv(remote_p_mv);
}

static inline uint32_t Sense_DnErrorMv(uint32_t remote_n_mv)
{
    return Sense_ChannelErrorMv(remote_n_mv);
}

static inline uint32_t Sense_VdErrorMv(uint32_t remote_p_mv, uint32_t remote_n_mv)
{
    return Sense_ChannelErrorMv(remote_p_mv) + Sense_ChannelErrorMv(remote_n_mv);
}

static inline uint32_t Sense_SumErrorMv(uint32_t local_mv, uint32_t remote_p_mv,
                                       uint32_t remote_n_mv)
{
    return Sense_ChannelErrorMv(local_mv) + Sense_ChannelErrorMv(remote_p_mv) +
           Sense_ChannelErrorMv(remote_n_mv);
}

/*
 * A lead parked on the wrong rail (drop == local) still has to clear the
 * wire limit after the two-channel error is subtracted. Below that voltage
 * every connection looks legal, so the sample is NOT_READY.
 *   local > (wire + 2*offset) / (1 − gain)
 */
static inline uint32_t Sense_MinLocalMv(uint32_t wire_limit_mv)
{
    uint64_t num = ((uint64_t)wire_limit_mv + (2ULL * (uint64_t)SENSE_OFFSET_MV)) *
                   1000000ULL;
    uint64_t den = 1000000ULL - (uint64_t)SENSE_GAIN_PPM;

    if (den == 0ULL) {
        return 0U;
    }
    return (uint32_t)((num + den - 1ULL) / den);
}

/* Voltage at an open REMOTE_P pin while K1 ties it to V_SNS_P. */
static inline uint32_t Sense_OpenPlusMv(uint32_t local_mv)
{
    return (uint32_t)(((uint64_t)local_mv * (uint64_t)SENSE_OPEN_PLUS_NUM) /
                      (uint64_t)SENSE_OPEN_PLUS_DEN);
}

static inline bool Sense_PeriodSettled(void)
{
    return ((uint32_t)SENSE_PERIOD_MS * 1000U) >=
           (20U * (uint32_t)SENSE_FILTER_TAU_US);
}

static inline SenseLimits Sense_DefaultLimits(void)
{
    SenseLimits lim;

    lim.wire_mv = SENSE_DROP_WIRE_MV;
    lim.sum_mv = SENSE_DROP_SUM_MV;
    lim.local_max_mv = SENSE_LOCAL_MAX_MV;
    return lim;
}

static inline uint32_t Sense_AbsDiffU32(uint32_t a, uint32_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static inline bool Sense_ReadingAgrees(uint32_t earlier_mv, uint32_t later_mv)
{
    uint32_t allow = Sense_ChannelErrorMv(earlier_mv);
    uint32_t allow_now = Sense_ChannelErrorMv(later_mv);

    if (allow_now > allow) {
        allow = allow_now;
    }
    return Sense_AbsDiffU32(earlier_mv, later_mv) <= allow;
}

static inline bool Sense_NearReading(uint32_t value_mv, uint32_t target_mv)
{
    uint32_t err = Sense_ChannelErrorMv(value_mv) + Sense_ChannelErrorMv(target_mv);

    return Sense_AbsDiffU32(value_mv, target_mv) <= err;
}

static inline bool Sense_DropAccepted(int32_t drop_mv, uint32_t err_mv,
                                     uint32_t limit_mv)
{
    int32_t err = (int32_t)err_mv;
    int32_t lim = (int32_t)limit_mv;

    return (drop_mv >= -err) && (drop_mv <= (lim + err));
}

static inline bool Sense_VdMatchesSetpoint(uint32_t remote_p_mv,
                                          uint32_t remote_n_mv,
                                          uint32_t setpoint_mv)
{
    int32_t vd = (int32_t)remote_p_mv - (int32_t)remote_n_mv;
    int32_t err = (int32_t)Sense_VdErrorMv(remote_p_mv, remote_n_mv);
    int32_t delta = vd - (int32_t)setpoint_mv;

    if (delta < 0) {
        delta = -delta;
    }
    return delta <= err;
}

static inline uint8_t Sense_ClassifyLimits(uint32_t local_mv, uint32_t remote_p_mv,
                                          uint32_t remote_n_mv,
                                          SenseLimits lim)
{
    uint32_t min_local;
    uint32_t dp_err;
    uint32_t dn_err;
    uint32_t vd_err;
    uint32_t sum_err;
    int32_t dp;
    int32_t dn;
    int32_t vd;
    int32_t sum;
    bool dp_ok;
    bool dn_ok;

    if ((local_mv >= SENSE_FULL_SCALE_MV) || (local_mv > lim.local_max_mv)) {
        return SENSE_LOCAL_HIGH;
    }

    min_local = Sense_MinLocalMv(lim.wire_mv);
    if (local_mv < min_local) {
        return SENSE_NOT_READY;
    }

    dp = (int32_t)local_mv - (int32_t)remote_p_mv;
    dn = (int32_t)remote_n_mv;
    vd = (int32_t)remote_p_mv - (int32_t)remote_n_mv;
    sum = dp + dn;
    dp_err = Sense_DpErrorMv(local_mv, remote_p_mv);
    dn_err = Sense_DnErrorMv(remote_n_mv);
    vd_err = Sense_VdErrorMv(remote_p_mv, remote_n_mv);
    sum_err = Sense_SumErrorMv(local_mv, remote_p_mv, remote_n_mv);
    dp_ok = Sense_DropAccepted(dp, dp_err, lim.wire_mv);
    dn_ok = Sense_DropAccepted(dn, dn_err, lim.wire_mv);

    if (Sense_NearReading(remote_p_mv, local_mv) &&
        Sense_NearReading(remote_n_mv, local_mv)) {
        return SENSE_N_ON_POUT;
    }
    if (Sense_NearReading(remote_n_mv, local_mv)) {
        return SENSE_REVERSED;
    }
    if ((!dp_ok) && (!dn_ok) && (vd < -(int32_t)vd_err)) {
        return SENSE_REVERSED;
    }
    if ((!dp_ok) && (!dn_ok) &&
        (Sense_AbsDiffU32((uint32_t)((vd < 0) ? -vd : vd), 0U) <= vd_err)) {
        return SENSE_SHORT;
    }
    if (!dp_ok) {
        return SENSE_DROP_P;
    }
    if (!dn_ok) {
        return SENSE_DROP_N;
    }
    if (!Sense_DropAccepted(sum, sum_err, lim.sum_mv)) {
        return SENSE_DROP_SUM;
    }
    return SENSE_OK;
}

static inline uint8_t Sense_Classify(uint32_t local_mv, uint32_t remote_p_mv,
                                    uint32_t remote_n_mv)
{
    return Sense_ClassifyLimits(local_mv, remote_p_mv, remote_n_mv,
                                Sense_DefaultLimits());
}

static inline void Sense_GateInit(SenseGate *gate)
{
    gate->wanted = false;
    gate->closed = false;
    gate->latched = false;
    gate->have_prev = false;
    gate->streak = 0U;
    gate->cv_streak = 0U;
    gate->code = SENSE_NO_SAMPLE;
    gate->prev_local_mv = 0U;
    gate->prev_p_mv = 0U;
    gate->prev_n_mv = 0U;
}

static inline void Sense_GateRequest(SenseGate *gate, bool enable)
{
    if (enable == gate->wanted) {
        return;
    }
    gate->wanted = enable;
    gate->streak = 0U;
    gate->cv_streak = 0U;
    gate->have_prev = false;
    if (!enable) {
        gate->latched = false;
        gate->closed = false;
    }
}

static inline void Sense_GateTrip(SenseGate *gate, SenseStep *step, uint8_t code)
{
    gate->code = code;
    gate->closed = false;
    gate->latched = true;
    gate->streak = 0U;
    gate->cv_streak = 0U;
    gate->have_prev = false;
    step->code = code;
    step->closed = false;
    step->latched = true;
    step->drop_permit = true;
}

static inline SenseStep Sense_GateStep(SenseGate *gate, const SenseInput *in)
{
    SenseStep step;
    uint8_t code;
    bool was_closed;

    step.code = gate->code;
    step.closed = gate->closed;
    step.latched = gate->latched;
    step.drop_permit = false;
    was_closed = gate->closed;

    if (!in->wanted) {
        gate->wanted = false;
        gate->latched = false;
        gate->closed = false;
        gate->streak = 0U;
        gate->cv_streak = 0U;
        gate->have_prev = false;
        if (in->have_sample) {
            gate->code = Sense_Classify(in->local_mv, in->remote_p_mv,
                                        in->remote_n_mv);
        } else {
            gate->code = SENSE_NO_SAMPLE;
        }
        step.code = gate->code;
        step.closed = false;
        step.latched = false;
        step.drop_permit = false;
        return step;
    }

    gate->wanted = true;

    if (gate->latched) {
        if (in->have_sample) {
            gate->code = Sense_Classify(in->local_mv, in->remote_p_mv,
                                        in->remote_n_mv);
        } else {
            gate->code = SENSE_NO_SAMPLE;
        }
        gate->closed = false;
        gate->streak = 0U;
        step.code = gate->code;
        step.closed = false;
        step.latched = true;
        step.drop_permit = false;
        return step;
    }

    /*
     * Host OFF. The output path has already dropped PERMIT. Release K1
     * without creating a latch, and leave a latch that a real fault set.
     */
    if (!in->output_on) {
        gate->closed = false;
        gate->streak = 0U;
        gate->cv_streak = 0U;
        gate->have_prev = false;
        if (in->have_sample) {
            gate->code = Sense_Classify(in->local_mv, in->remote_p_mv,
                                        in->remote_n_mv);
        } else {
            gate->code = SENSE_NO_SAMPLE;
        }
        step.code = gate->code;
        step.closed = false;
        step.latched = gate->latched;
        step.drop_permit = false;
        return step;
    }

    if (!in->have_sample) {
        gate->have_prev = false;
        gate->streak = 0U;
        gate->cv_streak = 0U;
        if (was_closed) {
            Sense_GateTrip(gate, &step, SENSE_NO_SAMPLE);
        } else {
            gate->code = SENSE_NO_SAMPLE;
            step.code = SENSE_NO_SAMPLE;
            step.closed = false;
            step.latched = false;
        }
        return step;
    }

    code = Sense_Classify(in->local_mv, in->remote_p_mv, in->remote_n_mv);
    gate->code = code;

    /*
     * Low voltage is not a wiring fault. A closed relay stays closed through
     * a CC short or any other descent under the budget floor. PERMIT stays.
     */
    if (code == SENSE_NOT_READY) {
        gate->have_prev = false;
        gate->streak = 0U;
        gate->cv_streak = 0U;
        gate->closed = was_closed;
        step.code = SENSE_NOT_READY;
        step.closed = was_closed;
        step.latched = false;
        step.drop_permit = false;
        return step;
    }

    if (code != SENSE_OK) {
        gate->have_prev = false;
        gate->streak = 0U;
        gate->cv_streak = 0U;
        if (was_closed) {
            Sense_GateTrip(gate, &step, code);
        } else {
            step.code = code;
            step.closed = false;
            step.latched = false;
        }
        return step;
    }

    if (!was_closed) {
        if ((gate->streak == 0U) || (!gate->have_prev)) {
            gate->streak = 1U;
        } else if (Sense_ReadingAgrees(gate->prev_local_mv, in->local_mv) &&
                   Sense_ReadingAgrees(gate->prev_p_mv, in->remote_p_mv) &&
                   Sense_ReadingAgrees(gate->prev_n_mv, in->remote_n_mv)) {
            if (gate->streak < SENSE_OK_STREAK) {
                gate->streak++;
            }
        } else {
            gate->streak = 1U;
        }
        gate->prev_local_mv = in->local_mv;
        gate->prev_p_mv = in->remote_p_mv;
        gate->prev_n_mv = in->remote_n_mv;
        gate->have_prev = true;
        if (gate->streak >= SENSE_OK_STREAK) {
            gate->closed = true;
        }
        step.code = SENSE_OK;
        step.closed = gate->closed;
        step.latched = false;
        return step;
    }

    gate->streak = SENSE_OK_STREAK;
    gate->prev_local_mv = in->local_mv;
    gate->prev_p_mv = in->remote_p_mv;
    gate->prev_n_mv = in->remote_n_mv;
    gate->have_prev = true;

    if (in->compare_setpoint &&
        (!Sense_VdMatchesSetpoint(in->remote_p_mv, in->remote_n_mv,
                                  in->setpoint_mv))) {
        if (gate->cv_streak < SENSE_OK_STREAK) {
            gate->cv_streak++;
        }
        gate->code = SENSE_CV_VD;
        if (gate->cv_streak >= SENSE_OK_STREAK) {
            Sense_GateTrip(gate, &step, SENSE_CV_VD);
        } else {
            step.code = SENSE_CV_VD;
            step.closed = true;
            step.latched = false;
        }
        return step;
    }

    gate->cv_streak = 0U;
    step.code = SENSE_OK;
    step.closed = true;
    step.latched = false;
    return step;
}

typedef void (*SensePermitOffFn)(void *ctx);
typedef void (*SenseRelayFn)(void *ctx, bool closed);

/*
 * A critical step drops the output (PERMIT pin included) and only then opens
 * K1. Host OFF has drop_permit clear, so this only moves the relay.
 */
static inline void Sense_Commit(const SenseStep *step, bool relay_closed,
                               void *ctx, SensePermitOffFn permit_off,
                               SenseRelayFn drive)
{
    if ((step->drop_permit) && (permit_off != NULL)) {
        permit_off(ctx);
    }
    if ((drive != NULL) && (relay_closed != step->closed)) {
        drive(ctx, step->closed);
    }
}

#endif /* SENSE_CHECK_H */
