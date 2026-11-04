#ifndef BMS_LOWBATT_POLICY_H
#define BMS_LOWBATT_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Host low-battery policy for the 5S LFP pack.
 *
 * The AFE's CUV (2.48 V) is the last-resort floor: once it opens DSG the host
 * loses power, the LFP cell rebounds above the CUV recovery (2.68 V), DSG
 * closes again and the board restarts in a loop.  The host therefore stops
 * the load first and puts the AFE into SHUTDOWN while the cells still have
 * enough rebound for a few more starts:
 *
 *   LOW    min cell < warn_mv (3.00 V) while not charging (clears at +50 mV)
 *   EMPTY  min cell < cutoff_mv (2.80 V) for cutoff_hold_ms while not charging,
 *          or, right after a start, min cell < boot_min_mv (2.95 V) with no
 *          charging current seen within boot_window_ms.
 *          EMPTY is shown for notice_ms, then the host shuts the pack down.
 *
 * Charging current (≥ charge_ma) cancels everything.  After one shutdown
 * attempt the host stays up only when externally powered; it does not retry
 * until charging is seen again.
 */

#define BMS_LOWBATT_FLAG_LOW    0x200U   /* joins BQ76922 fault_flags on the link */
#define BMS_LOWBATT_FLAG_EMPTY  0x400U

typedef struct {
    int16_t warn_mv;
    int16_t warn_hyst_mv;
    int16_t cutoff_mv;
    uint32_t cutoff_hold_ms;
    int16_t boot_min_mv;
    uint32_t boot_window_ms;
    uint32_t notice_ms;
    int32_t charge_ma;
} BmsLowBatt_Config_t;

typedef enum {
    BMS_LB_RUN = 0,
    BMS_LB_NOTICE,     /* EMPTY shown, shutdown at notice_until_ms */
    BMS_LB_DONE,       /* shutdown sent; host still alive = external power */
} BmsLowBatt_Phase_t;

typedef struct {
    BmsLowBatt_Phase_t phase;
    bool started;
    bool boot_done;
    bool charge_seen;
    bool low;
    bool below_cut;
    uint32_t start_ms;
    uint32_t below_cut_since_ms;
    uint32_t notice_until_ms;
} BmsLowBatt_State_t;

static inline void BmsLowBatt_DefaultConfig(BmsLowBatt_Config_t *c)
{
    c->warn_mv = 3000;
    c->warn_hyst_mv = 50;
    c->cutoff_mv = 2800;
    c->cutoff_hold_ms = 10000U;
    c->boot_min_mv = 2950;
    c->boot_window_ms = 15000U;
    c->notice_ms = 3000U;
    c->charge_ma = 100;
}

/* Pack disappeared (SHUTDOWN or I2C loss): the next sample is a fresh start. */
static inline void BmsLowBatt_PackLost(BmsLowBatt_State_t *s)
{
    bool keep_done = (s->phase == BMS_LB_DONE);

    *s = (BmsLowBatt_State_t){0};
    /* Host kept running through SHUTDOWN = external power that does not
     * charge.  Do not cycle the AFE again until charging is seen. */
    if (keep_done) {
        s->phase = BMS_LB_DONE;
    }
}

/*
 * One valid AFE sample.  Returns true exactly once when the host must run
 * the orderly shutdown now.
 */
static inline bool BmsLowBatt_Step(const BmsLowBatt_Config_t *c,
                                   BmsLowBatt_State_t *s,
                                   uint32_t now_ms,
                                   int16_t min_cell_mv,
                                   int32_t current_ma)
{
    bool charging = (current_ma >= c->charge_ma);

    if (!s->started) {
        s->started = true;
        s->start_ms = now_ms;
    }
    if (charging) {
        s->charge_seen = true;
        s->low = false;
        s->below_cut = false;
        if (s->phase != BMS_LB_RUN) {
            s->phase = BMS_LB_RUN;
        }
        return false;
    }

    if (min_cell_mv < c->warn_mv) {
        s->low = true;
    } else if (min_cell_mv >= (int16_t)(c->warn_mv + c->warn_hyst_mv)) {
        s->low = false;
    }

    if (s->phase == BMS_LB_DONE) {
        return false;
    }
    if (s->phase == BMS_LB_NOTICE) {
        if ((int32_t)(now_ms - s->notice_until_ms) >= 0) {
            s->phase = BMS_LB_DONE;
            return true;
        }
        return false;
    }

    if (!s->boot_done && ((now_ms - s->start_ms) >= c->boot_window_ms)) {
        s->boot_done = true;
        if (!s->charge_seen && (min_cell_mv < c->boot_min_mv)) {
            s->phase = BMS_LB_NOTICE;
            s->notice_until_ms = now_ms + c->notice_ms;
            return false;
        }
    }

    if (min_cell_mv < c->cutoff_mv) {
        if (!s->below_cut) {
            s->below_cut = true;
            s->below_cut_since_ms = now_ms;
        } else if ((now_ms - s->below_cut_since_ms) >= c->cutoff_hold_ms) {
            s->phase = BMS_LB_NOTICE;
            s->notice_until_ms = now_ms + c->notice_ms;
        }
    } else {
        s->below_cut = false;
    }
    return false;
}

static inline uint32_t BmsLowBatt_Flags(const BmsLowBatt_State_t *s)
{
    uint32_t f = 0U;

    if (s->low) f |= BMS_LOWBATT_FLAG_LOW;
    if (s->phase != BMS_LB_RUN) f |= BMS_LOWBATT_FLAG_EMPTY;
    return f;
}

#endif /* BMS_LOWBATT_POLICY_H */
