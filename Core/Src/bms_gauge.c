#include "bms_gauge.h"
#include "bms_board.h"

#include <stddef.h>
#include <string.h>

/*
 * Internal charge unit: 2^-32 mAh (the BQ76922 DASTATUS6 resolution), kept
 * in int64.  6.7 Ah ≈ 2.9e13 units — far from overflow, no rounding drift.
 */
#define Q_ONE_MAH          (1LL << 32)
#define Q_FROM_MAH(m)      ((int64_t)(m) * Q_ONE_MAH)
#define Q_TO_MAH(q)        ((int32_t)((q) / Q_ONE_MAH))

#define PASSQ_JUMP_MAX_MAH 2000      /* one sample can never move this much */
#define LEARN_MIN_DSOC     500U      /* ≥ 50 % between two anchors */
#define LEARN_MAX_STEP_PCT 5         /* FCC moves ≤ 5 % per update */
#define FCC_MIN_PCT        50
#define FCC_MAX_PCT        110
#define RCELL_DEFAULT_X10  250       /* 25 mΩ per cell incl. interconnect */
#define RCELL_MIN_X10      20
#define RCELL_MAX_X10      1000
#define RSTEP_MIN_MA       1000      /* current step for an R estimate */
#define KNEE_HOLD_MS       5000U
#define IAVG_TAU_MS        60000U
#define BOOT_OCV_MAX_DIFF  150U      /* permille: larger = trust voltage */

typedef struct {
    int16_t mv;
    uint16_t permille;
} OcvPoint_t;

/* Relaxed OCV per cell, 25 °C, typical LFP (graphite/LiFePO4).  The
 * discharge branch sits ~10–20 mV below the charge branch on the plateau
 * (hysteresis), so the branch is chosen by the last current direction. */
static const OcvPoint_t s_ocv_dsg[] = {
    { 2500,    0 }, { 2900,   20 }, { 3050,   50 }, { 3180,  100 },
    { 3210,  150 }, { 3240,  200 }, { 3265,  300 }, { 3280,  400 },
    { 3290,  500 }, { 3300,  600 }, { 3310,  700 }, { 3320,  800 },
    { 3330,  900 }, { 3340,  950 }, { 3360,  980 }, { 3400, 1000 },
};
static const OcvPoint_t s_ocv_chg[] = {
    { 2600,    0 }, { 3150,   50 }, { 3220,  100 }, { 3270,  200 },
    { 3290,  300 }, { 3310,  500 }, { 3330,  700 }, { 3340,  900 },
    { 3360,  950 }, { 3400,  980 }, { 3450, 1000 },
};
/* OCV is trusted for an absolute value only at the steep ends. */
#define OCV_STEEP_HIGH_MV  3360
#define OCV_STEEP_LOW_MV   3220
#define OCV_PLATEAU_LO_MV  3265
#define OCV_PLATEAU_HI_MV  3330

typedef struct {
    BmsGauge_Config_t cfg;
    BmsGauge_Persist_t p;
    int64_t q;                 /* remaining charge */
    int64_t q_saved;           /* q at the last persist snapshot */
    int64_t chg_frac;          /* residuals for the lifetime totals */
    int64_t dsg_frac;
    /* passQ baseline */
    bool have_passq;
    int64_t passq_last;
    uint32_t passq_time_last;
    /* learning */
    bool anchor_valid;
    uint16_t anchor_permille;
    int64_t anchor_counted;    /* net charge since that anchor */
    /* state */
    bool initialized;          /* first sample seen */
    bool have_value;
    uint16_t flags;
    bool after_charge;         /* hysteresis branch */
    uint32_t last_ms;
    bool have_time;
    uint32_t rest_since_ms;
    bool resting;
    bool relax_used;           /* OCV anchor already applied this rest */
    uint32_t taper_since_ms;
    bool full_latched;
    uint32_t knee1_since_ms;
    uint32_t knee2_since_ms;
    bool knee1_done;
    bool knee2_done;
    int32_t prev_i_ma;
    int32_t prev_vavg_mv;
    bool have_prev;
    int64_t iavg_x1000;        /* EMA of current, mA × 1000 */
    int16_t ocv_avg_mv;
    bool restored;
    bool restore_clean;
    bool boot_check_done;
} Gauge_t;

static Gauge_t g;

/* ------------------------------------------------------------ helpers -- */
static int64_t g_abs64(int64_t v) { return (v < 0) ? -v : v; }
static int32_t g_abs32(int32_t v) { return (v < 0) ? -v : v; }

static uint16_t ocv_lookup(const OcvPoint_t *t, uint8_t n, int16_t mv)
{
    uint8_t i;

    if (mv <= t[0].mv) {
        return t[0].permille;
    }
    if (mv >= t[n - 1U].mv) {
        return t[n - 1U].permille;
    }
    for (i = 1U; i < n; i++) {
        if (mv <= t[i].mv) {
            int32_t span = (int32_t)t[i].mv - (int32_t)t[i - 1U].mv;
            int32_t rise = (int32_t)t[i].permille - (int32_t)t[i - 1U].permille;
            int32_t num = ((int32_t)mv - (int32_t)t[i - 1U].mv) * rise;

            return (uint16_t)((int32_t)t[i - 1U].permille + (num / span));
        }
    }
    return t[n - 1U].permille;
}

uint16_t BmsGauge_OcvToPermille(int16_t cell_mv, bool after_charge)
{
    if (after_charge) {
        return ocv_lookup(s_ocv_chg, (uint8_t)(sizeof(s_ocv_chg) / sizeof(s_ocv_chg[0])), cell_mv);
    }
    return ocv_lookup(s_ocv_dsg, (uint8_t)(sizeof(s_ocv_dsg) / sizeof(s_ocv_dsg[0])), cell_mv);
}

static int64_t q_full(void)
{
    return Q_FROM_MAH(g.p.fcc_mah);
}

static void clamp_q(void)
{
    if (g.q < 0) {
        g.q = 0;
    } else if (g.q > q_full()) {
        g.q = q_full();
    }
}

static uint16_t soc_permille(void)
{
    int64_t full = q_full();

    if (!g.have_value || (full <= 0)) {
        return BMS_GAUGE_INVALID_PERMILLE;
    }
    return (uint16_t)((g.q * 1000LL + (full / 2)) / full);
}

static void set_soc(uint16_t permille)
{
    if (permille > 1000U) {
        permille = 1000U;
    }
    g.q = (q_full() * (int64_t)permille) / 1000LL;
    g.have_value = true;
    g.flags |= BMS_GAUGE_F_VALID;
}

static void mark_dirty(void)
{
    g.flags |= BMS_GAUGE_F_DIRTY;
}

static void learn_fcc(uint16_t new_permille)
{
    int32_t dsoc;
    int64_t counted;
    int64_t meas;
    int64_t fcc = g.p.fcc_mah;
    int64_t step;
    int64_t lo = ((int64_t)g.cfg.design_mah * FCC_MIN_PCT) / 100;
    int64_t hi = ((int64_t)g.cfg.design_mah * FCC_MAX_PCT) / 100;
    int64_t w_permille;

    if (!g.anchor_valid) {
        return;
    }
    dsoc = (int32_t)new_permille - (int32_t)g.anchor_permille;
    counted = g.anchor_counted;
    /* Direction must agree: SOC up only by charging and vice versa. */
    if ((dsoc > 0 && counted <= 0) || (dsoc < 0 && counted >= 0)) {
        return;
    }
    if ((uint32_t)g_abs32(dsoc) < LEARN_MIN_DSOC) {
        return;
    }
    meas = (g_abs64(counted) * 1000LL) / ((int64_t)g_abs32(dsoc) * Q_ONE_MAH);
    if ((meas < lo / 2) || (meas > hi * 2)) {
        return; /* nonsense (passQ glitch) */
    }
    /* Weight by span: 50 % SOC → 0.25, 100 % → 0.5. */
    w_permille = (int64_t)g_abs32(dsoc) / 2;
    step = ((meas - fcc) * w_permille) / 1000LL;
    if (step > (fcc * LEARN_MAX_STEP_PCT) / 100) {
        step = (fcc * LEARN_MAX_STEP_PCT) / 100;
    } else if (step < -(fcc * LEARN_MAX_STEP_PCT) / 100) {
        step = -(fcc * LEARN_MAX_STEP_PCT) / 100;
    }
    fcc += step;
    if (fcc < lo) {
        fcc = lo;
    } else if (fcc > hi) {
        fcc = hi;
    }
    if (fcc != g.p.fcc_mah) {
        /* Keep SOC, rescale the remaining charge to the new capacity. */
        uint16_t soc = soc_permille();

        g.p.fcc_mah = (int32_t)fcc;
        if (soc != BMS_GAUGE_INVALID_PERMILLE) {
            set_soc(soc);
        }
    }
    g.p.flags |= BMS_GAUGE_F_FCC_LEARNED;
    if (g.p.learn_count < 255U) {
        g.p.learn_count++;
    }
    mark_dirty();
}

static void apply_anchor(BmsAnchor_t kind, uint16_t permille, bool trusted)
{
    if (trusted) {
        learn_fcc(permille);
    }
    set_soc(permille);
    g.p.last_anchor = (uint8_t)kind;
    if (trusted) {
        g.flags &= (uint16_t)~BMS_GAUGE_F_UNCERTAIN;
        g.anchor_valid = true;
        g.anchor_permille = permille;
        g.anchor_counted = 0;
    } else {
        g.flags |= BMS_GAUGE_F_UNCERTAIN;
        /* An estimate is not a basis for learning. */
        g.anchor_valid = false;
    }
    mark_dirty();
}

static int16_t avg_cell_mv(const int16_t *cell, uint8_t n, int16_t *min_out)
{
    int32_t sum = 0;
    int16_t min = 32767;
    uint8_t i;

    for (i = 0U; i < n; i++) {
        sum += cell[i];
        if (cell[i] < min) {
            min = cell[i];
        }
    }
    *min_out = min;
    return (int16_t)(sum / (int32_t)n);
}

/* mV to add for IR compensation: |I| × R (R per cell in 0.1 mΩ). */
static int16_t ir_mv(int32_t i_ma)
{
    int64_t mv = ((int64_t)i_ma * (int64_t)g.p.rcell_mohm_x10) / 10000LL;

    return (int16_t)mv;
}

/* ----------------------------------------------------------- public ---- */
void BmsGauge_DefaultConfig(BmsGauge_Config_t *cfg)
{
    cfg->design_mah = 6700;
    cfg->series = 5U;
    cfg->taper_ma = 335;            /* C/20 */
    cfg->full_cell_mv = (int16_t)(BMS_CHARGE_TARGET_CELL_MV - 50U);
    cfg->full_hold_ms = 60000U;
    cfg->rest_ma = 400;             /* board standby draw is a few 100 mA */
    cfg->relax_ms = 30U * 60U * 1000U;
    cfg->low_cell_mv = 3050;
    cfg->low_permille = 60U;
    cfg->empty_cell_mv = 2850;
    cfg->empty_permille = 10U;
}

void BmsGauge_Init(const BmsGauge_Config_t *cfg)
{
    memset(&g, 0, sizeof(g));
    g.cfg = *cfg;
    g.p.fcc_mah = cfg->design_mah;
    g.p.rcell_mohm_x10 = RCELL_DEFAULT_X10;
    g.p.soc_permille = BMS_GAUGE_INVALID_PERMILLE;
    g.flags = BMS_GAUGE_F_UNCERTAIN;
}

void BmsGauge_Restore(const BmsGauge_Persist_t *p, bool clean)
{
    int64_t lo = ((int64_t)g.cfg.design_mah * FCC_MIN_PCT) / 100;
    int64_t hi = ((int64_t)g.cfg.design_mah * FCC_MAX_PCT) / 100;

    if (p == NULL) {
        return;
    }
    g.p = *p;
    if ((g.p.fcc_mah < lo) || (g.p.fcc_mah > hi)) {
        g.p.fcc_mah = g.cfg.design_mah;
        g.p.flags &= (uint16_t)~BMS_GAUGE_F_FCC_LEARNED;
    }
    if ((g.p.rcell_mohm_x10 < RCELL_MIN_X10) || (g.p.rcell_mohm_x10 > RCELL_MAX_X10)) {
        g.p.rcell_mohm_x10 = RCELL_DEFAULT_X10;
    }
    g.q = ((int64_t)g.p.remaining_mah_x10 * Q_ONE_MAH) / 10LL;
    g.have_value = true;
    clamp_q();
    g.q_saved = g.q;
    g.flags = (uint16_t)(BMS_GAUGE_F_VALID | BMS_GAUGE_F_RESTORED |
                         (clean ? BMS_GAUGE_F_CLEAN_RESTORE : 0U));
    /* A non-clean restore lost whatever happened after the last snapshot. */
    if (!clean || ((p->flags & BMS_GAUGE_F_UNCERTAIN) != 0U)) {
        g.flags |= BMS_GAUGE_F_UNCERTAIN;
    }
    g.restored = true;
    g.restore_clean = clean;
    g.anchor_valid = false;
}

void BmsGauge_ResetPack(void)
{
    BmsGauge_Config_t cfg = g.cfg;
    uint16_t soc = soc_permille();
    bool had = g.have_value;
    bool hp = g.have_passq;
    int64_t pl = g.passq_last;
    uint32_t pt = g.passq_time_last;

    BmsGauge_Init(&cfg);
    if (had && soc != BMS_GAUGE_INVALID_PERMILLE) {
        set_soc(soc);
    }
    g.have_passq = hp;
    g.passq_last = pl;
    g.passq_time_last = pt;
    g.initialized = true;
    g.boot_check_done = true;
    g.flags |= BMS_GAUGE_F_UNCERTAIN;
    mark_dirty();
}

void BmsGauge_NotePackLost(void)
{
    g.have_passq = false;
    g.have_prev = false;
    g.resting = false;
    g.relax_used = false;
    /* The next pack may be a different one: re-check against voltage. */
    g.boot_check_done = false;
}

static void boot_check(int16_t ocv, int32_t i_ma)
{
    uint16_t est = BmsGauge_OcvToPermille(ocv, false);
    uint16_t cur = soc_permille();

    g.boot_check_done = true;
    if (!g.have_value || (cur == BMS_GAUGE_INVALID_PERMILLE)) {
        apply_anchor(BMS_ANCHOR_BOOT_OCV, est, false);
        return;
    }
    if (g_abs32(i_ma) >= g.cfg.rest_ma) {
        return; /* loaded: voltage is not OCV, keep the stored value */
    }
    if ((ocv >= OCV_STEEP_HIGH_MV) || (ocv <= OCV_STEEP_LOW_MV)) {
        int32_t d = (int32_t)est - (int32_t)cur;

        if ((uint32_t)g_abs32(d) > BOOT_OCV_MAX_DIFF) {
            /* Pack was charged/discharged elsewhere, or swapped. */
            apply_anchor(BMS_ANCHOR_BOOT_OCV, est, false);
        }
        return;
    }
    if ((ocv >= OCV_PLATEAU_LO_MV) && (ocv <= OCV_PLATEAU_HI_MV)) {
        if (cur < 150U) {
            apply_anchor(BMS_ANCHOR_PLATEAU, 200U, false);
        } else if (cur > 960U) {
            apply_anchor(BMS_ANCHOR_PLATEAU, 900U, false);
        }
    }
}

static void count_passq(const BmsGauge_Input_t *in)
{
    int64_t d;

    if (!in->passq_valid) {
        g.have_passq = false;
        return;
    }
    {
        /* Signed 64-bit counter in 2^-32 mAh, exactly as the AFE holds it. */
        int64_t raw = (int64_t)(((uint64_t)(uint32_t)in->passq_whole << 32) | in->passq_frac);

        if (!g.have_passq || (in->passq_time_s < g.passq_time_last)) {
            g.passq_last = raw;
            g.passq_time_last = in->passq_time_s;
            g.have_passq = true;
            return;
        }
        d = raw - g.passq_last;
        g.passq_last = raw;
        g.passq_time_last = in->passq_time_s;
    }
    if (g_abs64(d) > Q_FROM_MAH(PASSQ_JUMP_MAX_MAH)) {
        return; /* AFE reset between two reads — rebaselined above */
    }
    if (d > 0) {
        g.chg_frac += d;
        while (g.chg_frac >= Q_ONE_MAH) {
            g.chg_frac -= Q_ONE_MAH;
            g.p.chg_total_mah++;
        }
    } else if (d < 0) {
        g.dsg_frac -= d;
        while (g.dsg_frac >= Q_ONE_MAH) {
            g.dsg_frac -= Q_ONE_MAH;
            g.p.dsg_total_mah++;
        }
    }
    if (g.have_value) {
        g.q += d;
        clamp_q();
    }
    g.anchor_counted += d;
}

static void update_resistance(int32_t i_ma, int16_t vavg)
{
    if (g.have_prev) {
        int32_t di = i_ma - g.prev_i_ma;
        int32_t dv = (int32_t)vavg - g.prev_vavg_mv;

        if (g_abs32(di) >= RSTEP_MIN_MA) {
            /* Discharge = negative current: V drops when I gets more negative,
             * so R = ΔV/ΔI is positive. */
            int64_t r_x10 = ((int64_t)dv * 10000LL) / (int64_t)di;

            if ((r_x10 >= RCELL_MIN_X10) && (r_x10 <= RCELL_MAX_X10)) {
                int32_t r = g.p.rcell_mohm_x10;

                r += (int32_t)((r_x10 - r) / 8);
                g.p.rcell_mohm_x10 = (uint16_t)r;
            }
        }
    }
    g.prev_i_ma = i_ma;
    g.prev_vavg_mv = vavg;
    g.have_prev = true;
}

void BmsGauge_Update(const BmsGauge_Input_t *in)
{
    uint32_t dt = 0U;
    int16_t vavg;
    int16_t vmin;
    int16_t ocv;
    int16_t ocv_min;
    int32_t i;
    uint16_t soc;

    if ((in == NULL) || !in->afe_valid || (in->cell_mv == NULL) || (g.cfg.series == 0U)) {
        return;
    }
    i = in->current_ma;
    if (g.have_time) {
        dt = in->now_ms - g.last_ms;
    }
    g.last_ms = in->now_ms;
    g.have_time = true;

    count_passq(in);

    vavg = avg_cell_mv(in->cell_mv, g.cfg.series, &vmin);
    update_resistance(i, vavg);
    /* OCV estimate: add back the IR drop (I<0 on discharge → +). */
    ocv = (int16_t)(vavg - ir_mv(i));
    ocv_min = (int16_t)(vmin - ir_mv(i));
    g.ocv_avg_mv = ocv;

    /* Average current for time estimates. */
    if (dt > 0U) {
        int64_t target = (int64_t)i * 1000LL;
        uint32_t k = (dt > IAVG_TAU_MS) ? IAVG_TAU_MS : dt;

        g.iavg_x1000 += ((target - g.iavg_x1000) * (int64_t)k) / (int64_t)IAVG_TAU_MS;
    } else {
        g.iavg_x1000 = (int64_t)i * 1000LL;
    }

    /* Direction / rest. */
    g.flags &= (uint16_t)~(BMS_GAUGE_F_CHARGING | BMS_GAUGE_F_DISCHARGING |
                           BMS_GAUGE_F_RESTING | BMS_GAUGE_F_RELAXED);
    if (i > g.cfg.rest_ma) {
        g.flags |= BMS_GAUGE_F_CHARGING;
        g.after_charge = true;
        g.resting = false;
    } else if (i < -g.cfg.rest_ma) {
        g.flags |= BMS_GAUGE_F_DISCHARGING;
        g.after_charge = false;
        g.resting = false;
    } else {
        if (!g.resting) {
            g.resting = true;
            g.relax_used = false;
            g.rest_since_ms = in->now_ms;
        }
        g.flags |= BMS_GAUGE_F_RESTING;
        if ((in->now_ms - g.rest_since_ms) >= g.cfg.relax_ms) {
            g.flags |= BMS_GAUGE_F_RELAXED;
        }
    }

    if (!g.initialized || !g.boot_check_done) {
        g.initialized = true;
        boot_check(ocv, i);
    }

    /* ---- top anchor: CV + taper ------------------------------------- */
    if ((i > 10) && (i < g.cfg.taper_ma) && (vavg >= g.cfg.full_cell_mv)) {
        if (g.taper_since_ms == 0U) {
            g.taper_since_ms = in->now_ms | 1U;
        }
        if (!g.full_latched &&
            ((in->now_ms - g.taper_since_ms) >= g.cfg.full_hold_ms)) {
            apply_anchor(BMS_ANCHOR_FULL, 1000U, true);
            g.full_latched = true;
            g.after_charge = true;
        }
    } else {
        g.taper_since_ms = 0U;
    }
    soc = soc_permille();
    if (g.full_latched && (soc != BMS_GAUGE_INVALID_PERMILLE) && (soc < 970U)) {
        g.full_latched = false;
    }

    /* ---- bottom knees under load (IR compensated, weakest cell) ------ */
    if (i < -g.cfg.rest_ma) {
        if (ocv_min <= g.cfg.empty_cell_mv) {
            if (g.knee2_since_ms == 0U) {
                g.knee2_since_ms = in->now_ms | 1U;
            }
            if (!g.knee2_done && ((in->now_ms - g.knee2_since_ms) >= KNEE_HOLD_MS)) {
                apply_anchor(BMS_ANCHOR_EMPTY, g.cfg.empty_permille, true);
                g.knee2_done = true;
                g.knee1_done = true;
            }
        } else {
            g.knee2_since_ms = 0U;
        }
        if (ocv_min <= g.cfg.low_cell_mv) {
            if (g.knee1_since_ms == 0U) {
                g.knee1_since_ms = in->now_ms | 1U;
            }
            if (!g.knee1_done && ((in->now_ms - g.knee1_since_ms) >= KNEE_HOLD_MS)) {
                apply_anchor(BMS_ANCHOR_EMPTY, g.cfg.low_permille, true);
                g.knee1_done = true;
            }
        } else {
            g.knee1_since_ms = 0U;
        }
    } else {
        g.knee1_since_ms = 0U;
        g.knee2_since_ms = 0U;
    }
    if ((g.flags & BMS_GAUGE_F_CHARGING) != 0U) {
        soc = soc_permille();
        if ((soc != BMS_GAUGE_INVALID_PERMILLE) && (soc > 200U)) {
            g.knee1_done = false;
            g.knee2_done = false;
        }
    }

    /* ---- relaxed OCV at the steep ends ------------------------------- */
    if (((g.flags & BMS_GAUGE_F_RELAXED) != 0U) && !g.relax_used) {
        uint16_t est = BmsGauge_OcvToPermille(ocv, g.after_charge);

        g.relax_used = true;
        if (ocv >= OCV_STEEP_HIGH_MV) {
            apply_anchor(BMS_ANCHOR_OCV_HIGH, est, true);
        } else if (ocv <= OCV_STEEP_LOW_MV) {
            apply_anchor(BMS_ANCHOR_OCV_LOW, est, true);
        } else if ((ocv >= OCV_PLATEAU_LO_MV) && (ocv <= OCV_PLATEAU_HI_MV)) {
            soc = soc_permille();
            if (soc != BMS_GAUGE_INVALID_PERMILLE) {
                if (soc < 150U) {
                    apply_anchor(BMS_ANCHOR_PLATEAU, 200U, false);
                } else if ((soc > 990U) && (ocv <= 3310)) {
                    apply_anchor(BMS_ANCHOR_PLATEAU, 900U, false);
                }
            }
        }
    }

    soc = soc_permille();
    g.flags &= (uint16_t)~(BMS_GAUGE_F_LOW | BMS_GAUGE_F_EMPTY | BMS_GAUGE_F_FULL);
    if (soc != BMS_GAUGE_INVALID_PERMILLE) {
        if (soc <= 100U || g.knee1_done) {
            g.flags |= BMS_GAUGE_F_LOW;
        }
        if (g.knee2_done || soc == 0U) {
            g.flags |= BMS_GAUGE_F_EMPTY;
        }
    }
    if (g.full_latched) {
        g.flags |= BMS_GAUGE_F_FULL;
    }
    g.p.soc_permille = soc;
}

void BmsGauge_GetOutput(BmsGauge_Output_t *out)
{
    int32_t iavg = (int32_t)(g.iavg_x1000 / 1000LL);
    int32_t rem = Q_TO_MAH(g.q);

    memset(out, 0, sizeof(*out));
    out->soc_permille = soc_permille();
    out->flags = (uint16_t)(g.flags | (g.p.flags & BMS_GAUGE_F_FCC_LEARNED));
    out->remaining_mah = g.have_value ? rem : 0;
    out->fcc_mah = g.p.fcc_mah;
    out->soh_permille = (uint16_t)(((int64_t)g.p.fcc_mah * 1000LL) / g.cfg.design_mah);
    out->cycles_x100 = (uint16_t)(((uint64_t)g.p.dsg_total_mah * 100ULL) /
                                  (uint64_t)g.cfg.design_mah);
    out->rcell_mohm_x10 = (int16_t)g.p.rcell_mohm_x10;
    out->ocv_avg_mv = g.ocv_avg_mv;
    out->tte_min = 0xFFFFU;
    out->ttf_min = 0xFFFFU;
    if (g.have_value && (iavg < -g.cfg.rest_ma)) {
        int64_t m = ((int64_t)rem * 60LL) / (int64_t)(-iavg);

        out->tte_min = (uint16_t)((m > 0xFFFE) ? 0xFFFE : m);
    } else if (g.have_value && (iavg > g.cfg.rest_ma)) {
        int64_t m = ((int64_t)(g.p.fcc_mah - rem) * 60LL) / (int64_t)iavg;

        out->ttf_min = (uint16_t)((m > 0xFFFE) ? 0xFFFE : ((m < 0) ? 0 : m));
    }
    out->last_anchor = g.p.last_anchor;
    out->learn_count = g.p.learn_count;
    out->learn_delta_mah = g.anchor_valid ? Q_TO_MAH(g.anchor_counted) : 0;
    out->chg_total_mah = g.p.chg_total_mah;
    out->dsg_total_mah = g.p.dsg_total_mah;
}

void BmsGauge_GetPersist(BmsGauge_Persist_t *p)
{
    g.p.remaining_mah_x10 = (int32_t)((g.q * 10LL) / Q_ONE_MAH);
    g.p.soc_permille = soc_permille();
    g.p.flags = (uint16_t)((g.p.flags & BMS_GAUGE_F_FCC_LEARNED) |
                           (g.flags & BMS_GAUGE_F_UNCERTAIN));
    *p = g.p;
    g.q_saved = g.q;
    g.flags &= (uint16_t)~BMS_GAUGE_F_DIRTY;
}

bool BmsGauge_IsDirty(void)
{
    return (g.flags & BMS_GAUGE_F_DIRTY) != 0U;
}

int32_t BmsGauge_UnsavedMah(void)
{
    return Q_TO_MAH(g_abs64(g.q - g.q_saved));
}
