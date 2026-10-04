#include "bms_soc.h"

#include <stddef.h>

#define BMS_SOC_DT_MAX_MS            5000U
#define BMS_SOC_MAH_SCALE            3600000LL
/* AFE drops a host balance mask after 20 s. Refresh sooner. RAM only. */
#define BMS_SOC_BALANCE_REFRESH_MS   5000U
#define BMS_SOC_BALANCE_CHG_MA       100
#define BMS_SOC_CAP_MIN_MAH          100
#define BMS_SOC_CAP_MAX_MAH          200000
#define BMS_SOC_LEARN_MIN_PERMILLE   400
#define BMS_SOC_LEARN_MIN_MAH        100
/* Live Li-ion correction time constant. Ends only; the flat middle is ignored. */
#define BMS_SOC_END_TAU_MS           180000U

#define BMS_SOC_LIION_REST_MA        40
#define BMS_SOC_LIION_REST_MS        (30U * 60U * 1000U)
/* Capacity anchors are the upper and lower knees, not empty and absolute full. */
#define BMS_SOC_LIION_ANCHOR_HIGH_MV 4000
#define BMS_SOC_LIION_ANCHOR_LOW_MV  3600
#define BMS_SOC_LIION_TRACK_LOW_MV   3450
#define BMS_SOC_LIION_TRACK_HIGH_MV  4000
#define BMS_SOC_LIION_BAL_START_MV   4000
#define BMS_SOC_LIION_BAL_STOP_MV    3900

#define BMS_SOC_LFP_REST_MA          30
#define BMS_SOC_LFP_REST_MS          (45U * 60U * 1000U)
#define BMS_SOC_LFP_ANCHOR_HIGH_MV   3400
#define BMS_SOC_LFP_ANCHOR_LOW_MV    3200
#define BMS_SOC_LFP_BAL_MV           3450

#define BMS_SOC_BAL_START_DELTA_MV   15
#define BMS_SOC_BAL_STOP_DELTA_MV    8

typedef struct {
    int16_t mv;
    uint16_t permille;
} BmsSoc_OcvPoint_t;

/* Tuning starts. Piecewise linear, clamped at the ends. */
static const BmsSoc_OcvPoint_t s_liion_ocv[] = {
    { 3000, 0 },
    { 3300, 50 },
    { 3400, 100 },
    { 3600, 300 },
    { 3700, 450 },
    { 3800, 600 },
    { 3900, 720 },
    { 4000, 820 },
    { 4100, 950 },
    { 4200, 1000 }
};

static const BmsSoc_OcvPoint_t s_lfp_ocv[] = {
    { 2500, 0 },
    { 3000, 50 },
    { 3200, 100 },
    { 3250, 200 },
    { 3300, 500 },
    { 3350, 800 },
    { 3400, 950 },
    { 3450, 1000 }
};

typedef struct {
    BmsSoc_Chem_t chem;
    int32_t capacity_mah;
    bool learned;
    bool valid;
    bool have_time;
    bool rest_timing;
    bool resting;
    uint32_t last_ms;
    uint32_t rest_since_ms;
    bool have_passq;
    int32_t passq_mah;
    int64_t soc_mA_ms;
    bool anchor_valid;
    bool anchor_high;
    uint16_t anchor_permille;
    int32_t anchor_passq_mah;
    uint8_t balance_mask;
    bool balance_sent;
    uint16_t balance_sent_mask;
    uint32_t balance_sent_ms;
} BmsSoc_State_t;

static BmsSoc_State_t s;

static int64_t BmsSoc_Abs64(int64_t value)
{
    return (value < 0) ? -value : value;
}

static void BmsSoc_ClampSoc(void)
{
    int64_t full;

    if (s.capacity_mah <= 0) {
        s.soc_mA_ms = 0;
        return;
    }
    full = (int64_t)s.capacity_mah * 1000LL * 3600LL;
    if (s.soc_mA_ms < 0) {
        s.soc_mA_ms = 0;
    } else if (s.soc_mA_ms > full) {
        s.soc_mA_ms = full;
    }
}

static uint16_t BmsSoc_Permille(void)
{
    if (!s.valid || (s.capacity_mah <= 0)) {
        return BMS_SOC_INVALID_PERMILLE;
    }
    return (uint16_t)(s.soc_mA_ms / ((int64_t)s.capacity_mah * 3600LL));
}

static uint16_t BmsSoc_OcvPermille(int16_t mv)
{
    const BmsSoc_OcvPoint_t *table;
    uint8_t count;
    uint8_t i;

    if (s.chem == BMS_SOC_CHEM_LFP) {
        table = s_lfp_ocv;
        count = (uint8_t)(sizeof(s_lfp_ocv) / sizeof(s_lfp_ocv[0]));
    } else {
        table = s_liion_ocv;
        count = (uint8_t)(sizeof(s_liion_ocv) / sizeof(s_liion_ocv[0]));
    }
    if (mv <= table[0].mv) {
        return table[0].permille;
    }
    if (mv >= table[count - 1U].mv) {
        return table[count - 1U].permille;
    }
    for (i = 1U; i < count; i++) {
        if (mv <= table[i].mv) {
            int32_t span = (int32_t)table[i].mv - (int32_t)table[i - 1U].mv;
            int32_t rise = (int32_t)table[i].permille -
                           (int32_t)table[i - 1U].permille;
            int32_t num = ((int32_t)mv - (int32_t)table[i - 1U].mv) * rise;

            return (uint16_t)((int32_t)table[i - 1U].permille + (num / span));
        }
    }
    return table[count - 1U].permille;
}

static bool BmsSoc_MinUsedMv(const int16_t *cell_mv, uint8_t cell_count,
                             uint8_t used, int16_t *min_mv)
{
    uint8_t i;
    bool any = false;
    int16_t min = 32767;

    for (i = 0U; i < cell_count; i++) {
        if ((used & (uint8_t)(1U << i)) == 0U) {
            continue;
        }
        any = true;
        if (cell_mv[i] < min) {
            min = cell_mv[i];
        }
    }
    if (!any) {
        return false;
    }
    *min_mv = min;
    return true;
}

static void BmsSoc_Learn(uint16_t permille, bool high)
{
    int64_t dmah;
    int32_t dperm;
    int64_t cap;

    if (!s.anchor_valid || !s.have_passq || (high == s.anchor_high)) {
        return;
    }
    dperm = (int32_t)permille - (int32_t)s.anchor_permille;
    if (dperm < 0) {
        dperm = -dperm;
    }
    dmah = BmsSoc_Abs64((int64_t)s.passq_mah - (int64_t)s.anchor_passq_mah);
    if ((dperm < BMS_SOC_LEARN_MIN_PERMILLE) || (dmah <= BMS_SOC_LEARN_MIN_MAH)) {
        return;
    }
    cap = (dmah * 1000LL) / (int64_t)dperm;
    if (cap < BMS_SOC_CAP_MIN_MAH) {
        cap = BMS_SOC_CAP_MIN_MAH;
    } else if (cap > BMS_SOC_CAP_MAX_MAH) {
        cap = BMS_SOC_CAP_MAX_MAH;
    }
    s.capacity_mah = (int32_t)cap;
    s.learned = true;
}

/* Li-ion only. Seed at a steep end, then walk toward that curve. */
static void BmsSoc_TrackEnds(int16_t min_mv, uint32_t dt, bool snapped)
{
    uint16_t target;
    int32_t err;
    int32_t step;

    if ((s.chem != BMS_SOC_CHEM_LIION) || snapped) {
        return;
    }
    if ((min_mv > BMS_SOC_LIION_TRACK_LOW_MV) &&
        (min_mv < BMS_SOC_LIION_TRACK_HIGH_MV)) {
        return;
    }
    target = BmsSoc_OcvPermille(min_mv);
    if (!s.valid) {
        s.valid = true;
        s.soc_mA_ms = (int64_t)target * (int64_t)s.capacity_mah * 3600LL;
        return;
    }
    if ((dt == 0U) || (s.capacity_mah <= 0)) {
        return;
    }
    err = (int32_t)target - (int32_t)BmsSoc_Permille();
    step = (int32_t)(((int64_t)err * (int64_t)dt) / (int64_t)BMS_SOC_END_TAU_MS);
    if ((step == 0) && (err != 0)) {
        step = (err > 0) ? 1 : -1;
    }
    if ((err > 0) && (step > err)) {
        step = err;
    } else if ((err < 0) && (step < err)) {
        step = err;
    }
    s.soc_mA_ms += (int64_t)step * (int64_t)s.capacity_mah * 3600LL;
    BmsSoc_ClampSoc();
}

static void BmsSoc_QualifyRest(int16_t ocv_mv)
{
    uint16_t permille = BmsSoc_OcvPermille(ocv_mv);
    bool high;
    bool low;

    if (s.chem == BMS_SOC_CHEM_LFP) {
        high = (ocv_mv >= BMS_SOC_LFP_ANCHOR_HIGH_MV);
        low = (ocv_mv <= BMS_SOC_LFP_ANCHOR_LOW_MV);
    } else {
        high = (ocv_mv >= BMS_SOC_LIION_ANCHOR_HIGH_MV);
        low = (ocv_mv <= BMS_SOC_LIION_ANCHOR_LOW_MV);
    }
    if (high || low) {
        BmsSoc_Learn(permille, high);
    }
    s.valid = true;
    s.soc_mA_ms = (int64_t)permille * (int64_t)s.capacity_mah * 3600LL;
    if ((high || low) && s.have_passq) {
        s.anchor_valid = true;
        s.anchor_high = high;
        s.anchor_permille = permille;
        s.anchor_passq_mah = s.passq_mah;
    }
}

static uint8_t BmsSoc_BalanceMask(int16_t cc2_ma, const int16_t *cell_mv,
                                  uint8_t cell_count, uint8_t used)
{
    uint8_t i;
    uint8_t max_i = 0U;
    bool any = false;
    int16_t vmin = 32767;
    int16_t vmax = -32768;
    int16_t delta;
    int16_t start_mv;
    int16_t stop_mv;

    for (i = 0U; i < cell_count; i++) {
        if ((used & (uint8_t)(1U << i)) == 0U) {
            continue;
        }
        any = true;
        if (cell_mv[i] < vmin) {
            vmin = cell_mv[i];
        }
        if (cell_mv[i] > vmax) {
            vmax = cell_mv[i];
            max_i = i;
        }
    }
    if (!any || (cc2_ma <= BMS_SOC_BALANCE_CHG_MA)) {
        return 0U;
    }
    delta = (int16_t)(vmax - vmin);
    if (s.chem == BMS_SOC_CHEM_LFP) {
        start_mv = BMS_SOC_LFP_BAL_MV;
        stop_mv = BMS_SOC_LFP_BAL_MV;
    } else {
        start_mv = BMS_SOC_LIION_BAL_START_MV;
        stop_mv = BMS_SOC_LIION_BAL_STOP_MV;
    }
    if ((vmax < stop_mv) || (delta <= BMS_SOC_BAL_STOP_DELTA_MV)) {
        return 0U;
    }
    if ((s.balance_mask == 0U) &&
        ((vmax < start_mv) || (delta < BMS_SOC_BAL_START_DELTA_MV))) {
        return 0U;
    }
    return (uint8_t)(1U << max_i);
}

static void BmsSoc_Fill(bool passq_valid, int32_t passq_mah, BmsSoc_Result_t *out)
{
    uint8_t flags = 0U;

    if (s.valid) {
        flags |= BMS_SOC_FLAG_VALID;
    }
    if (s.resting) {
        flags |= BMS_SOC_FLAG_RESTING;
    }
    if (s.learned) {
        flags |= BMS_SOC_FLAG_LEARNED;
    }
    if (passq_valid) {
        flags |= BMS_SOC_FLAG_PASSQ_VALID;
    }
    if (s.balance_mask != 0U) {
        flags |= BMS_SOC_FLAG_BALANCE;
    }
    out->session_mah = 0;
    out->passq_mah = passq_valid ? passq_mah : 0;
    out->soc_permille = BmsSoc_Permille();
    out->flags = flags;
    out->balance_mask = s.balance_mask;
    out->balance_cells = s.balance_mask;
    out->balance_write = false;
    if ((uint16_t)s.balance_mask != s.balance_sent_mask) {
        if ((s.balance_mask != 0U) || s.balance_sent) {
            out->balance_write = true;
        }
    } else if ((s.balance_mask != 0U) &&
               ((uint32_t)(s.last_ms - s.balance_sent_ms) >=
                BMS_SOC_BALANCE_REFRESH_MS)) {
        out->balance_write = true;
    }
}

void BmsSoc_Reset(void)
{
    s.chem = BMS_SOC_CHEM_LIION;
    s.capacity_mah = BMS_SOC_NOMINAL_CELL_MAH;
    s.learned = false;
    s.valid = false;
    s.have_time = false;
    s.rest_timing = false;
    s.resting = false;
    s.last_ms = 0U;
    s.rest_since_ms = 0U;
    s.have_passq = false;
    s.passq_mah = 0;
    s.soc_mA_ms = 0;
    s.anchor_valid = false;
    s.anchor_high = false;
    s.anchor_permille = 0U;
    s.anchor_passq_mah = 0;
    s.balance_mask = 0U;
    s.balance_sent = false;
    s.balance_sent_mask = 0U;
    s.balance_sent_ms = 0U;
}

void BmsSoc_SetChemistry(BmsSoc_Chem_t chem)
{
    s.chem = chem;
}

int32_t BmsSoc_UserAhToMah(int32_t whole, uint32_t frac)
{
    uint64_t bits = ((uint64_t)(uint32_t)whole << 32) | (uint64_t)frac;
    int64_t q = (int64_t)bits;
    int64_t mah;

    if (q >= 0) {
        mah = (q + 0x80000000LL) >> 32;
    } else {
        uint64_t mag = (uint64_t)(-(q + 1)) + 1U;

        mah = -((int64_t)((mag + 0x80000000ULL) >> 32));
    }
    if (mah > 2147483647LL) {
        return 2147483647;
    }
    if (mah < (-2147483647LL - 1)) {
        return (int32_t)(-2147483647 - 1);
    }
    return (int32_t)mah;
}

bool BmsSoc_OnSample(uint32_t now_ms,
                     int16_t cc2_ma,
                     const int16_t *cell_mv,
                     uint8_t cell_count,
                     uint8_t cell_used_mask,
                     bool passq_valid,
                     int32_t passq_mah,
                     BmsSoc_Result_t *out)
{
    uint32_t dt = 0U;
    bool gap = false;
    int32_t iabs;
    int32_t rest_ma;
    uint32_t rest_ms;
    int16_t ocv_mv = 0;
    bool snapped = false;
    bool have_cell = false;

    if ((out == NULL) || (cell_mv == NULL) || (cell_count == 0U) ||
        (cell_count > 8U)) {
        return false;
    }

    if (s.have_time) {
        dt = (uint32_t)(now_ms - s.last_ms);
        if (dt > BMS_SOC_DT_MAX_MS) {
            gap = true;
            dt = 0U;
        }
    }
    s.last_ms = now_ms;
    s.have_time = true;

    /* Charge comes from the AFE register. The first sample is the baseline
     * already stored in the chip; later samples move SOC by the delta. */
    if (passq_valid) {
        if (s.have_passq && s.valid && (passq_mah != s.passq_mah)) {
            s.soc_mA_ms += ((int64_t)passq_mah - (int64_t)s.passq_mah) *
                           BMS_SOC_MAH_SCALE;
            BmsSoc_ClampSoc();
        }
        s.passq_mah = passq_mah;
        s.have_passq = true;
    }

    iabs = (int32_t)cc2_ma;
    if (iabs < 0) {
        iabs = -iabs;
    }
    if (s.chem == BMS_SOC_CHEM_LFP) {
        rest_ma = BMS_SOC_LFP_REST_MA;
        rest_ms = BMS_SOC_LFP_REST_MS;
    } else {
        rest_ma = BMS_SOC_LIION_REST_MA;
        rest_ms = BMS_SOC_LIION_REST_MS;
    }
    if (gap || (iabs >= rest_ma)) {
        s.rest_timing = false;
        s.resting = false;
    } else {
        if (!s.rest_timing) {
            s.rest_timing = true;
            s.rest_since_ms = now_ms;
        }
        s.resting = true;
        if (((uint32_t)(now_ms - s.rest_since_ms) >= rest_ms) &&
            BmsSoc_MinUsedMv(cell_mv, cell_count, cell_used_mask, &ocv_mv)) {
            BmsSoc_QualifyRest(ocv_mv);
            snapped = true;
            have_cell = true;
        }
    }
    if (!have_cell) {
        have_cell = BmsSoc_MinUsedMv(cell_mv, cell_count, cell_used_mask,
                                     &ocv_mv);
    }
    if (have_cell) {
        BmsSoc_TrackEnds(ocv_mv, dt, snapped);
    }

    s.balance_mask = BmsSoc_BalanceMask(cc2_ma, cell_mv, cell_count,
                                        cell_used_mask);
    BmsSoc_Fill(passq_valid, passq_mah, out);
    return true;
}

void BmsSoc_NoteBalanceSent(uint32_t now_ms)
{
    s.balance_sent = true;
    s.balance_sent_mask = s.balance_mask;
    s.balance_sent_ms = now_ms;
}
