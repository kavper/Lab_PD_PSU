/* Host test for the LFP gauge: simulated 5S pack with a true capacity
 * different from the design value; checks anchors, FCC learning, cycle
 * count, persistence and passQ resets. */
#include "bms_gauge.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); exit(1); } } while (0)

typedef struct {
    double cap_mah;     /* true capacity */
    double soc;         /* true SOC 0..1 */
    double r_ohm;       /* per cell */
    double passq_mah;   /* AFE counter */
    uint32_t passq_t;
    uint32_t now;
} Pack;

static double ocv(double soc)
{
    /* Discharge-branch table + a steep top so CV tapers like a real cell. */
    int16_t mv = (int16_t)BmsGauge_OcvToPermille(0, false); (void)mv;
    static const double s[] = {0, .02, .05, .10, .15, .20, .30, .40, .50, .60, .70, .80, .90, .95, .98, 1.0};
    static const double v[] = {2.50, 2.90, 3.05, 3.18, 3.21, 3.24, 3.265, 3.28, 3.29, 3.30, 3.31, 3.32, 3.33, 3.34, 3.36, 3.55};
    int i;
    if (soc <= 0) return 2.5;
    if (soc >= 1) return 3.55;
    for (i = 1; i < 16; i++) if (soc <= s[i]) return v[i-1] + (soc - s[i-1]) / (s[i] - s[i-1]) * (v[i] - v[i-1]);
    return 3.55;
}

static void step(Pack *p, double i_ma, uint32_t dt_ms, int16_t cells[5], int32_t *i_out)
{
    double dq = i_ma * dt_ms / 3600000.0;
    p->soc += dq / p->cap_mah;
    if (p->soc < 0) p->soc = 0;
    if (p->soc > 1) p->soc = 1;
    p->passq_mah += dq;
    p->passq_t += dt_ms / 1000;
    p->now += dt_ms;
    for (int k = 0; k < 5; k++) cells[k] = (int16_t)lround((ocv(p->soc) + i_ma / 1000.0 * p->r_ohm) * 1000.0);
    *i_out = (int32_t)lround(i_ma);
}

static void feed(Pack *p, const int16_t cells[5], int32_t i)
{
    BmsGauge_Input_t in;
    double whole = floor(p->passq_mah);
    memset(&in, 0, sizeof(in));
    in.now_ms = p->now;
    in.afe_valid = true;
    in.passq_valid = true;
    in.passq_whole = (int32_t)whole;
    in.passq_frac = (uint32_t)((p->passq_mah - whole) * 4294967296.0);
    in.passq_time_s = p->passq_t;
    in.current_ma = i;
    in.cell_mv = cells;
    in.temp_dc = 250;
    BmsGauge_Update(&in);
}

/* CC/CV charge to 3.50 V/cell, until current < 100 mA. */
static void charge_full(Pack *p)
{
    int16_t c[5]; int32_t i;
    for (int n = 0; n < 200000; n++) {
        double imax = 3000.0;
        double icv = (3.50 - ocv(p->soc)) / p->r_ohm * 1000.0;
        double ic = icv < imax ? icv : imax;
        if (ic < 100.0) break;
        step(p, ic, 500, c, &i);
        feed(p, c, i);
    }
    /* charger keeps a trickle for a while (taper) */
    for (int n = 0; n < 240; n++) { step(p, 150.0, 500, c, &i); feed(p, c, i); }
}

static void discharge_to(Pack *p, double i_ma, double stop_cell_v)
{
    int16_t c[5]; int32_t i;
    for (int n = 0; n < 400000; n++) {
        step(p, -i_ma, 500, c, &i);
        feed(p, c, i);
        if (c[0] / 1000.0 <= stop_cell_v) break;
    }
}

static void rest(Pack *p, uint32_t ms)
{
    int16_t c[5]; int32_t i;
    for (uint32_t t = 0; t < ms; t += 1000) { step(p, -150.0, 1000, c, &i); feed(p, c, i); }
}

int main(void)
{
    BmsGauge_Config_t cfg;
    BmsGauge_Output_t o;
    BmsGauge_Persist_t per;
    Pack p = { 6000.0, 0.50, 0.020, 1234.5, 100, 1000 };
    int16_t c[5]; int32_t i;

    BmsGauge_DefaultConfig(&cfg);
    BmsGauge_Init(&cfg);

    /* 1: first sample, plateau voltage → estimate, uncertain */
    step(&p, 0, 500, c, &i); feed(&p, c, i);
    BmsGauge_GetOutput(&o);
    CHECK(o.soc_permille != BMS_GAUGE_INVALID_PERMILLE);
    CHECK(o.flags & BMS_GAUGE_F_UNCERTAIN);
    CHECK(o.soc_permille > 300 && o.soc_permille < 700);

    /* 2: full charge → FULL anchor, 100 %, certain */
    charge_full(&p);
    BmsGauge_GetOutput(&o);
    CHECK(o.last_anchor == BMS_ANCHOR_FULL);
    CHECK(o.soc_permille >= 995);
    CHECK(!(o.flags & BMS_GAUGE_F_UNCERTAIN));

    /* 3: cycles learn the real 6000 mAh */
    for (int cyc = 0; cyc < 12; cyc++) {
        discharge_to(&p, 2000.0, 2.86);
        BmsGauge_GetOutput(&o);
        CHECK(o.flags & BMS_GAUGE_F_LOW);
        charge_full(&p);
    }
    BmsGauge_GetOutput(&o);
    printf("fcc=%ld mAh soh=%u learn=%u cycles=%u.%02u rcell=%d.%d mOhm\n",
           (long)o.fcc_mah, o.soh_permille, o.learn_count,
           o.cycles_x100 / 100, o.cycles_x100 % 100, o.rcell_mohm_x10 / 10, o.rcell_mohm_x10 % 10);
    CHECK(o.fcc_mah > 5800 && o.fcc_mah < 6250);
    CHECK(o.learn_count >= 10);
    CHECK(o.cycles_x100 >= 900 && o.cycles_x100 <= 1200);
    CHECK(o.rcell_mohm_x10 > 150 && o.rcell_mohm_x10 < 260);

    /* 4: half discharge, SOC tracks true SOC within 3 % */
    discharge_to(&p, 1500.0, 0.0 + 3.29);
    BmsGauge_GetOutput(&o);
    printf("true=%.1f%% gauge=%.1f%%\n", p.soc * 100.0, o.soc_permille / 10.0);
    CHECK(fabs(p.soc * 1000.0 - o.soc_permille) < 30.0);

    /* 5: persist → restore (clean) keeps SOC/FCC/cycles */
    BmsGauge_GetPersist(&per);
    {
        uint16_t soc_before = o.soc_permille;
        int32_t fcc_before = o.fcc_mah;
        BmsGauge_Init(&cfg);
        BmsGauge_Restore(&per, true);
        p.passq_mah = 0.0; p.passq_t = 0; /* AFE was in SHUTDOWN: counter reset */
        rest(&p, 2000);
        BmsGauge_GetOutput(&o);
        CHECK(o.fcc_mah == fcc_before);
        CHECK(abs((int)o.soc_permille - (int)soc_before) <= 5);
        CHECK(o.flags & BMS_GAUGE_F_RESTORED);
    }

    /* 6: pack charged elsewhere while stored state says ~50 % → boot OCV fixes */
    BmsGauge_GetPersist(&per);
    BmsGauge_Init(&cfg);
    BmsGauge_Restore(&per, false);
    p.soc = 0.99;   /* relaxed at the steep top */
    rest(&p, 1000);
    BmsGauge_GetOutput(&o);
    printf("after external charge: gauge=%.1f%% anchor=%u\n", o.soc_permille / 10.0, o.last_anchor);
    CHECK(o.soc_permille > 900);
    CHECK(o.flags & BMS_GAUGE_F_UNCERTAIN);

    /* 7: passQ reset mid-run does not jump SOC */
    {
        uint16_t before;
        BmsGauge_GetOutput(&o); before = o.soc_permille;
        p.passq_mah = -50000.0; p.passq_t = 0;
        rest(&p, 2000);
        BmsGauge_GetOutput(&o);
        CHECK(abs((int)o.soc_permille - (int)before) <= 3);
    }

    printf("test_bms_gauge: PASS\n");
    return 0;
}
