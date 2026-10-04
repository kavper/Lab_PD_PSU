#include "bms_soc.h"
#include "h7_link_proto.h"

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

static void ExpectEqI(int32_t got, int32_t want, const char *msg)
{
    if (got != want) {
        g_failures++;
        printf("FAIL: %s (got %ld want %ld)\n", msg, (long)got, (long)want);
    }
}

static void ExpectEqU(unsigned got, unsigned want, const char *msg)
{
    if (got != want) {
        g_failures++;
        printf("FAIL: %s (got %u want %u)\n", msg, got, want);
    }
}

static void Sample(uint32_t now, int16_t ma, const int16_t *cells,
                   BmsSoc_Result_t *out)
{
    ExpectTrue(BmsSoc_OnSample(now, ma, cells, 5U, 0x17U, false, 0, out),
               "sample accepted");
}

/* Step in 5 s so the rest timer accumulates and the integral is not skipped. */
static void Pump(uint32_t *now, uint32_t total_ms, int16_t ma,
                 const int16_t *cells, BmsSoc_Result_t *out)
{
    uint32_t left = total_ms;

    while (left > 0U) {
        uint32_t step = (left > 5000U) ? 5000U : left;

        *now += step;
        left -= step;
        Sample(*now, ma, cells, out);
    }
}

static void FillMv(int16_t *cells, int16_t mv)
{
    cells[0] = mv;
    cells[1] = mv;
    cells[2] = mv;
    cells[3] = 0;
    cells[4] = mv;
}

int main(void)
{
    BmsSoc_Result_t out;
    int16_t cells[5];
    uint32_t now;
    uint32_t i;

    ExpectEqI(BmsSoc_UserAhToMah(-1, 0xC0000000U), 0,
              "-0.25 userAh rounds to 0 mAh");
    ExpectEqI(BmsSoc_UserAhToMah(1, 0x9999999AU), 2,
              "1.6 userAh rounds to 2 mAh");
    ExpectEqI(BmsSoc_UserAhToMah(1, 0x66666666U), 1,
              "1.4 userAh rounds to 1 mAh");

    ExpectTrue(H7_LINK_BMS_BYTES == 72U, "BMS payload is 72 bytes");
    ExpectTrue(H7_BMS_PASSQ_MAH == 56U, "passQ starts after the old payload");
    ExpectTrue(H7_BMS_SOC_FLAGS == 71U, "SOC flags are the last BMS byte");
    ExpectTrue((H7_LINK_BMS_BYTES + 7U) <= H7_LINK_MAX_FRAME,
               "BMS frame fits in 120 bytes");
    ExpectTrue(H7_LINK_AUX_TLM == 0x13U, "AUX telemetry type is 0x13");
    ExpectTrue(H7_LINK_AUX_BYTES == 24U, "AUX payload is 24 bytes");
    ExpectTrue(H7_AUX_VALID == 19U, "AUX valid flag sits at byte 19");
    ExpectTrue((H7_LINK_AUX_BYTES + 7U) <= H7_LINK_MAX_FRAME,
               "AUX frame fits in 120 bytes");

    BmsSoc_Reset();
    FillMv(cells, 3700);
    now = 0U;
    Sample(now, 3600, cells, &out);
    ExpectEqU(out.soc_permille, BMS_SOC_INVALID_PERMILLE,
              "SOC stays invalid before a qualified rest");
    ExpectEqI(out.session_mah, 0, "first sample has no dt");
    for (i = 0U; i < 10U; i++) {
        now += 1000U;
        Sample(now, 3600, cells, &out);
    }
    ExpectEqI(out.session_mah, 10, "10 s at 3.6 A is 10 mAh");
    ExpectEqU(out.soc_permille, BMS_SOC_INVALID_PERMILLE,
              "live voltage does not seed SOC");
    now += 5001U;
    Sample(now, 3600, cells, &out);
    ExpectEqI(out.session_mah, 10, "a gap above 5 s is not integrated");
    now += 1000U;
    Sample(now, -3600, cells, &out);
    ExpectEqI(out.session_mah, 9, "discharge counts against the session");

    BmsSoc_Reset();
    FillMv(cells, 3500);
    now = 0U;
    Sample(now, 0, cells, &out);
    Pump(&now, 30U * 60U * 1000U, 0, cells, &out);
    ExpectEqU(out.soc_permille, 200U, "3500 mV rests at 200 permille");
    ExpectTrue((out.flags & BMS_SOC_FLAG_VALID) != 0U, "rest marks SOC valid");
    ExpectTrue((out.flags & BMS_SOC_FLAG_RESTING) != 0U, "rest flag is set");
    ExpectTrue((out.flags & BMS_SOC_FLAG_LEARNED) == 0U,
               "one rest does not learn capacity");

    BmsSoc_Reset();
    FillMv(cells, 3300);
    now = 0U;
    Sample(now, 0, cells, &out);
    Pump(&now, 30U * 60U * 1000U, 0, cells, &out);
    ExpectEqU(out.soc_permille, 50U, "default chemistry is Li-ion");

    BmsSoc_Reset();
    BmsSoc_SetChemistry(BMS_SOC_CHEM_LFP);
    FillMv(cells, 3300);
    now = 0U;
    Sample(now, 0, cells, &out);
    Pump(&now, 45U * 60U * 1000U, 0, cells, &out);
    ExpectEqU(out.soc_permille, 500U, "LFP table is used only when selected");

    /* Low anchor, 277 mAh of charge, high anchor → capacity 325 mAh. */
    BmsSoc_Reset();
    FillMv(cells, 3400);
    now = 0U;
    Sample(now, 0, cells, &out);
    Pump(&now, 30U * 60U * 1000U, 0, cells, &out);
    ExpectEqU(out.soc_permille, 100U, "3400 mV is the low Li-ion anchor");
    Pump(&now, 200U * 5000U, 1000, cells, &out);
    ExpectEqI(out.session_mah, 277, "session counts the transfer");
    ExpectTrue((out.flags & BMS_SOC_FLAG_LEARNED) == 0U,
               "capacity waits for the opposite anchor");
    FillMv(cells, 4100);
    /* The rest timer starts on the first quiet sample, so the window is one step longer. */
    Pump(&now, (30U * 60U * 1000U) + 5000U, 0, cells, &out);
    ExpectEqU(out.soc_permille, 950U, "4100 mV snaps to 950 permille");
    ExpectTrue((out.flags & BMS_SOC_FLAG_LEARNED) != 0U,
               "opposite anchors learn capacity");
    for (i = 0U; i < 32U; i++) {
        now += 1000U;
        Sample(now, -3600, cells, &out);
    }
    ExpectEqU(out.soc_permille, 851U, "learned 325 mAh moves SOC by 32 mAh");

    BmsSoc_Reset();
    cells[0] = 4000;
    cells[1] = 4010;
    cells[2] = 4020;
    cells[3] = 5000;
    cells[4] = 4030;
    now = 0U;
    Sample(now, 200, cells, &out);
    ExpectEqU(out.balance_mask, 0x10U, "only the highest used cell is balanced");
    ExpectTrue(out.balance_write, "a new mask requests one CB write");
    BmsSoc_NoteBalanceSent(now);
    now += 1000U;
    Sample(now, 200, cells, &out);
    ExpectTrue(!out.balance_write, "an unchanged mask is not rewritten at once");
    now += 10000U;
    Sample(now, 200, cells, &out);
    ExpectTrue(out.balance_write, "a live mask is refreshed at 10 s");
    BmsSoc_NoteBalanceSent(now);
    cells[0] = 4025;
    cells[1] = 4026;
    cells[2] = 4027;
    cells[4] = 4030;
    now += 1000U;
    Sample(now, 200, cells, &out);
    ExpectEqU(out.balance_mask, 0U, "delta at the stop threshold ends balance");
    ExpectTrue(out.balance_write, "turning balance off is written once");
    BmsSoc_NoteBalanceSent(now);
    now += 1000U;
    Sample(now, 0, cells, &out);
    ExpectTrue(!out.balance_write, "a zero mask is not repeated");

    BmsSoc_Reset();
    FillMv(cells, 4100);
    Sample(0U, 50, cells, &out);
    ExpectEqU(out.balance_mask, 0U, "balance waits for charge above 100 mA");
    ExpectTrue(BmsSoc_OnSample(0U, 0, cells, 5U, 0x17U, true, -4, &out),
               "passQ sample accepted");
    ExpectTrue((out.flags & BMS_SOC_FLAG_PASSQ_VALID) != 0U, "passQ flag");
    ExpectEqI(out.passq_mah, -4, "passQ is reported and is not the session");
    ExpectEqI(out.session_mah, 0, "passQ does not move the session integral");

    if (g_failures != 0) {
        printf("%d failure(s)\n", g_failures);
        return EXIT_FAILURE;
    }
    printf("test_bms_soc: PASS\n");
    return EXIT_SUCCESS;
}
