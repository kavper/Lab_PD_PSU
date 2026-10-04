#include "bms_soc.h"
#include "h7_link_proto.h"
#include "ldo_tlm_layout.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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

static int64_t g_afe_mA_ms;

static int32_t AfeMah(void)
{
    return (int32_t)(g_afe_mA_ms / 3600000LL);
}

static void Boot(void)
{
    BmsSoc_Reset();
    g_afe_mA_ms = 0;
}

static void Sample(uint32_t now, int16_t ma, const int16_t *cells,
                   BmsSoc_Result_t *out)
{
    ExpectTrue(BmsSoc_OnSample(now, ma, cells, 5U, 0x17U, true, AfeMah(), out),
               "sample accepted");
}

/* The simulated AFE keeps integrating across a G4 time gap. */
static void Elapse(uint32_t *now, uint32_t dt_ms, int16_t ma,
                   const int16_t *cells, BmsSoc_Result_t *out)
{
    g_afe_mA_ms += (int64_t)ma * (int64_t)dt_ms;
    *now += dt_ms;
    Sample(*now, ma, cells, out);
}

static void Pump(uint32_t *now, uint32_t total_ms, int16_t ma,
                 const int16_t *cells, BmsSoc_Result_t *out)
{
    uint32_t left = total_ms;

    while (left > 0U) {
        uint32_t step = (left > 5000U) ? 5000U : left;

        left -= step;
        Elapse(now, step, ma, cells, out);
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
    ExpectTrue(H7_LINK_AUX_BYTES == 32U, "AUX payload is 32 bytes");
    ExpectTrue(H7_AUX_VALID == 19U, "AUX valid flag sits at byte 19");
    ExpectTrue(H7_AUX_LOCAL_MV == 20U, "local sense millivolts follow the valid flag");
    ExpectTrue(H7_AUX_SENSE_CODE == 26U, "sense code is a single byte");
    ExpectTrue(H7_AUX_FAN_RPM == 28U, "fan tach RPM is the u16 at byte 28");
    ExpectTrue((H7_LINK_AUX_BYTES + 7U) <= H7_LINK_MAX_FRAME,
               "AUX frame fits in 120 bytes");

    Boot();
    FillMv(cells, 3700);
    now = 0U;
    Sample(now, 3600, cells, &out);
    ExpectEqU(out.soc_permille, BMS_SOC_INVALID_PERMILLE,
              "SOC stays invalid before a qualified rest");
    ExpectEqI(out.passq_mah, 0, "the first passQ sample is the baseline");
    ExpectEqI(out.session_mah, 0, "G4 does not keep a session integral");
    for (i = 0U; i < 10U; i++) {
        Elapse(&now, 1000U, 3600, cells, &out);
    }
    ExpectEqI(out.passq_mah, 10, "10 s at 3.6 A is 10 mAh on passQ");
    ExpectEqI(out.session_mah, 0, "CC2 does not build a second counter");
    ExpectEqU(out.soc_permille, BMS_SOC_INVALID_PERMILLE,
              "live voltage does not seed SOC");
    Elapse(&now, 5001U, 3600, cells, &out);
    ExpectEqI(out.passq_mah, 15, "a G4 time gap does not drop AFE charge");
    Elapse(&now, 1000U, -3600, cells, &out);
    ExpectEqI(out.passq_mah, 14, "discharge counts on passQ");

    Boot();
    FillMv(cells, 3500);
    now = 0U;
    Sample(now, 0, cells, &out);
    Pump(&now, 30U * 60U * 1000U, 0, cells, &out);
    ExpectEqU(out.soc_permille, 200U, "3500 mV rests at 200 permille");
    ExpectTrue((out.flags & BMS_SOC_FLAG_VALID) != 0U, "rest marks SOC valid");
    ExpectTrue((out.flags & BMS_SOC_FLAG_RESTING) != 0U, "rest flag is set");
    ExpectTrue((out.flags & BMS_SOC_FLAG_LEARNED) == 0U,
               "one rest does not learn capacity");

    Boot();
    FillMv(cells, 3300);
    now = 0U;
    Sample(now, 0, cells, &out);
    Pump(&now, 30U * 60U * 1000U, 0, cells, &out);
    ExpectEqU(out.soc_permille, 50U, "default chemistry is Li-ion");

    Boot();
    BmsSoc_SetChemistry(BMS_SOC_CHEM_LFP);
    FillMv(cells, 3300);
    now = 0U;
    Sample(now, 0, cells, &out);
    Pump(&now, 45U * 60U * 1000U, 0, cells, &out);
    ExpectEqU(out.soc_permille, 500U, "LFP table is used only when selected");

    /* Low anchor, 277 mAh of charge, high anchor → capacity 325 mAh. */
    Boot();
    FillMv(cells, 3400);
    now = 0U;
    Sample(now, 0, cells, &out);
    Pump(&now, 30U * 60U * 1000U, 0, cells, &out);
    ExpectEqU(out.soc_permille, 100U, "3400 mV is the low Li-ion anchor");
    Pump(&now, 200U * 5000U, 1000, cells, &out);
    ExpectEqI(out.passq_mah, 277, "passQ counts the transfer");
    ExpectEqI(out.session_mah, 0, "the transfer is not a G4 integral");
    ExpectTrue((out.flags & BMS_SOC_FLAG_LEARNED) == 0U,
               "capacity waits for the opposite anchor");
    FillMv(cells, 4100);
    /* The rest timer starts on the first quiet sample, so the window is one step longer. */
    Pump(&now, (30U * 60U * 1000U) + 5000U, 0, cells, &out);
    ExpectEqU(out.soc_permille, 950U, "4100 mV snaps to 950 permille");
    ExpectTrue((out.flags & BMS_SOC_FLAG_LEARNED) != 0U,
               "opposite anchors learn capacity");
    /* Leave the steep end. Live tracking must not fight this coulomb step. */
    FillMv(cells, 3700);
    for (i = 0U; i < 32U; i++) {
        Elapse(&now, 1000U, -3600, cells, &out);
    }
    ExpectEqU(out.soc_permille, 851U, "learned 325 mAh moves SOC by 32 mAh");

    Boot();
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
    ExpectTrue(out.balance_write, "a live mask is refreshed inside 20 s");
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

    Boot();
    FillMv(cells, 4100);
    Sample(0U, 50, cells, &out);
    ExpectEqU(out.balance_mask, 0U, "balance waits for charge above 100 mA");
    ExpectTrue(BmsSoc_OnSample(0U, 0, cells, 5U, 0x17U, true, -4, &out),
               "passQ sample accepted");
    ExpectTrue((out.flags & BMS_SOC_FLAG_PASSQ_VALID) != 0U, "passQ flag");
    ExpectEqI(out.passq_mah, -4, "passQ is the DASTATUS6 reading");
    ExpectEqI(out.session_mah, 0, "passQ does not start a G4 integral");

    /* A G4 reboot keeps the AFE count and does not turn it into SOC. */
    Boot();
    g_afe_mA_ms = -40LL * 3600000LL;
    FillMv(cells, 3700);
    now = 0U;
    Sample(now, 0, cells, &out);
    ExpectEqI(out.passq_mah, -40, "boot reports the passQ the AFE kept");
    ExpectEqU(out.soc_permille, BMS_SOC_INVALID_PERMILLE,
              "a survived passQ is not a percentage by itself");
    for (i = 0U; i < 10U; i++) {
        now += 1000U;
        Sample(now, 3600, cells, &out);
    }
    ExpectEqI(out.passq_mah, -40, "CC2 alone does not move the counter");
    g_afe_mA_ms += 10LL * 3600000LL;
    now += 1000U;
    Sample(now, 0, cells, &out);
    ExpectEqI(out.passq_mah, -30, "the next DASTATUS6 reading is the counter");
    FillMv(cells, 4100);
    now += 1000U;
    Sample(now, 200, cells, &out);
    ExpectEqU(out.soc_permille, 950U, "the high end seeds SOC on the survived counter");
    FillMv(cells, 3700);
    g_afe_mA_ms += 25LL * 3600000LL;
    now += 1000U;
    Sample(now, 0, cells, &out);
    ExpectEqU(out.soc_permille, 960U, "25 mAh of passQ is 10 permille at 2500 mAh");
    now += 1000U;
    Sample(now, 3600, cells, &out);
    ExpectEqU(out.soc_permille, 960U, "CC2 does not move SOC while passQ holds");

    /* Steep ends estimate SOC under current. The flat middle does not. */
    Boot();
    FillMv(cells, 3700);
    Sample(0U, 500, cells, &out);
    ExpectEqU(out.soc_permille, BMS_SOC_INVALID_PERMILLE,
              "middle of the Li-ion curve does not invent SOC");
    FillMv(cells, 4100);
    Sample(1000U, 500, cells, &out);
    ExpectEqU(out.soc_permille, 950U, "the high end seeds SOC without a rest");
    ExpectTrue((out.flags & BMS_SOC_FLAG_VALID) != 0U, "seeded SOC is valid");
    FillMv(cells, 3700);
    Sample(2000U, 3600, cells, &out);
    ExpectEqU(out.soc_permille, 950U,
              "middle voltage under current does not pull SOC");
    FillMv(cells, 3300);
    Sample(3000U, -100, cells, &out);
    ExpectEqU(out.soc_permille, 945U,
              "the low end corrects gently instead of snapping");

    /* 3.6 V to 4.0 V is enough to learn. Empty (3.0 V) is not required. */
    Boot();
    FillMv(cells, 3600);
    now = 0U;
    Sample(now, 0, cells, &out);
    Pump(&now, 30U * 60U * 1000U, 0, cells, &out);
    ExpectEqU(out.soc_permille, 300U, "3600 mV rest is the low knee");
    Pump(&now, 108U * 5000U, 1000, cells, &out);
    ExpectEqI(out.passq_mah, 150, "partial transfer is 150 mAh on passQ");
    ExpectEqI(out.session_mah, 0, "partial transfer is not a G4 integral");
    FillMv(cells, 4000);
    Pump(&now, (30U * 60U * 1000U) + 5000U, 0, cells, &out);
    ExpectEqU(out.soc_permille, 820U, "4000 mV rest is the high knee");
    ExpectTrue((out.flags & BMS_SOC_FLAG_LEARNED) != 0U,
               "capacity learns without a discharge to empty");
    FillMv(cells, 3700);
    for (i = 0U; i < 28U; i++) {
        Elapse(&now, 1000U, -3600, cells, &out);
    }
    ExpectEqU(out.soc_permille, 722U, "learned 288 mAh moves SOC by 28 mAh");

    Boot();
    BmsSoc_SetChemistry(BMS_SOC_CHEM_LFP);
    FillMv(cells, 3450);
    Sample(0U, 500, cells, &out);
    ExpectEqU(out.soc_permille, BMS_SOC_INVALID_PERMILLE,
              "LFP does not take a live voltage seed");

    {
        uint8_t frame[LDO_TLM_BYTES];

        memset(frame, 0, sizeof(frame));
        frame[LDO_TLM_TEMP_RAW] = 0xD0;
        frame[LDO_TLM_TEMP_RAW + 1U] = 0x07; /* raw ADC 2000 */
        frame[LDO_TLM_TEMP_FILTERED] = 0x08;
        frame[LDO_TLM_TEMP_FILTERED + 1U] = 0x07; /* filtered ADC 1800 */
        frame[LDO_TLM_TEMP_CENTI] = 0xFD;
        frame[LDO_TLM_TEMP_CENTI + 1U] = 0x00; /* 253 = 25.3 °C */
        frame[LDO_TLM_TEMP_CENTI + 2U] = 0xF1;
        frame[LDO_TLM_TEMP_CENTI + 3U] = 0x00; /* 241 = 24.1 °C */
        frame[LDO_TLM_TEMP_CENTI + 4U] = 0x36;
        frame[LDO_TLM_TEMP_CENTI + 5U] = 0x01; /* 310 = 31.0 °C */
        frame[LDO_TLM_TEMP_CENTI + 6U] = 0x18;
        frame[LDO_TLM_TEMP_CENTI + 7U] = 0x01; /* 280 = 28.0 °C */
        ExpectEqI(LdoTlm_TempCenti(frame, 0U), 253,
                  "AUX temperature is °C×10, not the raw ADC block");
        ExpectEqI(LdoTlm_TempCenti(frame, 1U), 241, "T2 is °C×10");
        ExpectEqI(LdoTlm_TempCenti(frame, 2U), 310, "T3 is °C×10");
        ExpectEqI(LdoTlm_TempCenti(frame, 3U), 280, "T4 is °C×10");
        ExpectTrue(LDO_TLM_TEMP_RAW == 40U, "raw ADC stays at G0 offset 40");
        ExpectTrue(LDO_TLM_TEMP_FILTERED == 48U, "filtered ADC stays at G0 offset 48");
        ExpectTrue(LDO_TLM_TEMP_CENTI == 56U, "°C×10 starts at G0 offset 56");
        ExpectTrue(LDO_TLM_FAN == 64U, "fan stays at G0 offset 64");
        ExpectTrue(H7_AUX_T1_CC == 8U, "H7 AUX T1 is offset 8");
        ExpectTrue(H7_AUX_T2_CC == 10U, "H7 AUX T2 is offset 10");
        ExpectTrue(H7_AUX_T3_CC == 12U, "H7 AUX T3 is offset 12");
        ExpectTrue(H7_AUX_T4_CC == 14U, "H7 AUX T4 is offset 14");
    }

    if (g_failures != 0) {
        printf("%d failure(s)\n", g_failures);
        return EXIT_FAILURE;
    }
    printf("test_bms_soc: PASS\n");
    return EXIT_SUCCESS;
}
