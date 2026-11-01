/* Host test: FIELDS encode → decode round trip, multi-part split, unknown ids. */
#include "tlm_fields.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); exit(1); } } while (0)

static int32_t got[256];
static int seen[256];
static void cb(void *ctx, uint8_t id, int32_t v) { (void)ctx; got[id] = v; seen[id]++; }

int main(void)
{
    TlmBuilder_t b;
    uint8_t buf[TLM_MAX_PAYLOAD];
    char text[600];
    uint8_t parts, part, g, pp, i;

    Tlm_Begin(&b, TLM_GROUP_BMS);
    CHECK(Tlm_Put(&b, TLM_BMS_PRESENT, 1));
    CHECK(Tlm_Put(&b, TLM_BMS_C1_MV, -1));
    CHECK(Tlm_Put(&b, TLM_BMS_I_PACK_MA, -123456));
    CHECK(Tlm_Put(&b, TLM_BMS_PACK_MV, 16500));
    CHECK(!Tlm_Put(&b, TLM_BATT_FCC_MAH, 1));   /* wrong group */
    CHECK(!Tlm_Put(&b, 250, 1));                /* unknown id */
    for (i = 0; i < 40; i++) Tlm_Put(&b, TLM_BMS_FAULT, (int32_t)i); /* force split */
    parts = Tlm_Parts(&b);
    CHECK(parts >= 2);
    memset(seen, 0, sizeof(seen));
    for (part = 0; part < parts; part++) {
        uint8_t n = Tlm_EncodePart(&b, part, buf);
        CHECK(n > 4 && n <= TLM_MAX_PAYLOAD);
        CHECK(Tlm_Decode(buf, n, &g, &pp, &parts, cb, NULL));
        CHECK(g == TLM_GROUP_BMS && pp == part);
    }
    CHECK(got[TLM_BMS_C1_MV] == -1);
    CHECK(got[TLM_BMS_I_PACK_MA] == -123456);
    CHECK(got[TLM_BMS_PACK_MV] == 16500);
    CHECK(seen[TLM_BMS_FAULT] == 40 && got[TLM_BMS_FAULT] == 39);
    /* unknown id inside a payload is skipped by length */
    {
        uint8_t p[] = {1, 2, 0, 1, 200, 0x04, 1, 2, 3, 4, TLM_BATT_SOH, 0x02, 0xE8, 0x03};
        memset(seen, 0, sizeof(seen));
        CHECK(Tlm_Decode(p, sizeof(p), &g, &pp, &parts, cb, NULL));
        CHECK(got[TLM_BATT_SOH] == 1000 && seen[200] == 1);
    }
    /* truncated → rejected */
    {
        uint8_t p[] = {1, 1, 0, 1, TLM_BMS_PACK_MV, 0x02, 0x10};
        CHECK(!Tlm_Decode(p, sizeof(p), &g, &pp, &parts, cb, NULL));
    }
    Tlm_Begin(&b, TLM_GROUP_BATT);
    Tlm_Put(&b, TLM_BATT_SOC, 633); Tlm_Put(&b, TLM_BATT_RCELL, -5);
    Tlm_Format(&b, text, sizeof(text));
    CHECK(strcmp(text, "BATT soc_permille=633 rcell_x10=-5") == 0);
    printf("test_tlm_fields: PASS\n");
    return 0;
}
