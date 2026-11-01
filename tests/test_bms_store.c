/* Host test: gauge record codec round trip and corruption detection. */
#include "bms_store.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); exit(1); } } while (0)

int main(void)
{
    BmsStore_Record_t a, b;
    uint8_t buf[BMS_STORE_RECORD_BYTES];

    memset(&a, 0, sizeof(a));
    a.seq = 123456U;
    a.clean = true;
    a.gauge.remaining_mah_x10 = 41234;
    a.gauge.fcc_mah = 6512;
    a.gauge.dsg_total_mah = 987654U;
    a.gauge.chg_total_mah = 991000U;
    a.gauge.soc_permille = 633U;
    a.gauge.rcell_mohm_x10 = 212U;
    a.gauge.flags = BMS_GAUGE_F_FCC_LEARNED;
    a.gauge.last_anchor = BMS_ANCHOR_FULL;
    a.gauge.learn_count = 17U;
    BmsStore_Encode(&a, buf);
    CHECK(!BmsStore_IsErased(buf));
    CHECK(BmsStore_Decode(buf, &b));
    CHECK(b.seq == a.seq && b.clean == a.clean);
    CHECK(memcmp(&a.gauge, &b.gauge, sizeof(a.gauge)) == 0);
    for (unsigned i = 0; i < BMS_STORE_RECORD_BYTES; i++) {
        uint8_t t[BMS_STORE_RECORD_BYTES];
        memcpy(t, buf, sizeof(t));
        t[i] ^= 0x10U;
        CHECK(!BmsStore_Decode(t, &b));
    }
    memset(buf, 0xFF, sizeof(buf));
    CHECK(BmsStore_IsErased(buf));
    CHECK(!BmsStore_Decode(buf, &b));
    printf("test_bms_store: PASS\n");
    return 0;
}
