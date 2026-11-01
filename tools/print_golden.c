/* Host tool: print the BQ76922 golden table (exactly what RAM/OTP get).
 *   cc -std=c11 -ICore/Inc tools/print_golden.c Core/Src/bms_golden.c -o /tmp/pg && /tmp/pg */
#include "bms_golden.h"
#include <stdio.h>

int main(void)
{
    uint16_t i;

    printf("| Adres | Nazwa (TRM) | Typ | Wartość | Surowe bajty (LE) | FP |\n");
    printf("|---|---|---|---|---|---|\n");
    for (i = 0; i < g_bms_golden_count; i++) {
        const BmsDmEntry_t *e = &g_bms_golden[i];
        uint8_t b[4];
        char val[32];

        BmsGolden_Bytes(e, b);
        if (e->type == BMS_DM_F4) {
            snprintf(val, sizeof(val), "%.5f", (double)e->value.f);
        } else if (e->type == BMS_DM_U2) {
            snprintf(val, sizeof(val), "%u (0x%04X, i16 %d)", (unsigned)e->value.u,
                     (unsigned)e->value.u, (int)(int16_t)e->value.u);
        } else {
            snprintf(val, sizeof(val), "%u (0x%02X, i8 %d)", (unsigned)e->value.u,
                     (unsigned)e->value.u, (int)(int8_t)e->value.u);
        }
        printf("| 0x%04X | %s | %s | %s |", e->addr, e->name,
               e->type == 1 ? "U1" : e->type == 2 ? "U2" : "F4", val);
        for (int k = 0; k < e->type; k++) {
            printf(" %02X", b[k]);
        }
        printf(" | %s |\n", (e->flags & BMS_DM_FLAG_FINGERPRINT) ? "tak" : "");
    }
    printf("\nWpisów: %u, CRC-32 obrazu: 0x%08lX\n", (unsigned)g_bms_golden_count,
           (unsigned long)BmsGolden_Crc32());
    return 0;
}
