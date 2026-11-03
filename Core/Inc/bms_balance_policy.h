#ifndef BMS_BALANCE_POLICY_H
#define BMS_BALANCE_POLICY_H
#include <stdint.h>
#include <stdbool.h>
typedef struct { bool waiting; uint32_t since_ms; } BmsBalancePolicy;
/* COV may be balanced only with charge physically reported off. */
static inline bool BmsBalance_SafetyAllows(uint8_t sa, uint8_t sb, uint8_t sc,
                                          bool chg_on, bool dsg_on) {
    return sb==0U && sc==0U &&
           (sa==0U || (sa==0x08U && !chg_on && dsg_on));
}
/* Only low-current, near-full, valid samples may start balancing. */
static inline uint8_t BmsBalance_Select(BmsBalancePolicy *p, uint32_t now,
    bool valid, int32_t current_ma, uint16_t soc, const int16_t cell[5], uint8_t active)
{
    int16_t lo=32767; uint8_t result=0, count=0;
    for (unsigned i=0;i<5;i++) { if (cell[i]<lo) lo=cell[i]; }
    bool allow=valid && current_ma>-1000 && current_ma<1000 &&
               soc>=900 && soc<=1000 && lo>=3350;
    if (!allow) { p->waiting=false; return 0; }
    if (!p->waiting) { p->waiting=true; p->since_ms=now; return 0; }
    if ((uint32_t)(now-p->since_ms)<5000U) return 0;
    /* Highest first; at most two non-adjacent channels, with 20/8mV hysteresis. */
    for (unsigned pass=0;pass<5 && count<2;pass++) {
        int best=-1;
        for (unsigned i=0;i<5;i++) {
            uint8_t bit=(uint8_t)(1U<<i);
            int threshold=(active&bit)?8:20;
            if ((result&bit) || (result&(bit<<1)) || (result&(bit>>1))) continue;
            if ((cell[i]-lo)<threshold) continue;
            if (best<0 || cell[i]>cell[best]) best=(int)i;
        }
        if (best<0) break;
        result|=(uint8_t)(1U<<best); count++;
    }
    return result;
}
#endif
