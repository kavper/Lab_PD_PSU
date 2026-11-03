#ifndef PD_SOURCE_READINESS_H
#define PD_SOURCE_READINESS_H
#include <stdint.h>
#include <stdbool.h>
/* ADC telemetry is slow; never use a previous/invalid sample to authorize EN.
 * The fallback only recognizes OTG that is ALREADY running at the contract.
 * It does not initiate a cold converter at high VBUS or override a fault. */
static inline bool PdSource_BqReady(uint32_t bq_age_ms, bool bq_valid,
    uint32_t bq_mv, bool bq_otg, uint8_t bq_fault,
    bool coherent_contract, uint32_t tps_age_ms, uint32_t tps_mv, uint32_t contract_mv)
{
    if (!bq_valid || bq_age_ms>2500U || bq_fault!=0U) return false;
    if (bq_mv>=4500U && bq_mv<=5500U) return true;
    if (!bq_otg || !coherent_contract || tps_age_ms>1500U ||
        contract_mv<4500U || contract_mv>21000U) return false;
    uint32_t tolerance=contract_mv/20U+100U; /* 5% + coarse ADC resolution */
    uint32_t lo=contract_mv-tolerance, hi=contract_mv+tolerance;
    return bq_mv>=lo && bq_mv<=hi && tps_mv>=lo && tps_mv<=hi;
}
#endif
