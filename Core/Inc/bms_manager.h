#ifndef BMS_MANAGER_H
#define BMS_MANAGER_H

#include "bms_gauge.h"
#include "tlm_fields.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Glue between the BQ76922 driver, the LFP gauge and the flash store.
 *
 * Save policy:
 *   - BMS SHUTDOWN: always, marked clean (pack rests afterwards).
 *   - While the PSU output is off: at most every BMS_MGR_IDLE_SAVE_MS when
 *     SOC moved ≥ 0.1 % (≥ 7 mAh) since the last record, and promptly
 *     (≥ BMS_MGR_EVENT_SAVE_MS apart) after a gauge event — full anchor,
 *     capacity learned, cycle state.
 *   - Never while the converter runs (flash erase stalls the CPU).
 */
#define BMS_MGR_IDLE_SAVE_MS      (10U * 60U * 1000U)
#define BMS_MGR_EVENT_SAVE_MS     (30U * 1000U)
#define BMS_MGR_MIN_DELTA_MAH     7

void BmsMgr_Init(void);
void BmsMgr_Task(uint32_t now_ms);
/* Orderly shutdown: store a clean record. Returns true when written. */
bool BmsMgr_SaveForShutdown(void);
/* Manual save (BMS SAVE); refused while the output is on. */
bool BmsMgr_SaveNow(bool *refused_output_on);
void BmsMgr_ResetPack(void);
void BmsMgr_GetGauge(BmsGauge_Output_t *out);
bool BmsMgr_StoreLoaded(void);
uint32_t BmsMgr_StoreSeq(void);
bool BmsMgr_LastSaveOk(void);

/* Telemetry: fill the BMS (AFE) and BATT (gauge) groups. */
void BmsMgr_FillBms(TlmBuilder_t *b);
void BmsMgr_FillBatt(TlmBuilder_t *b);

#endif /* BMS_MANAGER_H */
