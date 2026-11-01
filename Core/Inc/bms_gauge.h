#ifndef BMS_GAUGE_H
#define BMS_GAUGE_H

#include <stdbool.h>
#include <stdint.h>

/*
 * LiFePO4 fuel gauge (5S1P).  Pure C, no HAL: unit-tested on the host
 * (tests/test_bms_gauge.c).
 *
 * Principle — the same split TI Impedance Track / Apple gauges use:
 *   - Charge is counted by the BQ76922 coulomb counter (DASTATUS6 passQ).
 *     The G4 only takes deltas, so its own loop timing does not matter.
 *   - Voltage corrects that count where the LFP curve actually carries
 *     information:  the top knee at end of charge (CV + taper = 100 %),
 *     the bottom knee under load (IR-compensated) and relaxed OCV at the
 *     steep ends.  The flat plateau (≈3.27…3.33 V) is never used for a
 *     precise value — only to clamp an obviously wrong SOC into its band.
 *   - Full-charge capacity (FCC, "Qmax") is learned from the charge counted
 *     between two voltage anchors that are far enough apart (≥ 50 % SOC),
 *     filtered and rate-limited.  SOH = FCC / design capacity.
 *   - Cycle count = total discharged charge / design capacity (equivalent
 *     full cycles, like Apple).
 *   - Cell resistance is estimated from current steps and used for the
 *     knee compensation.
 *
 * Persistence (see bms_store.h): the persistent part is BmsGauge_Persist_t.
 * It is written on BMS SHUTDOWN and opportunistically while the PSU output
 * is off; a hard battery pull loses at most the counting since then, and
 * the voltage anchors repair that on the way.
 */

#define BMS_GAUGE_INVALID_PERMILLE   0xFFFFU

/* Flags (telemetry soc_flags). */
#define BMS_GAUGE_F_VALID            0x0001U /* SOC has a value at all          */
#define BMS_GAUGE_F_UNCERTAIN        0x0002U /* not yet confirmed by an anchor  */
#define BMS_GAUGE_F_RESTING          0x0004U /* |I| below rest threshold        */
#define BMS_GAUGE_F_RELAXED          0x0008U /* rested long enough for OCV      */
#define BMS_GAUGE_F_FCC_LEARNED      0x0010U /* FCC measured at least once      */
#define BMS_GAUGE_F_CHARGING         0x0020U
#define BMS_GAUGE_F_DISCHARGING      0x0040U
#define BMS_GAUGE_F_FULL             0x0080U /* last anchor = full, not left    */
#define BMS_GAUGE_F_LOW              0x0100U /* warning window                  */
#define BMS_GAUGE_F_EMPTY            0x0200U /* load cut-off reached            */
#define BMS_GAUGE_F_RESTORED         0x0400U /* state came from the store       */
#define BMS_GAUGE_F_CLEAN_RESTORE    0x0800U /* ...and it was saved on SHUTDOWN */
#define BMS_GAUGE_F_DIRTY            0x1000U /* persistent part changed         */

typedef enum {
    BMS_ANCHOR_NONE = 0,
    BMS_ANCHOR_FULL,       /* CV + taper current          */
    BMS_ANCHOR_EMPTY,      /* knee under load             */
    BMS_ANCHOR_OCV_HIGH,   /* relaxed OCV, steep top      */
    BMS_ANCHOR_OCV_LOW,    /* relaxed OCV, steep bottom   */
    BMS_ANCHOR_PLATEAU,    /* clamp into the plateau band */
    BMS_ANCHOR_BOOT_OCV,   /* first estimate from voltage */
} BmsAnchor_t;

typedef struct {
    int32_t design_mah;           /* 6700 */
    uint8_t series;               /* 5 */
    int32_t taper_ma;             /* full detection: 0 < I < taper (C/20) */
    int16_t full_cell_mv;         /* average cell ≥ this while tapering */
    uint32_t full_hold_ms;        /* taper must hold this long */
    int32_t rest_ma;              /* |I| below = resting */
    uint32_t relax_ms;            /* resting this long = OCV valid */
    int16_t low_cell_mv;          /* warning (IR compensated) */
    int16_t empty_cell_mv;        /* load cut-off (IR compensated) */
    uint16_t empty_permille;      /* SOC at the empty knee */
    uint16_t low_permille;        /* SOC floor of the warning knee */
} BmsGauge_Config_t;

/* Persistent state — 28 bytes of payload, stored with header + CRC. */
typedef struct {
    int32_t remaining_mah_x10;    /* 0.1 mAh */
    int32_t fcc_mah;              /* learned full-charge capacity */
    uint32_t dsg_total_mah;       /* lifetime discharged charge */
    uint32_t chg_total_mah;       /* lifetime charged charge */
    uint16_t soc_permille;        /* redundant, for quick reads */
    uint16_t rcell_mohm_x10;      /* learned cell resistance, 0.1 mΩ */
    uint16_t flags;               /* BMS_GAUGE_F_FCC_LEARNED | _UNCERTAIN */
    uint8_t last_anchor;          /* BmsAnchor_t */
    uint8_t learn_count;          /* FCC updates so far (saturating) */
} BmsGauge_Persist_t;

typedef struct {
    uint32_t now_ms;
    bool afe_valid;               /* fresh cell/current sample */
    bool passq_valid;
    int32_t passq_whole;          /* DASTATUS6 integer part (mAh, signed) */
    uint32_t passq_frac;          /* DASTATUS6 fraction (2^-32 mAh) */
    uint32_t passq_time_s;        /* DASTATUS6 timer, resets with passQ */
    int32_t current_ma;           /* + charge / − discharge */
    const int16_t *cell_mv;       /* series entries */
    int16_t temp_dc;              /* cell temperature, 0.1 °C (INT16_MIN = none) */
} BmsGauge_Input_t;

typedef struct {
    uint16_t soc_permille;        /* 0..1000, 0.1 % steps */
    uint16_t flags;
    int32_t remaining_mah;
    int32_t fcc_mah;
    uint16_t soh_permille;
    uint16_t cycles_x100;         /* equivalent full cycles × 100 */
    int16_t rcell_mohm_x10;
    int16_t ocv_avg_mv;           /* IR-compensated average cell */
    uint16_t tte_min;             /* time to empty, 0xFFFF = n/a */
    uint16_t ttf_min;             /* time to full,  0xFFFF = n/a */
    uint8_t last_anchor;
    uint8_t learn_count;
    int32_t learn_delta_mah;      /* charge counted since the last anchor */
    uint32_t chg_total_mah;
    uint32_t dsg_total_mah;
} BmsGauge_Output_t;

void BmsGauge_DefaultConfig(BmsGauge_Config_t *cfg);
void BmsGauge_Init(const BmsGauge_Config_t *cfg);
/* Restore persisted state. clean = it was saved by an orderly SHUTDOWN. */
void BmsGauge_Restore(const BmsGauge_Persist_t *p, bool clean);
void BmsGauge_Update(const BmsGauge_Input_t *in);
void BmsGauge_GetOutput(BmsGauge_Output_t *out);
/* Snapshot of the persistent part; clears the DIRTY flag. */
void BmsGauge_GetPersist(BmsGauge_Persist_t *p);
bool BmsGauge_IsDirty(void);
/* Charge counted since the last stored snapshot (mAh, absolute). */
int32_t BmsGauge_UnsavedMah(void);
/* New pack: forget FCC/cycles/resistance, keep nothing but design data. */
void BmsGauge_ResetPack(void);
/* Pack disappeared (AFE absent): drop the passQ baseline. */
void BmsGauge_NotePackLost(void);

/* Helpers, exposed for the tests. */
uint16_t BmsGauge_OcvToPermille(int16_t cell_mv, bool after_charge);

#endif /* BMS_GAUGE_H */
