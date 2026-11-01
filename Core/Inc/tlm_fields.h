#ifndef TLM_FIELDS_H
#define TLM_FIELDS_H

/*
 * ONE telemetry dictionary for G4 → H7 (binary) and G4 → PC (text).
 * This file is shared verbatim with GUI_Lab_PD_PSU (Appli/Core/Inc/).
 *
 * Binary: frame type 0x15 FIELDS on the existing A5 5A link.
 *   payload = u8 version(=1) | u8 group | u8 part | u8 parts | entries…
 *   entry   = u8 id | u8 type | value (little-endian, length = type & 0x0F)
 *   type    = length 1/2/4 in bits 3..0, bit 4 = signed.
 * A group snapshot may span several frames (part 0..parts-1); the receiver
 * starts a fresh record on part 0 and publishes it after the last part.
 * A field that is not available is simply not sent (no sentinels).
 * Unknown ids are skipped by length, so adding a field is one line here
 * plus one TlmPut() on the G4 — old H7 firmware keeps working.
 *
 * Text: "TLM" on USART1/TEXT_CMD prints "<group> name=value …" lines from
 * the same table.
 *
 * Rules: never reuse or renumber an id; names are the keys the H7 UI reads.
 */

#include <stdint.h>

#define TLM_FIELDS_VERSION   1U
#define TLM_FRAME_FIELDS     0x15U

#define TLM_GROUP_BMS        1U   /* AFE: cells, FETs, protections  → H7 record TB  */
#define TLM_GROUP_BATT       2U   /* gauge: SOC/SOH/cycles/store    → H7 record BATT */
#define TLM_GROUP_PD         3U   /* charger + USB-C/PD             → H7 record TC  */
#define TLM_GROUP_AUX        4U   /* G0 DAC/NTC, fan, remote sense  → H7 record AUX */

#define TLM_U8   0x01U
#define TLM_U16  0x02U
#define TLM_U32  0x04U
#define TLM_I8   0x11U
#define TLM_I16  0x12U
#define TLM_I32  0x14U
#define TLM_TYPE_LEN(t)     ((uint8_t)((t) & 0x0FU))
#define TLM_TYPE_SIGNED(t)  (((t) & 0x10U) != 0U)

/* X(sym, id, group, "name", type) */
#define TLM_FIELDS(X) \
    /* ---- BMS (AFE) ---------------------------------------------------- */ \
    X(BMS_PRESENT,      1, BMS,  "bms",          TLM_U8)  \
    X(BMS_CFG,          2, BMS,  "cfg",          TLM_U8)  \
    X(BMS_STATE,        3, BMS,  "st",           TLM_U8)  \
    X(BMS_ALERT,        4, BMS,  "alert",        TLM_U8)  \
    X(BMS_FAULT,        5, BMS,  "fault",        TLM_U32) \
    X(BMS_ALARM,        6, BMS,  "alarm",        TLM_U16) \
    X(BMS_SA,           7, BMS,  "sa",           TLM_U8)  \
    X(BMS_SB,           8, BMS,  "sb",           TLM_U8)  \
    X(BMS_SC,           9, BMS,  "sc",           TLM_U8)  \
    X(BMS_FET,         10, BMS,  "fet",          TLM_U8)  \
    X(BMS_MANUF,       11, BMS,  "manuf",        TLM_U16) \
    X(BMS_INIT_STEP,   12, BMS,  "init_step",    TLM_U8)  \
    X(BMS_CFG_FAIL,    13, BMS,  "cfg_fail",     TLM_U16) \
    X(BMS_VCELL_RB,    14, BMS,  "vcell_rb",     TLM_U16) \
    X(BMS_BATT_STATUS, 15, BMS,  "batt",         TLM_U16) \
    X(BMS_CHG,         16, BMS,  "chg",          TLM_U8)  \
    X(BMS_DSG,         17, BMS,  "dsg",          TLM_U8)  \
    X(BMS_FETS,        18, BMS,  "fets",         TLM_U8)  \
    X(BMS_SERIES,      19, BMS,  "series",       TLM_U8)  \
    X(BMS_C1_MV,       20, BMS,  "c1_mv",        TLM_I16) \
    X(BMS_C2_MV,       21, BMS,  "c2_mv",        TLM_I16) \
    X(BMS_C3_MV,       22, BMS,  "c3_mv",        TLM_I16) \
    X(BMS_C4_MV,       23, BMS,  "c4_mv",        TLM_I16) \
    X(BMS_C5_MV,       24, BMS,  "c5_mv",        TLM_I16) \
    X(BMS_MIN_MV,      25, BMS,  "min_mv",       TLM_I16) \
    X(BMS_MAX_MV,      26, BMS,  "max_mv",       TLM_I16) \
    X(BMS_DV_MV,       27, BMS,  "dV_mv",        TLM_I16) \
    X(BMS_SUM_MV,      28, BMS,  "sum_mv",       TLM_U16) \
    X(BMS_PACK_MV,     29, BMS,  "pack_mv",      TLM_U16) \
    X(BMS_STACK_MV,    30, BMS,  "stack_mv",     TLM_U16) \
    X(BMS_I_PACK_MA,   31, BMS,  "i_pack_ma",    TLM_I32) \
    X(BMS_SAMPLE,      32, BMS,  "sample",       TLM_U8)  \
    X(BMS_PASSQ_MAH,   33, BMS,  "passq_mah",    TLM_I32) \
    X(BMS_CC1_MA,      34, BMS,  "cc1_ma",       TLM_I16) \
    X(BMS_INT_TEMP_DK, 35, BMS,  "int_temp_dK",  TLM_I16) \
    X(BMS_TS1_DK,      36, BMS,  "ts1_dK",       TLM_I16) \
    X(BMS_BALANCE,     37, BMS,  "balance_mask", TLM_U8)  \
    X(BMS_SOC,         38, BMS,  "soc_permille", TLM_U16) \
    X(BMS_SOC_FLAGS,   39, BMS,  "soc_flags",    TLM_U16) \
    X(BMS_GOLDEN,      40, BMS,  "golden",       TLM_U8)  \
    X(BMS_I2C_ERR,     41, BMS,  "i2c_err",      TLM_U32) \
    X(BMS_ALERTS,      42, BMS,  "alerts",       TLM_U32) \
    /* ---- BATT (gauge) -------------------------------------------------- */ \
    X(BATT_REM_MAH,    60, BATT, "rem_mah",      TLM_I32) \
    X(BATT_FCC_MAH,    61, BATT, "fcc_mah",      TLM_I32) \
    X(BATT_DESIGN_MAH, 62, BATT, "design_mah",   TLM_U16) \
    X(BATT_SOH,        63, BATT, "soh_permille", TLM_U16) \
    X(BATT_CYCLES,     64, BATT, "cycles_x100",  TLM_U16) \
    X(BATT_RCELL,      65, BATT, "rcell_x10",    TLM_I16) \
    X(BATT_OCV_MV,     66, BATT, "ocv_mv",       TLM_I16) \
    X(BATT_TTE_MIN,    67, BATT, "tte_min",      TLM_U16) \
    X(BATT_TTF_MIN,    68, BATT, "ttf_min",      TLM_U16) \
    X(BATT_ANCHOR,     69, BATT, "anchor",       TLM_U8)  \
    X(BATT_LEARN_N,    70, BATT, "learn_n",      TLM_U8)  \
    X(BATT_LEARN_MAH,  71, BATT, "learn_mah",    TLM_I32) \
    X(BATT_CHG_TOTAL,  72, BATT, "chg_total_mah", TLM_U32) \
    X(BATT_DSG_TOTAL,  73, BATT, "dsg_total_mah", TLM_U32) \
    X(BATT_STORE_SEQ,  74, BATT, "store_seq",    TLM_U32) \
    X(BATT_STORE_OK,   75, BATT, "store_ok",     TLM_U8)  \
    X(BATT_UNSAVED,    76, BATT, "unsaved_mah",  TLM_I32) \
    X(BATT_TEMP_DC,    77, BATT, "temp_dC",      TLM_I16) \
    X(BATT_CHEM,       78, BATT, "chem",         TLM_U8)  \
    X(BATT_CELLS,      79, BATT, "cell_n",       TLM_U8)  \
    X(BATT_SOC,        80, BATT, "soc_permille", TLM_U16) \
    X(BATT_FLAGS,      81, BATT, "soc_flags",    TLM_U16) \
    /* ---- PD (charger + USB-C) ----------------------------------------- */ \
    X(PD_BQ_OK,       100, PD,   "bq_ok",        TLM_U8)  \
    X(PD_BQ_IN,       101, PD,   "bq_in",        TLM_U8)  \
    X(PD_BQ_PRE,      102, PD,   "bq_pre",       TLM_U8)  \
    X(PD_BQ_FAST,     103, PD,   "bq_fast",      TLM_U8)  \
    X(PD_BQ_OTG,      104, PD,   "bq_otg",       TLM_U8)  \
    X(PD_BQ_IINDPM,   105, PD,   "bq_iindpm",    TLM_U8)  \
    X(PD_BQ_VINDPM,   106, PD,   "bq_vindpm",    TLM_U8)  \
    X(PD_RST_BUSY,    107, PD,   "rst_busy",     TLM_U8)  \
    X(PD_PLUG,        108, PD,   "plug",         TLM_U8)  \
    X(PD_BQ_ST,       109, PD,   "bq_st",        TLM_U16) \
    X(PD_BQ_FAULT,    110, PD,   "bq_fault",     TLM_U8)  \
    X(PD_CC1,         111, PD,   "cc1",          TLM_U8)  \
    X(PD_CC2,         112, PD,   "cc2",          TLM_U8)  \
    X(PD_ROLE,        113, PD,   "role",         TLM_U8)  \
    X(PD_CONN,        114, PD,   "conn",         TLM_U8)  \
    X(PD_TYPEC,       115, PD,   "typec",        TLM_U8)  \
    X(PD_PD_ROLE,     116, PD,   "pd_role",      TLM_U8)  \
    X(PD_ATTACH_GEN,  117, PD,   "attach_gen",   TLM_U32) \
    X(PD_VBAT_MV,     118, PD,   "bq_vbat_mv",   TLM_U32) \
    X(PD_VSYS_MV,     119, PD,   "bq_vsys_mv",   TLM_U32) \
    X(PD_IBAT_MA,     120, PD,   "bq_ibat_ma",   TLM_I32) \
    X(PD_ICHG_MA,     121, PD,   "bq_ichg_ma",   TLM_U32) \
    X(PD_IDCHG_MA,    122, PD,   "bq_idchg_ma",  TLM_U32) \
    X(PD_VBUS_MV,     123, PD,   "bq_vbus_mv",   TLM_U32) \
    X(PD_IIN_MA,      124, PD,   "bq_iin_ma",    TLM_U32) \
    X(PD_VREG_MV,     125, PD,   "bq_vreg_mv",   TLM_U32) \
    X(PD_ICHG_SET_MA, 126, PD,   "bq_ichg_set_ma", TLM_U32) \
    X(PD_IIN_SET_MA,  127, PD,   "bq_iin_set_ma",  TLM_U32) \
    X(PD_TPS_VBUS_MV, 128, PD,   "tps_vbus_mv",  TLM_U32) \
    X(PD_PD_MV,       129, PD,   "pd_mv",        TLM_U32) \
    X(PD_PD_MA,       130, PD,   "pd_ma",        TLM_U32) \
    /* ---- AUX (G0 / fan / remote sense) -------------------------------- */ \
    X(AUX_DAC_CV_MV,  150, AUX,  "dac_cv_mv",    TLM_U32) \
    X(AUX_DAC_CC_MV,  151, AUX,  "dac_cc_mv",    TLM_U32) \
    X(AUX_T1,         152, AUX,  "t1",           TLM_I16) \
    X(AUX_T2,         153, AUX,  "t2",           TLM_I16) \
    X(AUX_T3,         154, AUX,  "t3",           TLM_I16) \
    X(AUX_T4,         155, AUX,  "t4",           TLM_I16) \
    X(AUX_FAN,        156, AUX,  "fan",          TLM_U8)  \
    X(AUX_PGOOD,      157, AUX,  "pgood",        TLM_U8)  \
    X(AUX_BLEED,      158, AUX,  "bleed",        TLM_U8)  \
    X(AUX_VALID,      159, AUX,  "valid",        TLM_U8)  \
    X(AUX_LOCAL_MV,   160, AUX,  "local_mv",     TLM_U16) \
    X(AUX_REMOTE_P_MV,161, AUX,  "remote_p_mv",  TLM_U16) \
    X(AUX_REMOTE_N_MV,162, AUX,  "remote_n_mv",  TLM_U16) \
    X(AUX_SENSE_CODE, 163, AUX,  "sense_code",   TLM_U8)  \
    X(AUX_SENSE_FLAGS,164, AUX,  "sense_flags",  TLM_U8)  \
    X(AUX_FAN_RPM,    165, AUX,  "fan_rpm",      TLM_U16) \
    X(AUX_STOP_REASON,166, AUX,  "stop_reason",  TLM_U8)

typedef enum {
#define TLM_ENUM_(sym, id, grp, name, type) TLM_##sym = (id),
    TLM_FIELDS(TLM_ENUM_)
#undef TLM_ENUM_
} TlmId_t;

typedef struct {
    uint8_t id;
    uint8_t group;
    uint8_t type;
    const char *name;
} TlmFieldInfo_t;

/* Lookup by id (NULL when unknown). Implemented in tlm_fields.c on the G4
 * and by the same X-macro on the H7. */
const TlmFieldInfo_t *Tlm_Find(uint8_t id);
const char *Tlm_GroupName(uint8_t group);

/* ---------------------------------------------------- G4 side builder -- */
#define TLM_MAX_ENTRIES      64U
#define TLM_MAX_PAYLOAD      113U
#define TLM_HEADER_BYTES     4U

typedef struct {
    uint8_t group;
    uint8_t count;
    uint8_t id[TLM_MAX_ENTRIES];
    int32_t value[TLM_MAX_ENTRIES];
} TlmBuilder_t;

void Tlm_Begin(TlmBuilder_t *b, uint8_t group);
/* Ignored (and returns 0) for an unknown id or a full builder. */
int Tlm_Put(TlmBuilder_t *b, uint8_t id, int32_t value);
/* Number of frames this snapshot needs. */
uint8_t Tlm_Parts(const TlmBuilder_t *b);
/* Encode part `part` into out (≤ TLM_MAX_PAYLOAD). Returns payload length,
 * 0 when part is out of range. */
uint8_t Tlm_EncodePart(const TlmBuilder_t *b, uint8_t part, uint8_t *out);
/* "BMS bms=1 cfg=1 …" — returns length written (always NUL-terminated). */
unsigned Tlm_Format(const TlmBuilder_t *b, char *out, unsigned out_len);

/* ---------------------------------------------------- decoder (both) -- */
typedef void (*TlmFieldCb)(void *ctx, uint8_t id, int32_t value);
/* Returns 0 on a malformed payload. group/part/parts are returned. */
int Tlm_Decode(const uint8_t *payload, uint8_t len, uint8_t *group,
               uint8_t *part, uint8_t *parts, TlmFieldCb cb, void *ctx);

#endif /* TLM_FIELDS_H */
