#include "bms_golden.h"

#include "bms_board.h"

#include <stddef.h>

#define FP   BMS_DM_FLAG_FINGERPRINT
#define U1(a, v, n, f)  { (a), BMS_DM_U1, (f), { .u = (uint32_t)(uint8_t)(v) },  (n) }
#define I1(a, v, n, f)  { (a), BMS_DM_U1, (f), { .u = (uint32_t)(uint8_t)(int8_t)(v) }, (n) }
#define U2(a, v, n, f)  { (a), BMS_DM_U2, (f), { .u = (uint32_t)(uint16_t)(v) }, (n) }
#define I2(a, v, n, f)  { (a), BMS_DM_U2, (f), { .u = (uint32_t)(uint16_t)(int16_t)(v) }, (n) }
#define F4(a, v, n, fl) { (a), BMS_DM_F4, (fl), { .f = (v) }, (n) }

/*
 * Order matters only for readability. Addresses: BQ76922 TRM SLUUCG7 §13.
 * Every value here ends up in OTP — change it only together with
 * docs/BMS_5S_LFP.md and only before the burn.
 */
const BmsDmEntry_t g_bms_golden[] = {
    /* Configuration */
    U1(0x9236, BMS_REG1_CONFIG,            "REG1 Config", 0),
    U1(0x9237, BMS_REG0_CONFIG,            "REG0 Config", 0),
    U2(0x9304, BMS_VCELL_MODE,             "Vcell Mode", FP),
    U1(0x92FA, BMS_CFETOFF_PIN_CONFIG,     "CFETOFF Pin Config", 0),
    U1(0x92FB, BMS_DFETOFF_PIN_CONFIG,     "DFETOFF Pin Config", 0),
    U1(0x92FD, BMS_TS1_CONFIG,             "TS1 Config", FP),
    U1(0x92FE, BMS_TS2_CONFIG,             "TS2 Config", 0),
    U2(0x9343, BMS_MFG_STATUS_INIT,        "Mfg Status Init", FP),

    /* Current sense */
    F4(0x91A8, BMS_CC_GAIN,                "CC Gain", FP),
    F4(0x91AC, BMS_CAPACITY_GAIN,          "Capacity Gain", FP),
    I2(0x91C8, BMS_BOARD_OFFSET,           "Board Offset", FP),
    I1(0x91CE, BMS_TS1_TEMP_OFFSET_DC,     "TS1 Temp Offset", FP),

    /* Vishay NTCLE100E103JB0A, 18k model: TI GPC report 2026-11-02.
     * Reported nominal maximum fit error 0.5 C, range -40..125 C.
     * All coefficients are fingerprinted so old RAM forces a full update. */
    I2(0x91EA,  -16652, "Vishay 18K a1", FP),
    I2(0x91EC,   29010, "Vishay 18K a2", FP),
    I2(0x91EE,  -22024, "Vishay 18K a3", FP),
    I2(0x91F0,   26321, "Vishay 18K a4", FP),
    I2(0x91F2,    1343, "Vishay 18K a5", FP),
    I2(0x91F4,   -3958, "Vishay 18K b1", FP),
    I2(0x91F6,    5551, "Vishay 18K b2", FP),
    I2(0x91F8,   -4365, "Vishay 18K b3", FP),
    I2(0x91FA,    4554, "Vishay 18K b4", FP),
    I2(0x91FE,   11703, "Vishay 18K Adc0", FP),

    /* Protections enable / FET mapping */
    U1(0x9261, BMS_ENABLED_PROTECTIONS_A,  "Enabled Protections A", FP),
    U1(0x9262, BMS_ENABLED_PROTECTIONS_B,  "Enabled Protections B", FP),
    U1(0x9265, BMS_CHG_FET_PROTECTIONS_A,  "CHG FET Protections A", 0),
    U1(0x9266, BMS_CHG_FET_PROTECTIONS_B,  "CHG FET Protections B", 0),
    U1(0x9269, BMS_DSG_FET_PROTECTIONS_A,  "DSG FET Protections A", 0),
    U1(0x926A, BMS_DSG_FET_PROTECTIONS_B,  "DSG FET Protections B", 0),
    I2(0x9273, BMS_BODY_DIODE_THRESHOLD_MA, "Body Diode Threshold", 0),

    /* Cell voltage */
    U1(0x9275, BMS_CUV_THRESHOLD_CODE,     "CUV Threshold", FP),
    U1(0x927B, BMS_CUV_RECOVERY_HYST,      "CUV Recovery Hysteresis", 0),
    U1(0x9278, BMS_COV_THRESHOLD_CODE,     "COV Threshold", FP),
    U1(0x927C, BMS_COV_RECOVERY_HYST,      "COV Recovery Hysteresis", 0),

    /* Current */
    U1(0x9280, BMS_OCC_THRESHOLD_CODE,     "OCC Threshold", 0),
    I2(0x9288, BMS_OCC_RECOVERY_MA,        "OCC Recovery Threshold", 0),
    U1(0x9282, BMS_OCD1_THRESHOLD_CODE,    "OCD1 Threshold", 0),
    U1(0x9286, BMS_SCD_THRESHOLD,          "SCD Threshold", 0),

    /* Temperature (TS1 NTC = cell, internal die) */
    I1(0x929A, BMS_OTC_C,                  "OTC Threshold", FP),
    I1(0x929C, BMS_OTC_RECOVERY_C,         "OTC Recovery", FP),
    I1(0x929D, BMS_OTD_C,                  "OTD Threshold", 0),
    I1(0x929F, BMS_OTD_RECOVERY_C,         "OTD Recovery", 0),
    I1(0x92A3, BMS_OTINT_C,                "OTINT Threshold", 0),
    I1(0x92A5, BMS_OTINT_RECOVERY_C,       "OTINT Recovery", 0),
    I1(0x92A6, BMS_UTC_C,                  "UTC Threshold", 0),
    I1(0x92A8, BMS_UTC_RECOVERY_C,         "UTC Recovery", 0),
    I1(0x92A9, BMS_UTD_C,                  "UTD Threshold", 0),
    I1(0x92AB, BMS_UTD_RECOVERY_C,         "UTD Recovery", 0),

    /* FETs */
    U1(0x9308, BMS_FET_OPTIONS,            "FET Options", FP),
    I2(0x930A, BMS_PCHG_START_MV,          "Precharge Start Voltage", 0),
    I2(0x930C, BMS_PCHG_STOP_MV,           "Precharge Stop Voltage", 0),
    U1(0x930E, BMS_PDSG_TIMEOUT,           "Predischarge Timeout", FP),
    U1(0x930F, BMS_PDSG_STOP_DELTA,        "Predischarge Stop Delta", FP),

    /* Power / shutdown */
    U1(0x9254, BMS_AUTO_SHUTDOWN_TIME,     "Auto Shutdown Time", 0),
    I2(0x923F, BMS_SHUTDOWN_CELL_MV,       "Shutdown Cell Voltage", 0),
    I2(0x9241, BMS_SHUTDOWN_STACK_10MV,    "Shutdown Stack Voltage", 0),

    /* Cell balancing (autonomous, top of charge only) */
    U1(0x9335, BMS_BALANCING_CONFIG,       "Balancing Configuration", FP),
    I1(0x9336, BMS_CB_MIN_CELL_TEMP_C,     "Min Cell Temp", 0),
    I1(0x9337, BMS_CB_MAX_CELL_TEMP_C,     "Max Cell Temp", 0),
    I1(0x9338, BMS_CB_MAX_INTERNAL_TEMP_C, "Max Internal Temp", 0),
    U1(0x9339, BMS_CB_INTERVAL_S,          "Cell Balance Interval", 0),
    U1(0x933A, BMS_CB_MAX_CELLS,           "Cell Balance Max Cells", 0),
    I2(0x933B, BMS_CB_MIN_CELL_MV_CHG,     "Cell Balance Min Cell V (Charge)", FP),
    U1(0x933D, BMS_CB_MIN_DELTA_CHG_MV,    "Cell Balance Min Delta (Charge)", 0),
    U1(0x933E, BMS_CB_STOP_DELTA_CHG_MV,   "Cell Balance Stop Delta (Charge)", 0),
    I2(0x933F, BMS_CB_MIN_CELL_MV_RLX,     "Cell Balance Min Cell V (Relax)", 0),
    U1(0x9341, BMS_CB_MIN_DELTA_RLX_MV,    "Cell Balance Min Delta (Relax)", 0),
    U1(0x9342, BMS_CB_STOP_DELTA_RLX_MV,   "Cell Balance Stop Delta (Relax)", 0),
};

const uint16_t g_bms_golden_count =
    (uint16_t)(sizeof(g_bms_golden) / sizeof(g_bms_golden[0]));

void BmsGolden_Bytes(const BmsDmEntry_t *e, uint8_t out[4])
{
    uint32_t u = e->value.u;

    out[0] = (uint8_t)(u & 0xFFU);
    out[1] = (uint8_t)((u >> 8) & 0xFFU);
    out[2] = (uint8_t)((u >> 16) & 0xFFU);
    out[3] = (uint8_t)((u >> 24) & 0xFFU);
}

static uint32_t BmsGolden_CrcByte(uint32_t crc, uint8_t b)
{
    uint8_t i;

    crc ^= b;
    for (i = 0U; i < 8U; i++) {
        crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
    return crc;
}

uint32_t BmsGolden_Crc32(void)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint16_t i;

    for (i = 0U; i < g_bms_golden_count; i++) {
        const BmsDmEntry_t *e = &g_bms_golden[i];
        uint8_t b[4];
        uint8_t k;

        crc = BmsGolden_CrcByte(crc, (uint8_t)(e->addr & 0xFFU));
        crc = BmsGolden_CrcByte(crc, (uint8_t)(e->addr >> 8));
        crc = BmsGolden_CrcByte(crc, e->type);
        BmsGolden_Bytes(e, b);
        for (k = 0U; k < e->type; k++) {
            crc = BmsGolden_CrcByte(crc, b[k]);
        }
    }
    return ~crc;
}
