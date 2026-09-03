#ifndef BMS_BOARD_H
#define BMS_BOARD_H

#include "board_rev.h"

/*
 * BQ76922 on shared I2C4 with TPS25751.
 *
 * Hardware is a 4S Li-ion pack on the 5-channel AFE: VC4 is unused
 * (user pack / harness skips that tap). VCell Mode must omit cell 4
 * or CUV/imbalance on the empty input keeps CHG/DSG FETs off — button
 * wake and USB-C bring-up then look "alive" (G4 boots, cells 1/2/3/5
 * read OK) while the pack path stays locked.
 *
 * Wake hardware (BQ769x2):
 *   - Button → TS2 pulled to VSS (exit SHUTDOWN)
 *   - USB-C / charger → LD pin > VWAKEONLD (~1.45 V)
 *   - REG1/REG18 from BAT keep G4 alive with CHG/DSG OFF; PACK needs FETs
 *
 * Blank OTP boots FET Test Mode (Mfg Status Init FET_EN=0). Host must
 * FET_ENABLE() then ALL_FETS_ON() every wake — RAM only, no OTP burn.
 *
 * BMS_ENABLE=0: skip I2C init/scan and do not trip on BMS faults (bring-up).
 * BMS_ENABLE=1: full monitor + shutdown on hardware safety faults.
 */
#ifndef BMS_ENABLE
#define BMS_ENABLE                       1U
#endif

#define BMS_U1_THRESHOLD_CODE_MV(mv)     ((uint8_t)((((uint32_t)(mv) * 10U) + 253U) / 506U))
#define BMS_COV_THRESHOLD_CODE           BMS_U1_THRESHOLD_CODE_MV(BMS_COV_THRESHOLD_MV)
#define BMS_CUV_THRESHOLD_CODE           BMS_U1_THRESHOLD_CODE_MV(BMS_CUV_THRESHOLD_MV)
#define BMS_COV_RECOVERY_CODE            BMS_U1_THRESHOLD_CODE_MV(BMS_COV_RECOVERY_MV)
#define BMS_CUV_RECOVERY_CODE            BMS_U1_THRESHOLD_CODE_MV(BMS_CUV_RECOVERY_MV)

/* AFE has 5 cell registers; series count is 4 (skip VC4). */
#define BMS_AFE_CELL_CHANNELS            5U
#define BMS_CELL_COUNT                   BMS_AFE_CELL_CHANNELS
#define BMS_SERIES_COUNT                 4U

/* Settings:VCell Mode — bit0=C1 … bit4=C5. 4S skip VC4: cells 1,2,3,5. */
#define BMS_VCELL_MODE                   0x0017U
#define BMS_VCELL_MODE_4S                BMS_VCELL_MODE
#define BMS_VCELL_MODE_5S                0x001FU
#define BMS_UNUSED_CELL_MV               (-1)

#define BMS_COV_THRESHOLD_MV             4250U
#define BMS_CUV_THRESHOLD_MV             2800U
#define BMS_COV_RECOVERY_MV              4150U
#define BMS_CUV_RECOVERY_MV              2900U

#define BMS_WARN_CELL_DELTA_MV           200U
#define BMS_WARN_CELL_LOW_MV             3000U
#define BMS_WARN_CELL_HIGH_MV            4200U
#define BMS_WARN_PACK_LOW_MV             11000U
#define BMS_WARN_PACK_HIGH_MV            17000U

/* BQ76922 SRP/SRN sense resistor (pack current). Not the DCDC INA296 shunt. */
#define BMS_SENSE_MOHM                   5U
/*
 * CC Gain = 7.4768 / Rsense_mOhm (TI calib guide). Default OTP is for ~1 mOhm
 * (CC Gain 7.4768) — with a 5 mOhm shunt CC2/i_pack reads ~5× too high until
 * this is written in CONFIG_UPDATE.
 */
#define BMS_CC_GAIN                      (7.4768f / (float)BMS_SENSE_MOHM)
#define BMS_CAPACITY_GAIN                (BMS_CC_GAIN * 298261.6178f)

/*
 * Enabled Protections A (TI TRM bits 7..2):
 *   SCD=0x80 OCD2=0x40 OCD1=0x20 OCC=0x10 COV=0x08 CUV=0x04
 * 0xBC = SCD|OCD1|OCC|COV|CUV (no OCD2 — SCD covers hard shorts).
 * Never use the reversed bit0..5 map; that labeled real OCC (0x10) as COV.
 */
#define BMS_ENABLED_PROTECTIONS_A        0xBCU
/* Prot B: leave OT/UT off until TS pins are proven; TS2 is the wake button. */
#define BMS_ENABLED_PROTECTIONS_B        0x00U

/*
 * FET Options 0x9308: SFET|HOST_FETOFF_EN|FET_CTRL_EN|PDSG_EN (TI demo 0x1D).
 * PDSG_EN soft-starts PACK caps before DSG — without it ALL_FETS_ON into
 * VIN/GaN capacitance trips SCD (sa bit7) and the pack path collapses hard
 * enough to BOR the 3V3 rail (PIN+POR reboot loop).
 */
#define BMS_FET_OPTIONS                  0x1DU
/*
 * CFETOFF / DFETOFF Pin Config (0x92FA / 0x92FB): PIN_FXN=0 → unused.
 * HW rev2 leaves TP29/TP30 floating; if PIN_FXN=CFETOFF (0x02) and the pin
 * floats high (active-high default), CHG stays forced off forever
 * (fet≈0x14 = DSG + DCHG_PIN) even after ALL_FETS_ON. Explicit 0x00.
 */
#define BMS_CFETOFF_PIN_CONFIG           0x00U
#define BMS_DFETOFF_PIN_CONFIG           0x00U
/* Predischarge Timeout 0x930E (U1, ~10 ms/step); Stop Delta 0x930F (U1, 10 mV). */
#define BMS_PDSG_TIMEOUT                 0x32U  /* ~500 ms max predischarge */
#define BMS_PDSG_STOP_DELTA              50U    /* exit PDSG when |stack-pack| < 500 mV */
/*
 * SCD Threshold 0x9286 index — hardware mV on sense, independent of CC Gain.
 * 0x05 ≈ 100 mV → ~20 A across 5 mOhm.
 */
#define BMS_SCD_THRESHOLD                0x05U
/*
 * OCC / OCD1 Threshold: U1, 2 mV/LSB on the sense resistor (not CC Gain).
 * Default OCC=2 → 4 mV / 5 mOhm ≈ 0.8 A — trips immediately when BQ25731
 * tries ~8 A charge. Code = I_mA * R_mOhm / 2000, clamped to TI min/max.
 */
#define BMS_OCC_THRESHOLD_MA             10000U /* ~10 A @ 5 mOhm → 50 mV → code 25 */
#define BMS_OCD1_THRESHOLD_MA            15000U /* ~15 A @ 5 mOhm → 75 mV → code 38 */
#define BMS_SENSE_MV_CODE_2MV(ma) \
    ((uint8_t)((((uint32_t)(ma) * (uint32_t)BMS_SENSE_MOHM) + 1000U) / 2000U))
#define BMS_OCC_THRESHOLD_CODE \
    ((BMS_SENSE_MV_CODE_2MV(BMS_OCC_THRESHOLD_MA) < 2U) ? 2U : \
     ((BMS_SENSE_MV_CODE_2MV(BMS_OCC_THRESHOLD_MA) > 62U) ? 62U : \
      BMS_SENSE_MV_CODE_2MV(BMS_OCC_THRESHOLD_MA)))
#define BMS_OCD1_THRESHOLD_CODE \
    ((BMS_SENSE_MV_CODE_2MV(BMS_OCD1_THRESHOLD_MA) < 2U) ? 2U : \
     ((BMS_SENSE_MV_CODE_2MV(BMS_OCD1_THRESHOLD_MA) > 100U) ? 100U : \
      BMS_SENSE_MV_CODE_2MV(BMS_OCD1_THRESHOLD_MA)))
/*
 * OCC recovers when I <= this for Recovery:Time (default -200 mA needs
 * discharge). With CHG latched off and a USB charger still raising PACK,
 * I≈0 never meets -200 mA and PACK-TOS also fails — CHG stays off forever.
 * +100 mA lets I≈0 clear OCC once the charge pulse is gone.
 */
#define BMS_OCC_RECOVERY_MA              100
/* Body Diode Threshold 0x9273 (mA after CC Gain). */
#define BMS_BODY_DIODE_THRESHOLD_MA      2000U

#define BMS_CELL_USED(i)                 (((BMS_VCELL_MODE >> (i)) & 1U) != 0U)

#endif /* BMS_BOARD_H */
