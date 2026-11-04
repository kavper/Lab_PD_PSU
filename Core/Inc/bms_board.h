#ifndef BMS_BOARD_H
#define BMS_BOARD_H

#include "board_rev.h"

/*
 * BQ76922 on shared I2C4 with TPS25751.
 *
 * Pack: 5S1P LiFePO4, 6.7 Ah nominal (all five AFE inputs used, VCell Mode
 * 0x001F).  The complete Data Memory image the firmware writes to RAM and
 * the image that is burned to OTP are the same table: bms_golden.c.  This
 * header only holds the human-readable numbers that table is built from.
 *
 * Wake hardware (BQ769x2):
 *   - Button → TS2 pulled to VSS (exit SHUTDOWN) — product wake, no host cmd
 *   - USB-C / charger → LD pin > VWAKEONLD (~1.45 V)
 *   - Blank / foreign OTP: MCU writes the golden table to RAM on every wake,
 *     then FET_ENABLE + ALL_FETS_ON.
 *   - Golden OTP: BQ turns the FETs on from TS2 alone (FET_EN + FET_INIT_OFF=0)
 *     and the MCU skips CONFIG_UPDATE when the fingerprint matches.
 *
 * BMS_ENABLE=0: skip I2C init/scan and do not trip on BMS faults (bring-up).
 */
#ifndef BMS_ENABLE
#define BMS_ENABLE                       1U
#endif

/*
 * OTP programming commands (BMS OTP CHECK / BURN) and the OTP_WRITE code path
 * exist only in a service build: cmake --preset OtpService (or -DBMS_OTP_SERVICE=1).
 * The production image keeps the read-only BMS OTP STATUS.
 */
#ifndef BMS_OTP_SERVICE
#define BMS_OTP_SERVICE                  0U
#endif

/* ---------------------------------------------------------------- pack -- */
#define BMS_CHEMISTRY_LFP                1U
#define BMS_AFE_CELL_CHANNELS            5U
#define BMS_CELL_COUNT                   BMS_AFE_CELL_CHANNELS
#define BMS_SERIES_COUNT                 5U
#define BMS_DESIGN_CAPACITY_MAH          6700
/* Settings:VCell Mode — bit0=C1 … bit4=C5. 5S: every input is a cell. */
#define BMS_VCELL_MODE                   0x001FU
#define BMS_VCELL_MODE_5S                0x001FU
#define BMS_VCELL_MODE_4S_SKIP4          0x0017U /* old harness, reference only */
#define BMS_UNUSED_CELL_MV               (-1)
#define BMS_CELL_USED(i)                 (((BMS_VCELL_MODE >> (i)) & 1U) != 0U)

/* ------------------------------------------------- LFP voltage window -- */
/*
 * Hardware limits (AFE, autonomous, burned into OTP):
 *   COV 3.70 V   — backstop above the 3.60 V charge target, below the cell
 *                  absolute maximum region. Recovery 2 × 50.6 mV ≈ 3.59 V.
 *   CUV 2.50 V   — hard floor. Software stops the load at 2.80 V first.
 *   Shutdown 2.30 V/cell — BQ enters SHUTDOWN (nA drain) on a flat pack;
 *                  a charger on PACK/LD aborts that, so the pack can be
 *                  recovered by charging.
 * Software window (G4):
 *   charge target 3.60 V/cell (18.00 V pack), low warning 3.00 V,
 *   load cut-off 2.80 V (under load, filtered).
 */
#define BMS_COV_THRESHOLD_MV             3700U
#define BMS_CUV_THRESHOLD_MV             2500U
#define BMS_COV_RECOVERY_HYST            2U   /* × 50.6 mV */
#define BMS_CUV_RECOVERY_HYST            4U   /* × 50.6 mV ≈ 2.70 V recover */
#define BMS_SHUTDOWN_CELL_MV             2300U
#define BMS_SHUTDOWN_STACK_10MV          1000U /* 10.0 V stack */

#define BMS_CHARGE_TARGET_CELL_MV        3600U
#define BMS_CHARGE_TARGET_PACK_MV        (BMS_CHARGE_TARGET_CELL_MV * BMS_SERIES_COUNT)
#define BMS_WARN_CELL_LOW_MV             3000U
#define BMS_CUTOFF_CELL_MV               2800U
#define BMS_WARN_CELL_HIGH_MV            3600U
#define BMS_WARN_CELL_DELTA_MV           100U
#define BMS_WARN_PACK_LOW_MV             (BMS_WARN_CELL_LOW_MV * BMS_SERIES_COUNT)
#define BMS_WARN_PACK_HIGH_MV            (BMS_WARN_CELL_HIGH_MV * BMS_SERIES_COUNT)

/* U1 thresholds in 50.6 mV steps, rounded to nearest. */
#define BMS_U1_THRESHOLD_CODE_MV(mv)     ((uint8_t)((((uint32_t)(mv) * 10U) + 253U) / 506U))
#define BMS_COV_THRESHOLD_CODE           BMS_U1_THRESHOLD_CODE_MV(BMS_COV_THRESHOLD_MV)
#define BMS_CUV_THRESHOLD_CODE           BMS_U1_THRESHOLD_CODE_MV(BMS_CUV_THRESHOLD_MV)
/* Effective trip voltages after rounding (3694 mV / 2479 mV). */
#define BMS_COV_EFFECTIVE_MV             ((BMS_COV_THRESHOLD_CODE * 506U) / 10U)
#define BMS_CUV_EFFECTIVE_MV             ((BMS_CUV_THRESHOLD_CODE * 506U) / 10U)

/* ------------------------------------------------------- temperature -- */
/*
 * Actual fitted part: Vishay NTCLE100E103JB0A, 10 kOhm at 25 C,
 * B25/85=3977 K, located on PCB (not on the cells).
 * 18K polynomial coefficients are from the TI GPC report (2026-11-02).
 * Nominal curve fitting is complete; physical thermal validation and
 * actual cell temperature sensing are still required before OTP.
 */
/* Deliberate OTP release gate: do not set to 1 before fitting and verifying
 * the exact NTC polynomial and resolving actual cell temperature sensing. */
#define BMS_THERMISTOR_MODEL_VALIDATED   0U
#define BMS_TS1_CONFIG                   0x07U
#define BMS_OTC_C                        55
/* TS1 Temp Offset (0x91CE, 0.1 C).  2026-11-04: TS1 read 20.35 C against a
 * Fluke thermocouple at 21.9 C on the NTC.  NTCLE100E103JB0A is +-5 % R25;
 * that and the pull-up error scale as T^2/B, so 1.55 C at 22 C is ~1.9 C at
 * the 55 C OTC.  +1.8 C keeps the OTC within ~0.3 C. */
#define BMS_TS1_TEMP_OFFSET_DC           18
#define BMS_OTC_RECOVERY_C               40
#define BMS_UTC_C                        0
#define BMS_UTC_RECOVERY_C               5
#define BMS_OTD_C                        60
#define BMS_OTD_RECOVERY_C               55
#define BMS_UTD_C                        (-20)
#define BMS_UTD_RECOVERY_C               (-15)
#define BMS_OTINT_C                      85
#define BMS_OTINT_RECOVERY_C             80

/*
 * Enabled Protections A (TI bits 7..2): SCD=0x80 OCD2=0x40 OCD1=0x20
 * OCC=0x10 COV=0x08 CUV=0x04.  0xBC = SCD|OCD1|OCC|COV|CUV.
 * Enabled Protections B: OTF=0x80 OTINT=0x40 OTD=0x20 OTC=0x10 UTINT=0x04
 * UTD=0x02 UTC=0x01.  0x73 = OTINT|OTD|OTC|UTD|UTC.
 * CHG/DSG FET Protections B keep the TI defaults (0xD5 / 0xE6), which map
 * OTC/UTC to CHG and OTD/UTD to DSG.
 */
#define BMS_ENABLED_PROTECTIONS_A        0xBCU
#define BMS_ENABLED_PROTECTIONS_B        0x73U
#define BMS_CHG_FET_PROTECTIONS_A        0x98U
#define BMS_DSG_FET_PROTECTIONS_A        0xE4U
#define BMS_CHG_FET_PROTECTIONS_B        0xD5U
#define BMS_DSG_FET_PROTECTIONS_B        0xE6U

/* ---------------------------------------------------------- balancing -- */
/*
 * G4-controlled balancing, |I|<1 A and SOC>=90%. Autonomous CHG/RLX off.
 * Voltage gate remains (LFP plateau voltages say
 * nothing about SOC, so no balancing there).  Internal FETs through the
 * 100 Ω series resistors: ≈15 mA per cell.
 * CB_SLEEP|CB_NOSLEEP=0x0C; host command timeout 5 s.
 */
#define BMS_BALANCING_CONFIG             0x0CU
#define BMS_CB_MIN_CELL_TEMP_C           0
#define BMS_CB_MAX_CELL_TEMP_C           50
#define BMS_CB_MAX_INTERNAL_TEMP_C       70
#define BMS_CB_INTERVAL_S                5U
#define BMS_CB_MAX_CELLS                 2U
#define BMS_CB_MIN_CELL_MV_CHG           3350U
#define BMS_CB_MIN_DELTA_CHG_MV          20U
#define BMS_CB_STOP_DELTA_CHG_MV         8U
#define BMS_CB_MIN_CELL_MV_RLX           3350U
#define BMS_CB_MIN_DELTA_RLX_MV          20U
#define BMS_CB_STOP_DELTA_RLX_MV         8U

/* ------------------------------------------------------------ current -- */
/* BQ76922 SRP/SRN sense resistor (pack current). Not the DCDC INA296 shunt. */
#define BMS_SENSE_MOHM                   5U
/* Nominal CC Gain = 7.5684 / Rsense_mOhm (BQ76922 TRM, VREF2=1.24 V) =
 * 1.51368 read ~5.3 % high on this board.  Calibrated 2026-11-04 against a
 * Fluke on the 10 A range (TI SLUAA32A 2.6): -303 mA -> -211.40 CC2 counts,
 * -5290 mA -> -3685.65 counts; +2.25 A charge check within ~2 %.  That is
 * an effective 5.27 mOhm between SRP and SRN.  Capacity Gain = CC Gain ×
 * 298261.6178.  The zero-current offset fitted to -0.3 counts (~0.5 mA),
 * inside the noise, so Board Offset stays 0. */
#define BMS_CC_GAIN                      1.43542f
#define BMS_CAPACITY_GAIN                (BMS_CC_GAIN * 298261.6178f)
#define BMS_BOARD_OFFSET                 0

/* SCD Threshold index — hardware mV on sense. 0x05 ≈ 100 mV → ~20 A @ 5 mΩ. */
#define BMS_SCD_THRESHOLD                0x05U
/* OCC / OCD1: U1, 2 mV/LSB on the sense resistor.  6.7 Ah pack:
 * OCC 10 A (1.5 C, above any charger setting), OCD1 15 A (2.2 C). */
#define BMS_OCC_THRESHOLD_MA             10000U
#define BMS_OCD1_THRESHOLD_MA            15000U
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
/* OCC recovers when I <= this (default −200 mA never clears at I≈0). */
#define BMS_OCC_RECOVERY_MA              100
#define BMS_BODY_DIODE_THRESHOLD_MA      2000U

/* ------------------------------------------------------ FET / pins etc -- */
/* FET Options 0x9308: SFET|HOST_FET_EN|FET_CTRL_EN|PDSG_EN (TI demo 0x1D). */
#define BMS_FET_OPTIONS                  0x1DU
/* Mfg Status Init 0x9343: PF_EN (TI default) | FET_EN → autonomous FETs. */
#define BMS_MFG_STATUS_INIT              0x0050U
#define BMS_AUTO_SHUTDOWN_TIME           0U
#define BMS_CFETOFF_PIN_CONFIG           0x00U
#define BMS_DFETOFF_PIN_CONFIG           0x00U
#define BMS_REG1_CONFIG                  0x0DU
#define BMS_REG0_CONFIG                  0x01U
#define BMS_TS2_CONFIG                   0x00U /* wake push-button, not an NTC */
#define BMS_PCHG_START_MV                0U
#define BMS_PCHG_STOP_MV                 0U
#define BMS_PDSG_TIMEOUT                 0x32U /* ~500 ms */
#define BMS_PDSG_STOP_DELTA              50U   /* 500 mV */

#endif /* BMS_BOARD_H */
