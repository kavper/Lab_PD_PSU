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
 * read OK) while the pack path stays open.
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

#define BMS_ENABLED_PROTECTIONS_A        0xBCU
#define BMS_ENABLED_PROTECTIONS_B        0xF0U

#define BMS_CELL_USED(i)                 (((BMS_VCELL_MODE >> (i)) & 1U) != 0U)

#endif /* BMS_BOARD_H */
