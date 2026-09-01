#ifndef BMS_BOARD_H
#define BMS_BOARD_H

#include "board_rev.h"

/*
 * BQ76922 on shared I2C4 with TPS25751. 5S Li-ion pack on rev2 hardware.
 * U1 protection threshold encoding (TRM): mV = code * 50.6 (approx).
 *
 * BMS_ENABLE=0: skip I2C init/scan and do not trip on BMS faults (bring-up).
 * BMS_ENABLE=1: full monitor + shutdown on hardware safety faults.
 */
#ifndef BMS_ENABLE
#define BMS_ENABLE                       0U
#endif

#define BMS_U1_THRESHOLD_CODE_MV(mv)     ((uint8_t)((((uint32_t)(mv) * 10U) + 253U) / 506U))
#define BMS_COV_THRESHOLD_CODE           BMS_U1_THRESHOLD_CODE_MV(BMS_COV_THRESHOLD_MV)
#define BMS_CUV_THRESHOLD_CODE           BMS_U1_THRESHOLD_CODE_MV(BMS_CUV_THRESHOLD_MV)
#define BMS_COV_RECOVERY_CODE            BMS_U1_THRESHOLD_CODE_MV(BMS_COV_RECOVERY_MV)
#define BMS_CUV_RECOVERY_CODE            BMS_U1_THRESHOLD_CODE_MV(BMS_CUV_RECOVERY_MV)

#define BMS_CELL_COUNT                   5U

#define BMS_VCELL_MODE_5S                0x001FU
#define BMS_COV_THRESHOLD_MV             4250U
#define BMS_CUV_THRESHOLD_MV             2800U
#define BMS_COV_RECOVERY_MV              4150U
#define BMS_CUV_RECOVERY_MV              2900U

#define BMS_WARN_CELL_DELTA_MV           200U
#define BMS_WARN_CELL_LOW_MV             3000U
#define BMS_WARN_CELL_HIGH_MV            4200U
#define BMS_WARN_PACK_LOW_MV             14000U
#define BMS_WARN_PACK_HIGH_MV            21000U

#define BMS_ENABLED_PROTECTIONS_A        0xBCU
#define BMS_ENABLED_PROTECTIONS_B        0xF0U

#endif /* BMS_BOARD_H */
