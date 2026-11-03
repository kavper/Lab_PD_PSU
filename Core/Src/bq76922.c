#include "bq76922.h"

#include "bms_board.h"
#include "bms_golden.h"
#include "bms_balance_policy.h"
#include "bms_gauge.h"
#include "board_mx.h"
#include "power_manager.h"

#include <string.h>

#define BQ76922_CMD_SUBCMD_LOW           0x3EU
#define BQ76922_CMD_RAM_DATA             0x40U
#define BQ76922_CMD_RAM_CHECKSUM         0x60U
#define BQ76922_CMD_SAFETY_STATUS_A      0x03U
#define BQ76922_CMD_SAFETY_STATUS_B      0x05U
#define BQ76922_CMD_SAFETY_STATUS_C      0x07U
#define BQ76922_CMD_BATTERY_STATUS       0x12U
#define BQ76922_CMD_CELL1_V              0x14U
#define BQ76922_CMD_STACK_V              0x34U
#define BQ76922_CMD_PACK_V               0x36U
#define BQ76922_CMD_CC2                  0x3AU
#define BQ76922_CMD_INT_TEMP             0x68U
#define BQ76922_CMD_TS1_TEMP             0x70U
#define BQ76922_CMD_ALARM_STATUS         0x62U
#define BQ76922_CMD_FET_STATUS           0x7FU

#define BQ76922_SUBCMD_SHUTDOWN          0x0010U
#define BQ76922_SUBCMD_SET_CFGUPDATE     0x0090U
#define BQ76922_SUBCMD_EXIT_CFGUPDATE    0x0092U
#define BQ76922_SUBCMD_FET_ENABLE        0x0022U
#define BQ76922_SUBCMD_ALL_FETS_OFF      0x0095U
#define BQ76922_SUBCMD_ALL_FETS_ON       0x0096U
#define BQ76922_SUBCMD_SLEEP_DISABLE     0x009AU
#define BQ76922_SUBCMD_MANUF_STATUS      0x0057U
#define BQ76922_SUBCMD_RESET             0x0012U
#define BQ76922_SUBCMD_RESET_PASSQ       0x0082U
#define BQ76922_SUBCMD_DASTATUS5         0x0075U
#define BQ76922_SUBCMD_DASTATUS6         0x0076U
#define BQ76922_SUBCMD_CB_ACTIVE_CELLS   0x0083U
#define BQ76922_SUBCMD_OTP_WR_CHECK      0x00A0U
#define BQ76922_SUBCMD_OTP_WRITE         0x00A1U

#define BQ76922_RAM_REG1_CONFIG          0x9236U
#define BQ76922_RAM_REG0_CONFIG          0x9237U
#define BQ76922_RAM_VCELL_MODE           0x9304U
#define BQ76922_RAM_ENABLED_PROT_A       0x9261U
#define BQ76922_RAM_ENABLED_PROT_B       0x9262U
#define BQ76922_RAM_CHG_FET_PROT_A       0x9265U
#define BQ76922_RAM_DSG_FET_PROT_A       0x9269U
#define BQ76922_RAM_BODY_DIODE           0x9273U
#define BQ76922_RAM_CUV_THRESHOLD        0x9275U
#define BQ76922_RAM_COV_THRESHOLD        0x9278U
#define BQ76922_RAM_OCC_THRESHOLD        0x9280U
#define BQ76922_RAM_OCD1_THRESHOLD       0x9282U
#define BQ76922_RAM_SCD_THRESHOLD        0x9286U
#define BQ76922_RAM_OCC_RECOVERY         0x9288U
#define BQ76922_RAM_CC_GAIN              0x91A8U
#define BQ76922_RAM_CAPACITY_GAIN        0x91ACU
#define BQ76922_RAM_CFETOFF_PIN_CONFIG   0x92FAU
#define BQ76922_RAM_DFETOFF_PIN_CONFIG   0x92FBU
#define BQ76922_RAM_TS2_CONFIG           0x92FEU
#define BQ76922_RAM_FET_OPTIONS          0x9308U
#define BQ76922_RAM_PCHG_START           0x930AU
#define BQ76922_RAM_PCHG_STOP            0x930CU
#define BQ76922_RAM_PDSG_TIMEOUT         0x930EU
#define BQ76922_RAM_PDSG_STOP_DELTA      0x930FU
#define BQ76922_RAM_MFG_STATUS_INIT      0x9343U
#define BQ76922_RAM_AUTO_SHUTDOWN_TIME   0x9254U

#define BQ76922_I2C_TIMEOUT_MS           20U
#define BQ76922_SCAN_PERIOD_MS           50U
#define BQ76922_PROBE_RETRY_MS           200U
#define BQ76922_INIT_STEP_DELAY_MS       10U
#define BQ76922_POST_CFG_DELAY_MS        200U
#define BQ76922_BOOT_SETTLE_MS           300U
#define BQ76922_BOOT_SETTLE_WAKE_MS      500U
#define BQ76922_FET_RETRY_MS             250U
#define BQ76922_FET_VERIFY_MAX           8U
#define BQ76922_FET_RECOVER_REINIT_MAX   12U
#define BQ76922_BUS_HOLD_MAX_MS          15000U
#define BQ76922_CFG_POLL_MAX             50U
#define BQ76922_CFG_ENTER_MAX            6U
/* AFE in SHUTDOWN NACKs I2C; after this many losses treat as absent so
 * TS2 button wake re-probes and runs FET_ENABLE + ALL_FETS_ON with no host cmd. */
#define BQ76922_I2C_ABSENT_STREAK        5U
#define BQ76922_CC2_CHARGE_MA            50
#define BQ76922_CC2_DISCHARGE_MA         (-50)
#define BQ76922_MANUF_FET_EN             (1U << 4)
/* FET Status: CHG=bit0, PCHG=bit1, DSG=bit2, PDSG=bit3;
 * bit4 reports CFETOFF/DCHG pin assert on BQ769x2 family (RSVD in BQ76922 TRM). */
#define BQ76922_FET_STATUS_CHG           0x01U
#define BQ76922_FET_STATUS_DSG           0x04U
#define BQ76922_FET_STATUS_PDSG          0x08U
#define BQ76922_FET_STATUS_DCHG_PIN      0x10U
#define BQ76922_FET_STATUS_CHG_DSG       (BQ76922_FET_STATUS_CHG | BQ76922_FET_STATUS_DSG)
/* Safety Status A — TI TRM §12.2.3: SCD OCD2 OCD1 OCC COV CUV (bits 7..2). */
#define BQ76922_SAFETY_A_SCD             0x80U
#define BQ76922_SAFETY_A_OCD2            0x40U
#define BQ76922_SAFETY_A_OCD1            0x20U
#define BQ76922_SAFETY_A_OCC             0x10U
#define BQ76922_SAFETY_A_COV             0x08U
#define BQ76922_SAFETY_A_CUV             0x04U
#define BQ76922_SAFETY_A_CURRENT \
    (BQ76922_SAFETY_A_SCD | BQ76922_SAFETY_A_OCD1 | BQ76922_SAFETY_A_OCD2 | \
     BQ76922_SAFETY_A_OCC)
/* Battery Status 0x12 */
#define BQ76922_BATT_CFGUPDATE             (1U << 0)
#define BQ76922_BATT_OTPB                (1U << 7)
#define BQ76922_BATT_SEC_MASK            (3U << 8)
#define BQ76922_BATT_SEC_FULLACCESS      (1U << 8)
#define BQ76922_BATT_SEC_UNSEALED        (2U << 8)
#define BQ76922_BATT_SS                  (1U << 11)
#define BQ76922_OTP_OK_CODE              0x80U
#define BQ76922_OTP_NO_DATA              0x08U /* bit3 — bit/image exhausted */
#define BQ76922_OTP_NO_SIG               0x10U /* bit4 — signature slots gone */
#define BQ76922_OTP_CONFIRM_TOKEN        "I-UNDERSTAND-OTP"
/* Default DA Config USER_VOLTS_CV=1 → Stack/PACK in centivolts (10 mV). */
#define BQ76922_USERV_TO_MV(raw)         ((int16_t)((int16_t)(raw) * 10))

typedef enum {
    BQ76922_INIT_WAIT_READY = 0,
    BQ76922_INIT_SLEEP_DISABLE_PRE,
    BQ76922_INIT_CLR_ALARM_PRE,
    BQ76922_INIT_OTP_FAST_CHECK,
    BQ76922_INIT_ENTER_CFG,
    BQ76922_INIT_WAIT_CFG,
    BQ76922_INIT_WRITE_TABLE,
    BQ76922_INIT_EXIT_CFG,
    BQ76922_INIT_WAIT_EXIT,
    BQ76922_INIT_VERIFY_TABLE,
    BQ76922_INIT_SLEEP_DISABLE,
    BQ76922_INIT_CLR_ALARM,
    BQ76922_INIT_MANUF_READ,
    BQ76922_INIT_FET_ENABLE,
    BQ76922_INIT_MANUF_CONFIRM_READ,
    BQ76922_INIT_ALL_FETS_ON,
    BQ76922_INIT_FET_SETTLE,
    BQ76922_INIT_FET_VERIFY,
    BQ76922_INIT_RESET_PASSQ,
    BQ76922_INIT_DONE,
} BQ76922_InitStep_t;

static void BQ76922_ForceAbsent(BQ76922_Device_t *dev);
static bool BQ76922_NoteCommsResult(BQ76922_Device_t *dev,
                                    BQ76922_Status_t status);

/* Session guard — OTP burn is never automatic; UART may do it once per boot. */
static bool s_otp_burned_this_boot = false;
/* SHUTDOWN clears the AFE coulomb counter. RESET_PASSQ only after that
 * command, never on a normal boot and never when only the FETs turn off. */
static bool s_passq_reset_pending = false;

static void BQ76922_UpdateBusHold(BQ76922_Device_t *dev, uint32_t now_ms)
{
    bool need_hold;

    if ((dev == NULL) || !BQ76922_IsEnabled()) {
        PowerManager_SetBmsBusHold(false);
        return;
    }

    need_hold = dev->snapshot.present && !dev->snapshot.configured;
    if (need_hold) {
        if (dev->bus_hold_since_ms == 0U) {
            dev->bus_hold_since_ms = now_ms;
        }
        if ((uint32_t)(now_ms - dev->bus_hold_since_ms) > BQ76922_BUS_HOLD_MAX_MS) {
            /* Do not starve PD forever if AFE is wedged; keep retrying. */
            PowerManager_SetBmsBusHold(false);
            return;
        }
        PowerManager_SetBmsBusHold(true);
        return;
    }

    dev->bus_hold_since_ms = 0U;
    PowerManager_SetBmsBusHold(false);
}

static bool BQ76922_BusIdle(const BQ76922_Device_t *dev)
{
    if ((dev == NULL) || (dev->hi2c == NULL)) {
        return false;
    }
    /* During BMS bus-hold, do not wait on PM job flags — only the wire state.
     * Otherwise a stale TPS IT job can freeze CONFIG_UPDATE forever. */
    if (PowerManager_GetBmsBusHold()) {
        return HAL_I2C_GetState(dev->hi2c) == HAL_I2C_STATE_READY;
    }
    if (!PowerManager_IsI2cIdle()) {
        return false;
    }
    return HAL_I2C_GetState(dev->hi2c) == HAL_I2C_STATE_READY;
}

static int16_t BQ76922_Le16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static BQ76922_Status_t BQ76922_ReadDirect(BQ76922_Device_t *dev,
                                           uint8_t cmd,
                                           uint8_t *data,
                                           uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((dev == NULL) || (dev->hi2c == NULL) || (data == NULL) || (length == 0U)) {
        return BQ76922_INVALID_ARG;
    }
    if (!BQ76922_BusIdle(dev)) {
        return BQ76922_BUSY;
    }

    status = HAL_I2C_Mem_Read(dev->hi2c,
                              (uint16_t)(dev->address_7bit << 1),
                              cmd,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              length,
                              BQ76922_I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        dev->snapshot.i2c_error_count++;
        return BQ76922_I2C_ERROR;
    }
    return BQ76922_OK;
}

static BQ76922_Status_t BQ76922_WriteDirect(BQ76922_Device_t *dev,
                                            uint8_t cmd,
                                            const uint8_t *data,
                                            uint16_t length)
{
    HAL_StatusTypeDef status;

    if ((dev == NULL) || (dev->hi2c == NULL) || (data == NULL) || (length == 0U)) {
        return BQ76922_INVALID_ARG;
    }
    if (!BQ76922_BusIdle(dev)) {
        return BQ76922_BUSY;
    }

    status = HAL_I2C_Mem_Write(dev->hi2c,
                               (uint16_t)(dev->address_7bit << 1),
                               cmd,
                               I2C_MEMADD_SIZE_8BIT,
                               (uint8_t *)data,
                               length,
                               BQ76922_I2C_TIMEOUT_MS);
    if (status != HAL_OK) {
        dev->snapshot.i2c_error_count++;
        return BQ76922_I2C_ERROR;
    }
    return BQ76922_OK;
}

static BQ76922_Status_t BQ76922_SendSubcommand(BQ76922_Device_t *dev, uint16_t subcmd)
{
    uint8_t buf[2];
    BQ76922_Status_t status;
    if (subcmd == BQ76922_SUBCMD_ALL_FETS_ON &&
        !BmsFetGuard_AllowStart(&dev->fet_guard)) return BQ76922_NOT_READY;

    buf[0] = (uint8_t)(subcmd & 0xFFU);
    buf[1] = (uint8_t)((subcmd >> 8) & 0xFFU);
    status = BQ76922_WriteDirect(dev, BQ76922_CMD_SUBCMD_LOW, buf, 2U);
    /* A bus error is ambiguous: do not repeat a potentially accepted start.
     * BUSY means no transaction was attempted and can be retried. */
    if (subcmd == BQ76922_SUBCMD_ALL_FETS_ON && status != BQ76922_BUSY)
        dev->fet_guard.start_attempted = true;
    return status;
}

static BQ76922_Status_t BQ76922_WriteRamBlock(BQ76922_Device_t *dev,
                                              uint16_t address,
                                              const uint8_t *data,
                                              uint8_t length)
{
    uint8_t whole_block[6];
    uint8_t checksum_block[2];
    uint16_t sum = 0U;
    uint8_t i;
    uint8_t block_len;
    BQ76922_Status_t status;

    if ((dev == NULL) || (data == NULL) || (length == 0U) || (length > 4U)) {
        return BQ76922_INVALID_ARG;
    }

    /* TI protocol: one write to 0x3E with [addr_lo, addr_hi, data…], then
     * checksum+length at 0x60. Length byte = (addr+data bytes) + 2.
     * Splitting data to 0x40 with length=data_len+2 silently drops the write
     * (vcell_rb stays 0x0000 after EXIT_CFGUPDATE). */
    whole_block[0] = (uint8_t)(address & 0xFFU);
    whole_block[1] = (uint8_t)((address >> 8) & 0xFFU);
    for (i = 0U; i < length; i++) {
        whole_block[2U + i] = data[i];
    }
    block_len = (uint8_t)(length + 2U);

    for (i = 0U; i < block_len; i++) {
        sum = (uint16_t)(sum + whole_block[i]);
    }

    status = BQ76922_WriteDirect(dev, BQ76922_CMD_SUBCMD_LOW, whole_block, block_len);
    if (status != BQ76922_OK) {
        return status;
    }

    checksum_block[0] = (uint8_t)(~sum & 0xFFU);
    checksum_block[1] = (uint8_t)(block_len + 2U); /* +2 per TI / BQ769x2_SetRegister */
    status = BQ76922_WriteDirect(dev, BQ76922_CMD_RAM_CHECKSUM, checksum_block, 2U);
    if (status == BQ76922_OK) {
        /* Data Memory needs time to commit before the next address is issued. */
        HAL_Delay(3U);
    }
    return status;
}

static BQ76922_Status_t BQ76922_ReadRamU1(BQ76922_Device_t *dev,
                                          uint16_t address,
                                          uint8_t *out);
static BQ76922_Status_t BQ76922_ReadRamU2(BQ76922_Device_t *dev,
                                          uint16_t address,
                                          uint16_t *out);

static BQ76922_Status_t BQ76922_ReadRamBlock(BQ76922_Device_t *dev,
                                             uint16_t address,
                                             uint8_t *data,
                                             uint8_t length);

static BQ76922_Status_t BQ76922_WriteGoldenEntry(BQ76922_Device_t *dev,
                                                 const BmsDmEntry_t *e)
{
    uint8_t b[4];

    BmsGolden_Bytes(e, b);
    return BQ76922_WriteRamBlock(dev, e->addr, b, e->type);
}

/* Reads one entry back; *match = RAM value equals the golden value. */
static BQ76922_Status_t BQ76922_CheckGoldenEntry(BQ76922_Device_t *dev,
                                                 const BmsDmEntry_t *e,
                                                 bool *match)
{
    uint8_t want[4];
    uint8_t got[4] = {0};
    uint8_t k;
    BQ76922_Status_t status;

    status = BQ76922_ReadRamBlock(dev, e->addr, got, e->type);
    if (status != BQ76922_OK) {
        return status;
    }
    BmsGolden_Bytes(e, want);
    *match = true;
    for (k = 0U; k < e->type; k++) {
        if (got[k] != want[k]) {
            *match = false;
        }
    }
    return BQ76922_OK;
}

/* only_fp: check FINGERPRINT entries only (wake fast path) or every entry
 * (OTP preflight / post-burn readback). fail = first mismatching address. */
static BQ76922_Status_t BQ76922_GoldenCompare(BQ76922_Device_t *dev,
                                              bool only_fp,
                                              bool *match,
                                              uint16_t *fail)
{
    uint16_t i;
    BQ76922_Status_t status;

    *match = true;
    *fail = 0U;
    for (i = 0U; i < g_bms_golden_count; i++) {
        const BmsDmEntry_t *e = &g_bms_golden[i];
        bool ok = false;

        if (only_fp && ((e->flags & BMS_DM_FLAG_FINGERPRINT) == 0U)) {
            continue;
        }
        status = BQ76922_CheckGoldenEntry(dev, e, &ok);
        if (status != BQ76922_OK) {
            return status;
        }
        if (!ok) {
            *match = false;
            *fail = e->addr;
            return BQ76922_OK;
        }
    }
    return BQ76922_OK;
}

static BQ76922_Status_t BQ76922_GoldenFingerprint(BQ76922_Device_t *dev,
                                                  bool *match,
                                                  uint16_t *fail)
{
    return BQ76922_GoldenCompare(dev, true, match, fail);
}

/* Subcommand with a 16-bit response.  The 0x3E echo check inside
 * ReadRamBlock rejects a stale 0x40 buffer from the previous command. */
static BQ76922_Status_t BQ76922_ReadSubcmdU2(BQ76922_Device_t *dev,
                                             uint16_t subcmd,
                                             uint16_t *out)
{
    uint8_t buf[2];
    BQ76922_Status_t status = BQ76922_ReadRamBlock(dev, subcmd, buf, 2U);

    if (status == BQ76922_OK) {
        *out = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    }
    return status;
}

static bool BQ76922_CellUsed(uint8_t index)
{
    return (index < BQ76922_CELL_COUNT) && BMS_CELL_USED(index);
}

static void BQ76922_UpdateCellStats(BQ76922_Device_t *dev)
{
    int16_t min_mv = 32767;
    int16_t max_mv = -32768;
    int32_t sum_mv = 0;
    uint8_t i;
    uint8_t used = 0U;

    for (i = 0U; i < BQ76922_CELL_COUNT; i++) {
        int16_t mv;

        if (!BQ76922_CellUsed(i)) {
            dev->snapshot.cell_mv[i] = (int16_t)BMS_UNUSED_CELL_MV;
            continue;
        }

        mv = dev->snapshot.cell_mv[i];
        used++;
        sum_mv += (int32_t)mv;
        if (mv < min_mv) {
            min_mv = mv;
        }
        if (mv > max_mv) {
            max_mv = mv;
        }
    }

    if (dev->snapshot.sample_valid && (used > 0U)) {
        dev->snapshot.min_cell_mv = min_mv;
        dev->snapshot.max_cell_mv = max_mv;
        dev->snapshot.cell_sum_mv = (int16_t)sum_mv;
    }
}

static void BQ76922_DecodeSafetyFaults(BQ76922_Device_t *dev)
{
    uint32_t flags = BQ76922_FAULT_NONE;
    uint8_t sa = dev->snapshot.safety_status_a;
    uint8_t sb = dev->snapshot.safety_status_b;
    uint8_t sc = dev->snapshot.safety_status_c;
    bool real_cov;
    bool real_cuv;

    /* Report COV until the AFE itself clears its recovery hysteresis. */
    real_cov = ((sa & BQ76922_SAFETY_A_COV) != 0U);
    real_cuv = ((sa & BQ76922_SAFETY_A_CUV) != 0U) &&
               dev->snapshot.sample_valid &&
               (dev->snapshot.min_cell_mv > 0) &&
               (dev->snapshot.min_cell_mv <= (int16_t)(BMS_CUV_EFFECTIVE_MV + 20U));

    if (real_cov) {
        flags |= BQ76922_FAULT_CELL_OV;
    }
    if (real_cuv) {
        flags |= BQ76922_FAULT_CELL_UV;
    }
    if (((sa & BQ76922_SAFETY_A_CURRENT) != 0U) || (sb != 0U) || (sc != 0U)) {
        flags |= BQ76922_FAULT_SAFETY;
    }
    /* ALERT Pin Config defaults to unused/Hi-Z; STM32 pull-up would look
     * asserted forever. Only trust software-latched Alarm Status. */
    if (dev->snapshot.alert_latched) {
        flags |= BQ76922_FAULT_ALERT;
    }

    dev->snapshot.fault_flags = flags;

    /* COV keeps CHG protected by the AFE; retain host monitoring/balancing.
     * Confirmed CUV still requests host shutdown. No FET override here. */
    if ((flags & BQ76922_FAULT_CELL_UV) != 0U) {
        if (dev->snapshot.configured) {
            dev->snapshot.shutdown_request = true;
            dev->snapshot.state = BQ76922_STATE_FAULT;
        }
    }
}

static void BQ76922_UpdateWarnState(BQ76922_Device_t *dev)
{
    int16_t delta;
    bool warn = false;

    if (!dev->snapshot.sample_valid) {
        return;
    }

    delta = (int16_t)(dev->snapshot.max_cell_mv - dev->snapshot.min_cell_mv);
    if (delta >= (int16_t)BMS_WARN_CELL_DELTA_MV) {
        warn = true;
        dev->snapshot.fault_flags |= BQ76922_FAULT_IMBALANCE;
    }

    if ((dev->snapshot.fault_flags & BQ76922_FAULT_CELL_OV) != 0U) warn = true;
    if ((dev->snapshot.min_cell_mv <= (int16_t)BMS_WARN_CELL_LOW_MV) ||
        (dev->snapshot.max_cell_mv >= (int16_t)BMS_WARN_CELL_HIGH_MV)) {
        warn = true;
    }

    /* PACK pin is ~0 with FETs off — do not treat that as a 4S undervoltage. */
    if (dev->snapshot.fets_enabled && (dev->snapshot.pack_mv > 0) &&
        ((dev->snapshot.pack_mv <= (int16_t)BMS_WARN_PACK_LOW_MV) ||
         (dev->snapshot.pack_mv >= (int16_t)BMS_WARN_PACK_HIGH_MV))) {
        warn = true;
    }

    if (dev->snapshot.state == BQ76922_STATE_FAULT) {
        return;
    }

    if (warn) {
        dev->snapshot.state = BQ76922_STATE_WARN;
        return;
    }

    if (dev->snapshot.cc2_ma >= BQ76922_CC2_CHARGE_MA) {
        dev->snapshot.state = BQ76922_STATE_CHARGING;
    } else if (dev->snapshot.cc2_ma <= BQ76922_CC2_DISCHARGE_MA) {
        dev->snapshot.state = BQ76922_STATE_DISCHARGING;
    } else {
        dev->snapshot.state = BQ76922_STATE_READY;
    }
}

static BQ76922_Status_t BQ76922_ReadSafetyStatus(BQ76922_Device_t *dev)
{
    BQ76922_Status_t status;

    status = BQ76922_ReadDirect(dev, BQ76922_CMD_SAFETY_STATUS_A,
                                &dev->snapshot.safety_status_a, 1U);
    if (status != BQ76922_OK) {
        return status;
    }

    status = BQ76922_ReadDirect(dev, BQ76922_CMD_SAFETY_STATUS_B,
                                &dev->snapshot.safety_status_b, 1U);
    if (status != BQ76922_OK) {
        return status;
    }

    status = BQ76922_ReadDirect(dev, BQ76922_CMD_SAFETY_STATUS_C,
                                &dev->snapshot.safety_status_c, 1U);
    return status;
}

static BQ76922_Status_t BQ76922_ReadBatteryStatus(BQ76922_Device_t *dev)
{
    uint8_t buf[2];
    BQ76922_Status_t status;

    status = BQ76922_ReadDirect(dev, BQ76922_CMD_BATTERY_STATUS, buf, 2U);
    if (status != BQ76922_OK) {
        return status;
    }
    dev->snapshot.battery_status = (uint16_t)BQ76922_Le16(buf);
    return BQ76922_OK;
}

static BQ76922_Status_t BQ76922_ClearAlarms(BQ76922_Device_t *dev)
{
    uint8_t buf[2];
    BQ76922_Status_t status;

    status = BQ76922_ReadDirect(dev, BQ76922_CMD_ALARM_STATUS, buf, 2U);
    if (status != BQ76922_OK) {
        return status;
    }

    status = BQ76922_WriteDirect(dev, BQ76922_CMD_ALARM_STATUS, buf, 2U);
    if (status != BQ76922_OK) {
        return status;
    }

    dev->snapshot.alarm_status = (uint16_t)BQ76922_Le16(buf);
    dev->snapshot.alert_latched = false;
    (void)BQ76922_ReadSafetyStatus(dev);
    return BQ76922_OK;
}

static void BQ76922_InitRestart(BQ76922_Device_t *dev, uint32_t delay_ms)
{
    dev->snapshot.fault_flags |= BQ76922_FAULT_I2C;
    dev->snapshot.cfg_fail_count++;
    dev->init_step = (uint8_t)BQ76922_INIT_WAIT_READY;
    dev->cfg_poll_tries = 0U;
    dev->cfg_enter_tries = 0U;
    dev->fet_en_attempts = 0U;
    dev->fet_verify_attempts = 0U;
    dev->fet_recover_fails = 0U;
    /* Allow a fresh exclusive-bus window after a failed CFGUPDATE cycle. */
    dev->bus_hold_since_ms = 0U;
    dev->next_action_ms = HAL_GetTick() + delay_ms;
    dev->snapshot.init_step = dev->init_step;
}

static BQ76922_Status_t BQ76922_HandleAlert(BQ76922_Device_t *dev)
{
    BQ76922_Status_t status;

    status = BQ76922_ClearAlarms(dev);
    if (status != BQ76922_OK) {
        return status;
    }
    BQ76922_DecodeSafetyFaults(dev);
    return BQ76922_OK;
}

static bool BQ76922_RunInitStep(BQ76922_Device_t *dev)
{
    BQ76922_Status_t status = BQ76922_OK;
    uint32_t delay_ms = BQ76922_INIT_STEP_DELAY_MS;
    uint8_t buf[2];
    uint16_t sec;

    dev->snapshot.init_step = dev->init_step;

    switch ((BQ76922_InitStep_t)dev->init_step) {
        case BQ76922_INIT_WAIT_READY:
            /* After button/charger wake SEC may be 0 until AFE finishes boot.
             * Also wait BOOT_SETTLE_MS — TI recommends ~300 ms before CFGUPDATE. */
            status = BQ76922_ReadBatteryStatus(dev);
            if (status == BQ76922_OK) {
                sec = (uint16_t)(dev->snapshot.battery_status & BQ76922_BATT_SEC_MASK);
                if (sec == 0U) {
                    delay_ms = 50U;
                    dev->next_action_ms = HAL_GetTick() + delay_ms;
                    return false;
                }
                if ((sec != BQ76922_BATT_SEC_FULLACCESS) &&
                    (sec != BQ76922_BATT_SEC_UNSEALED)) {
                    /* SEALED: cannot write VCell Mode — keep retrying / expose batt. */
                    BQ76922_InitRestart(dev, BQ76922_PROBE_RETRY_MS);
                    return false;
                }
                if (dev->boot_settle_ms == 0U) {
                    dev->boot_settle_ms = HAL_GetTick();
                }
                {
                    uint32_t settle = dev->wake_from_absent ?
                                         BQ76922_BOOT_SETTLE_WAKE_MS :
                                         BQ76922_BOOT_SETTLE_MS;
                    if ((uint32_t)(HAL_GetTick() - dev->boot_settle_ms) < settle) {
                        delay_ms = 50U;
                        dev->next_action_ms = HAL_GetTick() + delay_ms;
                        return false;
                    }
                }
            }
            break;

        case BQ76922_INIT_SLEEP_DISABLE_PRE:
            /* SLEEP_EN was set in user log (batt=0x0184). Disable before CFGUPDATE. */
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_SLEEP_DISABLE);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_CLR_ALARM_PRE:
            status = BQ76922_ReadDirect(dev, BQ76922_CMD_ALARM_STATUS, buf, 2U);
            if (status == BQ76922_OK) {
                (void)BQ76922_WriteDirect(dev, BQ76922_CMD_ALARM_STATUS, buf, 2U);
                dev->snapshot.alarm_status = (uint16_t)BQ76922_Le16(buf);
                dev->snapshot.alert_latched = false;
            }
            delay_ms = 10U;
            break;

        case BQ76922_INIT_OTP_FAST_CHECK:
            /* Golden OTP already loaded (SHUTDOWN wake on a burned chip):
             * SET_CFGUPDATE would hold the FETs off — skip it. Any other
             * image (blank chip, old 4S Li-ion OTP) gets the RAM table. */
            {
                bool match = false;
                uint16_t fail = 0U;

                status = BQ76922_GoldenFingerprint(dev, &match, &fail);
                if (status != BQ76922_OK) {
                    break;
                }
                dev->snapshot.golden_otp = match;
                dev->snapshot.fp_fail_addr = fail;
                if (match) {
                    dev->snapshot.golden_ok = true;
                    dev->snapshot.vcell_mode_rb = BMS_VCELL_MODE;
                    dev->init_step = (uint8_t)BQ76922_INIT_MANUF_READ;
                    dev->snapshot.init_step = dev->init_step;
                    delay_ms = 20U;
                    dev->next_action_ms = HAL_GetTick() + delay_ms;
                    return false;
                }
            }
            break;

        case BQ76922_INIT_ENTER_CFG:
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_SET_CFGUPDATE);
            delay_ms = 10U; /* SET_CFGUPDATE needs ~2 ms; give margin before poll */
            dev->cfg_poll_tries = 0U;
            break;

        case BQ76922_INIT_WAIT_CFG:
            status = BQ76922_ReadBatteryStatus(dev);
            if (status == BQ76922_OK) {
                if ((dev->snapshot.battery_status & BQ76922_BATT_CFGUPDATE) == 0U) {
                    dev->cfg_poll_tries++;
                    if (dev->cfg_poll_tries >= BQ76922_CFG_POLL_MAX) {
                        dev->snapshot.cfg_fail_count++;
                        dev->cfg_enter_tries++;
                        if (dev->cfg_enter_tries >= BQ76922_CFG_ENTER_MAX) {
                            /* HW suspect: RST_SHUT must be LOW (TP28). */
                            BQ76922_InitRestart(dev, BQ76922_PROBE_RETRY_MS);
                            return false;
                        }
                        /* Resend SET_CFGUPDATE without full restart. */
                        dev->init_step = (uint8_t)BQ76922_INIT_ENTER_CFG;
                        dev->snapshot.init_step = dev->init_step;
                        dev->cfg_poll_tries = 0U;
                        dev->next_action_ms = HAL_GetTick() + 50U;
                        return false;
                    }
                    delay_ms = 20U;
                    dev->next_action_ms = HAL_GetTick() + delay_ms;
                    return false;
                }
                dev->cfg_poll_tries = 0U;
                dev->cfg_enter_tries = 0U;
                dev->table_index = 0U;
            }
            break;

        case BQ76922_INIT_WRITE_TABLE:
            /* One golden entry per step (≈3 ms commit each) so the task
             * stays responsive; same table as the OTP image. */
            if (dev->table_index < g_bms_golden_count) {
                status = BQ76922_WriteGoldenEntry(dev, &g_bms_golden[dev->table_index]);
                if (status == BQ76922_OK) {
                    dev->table_index++;
                }
                if ((status == BQ76922_OK) && (dev->table_index < g_bms_golden_count)) {
                    dev->next_action_ms = HAL_GetTick() + 2U;
                    return false;
                }
                delay_ms = 5U;
            }
            break;

        case BQ76922_INIT_EXIT_CFG:
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_EXIT_CFGUPDATE);
            delay_ms = 5U;
            dev->cfg_poll_tries = 0U;
            break;

        case BQ76922_INIT_WAIT_EXIT:
            status = BQ76922_ReadBatteryStatus(dev);
            if (status == BQ76922_OK) {
                if ((dev->snapshot.battery_status & BQ76922_BATT_CFGUPDATE) != 0U) {
                    dev->cfg_poll_tries++;
                    if (dev->cfg_poll_tries >= BQ76922_CFG_POLL_MAX) {
                        BQ76922_InitRestart(dev, BQ76922_PROBE_RETRY_MS);
                        return false;
                    }
                    delay_ms = 10U;
                    dev->next_action_ms = HAL_GetTick() + delay_ms;
                    return false;
                }
                /* FW restarted with new RAM — allow measurements before FETs. */
                delay_ms = BQ76922_POST_CFG_DELAY_MS;
                dev->cfg_poll_tries = 0U;
            }
            break;

        case BQ76922_INIT_VERIFY_TABLE:
            {
                bool match = false;
                uint16_t fail = 0U;

                status = BQ76922_GoldenFingerprint(dev, &match, &fail);
                if (status == BQ76922_OK) {
                    dev->snapshot.fp_fail_addr = fail;
                    if (!match) {
                        BQ76922_InitRestart(dev, BQ76922_PROBE_RETRY_MS);
                        return false;
                    }
                    dev->snapshot.golden_ok = true;
                    dev->snapshot.vcell_mode_rb = BMS_VCELL_MODE;
                }
            }
            break;

        case BQ76922_INIT_SLEEP_DISABLE:
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_SLEEP_DISABLE);
            break;

        case BQ76922_INIT_CLR_ALARM:
            status = BQ76922_ReadDirect(dev, BQ76922_CMD_ALARM_STATUS, buf, 2U);
            if (status == BQ76922_OK) {
                (void)BQ76922_WriteDirect(dev, BQ76922_CMD_ALARM_STATUS, buf, 2U);
                (void)BQ76922_ReadSafetyStatus(dev);
                (void)BQ76922_ReadBatteryStatus(dev);
                dev->snapshot.alert_latched = false;
            }
            break;

        case BQ76922_INIT_MANUF_READ:
            {
                uint16_t manuf = 0U;

                status = BQ76922_ReadSubcmdU2(dev, BQ76922_SUBCMD_MANUF_STATUS, &manuf);
                if (status == BQ76922_OK) {
                    dev->snapshot.manuf_status = manuf;
                    /* FET_ENABLE is a toggle — do not send it if FET_EN is already 1. */
                    if ((manuf & BQ76922_MANUF_FET_EN) != 0U) {
                        dev->init_step = (uint8_t)BQ76922_INIT_ALL_FETS_ON;
                        dev->snapshot.init_step = dev->init_step;
                        dev->next_action_ms = HAL_GetTick() + delay_ms;
                        return false;
                    }
                }
            }
            break;

        case BQ76922_INIT_FET_ENABLE:
            /* Blank OTP: FET_EN=0 (FET Test Mode). Toggle into firmware FET control. */
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_FET_ENABLE);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_MANUF_CONFIRM_READ:
            {
                uint16_t manuf = 0U;

                status = BQ76922_ReadSubcmdU2(dev, BQ76922_SUBCMD_MANUF_STATUS, &manuf);
                if (status == BQ76922_OK) {
                    dev->snapshot.manuf_status = manuf;
                    if ((manuf & BQ76922_MANUF_FET_EN) == 0U) {
                        if (dev->fet_en_attempts < 4U) {
                            dev->fet_en_attempts++;
                            dev->init_step = (uint8_t)BQ76922_INIT_FET_ENABLE;
                            dev->snapshot.init_step = dev->init_step;
                            dev->next_action_ms = HAL_GetTick() + 30U;
                            return false;
                        }
                        BQ76922_InitRestart(dev, BQ76922_PROBE_RETRY_MS);
                        return false;
                    }
                    dev->fet_en_attempts = 0U;
                }
            }
            break;

        case BQ76922_INIT_ALL_FETS_ON:
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_ALL_FETS_ON);
            dev->last_fet_cmd_ms = HAL_GetTick();
            /* PDSG soft-start; after SHUTDOWN wake PACK caps are empty — wait longer. */
            delay_ms = dev->wake_from_absent ? 200U : 80U;
            break;

        case BQ76922_INIT_FET_SETTLE:
            status = BQ76922_ClearAlarms(dev);
            delay_ms = dev->wake_from_absent ? 80U : 40U;
            break;

        case BQ76922_INIT_FET_VERIFY:
            status = BQ76922_ReadDirect(dev, BQ76922_CMD_FET_STATUS, buf, 1U);
            if (status == BQ76922_OK) {
                uint8_t fet = buf[0];
                bool path_on = ((fet & BQ76922_FET_STATUS_DSG) != 0U) &&
                               ((fet & BQ76922_FET_STATUS_PDSG) == 0U);
                BmsFetGuard_Observe(&dev->fet_guard, fet, HAL_GetTick());
                bool chg_on = ((fet & BQ76922_FET_STATUS_CHG) != 0U);


                dev->snapshot.fet_status = fet;
                dev->snapshot.chg_fet_on = chg_on;
                dev->snapshot.dsg_fet_on =
                    ((fet & BQ76922_FET_STATUS_DSG) != 0U);
                dev->snapshot.fets_enabled = (chg_on || path_on);
                (void)BQ76922_ReadSafetyStatus(dev);
                if (BmsFetGuard_StartComplete(fet, dev->snapshot.safety_status_a,
                        dev->snapshot.safety_status_b, dev->snapshot.safety_status_c)) {
                    dev->fet_verify_attempts = 0U;
                    dev->fet_recover_fails = 0U;
                    break;
                }

                /* Poll completion without reissuing ALL_FETS_ON. */
                if ((uint32_t)(HAL_GetTick() - dev->last_fet_cmd_ms) <
                    BMS_PDSG_HOST_LIMIT_MS) {
                    dev->next_action_ms = HAL_GetTick() + 50U;
                    return false;
                }
                dev->fet_guard.latched = true;
                return false;
            }
            break;

        case BQ76922_INIT_RESET_PASSQ:
            /* Both the full CFGUPDATE path and the OTP fast path reach this
             * step. A normal boot leaves the flag clear and does not touch
             * a counter that survived in the AFE. */
            if (s_passq_reset_pending) {
                status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_RESET_PASSQ);
                if (status == BQ76922_OK) {
                    s_passq_reset_pending = false;
                }
            }
            break;

        case BQ76922_INIT_DONE:
        default:
            dev->snapshot.configured = true;
            dev->snapshot.fault_flags &= (uint32_t)~BQ76922_FAULT_I2C;
            dev->snapshot.shutdown_request = false;
            dev->snapshot.state = BQ76922_STATE_READY;
            dev->snapshot.init_step = (uint8_t)BQ76922_INIT_DONE;
            dev->wake_from_absent = false;
            BQ76922_UpdateBusHold(dev, HAL_GetTick());
            return true;
    }

    if (status == BQ76922_BUSY) {
        return false;
    }
    if (status != BQ76922_OK) {
        BQ76922_InitRestart(dev, BQ76922_PROBE_RETRY_MS);
        return false;
    }

    dev->init_step++;
    dev->snapshot.init_step = dev->init_step;
    dev->next_action_ms = HAL_GetTick() + delay_ms;
    return false;
}

void BQ76922_Bind(BQ76922_Device_t *dev, I2C_HandleTypeDef *hi2c)
{
    if (dev == NULL) {
        return;
    }

    memset(dev, 0, sizeof(*dev));
    dev->hi2c = hi2c;
    dev->address_7bit = BQ76922_I2C_ADDR_7BIT;
    dev->snapshot.state = BQ76922_STATE_ABSENT;
    {
        uint8_t i;
        for (i = 0U; i < BQ76922_CELL_COUNT; i++) {
            if (!BQ76922_CellUsed(i)) {
                dev->snapshot.cell_mv[i] = (int16_t)BMS_UNUSED_CELL_MV;
            }
        }
    }
}

BQ76922_Status_t BQ76922_Probe(BQ76922_Device_t *dev)
{
    HAL_StatusTypeDef status;

    if ((dev == NULL) || (dev->hi2c == NULL)) {
        return BQ76922_INVALID_ARG;
    }
    if (!BQ76922_BusIdle(dev)) {
        return BQ76922_BUSY;
    }

    status = HAL_I2C_IsDeviceReady(dev->hi2c,
                                   (uint16_t)(dev->address_7bit << 1),
                                   2U,
                                   20U);
    dev->snapshot.probed = true;
    dev->snapshot.present = (status == HAL_OK);
    if (dev->snapshot.present) {
        dev->snapshot.state = BQ76922_STATE_INIT;
    } else {
        dev->snapshot.state = BQ76922_STATE_ABSENT;
    }
    return (status == HAL_OK) ? BQ76922_OK : BQ76922_NOT_READY;
}

bool BQ76922_IsPresent(const BQ76922_Device_t *dev)
{
    return (dev != NULL) && dev->snapshot.present;
}

bool BQ76922_IsEnabled(void)
{
#if (BMS_ENABLE != 0U)
    return true;
#else
    return false;
#endif
}

bool BQ76922_IsShutdownRequested(const BQ76922_Device_t *dev)
{
#if (BMS_ENABLE == 0U)
    (void)dev;
    return false;
#else
    return (dev != NULL) && dev->snapshot.shutdown_request;
#endif
}

BQ76922_State_t BQ76922_GetState(const BQ76922_Device_t *dev)
{
    if (dev == NULL) {
        return BQ76922_STATE_ABSENT;
    }
    return dev->snapshot.state;
}

void BQ76922_AlertFromIsr(BQ76922_Device_t *dev)
{
    if (dev == NULL) {
        return;
    }
    dev->snapshot.alert_latched = true;
    dev->snapshot.alert_count++;
}

BQ76922_Status_t BQ76922_EnterShutdown(BQ76922_Device_t *dev)
{
    BQ76922_Status_t status;

#if (BMS_ENABLE == 0U)
    (void)dev;
    return BQ76922_NOT_READY;
#else
    if ((dev == NULL) || (dev->hi2c == NULL)) {
        return BQ76922_INVALID_ARG;
    }

    /* Keep the pack path powered until the AFE accepts SHUTDOWN. Sending
     * ALL_FETS_OFF first can brown out this MCU before either shutdown
     * write, leaving an awake AFE with disabled FETs: TS2 cannot wake it.
     * TI: sealed needs two writes within 4 s; unsealed accepts the first
     * and the second skips the configured shutdown delays. */
    status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_SHUTDOWN);
    if (status != BQ76922_OK) {
        return status;
    }
    /* SHUTDOWN clears accumulated charge. RESET_PASSQ runs once on the
     * next init for that reason only. FET-off and a normal boot do not. */
    s_passq_reset_pending = true;
    HAL_Delay(2U);
    status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_SHUTDOWN);
    /* Always invalidate session after the dual SHUTDOWN write. Leaving
     * init_step at DONE + configured=0 falsely re-marked READY without
     * FET_ENABLE — button wake then left MOSFETs off with no host cmd. */
    BQ76922_ForceAbsent(dev);
    return status;
#endif
}

#if (BMS_OTP_SERVICE != 0U)
static BQ76922_Status_t BQ76922_WaitSubcommandDone(BQ76922_Device_t *dev,
                                                   uint16_t subcmd)
{
    uint8_t buf[2];
    uint32_t t0 = HAL_GetTick();
    BQ76922_Status_t status;

    do {
        status = BQ76922_ReadDirect(dev, BQ76922_CMD_SUBCMD_LOW, buf, 2U);
        if (status != BQ76922_OK) {
            return status;
        }
        if (((uint16_t)buf[0] | ((uint16_t)buf[1] << 8)) == subcmd) {
            return BQ76922_OK;
        }
        HAL_Delay(2U);
    } while ((uint32_t)(HAL_GetTick() - t0) < 100U);

    return BQ76922_NOT_READY;
}

/* OTP_WR_CHECK / OTP_WRITE: byte0 = status; if No_DATA, byte1:2 = fail addr. */
static BQ76922_Status_t BQ76922_CallOtpStatus(BQ76922_Device_t *dev,
                                              uint16_t subcmd,
                                              uint8_t *status_byte,
                                              uint16_t *fail_addr)
{
    uint8_t buf[3];
    BQ76922_Status_t status;

    status = BQ76922_SendSubcommand(dev, subcmd);
    if (status != BQ76922_OK) {
        return status;
    }
    /* TI SLUAA32A requires 100 ms before reading OTP_WRITE result.  Polling
     * 0x3E immediately is insufficient: the command echo can already match
     * while programming is still in progress. */
    if (subcmd == BQ76922_SUBCMD_OTP_WRITE) {
        HAL_Delay(100U);
    }
    status = BQ76922_WaitSubcommandDone(dev, subcmd);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_ReadDirect(dev, BQ76922_CMD_RAM_DATA, buf, 3U);
    if (status != BQ76922_OK) {
        return status;
    }
    if (status_byte != NULL) {
        *status_byte = buf[0];
    }
    if (fail_addr != NULL) {
        if ((buf[0] & BQ76922_OTP_NO_DATA) != 0U) {
            *fail_addr = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
        } else {
            *fail_addr = 0U;
        }
    }
    return BQ76922_OK;
}

#endif /* BMS_OTP_SERVICE */

static BQ76922_Status_t BQ76922_ReadRamBlock(BQ76922_Device_t *dev,
                                             uint16_t address,
                                             uint8_t *data,
                                             uint8_t length)
{
    uint8_t addr[2];
    uint8_t echo[2];
    BQ76922_Status_t status;

    if ((data == NULL) || (length == 0U) || (length > 32U)) {
        return BQ76922_INVALID_ARG;
    }
    addr[0] = (uint8_t)(address & 0xFFU);
    addr[1] = (uint8_t)((address >> 8) & 0xFFU);
    status = BQ76922_WriteDirect(dev, BQ76922_CMD_SUBCMD_LOW, addr, 2U);
    if (status != BQ76922_OK) {
        return status;
    }
    HAL_Delay(3U);
    status = BQ76922_ReadDirect(dev, BQ76922_CMD_SUBCMD_LOW, echo, 2U);
    if (status != BQ76922_OK) {
        return status;
    }
    if ((echo[0] != addr[0]) || (echo[1] != addr[1])) {
        return BQ76922_I2C_ERROR;
    }
    return BQ76922_ReadDirect(dev, BQ76922_CMD_RAM_DATA, data, length);
}

static BQ76922_Status_t BQ76922_ReadRamU1(BQ76922_Device_t *dev,
                                          uint16_t address,
                                          uint8_t *out)
{
    return BQ76922_ReadRamBlock(dev, address, out, 1U);
}

static BQ76922_Status_t BQ76922_ReadRamU2(BQ76922_Device_t *dev,
                                          uint16_t address,
                                          uint16_t *out)
{
    uint8_t buf[2];
    BQ76922_Status_t status;

    status = BQ76922_ReadRamBlock(dev, address, buf, 2U);
    if (status != BQ76922_OK) {
        return status;
    }
    *out = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return BQ76922_OK;
}

#if (BMS_OTP_SERVICE != 0U)
static BQ76922_Status_t BQ76922_OtpEnterCfgupdate(BQ76922_Device_t *dev)
{
    BQ76922_Status_t status;
    uint32_t t0;

    status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_SET_CFGUPDATE);
    if (status != BQ76922_OK) {
        return status;
    }
    t0 = HAL_GetTick();
    do {
        status = BQ76922_ReadBatteryStatus(dev);
        if (status != BQ76922_OK) {
            return status;
        }
        if ((dev->snapshot.battery_status & BQ76922_BATT_CFGUPDATE) != 0U) {
            return BQ76922_OK;
        }
        HAL_Delay(5U);
    } while ((uint32_t)(HAL_GetTick() - t0) < 500U);

    return BQ76922_NOT_READY;
}

static BQ76922_Status_t BQ76922_OtpExitCfgupdate(BQ76922_Device_t *dev)
{
    BQ76922_Status_t status;
    uint32_t t0;

    status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_EXIT_CFGUPDATE);
    if (status != BQ76922_OK) {
        return status;
    }
    t0 = HAL_GetTick();
    do {
        status = BQ76922_ReadBatteryStatus(dev);
        if (status != BQ76922_OK) {
            return status;
        }
        if ((dev->snapshot.battery_status & BQ76922_BATT_CFGUPDATE) == 0U) {
            HAL_Delay(20U);
            return BQ76922_OK;
        }
        HAL_Delay(5U);
    } while ((uint32_t)(HAL_GetTick() - t0) < 500U);

    return BQ76922_NOT_READY;
}

static BQ76922_Status_t BQ76922_OtpWriteGolden(BQ76922_Device_t *dev)
{
    uint16_t i;
    BQ76922_Status_t status;

    /* Complete golden image — same table as the runtime RAM path. */
    for (i = 0U; i < g_bms_golden_count; i++) {
        status = BQ76922_WriteGoldenEntry(dev, &g_bms_golden[i]);
        if (status != BQ76922_OK) {
            return status;
        }
    }
    return BQ76922_OK;
}

static BQ76922_Status_t BQ76922_OtpVerifyGolden(BQ76922_Device_t *dev,
                                                uint16_t *fail_addr)
{
    bool match = false;
    uint16_t fail = 0U;
    BQ76922_Status_t status = BQ76922_GoldenCompare(dev, false, &match, &fail);

    if (fail_addr != NULL) {
        *fail_addr = fail;
    }
    if (status != BQ76922_OK) {
        return status;
    }
    return match ? BQ76922_OK : BQ76922_NOT_READY;
}

#endif /* BMS_OTP_SERVICE */

BQ76922_Status_t BQ76922_OtpGetReport(BQ76922_Device_t *dev,
                                      BQ76922_OtpReport_t *out)
{
    BQ76922_Status_t status;
    bool held = false;

#if (BMS_ENABLE == 0U)
    (void)dev;
    (void)out;
    return BQ76922_NOT_READY;
#else
    if ((dev == NULL) || (dev->hi2c == NULL) || (out == NULL)) {
        return BQ76922_INVALID_ARG;
    }

    /* Read-only — never enter CONFIG_UPDATE here. EXIT_CFGUPDATE after
     * OTP_WR_CHECK used to drop FETs and leave the driver thinking cfg=1. */
    memset(out, 0, sizeof(*out));
    out->session_already_burned = s_otp_burned_this_boot;
    out->fet_options_want = BMS_FET_OPTIONS;

    PowerManager_SetBmsBusHold(true);
    held = true;

    status = BQ76922_ReadBatteryStatus(dev);
    if (status != BQ76922_OK) {
        goto out_hold;
    }
    out->battery_status = dev->snapshot.battery_status;
    out->cfgupdate =
        ((dev->snapshot.battery_status & BQ76922_BATT_CFGUPDATE) != 0U);
    out->fullaccess =
        ((dev->snapshot.battery_status & BQ76922_BATT_SEC_MASK) ==
         BQ76922_BATT_SEC_FULLACCESS);
    out->otpb_blocked =
        ((dev->snapshot.battery_status & BQ76922_BATT_OTPB) != 0U);

    status = BQ76922_ReadRamU2(dev, BQ76922_RAM_MFG_STATUS_INIT,
                               &out->mfg_status_init);
    if (status != BQ76922_OK) {
        goto out_hold;
    }
    status = BQ76922_ReadRamU2(dev, BQ76922_RAM_VCELL_MODE, &out->vcell_mode);
    if (status != BQ76922_OK) {
        goto out_hold;
    }
    status = BQ76922_ReadRamU1(dev, BQ76922_RAM_FET_OPTIONS, &out->fet_options);
    if (status != BQ76922_OK) {
        goto out_hold;
    }
    status = BQ76922_GoldenCompare(dev, true, &out->fp_match, &out->fp_fail_addr);
    if (status != BQ76922_OK) {
        goto out_hold;
    }
    status = BQ76922_GoldenCompare(dev, false, &out->ram_match, &out->fail_addr);
    out->golden_crc = BmsGolden_Crc32();
    out->golden_count = g_bms_golden_count;
    out->golden_otp_at_wake = dev->snapshot.golden_otp;

out_hold:
    if (held) {
        PowerManager_SetBmsBusHold(false);
    }
    return status;
#endif
}

#if (BMS_OTP_SERVICE != 0U)
/*
 * RESET reloads RAM from OTP, so what is read right after it IS the burned
 * image.  A blank BQ76922 still shows the TI defaults in every fingerprint
 * field.  Anything else (e.g. the old 4S Li-ion chip) must never be burned
 * again: OTP bits only go 0→1 and the result would be a mix of both images.
 * Drops the FETs for ~0.5 s; the caller re-inits afterwards.
 */
static BQ76922_Status_t BQ76922_OtpBlankCheck(BQ76922_Device_t *dev,
                                              BQ76922_OtpBlank_t *b)
{
    BQ76922_Status_t status;

    memset(b, 0, sizeof(*b));
    status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_RESET);
    if (status != BQ76922_OK) {
        return status;
    }
    HAL_Delay(500U);
    status = BQ76922_ReadRamU2(dev, BQ76922_RAM_VCELL_MODE, &b->vcell_mode);
    if (status != BQ76922_OK) return status;
    status = BQ76922_ReadRamU1(dev, BQ76922_RAM_FET_OPTIONS, &b->fet_options);
    if (status != BQ76922_OK) return status;
    status = BQ76922_ReadRamU2(dev, BQ76922_RAM_MFG_STATUS_INIT, &b->mfg_status_init);
    if (status != BQ76922_OK) return status;
    status = BQ76922_ReadRamU1(dev, BQ76922_RAM_COV_THRESHOLD, &b->cov_threshold);
    if (status != BQ76922_OK) return status;
    status = BQ76922_ReadRamU1(dev, BQ76922_RAM_ENABLED_PROT_B, &b->prot_b);
    if (status != BQ76922_OK) return status;
    b->blank = (b->vcell_mode == BMS_BLANK_VCELL_MODE) &&
               (b->fet_options == BMS_BLANK_FET_OPTIONS) &&
               (b->mfg_status_init == BMS_BLANK_MFG_STATUS_INIT) &&
               (b->cov_threshold == BMS_BLANK_COV_THRESHOLD) &&
               (b->prot_b == BMS_BLANK_PROT_B);
    return BQ76922_OK;
}

BQ76922_Status_t BQ76922_OtpRunWrCheck(BQ76922_Device_t *dev,
                                       uint8_t *wr_check,
                                       uint16_t *fail_addr,
                                       BQ76922_OtpBlank_t *blank)
{
    BQ76922_Status_t status;
    uint8_t check = 0U;
    uint16_t fail = 0U;
    bool held = false;
    bool cfgupdate = false;

#if (BMS_ENABLE == 0U)
    (void)dev;
    (void)wr_check;
    (void)fail_addr;
    return BQ76922_NOT_READY;
#else
    if ((dev == NULL) || (dev->hi2c == NULL)) {
        return BQ76922_INVALID_ARG;
    }

    PowerManager_SetBmsBusHold(true);
    held = true;

    if (blank != NULL) {
        /* Informational here: CHECK may run on any chip. BURN enforces it. */
        status = BQ76922_OtpBlankCheck(dev, blank);
        if (status != BQ76922_OK) {
            goto out_reinit;
        }
    }

    status = BQ76922_OtpEnterCfgupdate(dev);
    if (status != BQ76922_OK) {
        goto out_reinit;
    }
    cfgupdate = true;

    /* Must load golden into RAM first — otherwise CHECK only validates the
     * currently effective OTP image (PDSG already cleared) and looks "OK". */
    status = BQ76922_OtpWriteGolden(dev);
    if (status != BQ76922_OK) {
        goto out_exit_cfg;
    }

    /* TI production sequence: EXIT, read every staged value back, then enter
     * CONFIG_UPDATE again for OTP_WR_CHECK. */
    status = BQ76922_OtpExitCfgupdate(dev);
    if (status != BQ76922_OK) {
        goto out_exit_cfg;
    }
    cfgupdate = false;
    status = BQ76922_OtpVerifyGolden(dev, &fail);
    if (status != BQ76922_OK) {
        goto out_reinit;
    }
    status = BQ76922_OtpEnterCfgupdate(dev);
    if (status != BQ76922_OK) {
        goto out_reinit;
    }
    cfgupdate = true;
    status = BQ76922_ReadBatteryStatus(dev);
    if (status != BQ76922_OK) {
        goto out_exit_cfg;
    }
    if (((dev->snapshot.battery_status & BQ76922_BATT_SEC_MASK) !=
         BQ76922_BATT_SEC_FULLACCESS) ||
        ((dev->snapshot.battery_status & BQ76922_BATT_OTPB) != 0U)) {
        status = BQ76922_NOT_READY;
        goto out_exit_cfg;
    }

    status = BQ76922_CallOtpStatus(dev, BQ76922_SUBCMD_OTP_WR_CHECK,
                                   &check, &fail);
    if (wr_check != NULL) {
        *wr_check = check;
    }
    if (fail_addr != NULL) {
        *fail_addr = fail;
    }

out_exit_cfg:
    if (cfgupdate) {
        (void)BQ76922_OtpExitCfgupdate(dev);
    }
out_reinit:
    if (fail_addr != NULL) {
        *fail_addr = fail;
    }
    /* EXIT_CFGUPDATE kills FET drivers — bring FETs back. */
    BQ76922_RequestReinit(dev);

    if (held) {
        PowerManager_SetBmsBusHold(false);
    }
    if ((status == BQ76922_OK) && (check != BQ76922_OTP_OK_CODE)) {
        return BQ76922_NOT_READY;
    }
    return status;
#endif
}

BQ76922_Status_t BQ76922_OtpBurn(BQ76922_Device_t *dev,
                                 const char *confirm_token,
                                 uint8_t *wr_check,
                                 uint8_t *wr_result,
                                 uint16_t *fail_addr,
                                 BQ76922_OtpBlank_t *blank)
{
    BQ76922_OtpBlank_t local_blank;
    BQ76922_Status_t status;
    uint8_t check = 0U;
    uint8_t result = 0U;
    uint16_t fail = 0U;
    bool held = false;
    bool cfgupdate = false;

#if (BMS_ENABLE == 0U)
    (void)dev;
    (void)confirm_token;
    (void)wr_check;
    (void)wr_result;
    (void)fail_addr;
    return BQ76922_NOT_READY;
#else
    if ((dev == NULL) || (dev->hi2c == NULL)) {
        return BQ76922_INVALID_ARG;
    }
    if ((confirm_token == NULL) ||
        (strcmp(confirm_token, BQ76922_OTP_CONFIRM_TOKEN) != 0)) {
        return BQ76922_INVALID_ARG;
    }
    if (s_otp_burned_this_boot) {
        if (wr_check != NULL) {
            *wr_check = 0U;
        }
        if (wr_result != NULL) {
            *wr_result = 0U;
        }
        if (fail_addr != NULL) {
            *fail_addr = 0U;
        }
        return BQ76922_NOT_READY;
    }

#if (BMS_THERMISTOR_MODEL_VALIDATED == 0U)
    /* Await physical thermal validation and resolution of PCB-vs-cell sensing. */
    return BQ76922_NOT_READY;
#endif

    PowerManager_SetBmsBusHold(true);
    held = true;

    if (blank == NULL) {
        blank = &local_blank;
    }
    status = BQ76922_OtpBlankCheck(dev, blank);
    if (status != BQ76922_OK) {
        goto out_reinit;
    }
    if (!blank->blank) {
        /* Not a factory-blank chip: refuse before anything is staged. */
        status = BQ76922_NOT_BLANK;
        goto out_reinit;
    }

    status = BQ76922_OtpEnterCfgupdate(dev);
    if (status != BQ76922_OK) {
        goto out_reinit;
    }
    cfgupdate = true;

    status = BQ76922_OtpWriteGolden(dev);
    if (status != BQ76922_OK) {
        goto out_exit_cfg;
    }

    /* Do not trust successful I2C writes as proof.  Commit RAM, verify every
     * golden field, and only then reopen CONFIG_UPDATE for OTP. */
    status = BQ76922_OtpExitCfgupdate(dev);
    if (status != BQ76922_OK) {
        goto out_exit_cfg;
    }
    cfgupdate = false;
    status = BQ76922_OtpVerifyGolden(dev, &fail);
    if (status != BQ76922_OK) {
        goto out_reinit;
    }
    status = BQ76922_OtpEnterCfgupdate(dev);
    if (status != BQ76922_OK) {
        goto out_reinit;
    }
    cfgupdate = true;

    status = BQ76922_ReadBatteryStatus(dev);
    if (status != BQ76922_OK) {
        goto out_exit_cfg;
    }
    if (((dev->snapshot.battery_status & BQ76922_BATT_SEC_MASK) !=
         BQ76922_BATT_SEC_FULLACCESS) ||
        ((dev->snapshot.battery_status & BQ76922_BATT_OTPB) != 0U)) {
        status = BQ76922_NOT_READY;
        goto out_exit_cfg;
    }

    status = BQ76922_CallOtpStatus(dev, BQ76922_SUBCMD_OTP_WR_CHECK,
                                   &check, &fail);
    if (wr_check != NULL) {
        *wr_check = check;
    }
    if (fail_addr != NULL) {
        *fail_addr = fail;
    }
    if ((status != BQ76922_OK) || (check != BQ76922_OTP_OK_CODE)) {
        if (status == BQ76922_OK) {
            status = BQ76922_NOT_READY;
        }
        goto out_exit_cfg;
    }

    /* A failed response does not prove that OTP_WRITE never reached the AFE.
     * Refuse all further attempts this MCU boot, including transport errors. */
    s_otp_burned_this_boot = true;
    status = BQ76922_CallOtpStatus(dev, BQ76922_SUBCMD_OTP_WRITE,
                                   &result, &fail);
    if (wr_result != NULL) {
        *wr_result = result;
    }
    if (fail_addr != NULL) {
        *fail_addr = fail;
    }
    if ((status != BQ76922_OK) || (result != BQ76922_OTP_OK_CODE)) {
        if (status == BQ76922_OK) {
            status = BQ76922_NOT_READY;
        }
        goto out_exit_cfg;
    }

    s_otp_burned_this_boot = true;
    status = BQ76922_OK;

out_exit_cfg:
    if (cfgupdate) {
        BQ76922_Status_t exit_status = BQ76922_OtpExitCfgupdate(dev);
        cfgupdate = false;
        if ((status == BQ76922_OK) && (exit_status != BQ76922_OK)) {
            status = exit_status;
        }
    }
    if (status == BQ76922_OK) {
        /* Only RESET proves that values reload from OTP rather than remaining
         * in the staged RAM image. */
        status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_RESET);
        if (status == BQ76922_OK) {
            HAL_Delay(500U);
            status = BQ76922_OtpVerifyGolden(dev, &fail);
        }
        /* Keep the one-shot guard set even if reset readback fails: OTP_WRITE
         * already returned success and repeating it would consume another
         * signature image. */
    }
out_reinit:
    if (fail_addr != NULL) {
        *fail_addr = fail;
    }
    /* Reload FETs after CFGUPDATE/reset. */
    BQ76922_RequestReinit(dev);

    if (held) {
        PowerManager_SetBmsBusHold(false);
    }
    return status;
#endif
}

#endif /* BMS_OTP_SERVICE */

void BQ76922_ClearShutdownRequest(BQ76922_Device_t *dev)
{
    if (dev == NULL) {
        return;
    }

    dev->snapshot.shutdown_request = false;
    if (dev->snapshot.state == BQ76922_STATE_FAULT) {
        dev->snapshot.state = dev->snapshot.configured ?
                              BQ76922_STATE_READY :
                              BQ76922_STATE_INIT;
    }
    dev->snapshot.fault_flags &= (uint32_t)~(BQ76922_FAULT_CELL_OV |
                                             BQ76922_FAULT_CELL_UV |
                                             BQ76922_FAULT_SAFETY);
}

bool BQ76922_IsConfiguredHealthy(const BQ76922_Device_t *dev)
{
    if ((dev == NULL) || !BQ76922_IsEnabled()) {
        return false;
    }
    if (!dev->snapshot.present || !dev->snapshot.configured) {
        return false;
    }
    if (!dev->snapshot.golden_ok) {
        return false;
    }
    /* Require both charge and discharge paths — matches a good TB line. */
    return (dev->snapshot.chg_fet_on &&
            (dev->snapshot.dsg_fet_on ||
             ((dev->snapshot.fet_status & BQ76922_FET_STATUS_PDSG) != 0U)));
}

/* Drop AFE session so the next presence (TS2 / LD wake) re-probes and runs
 * full CFGUPDATE + FET_ENABLE + ALL_FETS_ON — no host command. */
static void BQ76922_ForceAbsent(BQ76922_Device_t *dev)
{
    if (dev == NULL) {
        return;
    }

    dev->snapshot.present = false;
    dev->snapshot.probed = false;
    dev->snapshot.configured = false;
    dev->snapshot.fets_enabled = false;
    dev->snapshot.chg_fet_on = false;
    dev->snapshot.dsg_fet_on = false;
    dev->snapshot.sample_valid = false;
    dev->snapshot.shutdown_request = false;
    dev->snapshot.vcell_mode_rb = 0U;
    dev->snapshot.golden_ok = false;
    dev->snapshot.manuf_status = 0U;
    dev->snapshot.fet_status = 0U;
    dev->snapshot.state = BQ76922_STATE_ABSENT;
    dev->init_step = (uint8_t)BQ76922_INIT_WAIT_READY;
    dev->snapshot.init_step = dev->init_step;
    dev->cfg_poll_tries = 0U;
    dev->cfg_enter_tries = 0U;
    dev->fet_en_attempts = 0U;
    dev->fet_verify_attempts = 0U;
    dev->fet_recover_fails = 0U;
    dev->i2c_absent_streak = 0U;
    dev->scan_index = 0U;
    dev->bus_hold_since_ms = 0U;
    dev->boot_settle_ms = 0U;
    PowerManager_SetBmsBusHold(false);
    dev->next_action_ms = HAL_GetTick() + BQ76922_PROBE_RETRY_MS;
}

/* True when AFE should be treated as gone (SHUTDOWN / unpowered). */
static bool BQ76922_NoteCommsResult(BQ76922_Device_t *dev,
                                    BQ76922_Status_t status)
{
    if (status == BQ76922_OK) {
        dev->i2c_absent_streak = 0U;
        return false;
    }
    if (status != BQ76922_I2C_ERROR) {
        return false;
    }
    if (dev->i2c_absent_streak < 255U) {
        dev->i2c_absent_streak++;
    }
    if (dev->i2c_absent_streak >= BQ76922_I2C_ABSENT_STREAK) {
        BQ76922_ForceAbsent(dev);
        return true;
    }
    return false;
}

void BQ76922_RequestReinit(BQ76922_Device_t *dev)
{
    if (dev == NULL) {
        return;
    }

    /* Clears configured → I2C bus-hold starves BQ25731/TPS until init finishes.
     * EXIT_CFGUPDATE restarts AFE digital (FETs off) then ALL_FETS_ON can
     * inrush PACK caps hard enough to dip VIN / NRST (PIN+POR). */
    dev->snapshot.configured = false;
    dev->snapshot.fets_enabled = false;
    dev->snapshot.chg_fet_on = false;
    dev->snapshot.dsg_fet_on = false;
    dev->snapshot.sample_valid = false;
    dev->snapshot.shutdown_request = false;
    dev->snapshot.vcell_mode_rb = 0U;
    dev->snapshot.golden_ok = false;
    dev->snapshot.manuf_status = 0U;
    dev->snapshot.state = BQ76922_STATE_INIT;
    dev->init_step = (uint8_t)BQ76922_INIT_WAIT_READY;
    dev->snapshot.init_step = dev->init_step;
    dev->cfg_poll_tries = 0U;
    dev->cfg_enter_tries = 0U;
    dev->fet_en_attempts = 0U;
    dev->fet_verify_attempts = 0U;
    dev->fet_recover_fails = 0U;
    dev->i2c_absent_streak = 0U;
    dev->scan_index = 0U;
    dev->bus_hold_since_ms = 0U;
    dev->boot_settle_ms = 0U;
    dev->next_action_ms = HAL_GetTick();
    BQ76922_UpdateBusHold(dev, HAL_GetTick());
}

int32_t BQ76922_UserAhToMah(int32_t whole, uint32_t frac)
{
    uint64_t bits = ((uint64_t)(uint32_t)whole << 32) | (uint64_t)frac;
    int64_t q = (int64_t)bits;
    int64_t mah;

    if (q >= 0) {
        mah = (q + 0x80000000LL) >> 32;
    } else {
        uint64_t mag = (uint64_t)(-(q + 1)) + 1U;

        mah = -((int64_t)((mag + 0x80000000ULL) >> 32));
    }
    if (mah > 2147483647LL) {
        return 2147483647;
    }
    if (mah < (-2147483647LL - 1)) {
        return (int32_t)(-2147483647 - 1);
    }
    return (int32_t)mah;
}

void BQ76922_Task(BQ76922_Device_t *dev, uint32_t now_ms)
{
#if (BMS_ENABLE == 0U)
    (void)dev;
    (void)now_ms;
    PowerManager_SetBmsBusHold(false);
    return;
#else
    uint8_t buf[2];
    BQ76922_Status_t status;

    if ((dev == NULL) || (dev->hi2c == NULL)) {
        return;
    }

    if (BmsFetGuard_Expired(&dev->fet_guard, now_ms)) {
        dev->snapshot.fault_flags |= BQ76922_FAULT_FET_START;
        dev->snapshot.shutdown_request = true;
        dev->snapshot.state = BQ76922_STATE_FAULT;
        dev->snapshot.configured = false;
        PowerManager_SetBmsBusHold(false);
        if (!dev->fet_guard_off_sent) {
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_ALL_FETS_OFF);
            if (status == BQ76922_OK) dev->fet_guard_off_sent = true;
        }
        return;
    }

    BQ76922_UpdateBusHold(dev, now_ms);

    dev->snapshot.alert_pin =
        (HAL_GPIO_ReadPin(BMS_ALERT_GPIO_Port, BMS_ALERT_Pin) == GPIO_PIN_SET);

    if (BoardMx_TakeBmsAlert()) {
        BQ76922_AlertFromIsr(dev);
    }

    if ((int32_t)(now_ms - dev->next_action_ms) < 0) {
        return;
    }

    if (!dev->snapshot.probed || !dev->snapshot.present) {
        status = BQ76922_Probe(dev);
        if (status == BQ76922_BUSY) {
            return;
        }
        if (dev->snapshot.present) {
            /* Button/LD wake: AFE just returned — FET bring-up, no host cmd.
             * Prefer OTP fast path (skip CFGUPDATE) so FETs are not held off. */
            dev->i2c_absent_streak = 0U;
            dev->boot_settle_ms = 0U;
            dev->wake_from_absent = true;
            dev->init_step = (uint8_t)BQ76922_INIT_WAIT_READY;
            dev->snapshot.init_step = dev->init_step;
            dev->snapshot.configured = false;
            dev->snapshot.manuf_status = 0U;
            dev->bus_hold_since_ms = 0U;
            BQ76922_UpdateBusHold(dev, now_ms);
            dev->next_action_ms = now_ms + BQ76922_INIT_STEP_DELAY_MS;
        } else {
            BQ76922_UpdateBusHold(dev, now_ms);
            dev->next_action_ms = now_ms + BQ76922_PROBE_RETRY_MS;
        }
        return;
    }

    if (!dev->snapshot.configured) {
        (void)BQ76922_RunInitStep(dev);
        BQ76922_UpdateBusHold(dev, now_ms);
        return;
    }

    if (dev->snapshot.alert_latched) {
        status = BQ76922_HandleAlert(dev);
        if (BQ76922_NoteCommsResult(dev, status)) {
            return;
        }
        if (status == BQ76922_BUSY) {
            return;
        }
        dev->next_action_ms = now_ms + BQ76922_SCAN_PERIOD_MS;
        return;
    }

    if (dev->scan_index < BQ76922_CELL_COUNT) {
        if (!BQ76922_CellUsed(dev->scan_index)) {
            dev->snapshot.cell_mv[dev->scan_index] = (int16_t)BMS_UNUSED_CELL_MV;
            dev->scan_index++;
        } else {
            status = BQ76922_ReadDirect(dev,
                                        (uint8_t)(BQ76922_CMD_CELL1_V +
                                                  (dev->scan_index * 2U)),
                                        buf,
                                        2U);
            if (BQ76922_NoteCommsResult(dev, status)) {
                return;
            }
            if (status == BQ76922_OK) {
                dev->snapshot.cell_mv[dev->scan_index] = BQ76922_Le16(buf);
                dev->scan_index++;
            } else if (status == BQ76922_BUSY) {
                return;
            }
        }
    } else if (dev->scan_index == BQ76922_CELL_COUNT) {
        status = BQ76922_ReadDirect(dev, BQ76922_CMD_PACK_V, buf, 2U);
        if (BQ76922_NoteCommsResult(dev, status)) {
            return;
        }
        if (status == BQ76922_OK) {
            dev->snapshot.pack_mv = BQ76922_USERV_TO_MV(BQ76922_Le16(buf));
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else if (dev->scan_index == (BQ76922_CELL_COUNT + 1U)) {
        status = BQ76922_ReadDirect(dev, BQ76922_CMD_STACK_V, buf, 2U);
        if (BQ76922_NoteCommsResult(dev, status)) {
            return;
        }
        if (status == BQ76922_OK) {
            dev->snapshot.stack_mv = BQ76922_USERV_TO_MV(BQ76922_Le16(buf));
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else if (dev->scan_index == (BQ76922_CELL_COUNT + 2U)) {
        status = BQ76922_ReadDirect(dev, BQ76922_CMD_CC2, buf, 2U);
        if (BQ76922_NoteCommsResult(dev, status)) {
            return;
        }
        if (status == BQ76922_OK) {
            dev->snapshot.cc2_ma = BQ76922_Le16(buf);
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else if (dev->scan_index == (BQ76922_CELL_COUNT + 3U)) {
        status = BQ76922_ReadDirect(dev, BQ76922_CMD_FET_STATUS, buf, 1U);
        if (BQ76922_NoteCommsResult(dev, status)) {
            return;
        }
        if (status == BQ76922_OK) {
            dev->snapshot.fet_status = buf[0];
            BmsFetGuard_Observe(&dev->fet_guard, buf[0], HAL_GetTick());
            dev->snapshot.chg_fet_on = ((buf[0] & BQ76922_FET_STATUS_CHG) != 0U);
            dev->snapshot.dsg_fet_on = ((buf[0] & BQ76922_FET_STATUS_DSG) != 0U);
            /* PDSG counts as discharge path while soft-starting PACK caps. */
            dev->snapshot.fets_enabled =
                (dev->snapshot.chg_fet_on ||
                 dev->snapshot.dsg_fet_on ||
                 ((buf[0] & BQ76922_FET_STATUS_PDSG) != 0U));
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else if (dev->scan_index == (BQ76922_CELL_COUNT + 4U)) {
        status = BQ76922_ReadSafetyStatus(dev);
        if (BQ76922_NoteCommsResult(dev, status)) {
            return;
        }
        if (status == BQ76922_OK) {
            (void)BQ76922_ReadBatteryStatus(dev);
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else if (dev->scan_index == (BQ76922_CELL_COUNT + 5U)) {
        uint8_t ram[12];
        int32_t whole;
        uint32_t frac;

        status = BQ76922_ReadRamBlock(dev, BQ76922_SUBCMD_DASTATUS6, ram, 12U);
        if (BQ76922_NoteCommsResult(dev, status)) {
            return;
        }
        if (status == BQ76922_OK) {
            /* userAh, integer then fraction. CC2 is mA, so the integer is mAh. */
            whole = (int32_t)((uint32_t)ram[0] |
                              ((uint32_t)ram[1] << 8) |
                              ((uint32_t)ram[2] << 16) |
                              ((uint32_t)ram[3] << 24));
            frac = (uint32_t)ram[4] |
                   ((uint32_t)ram[5] << 8) |
                   ((uint32_t)ram[6] << 16) |
                   ((uint32_t)ram[7] << 24);
            dev->snapshot.passq_mah = BQ76922_UserAhToMah(whole, frac);
            dev->snapshot.passq_whole = whole;
            dev->snapshot.passq_frac = frac;
            dev->snapshot.passq_time_s = (uint32_t)ram[8] |
                                         ((uint32_t)ram[9] << 8) |
                                         ((uint32_t)ram[10] << 16) |
                                         ((uint32_t)ram[11] << 24);
            dev->snapshot.passq_valid = true;
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else if (dev->scan_index == (BQ76922_CELL_COUNT + 6U)) {
        uint8_t ram[24];

        status = BQ76922_ReadRamBlock(dev, BQ76922_SUBCMD_DASTATUS5, ram, 24U);
        if (BQ76922_NoteCommsResult(dev, status)) {
            return;
        }
        if (status == BQ76922_OK) {
            dev->snapshot.cc1_ma =
                (int16_t)((uint16_t)ram[22] | ((uint16_t)ram[23] << 8));
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else if (dev->scan_index == (BQ76922_CELL_COUNT + 7U)) {
        status = BQ76922_ReadDirect(dev, BQ76922_CMD_INT_TEMP, buf, 2U);
        if (BQ76922_NoteCommsResult(dev, status)) {
            return;
        }
        if (status == BQ76922_OK) {
            dev->snapshot.int_temp_dk = BQ76922_Le16(buf);
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else if (dev->scan_index == (BQ76922_CELL_COUNT + 8U)) {
        status = BQ76922_ReadDirect(dev, BQ76922_CMD_TS1_TEMP, buf, 2U);
        if (BQ76922_NoteCommsResult(dev, status)) {
            return;
        }
        if (status == BQ76922_OK) {
            dev->snapshot.ts1_temp_dk = BQ76922_Le16(buf);
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else if (dev->scan_index == (BQ76922_CELL_COUNT + 9U)) {
        uint16_t mask = 0U;

        static BmsBalancePolicy balance_policy;
        BmsGauge_Output_t gauge;
        uint8_t desired;
        uint8_t payload[2];
        BmsGauge_GetOutput(&gauge);
        bool valid = dev->snapshot.configured && dev->snapshot.sample_valid &&
            (uint32_t)(now_ms-dev->snapshot.sample_tick_ms)<3000U &&
            BmsBalance_SafetyAllows(dev->snapshot.safety_status_a,
                dev->snapshot.safety_status_b,dev->snapshot.safety_status_c,
                dev->snapshot.chg_fet_on,dev->snapshot.dsg_fet_on) &&
            !dev->snapshot.shutdown_request && !dev->fet_guard.latched &&
            dev->snapshot.ts1_temp_dk>=2732 && dev->snapshot.ts1_temp_dk<=3232 &&
            dev->snapshot.int_temp_dk>0 && dev->snapshot.int_temp_dk<=3432;
        desired=BmsBalance_Select(&balance_policy,now_ms,valid,
            dev->snapshot.cc2_ma,gauge.soc_permille,dev->snapshot.cell_mv,
            dev->snapshot.balance_mask);
        payload[0]=desired; payload[1]=0;
        /* Renew host timeout; readback below, never report the requested mask. */
        status=BQ76922_WriteRamBlock(dev,BQ76922_SUBCMD_CB_ACTIVE_CELLS,payload,2U);
        /* BUSY is arbitration, not a failed command. Preserve eligibility
         * and retry this scan step when the shared bus becomes available. */
        if (status==BQ76922_BUSY) return;
        if (status!=BQ76922_OK) {
            balance_policy.waiting=false;
            dev->snapshot.balance_mask=0;
            if (BQ76922_NoteCommsResult(dev,status)) return;
            if (status==BQ76922_BUSY) return;
        }

        status = BQ76922_ReadSubcmdU2(dev, BQ76922_SUBCMD_CB_ACTIVE_CELLS, &mask);
        if (BQ76922_NoteCommsResult(dev, status)) {
            return;
        }
        if (status == BQ76922_OK) {
            dev->snapshot.balance_mask = (uint8_t)(mask & 0x1FU);
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else {
        dev->snapshot.sample_valid = true;
        dev->snapshot.sample_tick_ms = now_ms;
        dev->scan_index = 0U;
        BQ76922_UpdateCellStats(dev);
        BQ76922_DecodeSafetyFaults(dev);
        BQ76922_UpdateWarnState(dev);

        /* Let the AFE's protection/recovery policy run. Never restart a
         * failed power path repeatedly through the predischarge resistor. */
        if (!dev->snapshot.dsg_fet_on &&
            !(dev->snapshot.fet_status & BQ76922_FET_STATUS_PDSG)) {
            dev->fet_guard.latched = true;
        }

    }

    dev->next_action_ms = now_ms + BQ76922_SCAN_PERIOD_MS;
#endif /* BMS_ENABLE */
}

void BQ76922_GetSnapshot(const BQ76922_Device_t *dev, BQ76922_Snapshot_t *out)
{
    if ((dev == NULL) || (out == NULL)) {
        return;
    }
    *out = dev->snapshot;
}
