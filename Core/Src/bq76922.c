#include "bq76922.h"

#include "bms_board.h"
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
#define BQ76922_SUBCMD_OTP_WR_CHECK      0x00A0U
#define BQ76922_SUBCMD_OTP_WRITE         0x00A1U

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
#define BQ76922_RAM_FET_OPTIONS          0x9308U
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
#define BQ76922_OTP_CONFIRM_TOKEN        "I-UNDERSTAND-OTP"
/* Default DA Config USER_VOLTS_CV=1 → Stack/PACK in centivolts (10 mV). */
#define BQ76922_USERV_TO_MV(raw)         ((int16_t)((int16_t)(raw) * 10))

typedef enum {
    BQ76922_INIT_WAIT_READY = 0,
    BQ76922_INIT_SLEEP_DISABLE_PRE,
    BQ76922_INIT_CLR_ALARM_PRE,
    BQ76922_INIT_ENTER_CFG,
    BQ76922_INIT_WAIT_CFG,
    BQ76922_INIT_VCELL_MODE,
    BQ76922_INIT_PROT_A,
    BQ76922_INIT_PROT_B,
    BQ76922_INIT_CHG_FET_PROT_A,
    BQ76922_INIT_DSG_FET_PROT_A,
    BQ76922_INIT_CUV_THRESH,
    BQ76922_INIT_COV_THRESH,
    BQ76922_INIT_OCC_THRESH,
    BQ76922_INIT_OCD1_THRESH,
    BQ76922_INIT_OCC_RECOVERY,
    BQ76922_INIT_CC_GAIN,
    BQ76922_INIT_CAPACITY_GAIN,
    BQ76922_INIT_CFETOFF_PIN,
    BQ76922_INIT_DFETOFF_PIN,
    BQ76922_INIT_FET_OPTIONS,
    BQ76922_INIT_MFG_STATUS_INIT,
    BQ76922_INIT_AUTO_SHUTDOWN,
    BQ76922_INIT_PDSG_TIMEOUT,
    BQ76922_INIT_PDSG_STOP_DELTA,
    BQ76922_INIT_SCD_THRESH,
    BQ76922_INIT_BODY_DIODE,
    BQ76922_INIT_EXIT_CFG,
    BQ76922_INIT_WAIT_EXIT,
    BQ76922_INIT_READ_VCELL_ADDR,
    BQ76922_INIT_READ_VCELL_DATA,
    BQ76922_INIT_SLEEP_DISABLE,
    BQ76922_INIT_CLR_ALARM,
    BQ76922_INIT_MANUF_ISSUE,
    BQ76922_INIT_MANUF_READ,
    BQ76922_INIT_FET_ENABLE,
    BQ76922_INIT_MANUF_CONFIRM_ISSUE,
    BQ76922_INIT_MANUF_CONFIRM_READ,
    BQ76922_INIT_ALL_FETS_ON,
    BQ76922_INIT_FET_SETTLE,
    BQ76922_INIT_FET_VERIFY,
    BQ76922_INIT_DONE,
} BQ76922_InitStep_t;

/*
 * VCell Mode 0x0017 has bit4 set — the same bit as Manufacturing Status
 * FET_EN. A stale 0x40 buffer after VCELL readback must never be treated
 * as manuf status or we skip FET_ENABLE and leave FETs in test mode.
 */
static bool BQ76922_ManufLooksStaleVcell(uint16_t manuf)
{
    return manuf == BMS_VCELL_MODE;
}

static void BQ76922_ForceAbsent(BQ76922_Device_t *dev);
static bool BQ76922_NoteCommsResult(BQ76922_Device_t *dev,
                                    BQ76922_Status_t status);

/* Session guard — OTP burn is never automatic; UART may do it once per boot. */
static bool s_otp_burned_this_boot = false;

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

    buf[0] = (uint8_t)(subcmd & 0xFFU);
    buf[1] = (uint8_t)((subcmd >> 8) & 0xFFU);
    return BQ76922_WriteDirect(dev, BQ76922_CMD_SUBCMD_LOW, buf, 2U);
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
    return BQ76922_WriteDirect(dev, BQ76922_CMD_RAM_CHECKSUM, checksum_block, 2U);
}

static BQ76922_Status_t BQ76922_WriteRamU1(BQ76922_Device_t *dev,
                                           uint16_t address,
                                           uint8_t value)
{
    return BQ76922_WriteRamBlock(dev, address, &value, 1U);
}

static BQ76922_Status_t BQ76922_WriteRamU2(BQ76922_Device_t *dev,
                                           uint16_t address,
                                           uint16_t value)
{
    uint8_t data[2];

    data[0] = (uint8_t)(value & 0xFFU);
    data[1] = (uint8_t)((value >> 8) & 0xFFU);
    return BQ76922_WriteRamBlock(dev, address, data, 2U);
}

static BQ76922_Status_t BQ76922_WriteRamF4(BQ76922_Device_t *dev,
                                           uint16_t address,
                                           float value)
{
    uint8_t data[4];
    union {
        float f;
        uint32_t u;
    } conv;

    conv.f = value;
    data[0] = (uint8_t)(conv.u & 0xFFU);
    data[1] = (uint8_t)((conv.u >> 8) & 0xFFU);
    data[2] = (uint8_t)((conv.u >> 16) & 0xFFU);
    data[3] = (uint8_t)((conv.u >> 24) & 0xFFU);
    return BQ76922_WriteRamBlock(dev, address, data, 4U);
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

    /* Only treat COV/CUV as cell faults when voltages agree — a capacitive
     * ALL_FETS_ON transient can latch safety bits without a real OV/UV. */
    real_cov = ((sa & BQ76922_SAFETY_A_COV) != 0U) &&
               dev->snapshot.sample_valid &&
               (dev->snapshot.max_cell_mv >= (int16_t)BMS_COV_THRESHOLD_MV);
    real_cuv = ((sa & BQ76922_SAFETY_A_CUV) != 0U) &&
               dev->snapshot.sample_valid &&
               (dev->snapshot.min_cell_mv > 0) &&
               (dev->snapshot.min_cell_mv <= (int16_t)BMS_CUV_THRESHOLD_MV);

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

    /* SCD/OCD/OCC and false COV are recoverable — clear + ALL_FETS_ON.
     * Only confirmed cell UV/OV may request a host rail shutdown. */
    if ((flags & (BQ76922_FAULT_CELL_OV | BQ76922_FAULT_CELL_UV)) != 0U) {
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
                if ((uint32_t)(HAL_GetTick() - dev->boot_settle_ms) <
                    BQ76922_BOOT_SETTLE_MS) {
                    delay_ms = 50U;
                    dev->next_action_ms = HAL_GetTick() + delay_ms;
                    return false;
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
            }
            break;

        case BQ76922_INIT_VCELL_MODE:
            status = BQ76922_WriteRamU2(dev, BQ76922_RAM_VCELL_MODE, BMS_VCELL_MODE);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_PROT_A:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_ENABLED_PROT_A,
                                        BMS_ENABLED_PROTECTIONS_A);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_PROT_B:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_ENABLED_PROT_B,
                                        BMS_ENABLED_PROTECTIONS_B);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_CHG_FET_PROT_A:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_CHG_FET_PROT_A,
                                        BMS_CHG_FET_PROTECTIONS_A);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_DSG_FET_PROT_A:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_DSG_FET_PROT_A,
                                        BMS_DSG_FET_PROTECTIONS_A);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_CUV_THRESH:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_CUV_THRESHOLD,
                                        BMS_CUV_THRESHOLD_CODE);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_COV_THRESH:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_COV_THRESHOLD,
                                        BMS_COV_THRESHOLD_CODE);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_OCC_THRESH:
            /* Raise above default ~0.8 A @ 5 mOhm so USB-C charge works. */
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_OCC_THRESHOLD,
                                        BMS_OCC_THRESHOLD_CODE);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_OCD1_THRESH:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_OCD1_THRESHOLD,
                                        BMS_OCD1_THRESHOLD_CODE);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_OCC_RECOVERY:
            /* I2 little-endian; +100 mA lets I≈0 clear a latched OCC. */
            status = BQ76922_WriteRamU2(dev, BQ76922_RAM_OCC_RECOVERY,
                                        (uint16_t)(int16_t)BMS_OCC_RECOVERY_MA);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_CC_GAIN:
            /* 5 mOhm pack shunt — TRM CC Gain = 7.5684 / Rsense_mOhm. */
            status = BQ76922_WriteRamF4(dev, BQ76922_RAM_CC_GAIN, BMS_CC_GAIN);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_CAPACITY_GAIN:
            status = BQ76922_WriteRamF4(dev, BQ76922_RAM_CAPACITY_GAIN,
                                        BMS_CAPACITY_GAIN);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_CFETOFF_PIN:
            /* Unused — floating TP29 must not force CHG off. */
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_CFETOFF_PIN_CONFIG,
                                        BMS_CFETOFF_PIN_CONFIG);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_DFETOFF_PIN:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_DFETOFF_PIN_CONFIG,
                                        BMS_DFETOFF_PIN_CONFIG);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_FET_OPTIONS:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_FET_OPTIONS,
                                        BMS_FET_OPTIONS);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_MFG_STATUS_INIT:
            /* OTP wake needs FET_EN=1 here — FET_ENABLE() alone is RAM-only. */
            status = BQ76922_WriteRamU2(dev, BQ76922_RAM_MFG_STATUS_INIT,
                                        BMS_MFG_STATUS_INIT);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_AUTO_SHUTDOWN:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_AUTO_SHUTDOWN_TIME,
                                        BMS_AUTO_SHUTDOWN_TIME);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_PDSG_TIMEOUT:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_PDSG_TIMEOUT,
                                        BMS_PDSG_TIMEOUT);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_PDSG_STOP_DELTA:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_PDSG_STOP_DELTA,
                                        BMS_PDSG_STOP_DELTA);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_SCD_THRESH:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_SCD_THRESHOLD,
                                        BMS_SCD_THRESHOLD);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_BODY_DIODE:
            status = BQ76922_WriteRamU2(dev, BQ76922_RAM_BODY_DIODE,
                                        BMS_BODY_DIODE_THRESHOLD_MA);
            delay_ms = 20U;
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

        case BQ76922_INIT_READ_VCELL_ADDR:
            buf[0] = (uint8_t)(BQ76922_RAM_VCELL_MODE & 0xFFU);
            buf[1] = (uint8_t)((BQ76922_RAM_VCELL_MODE >> 8) & 0xFFU);
            status = BQ76922_WriteDirect(dev, BQ76922_CMD_SUBCMD_LOW, buf, 2U);
            delay_ms = 15U;
            break;

        case BQ76922_INIT_READ_VCELL_DATA:
            status = BQ76922_ReadDirect(dev, BQ76922_CMD_RAM_DATA, buf, 2U);
            if (status == BQ76922_OK) {
                dev->snapshot.vcell_mode_rb = (uint16_t)BQ76922_Le16(buf);
                if (dev->snapshot.vcell_mode_rb != BMS_VCELL_MODE) {
                    BQ76922_InitRestart(dev, BQ76922_PROBE_RETRY_MS);
                    return false;
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

        case BQ76922_INIT_MANUF_ISSUE:
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_MANUF_STATUS);
            delay_ms = 15U;
            break;

        case BQ76922_INIT_MANUF_READ:
            status = BQ76922_ReadDirect(dev, BQ76922_CMD_RAM_DATA, buf, 2U);
            if (status == BQ76922_OK) {
                uint16_t manuf = (uint16_t)BQ76922_Le16(buf);

                if (BQ76922_ManufLooksStaleVcell(manuf)) {
                    /* 0x40 still has VCell Mode — re-issue ManufacturingStatus. */
                    status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_MANUF_STATUS);
                    if (status == BQ76922_OK) {
                        delay_ms = 20U;
                        dev->next_action_ms = HAL_GetTick() + delay_ms;
                        return false;
                    }
                    break;
                }
                dev->snapshot.manuf_status = manuf;
                /* FET_ENABLE is a toggle — do not send it if FET_EN is already 1. */
                if ((manuf & BQ76922_MANUF_FET_EN) != 0U) {
                    dev->init_step = (uint8_t)BQ76922_INIT_ALL_FETS_ON;
                    dev->snapshot.init_step = dev->init_step;
                    dev->next_action_ms = HAL_GetTick() + delay_ms;
                    return false;
                }
            }
            break;

        case BQ76922_INIT_FET_ENABLE:
            /* Blank OTP: FET_EN=0 (FET Test Mode). Toggle into firmware FET control. */
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_FET_ENABLE);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_MANUF_CONFIRM_ISSUE:
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_MANUF_STATUS);
            delay_ms = 15U;
            break;

        case BQ76922_INIT_MANUF_CONFIRM_READ:
            status = BQ76922_ReadDirect(dev, BQ76922_CMD_RAM_DATA, buf, 2U);
            if (status == BQ76922_OK) {
                uint16_t manuf = (uint16_t)BQ76922_Le16(buf);

                if (BQ76922_ManufLooksStaleVcell(manuf)) {
                    status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_MANUF_STATUS);
                    if (status == BQ76922_OK) {
                        delay_ms = 20U;
                        dev->next_action_ms = HAL_GetTick() + delay_ms;
                        return false;
                    }
                    break;
                }
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
            break;

        case BQ76922_INIT_ALL_FETS_ON:
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_ALL_FETS_ON);
            dev->last_fet_cmd_ms = HAL_GetTick();
            /* Let PDSG soft-start PACK caps before we decide FETs failed. */
            delay_ms = 80U;
            break;

        case BQ76922_INIT_FET_SETTLE:
            status = BQ76922_ClearAlarms(dev);
            delay_ms = 40U;
            break;

        case BQ76922_INIT_FET_VERIFY:
            status = BQ76922_ReadDirect(dev, BQ76922_CMD_FET_STATUS, buf, 1U);
            if (status == BQ76922_OK) {
                uint8_t fet = buf[0];
                bool path_on = ((fet & BQ76922_FET_STATUS_DSG) != 0U) ||
                               ((fet & BQ76922_FET_STATUS_PDSG) != 0U);
                bool chg_on = ((fet & BQ76922_FET_STATUS_CHG) != 0U);
                bool false_cov;
                bool cfetoff_force;

                dev->snapshot.fet_status = fet;
                dev->snapshot.chg_fet_on = chg_on;
                dev->snapshot.dsg_fet_on =
                    ((fet & BQ76922_FET_STATUS_DSG) != 0U);
                dev->snapshot.fets_enabled = (chg_on || path_on);
                (void)BQ76922_ReadSafetyStatus(dev);
                false_cov =
                    ((dev->snapshot.safety_status_a & BQ76922_SAFETY_A_COV) != 0U) &&
                    (!dev->snapshot.sample_valid ||
                     (dev->snapshot.max_cell_mv < (int16_t)BMS_COV_THRESHOLD_MV));
                cfetoff_force = ((fet & BQ76922_FET_STATUS_DCHG_PIN) != 0U) &&
                                !chg_on;

                if (chg_on && path_on) {
                    dev->fet_verify_attempts = 0U;
                    dev->fet_recover_fails = 0U;
                    break;
                }

                /* SCD/inrush, floating CFETOFF, or transient COV — clear +
                 * ALL_FETS_ON; do not reboot. After CFETOFF Pin Config=0,
                 * CHG should come back once any false COV latch clears. */
                if (dev->fet_verify_attempts < BQ76922_FET_VERIFY_MAX) {
                    dev->fet_verify_attempts++;
                    (void)BQ76922_ClearAlarms(dev);
                    status = BQ76922_SendSubcommand(dev,
                                                    BQ76922_SUBCMD_ALL_FETS_ON);
                    if (status == BQ76922_OK) {
                        dev->last_fet_cmd_ms = HAL_GetTick();
                        /* Longer settle when COV/CFETOFF was blocking CHG. */
                        delay_ms = (false_cov || cfetoff_force) ? 200U : 120U;
                        dev->init_step = (uint8_t)BQ76922_INIT_FET_SETTLE;
                        dev->snapshot.init_step = dev->init_step;
                        dev->next_action_ms = HAL_GetTick() + delay_ms;
                        return false;
                    }
                    break;
                }
                /* Keep going READY even if FETs still off — runtime retry
                 * will keep clearing SCD/false-COV and re-issuing ALL_FETS_ON. */
                dev->fet_verify_attempts = 0U;
            }
            break;

        case BQ76922_INIT_DONE:
        default:
            dev->snapshot.configured = true;
            dev->snapshot.fault_flags &= (uint32_t)~BQ76922_FAULT_I2C;
            dev->snapshot.shutdown_request = false;
            dev->snapshot.state = BQ76922_STATE_READY;
            dev->snapshot.init_step = (uint8_t)BQ76922_INIT_DONE;
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

    /* Clean FET path first, then SHUTDOWN twice to bypass command delay
     * (TI BQ769x2: single write is ignored as accidental-shutdown guard). */
    (void)BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_ALL_FETS_OFF);
    HAL_Delay(2U);
    status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_SHUTDOWN);
    if (status != BQ76922_OK) {
        return status;
    }
    HAL_Delay(2U);
    status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_SHUTDOWN);
    /* Always invalidate session after the dual SHUTDOWN write. Leaving
     * init_step at DONE + configured=0 falsely re-marked READY without
     * FET_ENABLE — button wake then left MOSFETs off with no host cmd. */
    BQ76922_ForceAbsent(dev);
    return status;
#endif
}

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

static BQ76922_Status_t BQ76922_CallSubcommandU1(BQ76922_Device_t *dev,
                                                 uint16_t subcmd,
                                                 uint8_t *out)
{
    BQ76922_Status_t status;

    status = BQ76922_SendSubcommand(dev, subcmd);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WaitSubcommandDone(dev, subcmd);
    if (status != BQ76922_OK) {
        return status;
    }
    return BQ76922_ReadDirect(dev, BQ76922_CMD_RAM_DATA, out, 1U);
}

static BQ76922_Status_t BQ76922_ReadRamBlock(BQ76922_Device_t *dev,
                                             uint16_t address,
                                             uint8_t *data,
                                             uint8_t length)
{
    uint8_t addr[2];
    BQ76922_Status_t status;

    if ((data == NULL) || (length == 0U) || (length > 4U)) {
        return BQ76922_INVALID_ARG;
    }
    addr[0] = (uint8_t)(address & 0xFFU);
    addr[1] = (uint8_t)((address >> 8) & 0xFFU);
    status = BQ76922_WriteDirect(dev, BQ76922_CMD_SUBCMD_LOW, addr, 2U);
    if (status != BQ76922_OK) {
        return status;
    }
    HAL_Delay(2U);
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

static BQ76922_Status_t BQ76922_OtpWriteGolden(BQ76922_Device_t *dev)
{
    BQ76922_Status_t status;

    /* Minimal golden for autonomous TS2 wake — same values as CFGUPDATE path. */
    status = BQ76922_WriteRamU2(dev, BQ76922_RAM_VCELL_MODE, BMS_VCELL_MODE);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_ENABLED_PROT_A,
                                BMS_ENABLED_PROTECTIONS_A);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_ENABLED_PROT_B,
                                BMS_ENABLED_PROTECTIONS_B);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_CHG_FET_PROT_A,
                                BMS_CHG_FET_PROTECTIONS_A);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_DSG_FET_PROT_A,
                                BMS_DSG_FET_PROTECTIONS_A);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_CUV_THRESHOLD,
                                BMS_CUV_THRESHOLD_CODE);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_COV_THRESHOLD,
                                BMS_COV_THRESHOLD_CODE);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_OCC_THRESHOLD,
                                BMS_OCC_THRESHOLD_CODE);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_OCD1_THRESHOLD,
                                BMS_OCD1_THRESHOLD_CODE);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU2(dev, BQ76922_RAM_OCC_RECOVERY,
                                (uint16_t)(int16_t)BMS_OCC_RECOVERY_MA);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamF4(dev, BQ76922_RAM_CC_GAIN, BMS_CC_GAIN);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamF4(dev, BQ76922_RAM_CAPACITY_GAIN, BMS_CAPACITY_GAIN);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_CFETOFF_PIN_CONFIG,
                                BMS_CFETOFF_PIN_CONFIG);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_DFETOFF_PIN_CONFIG,
                                BMS_DFETOFF_PIN_CONFIG);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_FET_OPTIONS, BMS_FET_OPTIONS);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU2(dev, BQ76922_RAM_MFG_STATUS_INIT,
                                BMS_MFG_STATUS_INIT);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_AUTO_SHUTDOWN_TIME,
                                BMS_AUTO_SHUTDOWN_TIME);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_PDSG_TIMEOUT, BMS_PDSG_TIMEOUT);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_PDSG_STOP_DELTA,
                                BMS_PDSG_STOP_DELTA);
    if (status != BQ76922_OK) {
        return status;
    }
    status = BQ76922_WriteRamU1(dev, BQ76922_RAM_SCD_THRESHOLD, BMS_SCD_THRESHOLD);
    if (status != BQ76922_OK) {
        return status;
    }
    return BQ76922_WriteRamU2(dev, BQ76922_RAM_BODY_DIODE,
                               BMS_BODY_DIODE_THRESHOLD_MA);
}

BQ76922_Status_t BQ76922_OtpGetReport(BQ76922_Device_t *dev,
                                      BQ76922_OtpReport_t *out)
{
    BQ76922_Status_t status;
    uint8_t check = 0U;
    bool held = false;

#if (BMS_ENABLE == 0U)
    (void)dev;
    (void)out;
    return BQ76922_NOT_READY;
#else
    if ((dev == NULL) || (dev->hi2c == NULL) || (out == NULL)) {
        return BQ76922_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));
    out->session_already_burned = s_otp_burned_this_boot;

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

    (void)BQ76922_ReadRamU2(dev, BQ76922_RAM_MFG_STATUS_INIT,
                            &out->mfg_status_init);
    (void)BQ76922_ReadRamU2(dev, BQ76922_RAM_VCELL_MODE, &out->vcell_mode);
    (void)BQ76922_ReadRamU1(dev, BQ76922_RAM_FET_OPTIONS, &out->fet_options);

    /* OTP_WR_CHECK requires CONFIG_UPDATE + FULLACCESS. */
    if (!out->cfgupdate) {
        status = BQ76922_OtpEnterCfgupdate(dev);
        if (status != BQ76922_OK) {
            goto out_hold;
        }
        out->cfgupdate = true;
        status = BQ76922_ReadBatteryStatus(dev);
        if (status == BQ76922_OK) {
            out->battery_status = dev->snapshot.battery_status;
            out->fullaccess =
                ((dev->snapshot.battery_status & BQ76922_BATT_SEC_MASK) ==
                 BQ76922_BATT_SEC_FULLACCESS);
            out->otpb_blocked =
                ((dev->snapshot.battery_status & BQ76922_BATT_OTPB) != 0U);
        }
    }

    status = BQ76922_CallSubcommandU1(dev, BQ76922_SUBCMD_OTP_WR_CHECK, &check);
    if (status == BQ76922_OK) {
        out->wr_check = check;
    }

    (void)BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_EXIT_CFGUPDATE);
    HAL_Delay(10U);
    out->cfgupdate = false;

out_hold:
    if (held) {
        PowerManager_SetBmsBusHold(false);
    }
    return status;
#endif
}

BQ76922_Status_t BQ76922_OtpBurn(BQ76922_Device_t *dev,
                                 const char *confirm_token,
                                 uint8_t *wr_check,
                                 uint8_t *wr_result)
{
    BQ76922_Status_t status;
    uint8_t check = 0U;
    uint8_t result = 0U;
    bool held = false;

#if (BMS_ENABLE == 0U)
    (void)dev;
    (void)confirm_token;
    (void)wr_check;
    (void)wr_result;
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
        return BQ76922_NOT_READY;
    }

    PowerManager_SetBmsBusHold(true);
    held = true;

    status = BQ76922_OtpEnterCfgupdate(dev);
    if (status != BQ76922_OK) {
        goto out_hold;
    }

    status = BQ76922_OtpWriteGolden(dev);
    if (status != BQ76922_OK) {
        goto out_exit_cfg;
    }

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

    status = BQ76922_CallSubcommandU1(dev, BQ76922_SUBCMD_OTP_WR_CHECK, &check);
    if (wr_check != NULL) {
        *wr_check = check;
    }
    if ((status != BQ76922_OK) || (check != BQ76922_OTP_OK_CODE)) {
        if (status == BQ76922_OK) {
            status = BQ76922_NOT_READY;
        }
        goto out_exit_cfg;
    }

    status = BQ76922_CallSubcommandU1(dev, BQ76922_SUBCMD_OTP_WRITE, &result);
    if (wr_result != NULL) {
        *wr_result = result;
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
    (void)BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_EXIT_CFGUPDATE);
    HAL_Delay(20U);
    /* Reload FETs after CFGUPDATE exit — OTP does not replace that need this boot. */
    BQ76922_RequestReinit(dev);

out_hold:
    if (held) {
        PowerManager_SetBmsBusHold(false);
    }
    return status;
#endif
}

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
    if (dev->snapshot.vcell_mode_rb != BMS_VCELL_MODE) {
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
            /* Button/LD wake: AFE just returned — full FET bring-up, no host cmd. */
            dev->i2c_absent_streak = 0U;
            dev->boot_settle_ms = 0U;
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
    } else {
        bool hard_uv_ov;

        dev->snapshot.sample_valid = true;
        dev->snapshot.sample_tick_ms = now_ms;
        dev->scan_index = 0U;
        BQ76922_UpdateCellStats(dev);
        BQ76922_DecodeSafetyFaults(dev);
        BQ76922_UpdateWarnState(dev);

        /* Button / USB-C wake unlock + charger-unplug recovery:
         * clear recoverable SCD/OCC/false-COV, keep CHG+DSG both on.
         * Floating CFETOFF (fet bit4) used to hold CHG off forever — pin
         * config is now forced unused in CFGUPDATE; still retry ALL_FETS_ON
         * and re-run CFGUPDATE if CHG never returns. */
        hard_uv_ov = ((dev->snapshot.fault_flags &
                       (BQ76922_FAULT_CELL_OV | BQ76922_FAULT_CELL_UV)) != 0U);
        {
            bool dsg_path =
                dev->snapshot.dsg_fet_on ||
                ((dev->snapshot.fet_status & BQ76922_FET_STATUS_PDSG) != 0U);
            bool false_cov =
                ((dev->snapshot.safety_status_a & BQ76922_SAFETY_A_COV) != 0U) &&
                !hard_uv_ov;
            bool cfetoff_force =
                ((dev->snapshot.fet_status & BQ76922_FET_STATUS_DCHG_PIN) != 0U) &&
                !dev->snapshot.chg_fet_on;
            bool need_fet_cmd =
                (!dev->snapshot.chg_fet_on || !dsg_path) ||
                ((dev->snapshot.safety_status_a & BQ76922_SAFETY_A_CURRENT) != 0U) ||
                false_cov;

            if (need_fet_cmd &&
                ((int32_t)(now_ms - dev->last_fet_cmd_ms) >=
                 (int32_t)BQ76922_FET_RETRY_MS)) {
                if (hard_uv_ov && (dev->snapshot.vcell_mode_rb != BMS_VCELL_MODE)) {
                    BQ76922_RequestReinit(dev);
                    return;
                }
                if (!hard_uv_ov) {
                    bool manuf_unknown =
                        BQ76922_ManufLooksStaleVcell(dev->snapshot.manuf_status) ||
                        ((dev->snapshot.manuf_status & BQ76922_MANUF_FET_EN) == 0U);

                    if (manuf_unknown) {
                        /* Need a verified FET_ENABLE path — do not toggle blindly. */
                        BQ76922_RequestReinit(dev);
                        return;
                    }
                    /* Sticky CHG-off with DCHG_PIN or false COV → re-apply
                     * CFETOFF=unused + ALL_FETS_ON via full CFGUPDATE. */
                    if (!dev->snapshot.chg_fet_on &&
                        (cfetoff_force || false_cov) &&
                        (dev->fet_recover_fails >= BQ76922_FET_RECOVER_REINIT_MAX)) {
                        dev->fet_recover_fails = 0U;
                        BQ76922_RequestReinit(dev);
                        return;
                    }
                    status = BQ76922_ClearAlarms(dev);
                    if (status == BQ76922_BUSY) {
                        return;
                    }
                    if (status == BQ76922_OK) {
                        status = BQ76922_SendSubcommand(dev,
                                                        BQ76922_SUBCMD_ALL_FETS_ON);
                    }
                    if (status == BQ76922_BUSY) {
                        return;
                    }
                    if (status == BQ76922_OK) {
                        dev->last_fet_cmd_ms = now_ms;
                        if (!dev->snapshot.chg_fet_on || !dsg_path) {
                            if (dev->fet_recover_fails < 255U) {
                                dev->fet_recover_fails++;
                            }
                        } else {
                            dev->fet_recover_fails = 0U;
                        }
                        /* Recoverable current/COV latch must not keep FAULT. */
                        if ((dev->snapshot.fault_flags &
                             (BQ76922_FAULT_CELL_OV | BQ76922_FAULT_CELL_UV)) == 0U) {
                            dev->snapshot.shutdown_request = false;
                            if (dev->snapshot.state == BQ76922_STATE_FAULT) {
                                dev->snapshot.state = BQ76922_STATE_READY;
                            }
                        }
                    }
                }
            } else if (dev->snapshot.chg_fet_on && dsg_path &&
                       ((dev->snapshot.safety_status_a &
                         (BQ76922_SAFETY_A_CURRENT | BQ76922_SAFETY_A_COV)) == 0U)) {
                dev->fet_recover_fails = 0U;
            }
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
