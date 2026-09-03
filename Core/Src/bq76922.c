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
#define BQ76922_CMD_CELL1_V              0x14U
#define BQ76922_CMD_STACK_V              0x34U
#define BQ76922_CMD_PACK_V               0x36U
#define BQ76922_CMD_CC2                  0x3AU
#define BQ76922_CMD_ALARM_STATUS         0x62U
#define BQ76922_CMD_FET_STATUS           0x7FU

#define BQ76922_SUBCMD_SET_CFGUPDATE     0x0090U
#define BQ76922_SUBCMD_EXIT_CFGUPDATE    0x0092U
#define BQ76922_SUBCMD_FET_ENABLE        0x0022U
#define BQ76922_SUBCMD_ALL_FETS_ON       0x0096U
#define BQ76922_SUBCMD_SLEEP_DISABLE     0x009AU
#define BQ76922_SUBCMD_MANUF_STATUS      0x0057U

#define BQ76922_RAM_VCELL_MODE           0x9304U
#define BQ76922_RAM_ENABLED_PROT_A       0x9261U
#define BQ76922_RAM_ENABLED_PROT_B       0x9262U
#define BQ76922_RAM_CUV_THRESHOLD        0x9275U
#define BQ76922_RAM_COV_THRESHOLD        0x9278U

#define BQ76922_I2C_TIMEOUT_MS           20U
#define BQ76922_SCAN_PERIOD_MS           50U
#define BQ76922_PROBE_RETRY_MS           200U
#define BQ76922_INIT_STEP_DELAY_MS       10U
#define BQ76922_POST_CFG_DELAY_MS        120U
#define BQ76922_FET_RETRY_MS             250U
#define BQ76922_CC2_CHARGE_MA            50
#define BQ76922_CC2_DISCHARGE_MA         (-50)
#define BQ76922_MANUF_FET_EN             (1U << 4)
/* FET Status: CHG=bit0, PCHG=bit1, DSG=bit2, PDSG=bit3 */
#define BQ76922_FET_STATUS_CHG           0x01U
#define BQ76922_FET_STATUS_DSG           0x04U
#define BQ76922_FET_STATUS_CHG_DSG       (BQ76922_FET_STATUS_CHG | BQ76922_FET_STATUS_DSG)
#define BQ76922_SAFETY_A_CUV             0x04U
#define BQ76922_SAFETY_A_COV             0x08U
/* Default DA Config USER_VOLTS_CV=1 → Stack/PACK in centivolts (10 mV). */
#define BQ76922_USERV_TO_MV(raw)         ((int16_t)((int16_t)(raw) * 10))

typedef enum {
    BQ76922_INIT_ENTER_CFG = 0,
    BQ76922_INIT_VCELL_MODE,
    BQ76922_INIT_PROT_A,
    BQ76922_INIT_PROT_B,
    BQ76922_INIT_CUV_THRESH,
    BQ76922_INIT_COV_THRESH,
    BQ76922_INIT_EXIT_CFG,
    BQ76922_INIT_READ_VCELL,
    BQ76922_INIT_SLEEP_DISABLE,
    BQ76922_INIT_CLR_ALARM,
    BQ76922_INIT_MANUF_ISSUE,
    BQ76922_INIT_MANUF_READ,
    BQ76922_INIT_FET_ENABLE,
    BQ76922_INIT_ALL_FETS_ON,
    BQ76922_INIT_DONE,
} BQ76922_InitStep_t;

static bool BQ76922_BusIdle(const BQ76922_Device_t *dev)
{
    if ((dev == NULL) || (dev->hi2c == NULL)) {
        return false;
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
    uint8_t addr_buf[2];
    uint8_t whole_block[6];
    uint8_t checksum_block[2];
    uint16_t sum = 0U;
    uint8_t i;

    if ((dev == NULL) || (data == NULL) || (length == 0U) || (length > 4U)) {
        return BQ76922_INVALID_ARG;
    }

    addr_buf[0] = (uint8_t)(address & 0xFFU);
    addr_buf[1] = (uint8_t)((address >> 8) & 0xFFU);

    if (BQ76922_WriteDirect(dev, BQ76922_CMD_SUBCMD_LOW, addr_buf, 2U) != BQ76922_OK) {
        return BQ76922_I2C_ERROR;
    }
    if (BQ76922_WriteDirect(dev, BQ76922_CMD_RAM_DATA, data, length) != BQ76922_OK) {
        return BQ76922_I2C_ERROR;
    }

    whole_block[0] = addr_buf[0];
    whole_block[1] = addr_buf[1];
    for (i = 0U; i < length; i++) {
        whole_block[2U + i] = data[i];
    }

    for (i = 0U; i < (uint8_t)(length + 2U); i++) {
        sum = (uint16_t)(sum + whole_block[i]);
    }

    checksum_block[0] = (uint8_t)(~sum & 0xFFU);
    checksum_block[1] = (uint8_t)(length + 2U);
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

    if ((sa & 0x08U) != 0U) {
        flags |= BQ76922_FAULT_CELL_OV;
    }
    if ((sa & 0x04U) != 0U) {
        flags |= BQ76922_FAULT_CELL_UV;
    }
    if ((sb != 0U) || (sc != 0U)) {
        flags |= BQ76922_FAULT_SAFETY;
    }
    if (dev->snapshot.alert_latched || dev->snapshot.alert_pin) {
        flags |= BQ76922_FAULT_ALERT;
    }

    dev->snapshot.fault_flags = flags;

    if ((flags & (BQ76922_FAULT_CELL_OV |
                  BQ76922_FAULT_CELL_UV |
                  BQ76922_FAULT_SAFETY)) != 0U) {
        /* Do not kill the rail until 4S config stuck and FETs were attempted.
         * Blank OTP boots as "all cells" — a skipped VC4 looks like CUV and
         * would otherwise latch FAULT_BMS before VCell Mode is applied. */
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

static BQ76922_Status_t BQ76922_HandleAlert(BQ76922_Device_t *dev)
{
    uint8_t buf[2];
    BQ76922_Status_t status;

    status = BQ76922_ReadDirect(dev, BQ76922_CMD_ALARM_STATUS, buf, 2U);
    if (status != BQ76922_OK) {
        return status;
    }

    dev->snapshot.alarm_status = (uint16_t)BQ76922_Le16(buf);
    (void)BQ76922_ReadSafetyStatus(dev);

    (void)BQ76922_WriteDirect(dev, BQ76922_CMD_ALARM_STATUS, buf, 2U);
    dev->snapshot.alert_latched = false;
    BQ76922_DecodeSafetyFaults(dev);
    return BQ76922_OK;
}

static bool BQ76922_RunInitStep(BQ76922_Device_t *dev)
{
    BQ76922_Status_t status = BQ76922_OK;
    uint32_t delay_ms = BQ76922_INIT_STEP_DELAY_MS;
    uint8_t buf[2];
    uint16_t vcell_mode = 0U;

    switch ((BQ76922_InitStep_t)dev->init_step) {
        case BQ76922_INIT_ENTER_CFG:
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_SET_CFGUPDATE);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_VCELL_MODE:
            status = BQ76922_WriteRamU2(dev, BQ76922_RAM_VCELL_MODE, BMS_VCELL_MODE);
            break;

        case BQ76922_INIT_PROT_A:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_ENABLED_PROT_A,
                                        BMS_ENABLED_PROTECTIONS_A);
            break;

        case BQ76922_INIT_PROT_B:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_ENABLED_PROT_B,
                                        BMS_ENABLED_PROTECTIONS_B);
            break;

        case BQ76922_INIT_CUV_THRESH:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_CUV_THRESHOLD,
                                        BMS_CUV_THRESHOLD_CODE);
            break;

        case BQ76922_INIT_COV_THRESH:
            status = BQ76922_WriteRamU1(dev, BQ76922_RAM_COV_THRESHOLD,
                                        BMS_COV_THRESHOLD_CODE);
            break;

        case BQ76922_INIT_EXIT_CFG:
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_EXIT_CFGUPDATE);
            delay_ms = BQ76922_POST_CFG_DELAY_MS;
            break;

        case BQ76922_INIT_READ_VCELL:
            /* Issue address then read 0x40 on the next step delay. */
            buf[0] = (uint8_t)(BQ76922_RAM_VCELL_MODE & 0xFFU);
            buf[1] = (uint8_t)((BQ76922_RAM_VCELL_MODE >> 8) & 0xFFU);
            status = BQ76922_WriteDirect(dev, BQ76922_CMD_SUBCMD_LOW, buf, 2U);
            delay_ms = 15U;
            break;

        case BQ76922_INIT_SLEEP_DISABLE:
            /* Completes VCELL readback first (data still in 0x40), then sleep off. */
            status = BQ76922_ReadDirect(dev, BQ76922_CMD_RAM_DATA, buf, 2U);
            if (status == BQ76922_OK) {
                vcell_mode = (uint16_t)BQ76922_Le16(buf);
                if (vcell_mode != BMS_VCELL_MODE) {
                    /* Config did not stick — retry whole sequence (no OTP needed). */
                    dev->snapshot.fault_flags |= BQ76922_FAULT_I2C;
                    dev->init_step = BQ76922_INIT_ENTER_CFG;
                    dev->next_action_ms = HAL_GetTick() + BQ76922_PROBE_RETRY_MS;
                    return false;
                }
            } else if (status == BQ76922_BUSY) {
                return false;
            }
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_SLEEP_DISABLE);
            break;

        case BQ76922_INIT_CLR_ALARM:
            status = BQ76922_ReadDirect(dev, BQ76922_CMD_ALARM_STATUS, buf, 2U);
            if (status == BQ76922_OK) {
                (void)BQ76922_WriteDirect(dev, BQ76922_CMD_ALARM_STATUS, buf, 2U);
                (void)BQ76922_ReadSafetyStatus(dev);
                dev->snapshot.alert_latched = false;
                /* Clear stale CUV from blank-OTP 5S view of unused VC4. */
                if ((dev->snapshot.safety_status_a &
                     (BQ76922_SAFETY_A_CUV | BQ76922_SAFETY_A_COV)) != 0U) {
                    /* Re-read after sleep disable; FETs still attempted below. */
                }
            }
            break;

        case BQ76922_INIT_MANUF_ISSUE:
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_MANUF_STATUS);
            delay_ms = 15U;
            break;

        case BQ76922_INIT_MANUF_READ:
            status = BQ76922_ReadDirect(dev, BQ76922_CMD_RAM_DATA, buf, 2U);
            if (status == BQ76922_OK) {
                dev->snapshot.manuf_status = (uint16_t)BQ76922_Le16(buf);
                /* FET_ENABLE is a toggle — do not send it if FET_EN is already 1. */
                if ((dev->snapshot.manuf_status & BQ76922_MANUF_FET_EN) != 0U) {
                    dev->init_step = (uint8_t)BQ76922_INIT_ALL_FETS_ON;
                    dev->next_action_ms = HAL_GetTick() + delay_ms;
                    return false;
                }
            }
            break;

        case BQ76922_INIT_FET_ENABLE:
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_FET_ENABLE);
            delay_ms = 20U;
            break;

        case BQ76922_INIT_ALL_FETS_ON:
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_ALL_FETS_ON);
            dev->last_fet_cmd_ms = HAL_GetTick();
            delay_ms = 30U;
            break;

        case BQ76922_INIT_DONE:
        default:
            dev->snapshot.configured = true;
            dev->snapshot.fault_flags &= (uint32_t)~BQ76922_FAULT_I2C;
            dev->snapshot.shutdown_request = false;
            if (dev->snapshot.state == BQ76922_STATE_FAULT) {
                dev->snapshot.state = BQ76922_STATE_READY;
            } else {
                dev->snapshot.state = BQ76922_STATE_READY;
            }
            return true;
    }

    if (status == BQ76922_BUSY) {
        return false;
    }
    if (status != BQ76922_OK) {
        dev->snapshot.fault_flags |= BQ76922_FAULT_I2C;
        dev->init_step = BQ76922_INIT_ENTER_CFG;
        dev->next_action_ms = HAL_GetTick() + BQ76922_PROBE_RETRY_MS;
        return false;
    }

    /* CLR_ALARM may return BUSY-equivalent via read path already handled. */
    if ((BQ76922_InitStep_t)dev->init_step == BQ76922_INIT_CLR_ALARM &&
        status == BQ76922_OK) {
        /* fall through */
    }

    (void)vcell_mode;
    dev->init_step++;
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

void BQ76922_RequestReinit(BQ76922_Device_t *dev)
{
    if (dev == NULL) {
        return;
    }

    dev->snapshot.configured = false;
    dev->snapshot.fets_enabled = false;
    dev->snapshot.chg_fet_on = false;
    dev->snapshot.dsg_fet_on = false;
    dev->snapshot.sample_valid = false;
    dev->snapshot.shutdown_request = false;
    dev->snapshot.state = BQ76922_STATE_INIT;
    dev->init_step = BQ76922_INIT_ENTER_CFG;
    dev->scan_index = 0U;
    dev->next_action_ms = HAL_GetTick();
}

void BQ76922_Task(BQ76922_Device_t *dev, uint32_t now_ms)
{
#if (BMS_ENABLE == 0U)
    (void)dev;
    (void)now_ms;
    return;
#else
    uint8_t buf[2];
    BQ76922_Status_t status;

    if ((dev == NULL) || (dev->hi2c == NULL)) {
        return;
    }

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
            dev->init_step = BQ76922_INIT_ENTER_CFG;
            dev->next_action_ms = now_ms + BQ76922_INIT_STEP_DELAY_MS;
        } else {
            dev->next_action_ms = now_ms + BQ76922_PROBE_RETRY_MS;
        }
        return;
    }

    if (!dev->snapshot.configured) {
        (void)BQ76922_RunInitStep(dev);
        return;
    }

    if (dev->snapshot.alert_latched) {
        status = BQ76922_HandleAlert(dev);
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
            if (status == BQ76922_OK) {
                dev->snapshot.cell_mv[dev->scan_index] = BQ76922_Le16(buf);
                dev->scan_index++;
            } else if (status == BQ76922_BUSY) {
                return;
            }
        }
    } else if (dev->scan_index == BQ76922_CELL_COUNT) {
        status = BQ76922_ReadDirect(dev, BQ76922_CMD_PACK_V, buf, 2U);
        if (status == BQ76922_OK) {
            dev->snapshot.pack_mv = BQ76922_USERV_TO_MV(BQ76922_Le16(buf));
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else if (dev->scan_index == (BQ76922_CELL_COUNT + 1U)) {
        status = BQ76922_ReadDirect(dev, BQ76922_CMD_STACK_V, buf, 2U);
        if (status == BQ76922_OK) {
            dev->snapshot.stack_mv = BQ76922_USERV_TO_MV(BQ76922_Le16(buf));
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else if (dev->scan_index == (BQ76922_CELL_COUNT + 2U)) {
        status = BQ76922_ReadDirect(dev, BQ76922_CMD_CC2, buf, 2U);
        if (status == BQ76922_OK) {
            dev->snapshot.cc2_ma = BQ76922_Le16(buf);
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else if (dev->scan_index == (BQ76922_CELL_COUNT + 3U)) {
        status = BQ76922_ReadDirect(dev, BQ76922_CMD_FET_STATUS, buf, 1U);
        if (status == BQ76922_OK) {
            dev->snapshot.fet_status = buf[0];
            dev->snapshot.chg_fet_on = ((buf[0] & BQ76922_FET_STATUS_CHG) != 0U);
            dev->snapshot.dsg_fet_on = ((buf[0] & BQ76922_FET_STATUS_DSG) != 0U);
            dev->snapshot.fets_enabled =
                (dev->snapshot.chg_fet_on || dev->snapshot.dsg_fet_on);
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else if (dev->scan_index == (BQ76922_CELL_COUNT + 4U)) {
        status = BQ76922_ReadSafetyStatus(dev);
        if (status == BQ76922_OK) {
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
        BQ76922_UpdateWarnState(dev);

        /* Button / USB-C wake: AFE is up but CHG/DSG stay off until ALL_FETS_ON
         * after 4S VCell Mode. Retry while CUV/COV are clear (unused VC4 used
         * to latch CUV and block the pack path). */
        hard_uv_ov = ((dev->snapshot.safety_status_a &
                       (BQ76922_SAFETY_A_CUV | BQ76922_SAFETY_A_COV)) != 0U);
        if (!dev->snapshot.fets_enabled && !hard_uv_ov &&
            ((int32_t)(now_ms - dev->last_fet_cmd_ms) >= (int32_t)BQ76922_FET_RETRY_MS)) {
            status = BQ76922_SendSubcommand(dev, BQ76922_SUBCMD_ALL_FETS_ON);
            if (status == BQ76922_BUSY) {
                return;
            }
            if (status == BQ76922_OK) {
                dev->last_fet_cmd_ms = now_ms;
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
