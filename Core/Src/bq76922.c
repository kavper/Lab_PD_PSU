#include "bq76922.h"

#include "board_mx.h"
#include "power_manager.h"

#include <string.h>

#define BQ76922_CMD_SAFETY_STATUS_A      0x03U
#define BQ76922_CMD_CELL1_V              0x14U
#define BQ76922_CMD_PACK_V               0x28U
#define BQ76922_CMD_CC2                  0x3AU
#define BQ76922_CMD_ALARM_STATUS         0x62U
#define BQ76922_I2C_TIMEOUT_MS           8U
#define BQ76922_SCAN_PERIOD_MS           50U
#define BQ76922_PROBE_RETRY_MS           500U

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

static int16_t BQ76922_Le16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

void BQ76922_Bind(BQ76922_Device_t *dev, I2C_HandleTypeDef *hi2c)
{
    if (dev == NULL) {
        return;
    }

    memset(dev, 0, sizeof(*dev));
    dev->hi2c = hi2c;
    dev->address_7bit = BQ76922_I2C_ADDR_7BIT;
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
    return (status == HAL_OK) ? BQ76922_OK : BQ76922_NOT_READY;
}

bool BQ76922_IsPresent(const BQ76922_Device_t *dev)
{
    return (dev != NULL) && dev->snapshot.present;
}

void BQ76922_AlertFromIsr(BQ76922_Device_t *dev)
{
    if (dev == NULL) {
        return;
    }
    dev->snapshot.alert_latched = true;
    dev->snapshot.alert_count++;
}

void BQ76922_Task(BQ76922_Device_t *dev, uint32_t now_ms)
{
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
        dev->next_action_ms = now_ms + BQ76922_PROBE_RETRY_MS;
        return;
    }

    if (dev->snapshot.alert_latched) {
        status = BQ76922_ReadDirect(dev, BQ76922_CMD_ALARM_STATUS, buf, 2U);
        if (status == BQ76922_OK) {
            dev->snapshot.alarm_status = (uint16_t)BQ76922_Le16(buf);
            (void)BQ76922_ReadDirect(dev, BQ76922_CMD_SAFETY_STATUS_A,
                                     &dev->snapshot.safety_status_a, 1U);
            /* W1C: write back bits that were set. */
            (void)HAL_I2C_Mem_Write(dev->hi2c,
                                    (uint16_t)(dev->address_7bit << 1),
                                    BQ76922_CMD_ALARM_STATUS,
                                    I2C_MEMADD_SIZE_8BIT,
                                    buf,
                                    2U,
                                    BQ76922_I2C_TIMEOUT_MS);
            dev->snapshot.alert_latched = false;
        } else if (status == BQ76922_BUSY) {
            return;
        }
        dev->next_action_ms = now_ms + BQ76922_SCAN_PERIOD_MS;
        return;
    }

    if (dev->scan_index < BQ76922_CELL_COUNT) {
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
    } else if (dev->scan_index == BQ76922_CELL_COUNT) {
        status = BQ76922_ReadDirect(dev, BQ76922_CMD_PACK_V, buf, 2U);
        if (status == BQ76922_OK) {
            dev->snapshot.pack_mv = BQ76922_Le16(buf);
            dev->scan_index++;
        } else if (status == BQ76922_BUSY) {
            return;
        }
    } else {
        status = BQ76922_ReadDirect(dev, BQ76922_CMD_CC2, buf, 2U);
        if (status == BQ76922_OK) {
            dev->snapshot.cc2_ma = BQ76922_Le16(buf);
            dev->snapshot.sample_valid = true;
            dev->snapshot.sample_tick_ms = now_ms;
            dev->scan_index = 0U;
        } else if (status == BQ76922_BUSY) {
            return;
        } else {
            dev->scan_index = 0U;
        }
    }

    dev->next_action_ms = now_ms + BQ76922_SCAN_PERIOD_MS;
}

void BQ76922_GetSnapshot(const BQ76922_Device_t *dev, BQ76922_Snapshot_t *out)
{
    if ((dev == NULL) || (out == NULL)) {
        return;
    }
    *out = dev->snapshot;
}
