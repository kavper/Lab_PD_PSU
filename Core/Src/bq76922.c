#include "bq76922.h"

#include <string.h>

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

    /* Direct I2C4 access. Do not call this while PowerManager owns the bus. */
    status = HAL_I2C_IsDeviceReady(dev->hi2c,
                                   (uint16_t)(dev->address_7bit << 1),
                                   2U,
                                   20U);
    dev->probed = true;
    dev->present = (status == HAL_OK);
    return (status == HAL_OK) ? BQ76922_OK : BQ76922_NOT_READY;
}

bool BQ76922_IsPresent(const BQ76922_Device_t *dev)
{
    return (dev != NULL) && dev->present;
}
