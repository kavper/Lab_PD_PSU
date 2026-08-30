#ifndef BQ76922_H
#define BQ76922_H

#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/* BQ76922 on I2C_USBPD together with TPS25751. Schematic note: 0x08. */
#define BQ76922_I2C_ADDR_7BIT            0x08U
#define BQ76922_CELL_COUNT               5U

typedef enum {
    BQ76922_OK = 0,
    BQ76922_NOT_READY,
    BQ76922_I2C_ERROR,
    BQ76922_BUSY,
    BQ76922_INVALID_ARG
} BQ76922_Status_t;

typedef struct {
    bool probed;
    bool present;
    bool sample_valid;
    bool alert_latched;
    bool alert_pin;
    uint16_t alarm_status;
    uint8_t safety_status_a;
    int16_t cell_mv[BQ76922_CELL_COUNT];
    int16_t pack_mv;
    int16_t cc2_ma;
    uint32_t sample_tick_ms;
    uint32_t alert_count;
    uint32_t i2c_error_count;
} BQ76922_Snapshot_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address_7bit;
    BQ76922_Snapshot_t snapshot;
    uint8_t scan_index;
    uint32_t next_action_ms;
} BQ76922_Device_t;

void BQ76922_Bind(BQ76922_Device_t *dev, I2C_HandleTypeDef *hi2c);
BQ76922_Status_t BQ76922_Probe(BQ76922_Device_t *dev);
bool BQ76922_IsPresent(const BQ76922_Device_t *dev);

/* One short I2C access if the shared bus is idle. Safe to call every App_Run. */
void BQ76922_Task(BQ76922_Device_t *dev, uint32_t now_ms);
void BQ76922_AlertFromIsr(BQ76922_Device_t *dev);
void BQ76922_GetSnapshot(const BQ76922_Device_t *dev, BQ76922_Snapshot_t *out);

#endif /* BQ76922_H */
