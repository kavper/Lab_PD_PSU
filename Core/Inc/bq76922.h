#ifndef BQ76922_H
#define BQ76922_H

#include "bms_board.h"
#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

#define BQ76922_I2C_ADDR_7BIT            0x08U
#define BQ76922_CELL_COUNT               BMS_AFE_CELL_CHANNELS

typedef enum {
    BQ76922_OK = 0,
    BQ76922_NOT_READY,
    BQ76922_I2C_ERROR,
    BQ76922_BUSY,
    BQ76922_INVALID_ARG
} BQ76922_Status_t;

typedef enum {
    BQ76922_STATE_ABSENT = 0,
    BQ76922_STATE_INIT,
    BQ76922_STATE_READY,
    BQ76922_STATE_DISCHARGING,
    BQ76922_STATE_CHARGING,
    BQ76922_STATE_WARN,
    BQ76922_STATE_FAULT,
} BQ76922_State_t;

typedef enum {
    BQ76922_FAULT_NONE              = 0U,
    BQ76922_FAULT_CELL_OV           = (1U << 0),
    BQ76922_FAULT_CELL_UV           = (1U << 1),
    BQ76922_FAULT_PACK_OV           = (1U << 2),
    BQ76922_FAULT_PACK_UV           = (1U << 3),
    BQ76922_FAULT_IMBALANCE         = (1U << 4),
    BQ76922_FAULT_SAFETY            = (1U << 5),
    BQ76922_FAULT_ALERT             = (1U << 6),
    BQ76922_FAULT_I2C               = (1U << 7),
} BQ76922_FaultFlags_t;

typedef struct {
    bool probed;
    bool present;
    bool configured;
    bool fets_enabled;
    bool sample_valid;
    bool alert_latched;
    bool alert_pin;
    bool shutdown_request;
    BQ76922_State_t state;
    uint16_t alarm_status;
    uint8_t safety_status_a;
    uint8_t safety_status_b;
    uint8_t safety_status_c;
    uint16_t device_number;
    int16_t cell_mv[BQ76922_CELL_COUNT];
    int16_t min_cell_mv;
    int16_t max_cell_mv;
    int16_t pack_mv;
    int16_t stack_mv;
    int16_t cell_sum_mv;
    int16_t cc2_ma;
    uint32_t fault_flags;
    uint32_t sample_tick_ms;
    uint32_t alert_count;
    uint32_t i2c_error_count;
    uint8_t fet_status;
    uint16_t manuf_status;
    uint16_t battery_status;
    /* Wake debug on TB: init_step / vcell_mode_rb / manuf / fet / cfg_fail */
    uint8_t init_step;
    uint16_t vcell_mode_rb;
    uint16_t cfg_fail_count;
    bool chg_fet_on;
    bool dsg_fet_on;
} BQ76922_Snapshot_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address_7bit;
    BQ76922_Snapshot_t snapshot;
    uint8_t scan_index;
    uint8_t init_step;
    uint8_t cfg_poll_tries;
    uint8_t cfg_enter_tries;
    uint8_t fet_en_attempts;
    uint8_t fet_verify_attempts;
    uint8_t fet_recover_fails;
    uint32_t next_action_ms;
    uint32_t last_fet_cmd_ms;
    uint32_t bus_hold_since_ms;
    uint32_t boot_settle_ms;
} BQ76922_Device_t;

void BQ76922_Bind(BQ76922_Device_t *dev, I2C_HandleTypeDef *hi2c);
BQ76922_Status_t BQ76922_Probe(BQ76922_Device_t *dev);
bool BQ76922_IsPresent(const BQ76922_Device_t *dev);
bool BQ76922_IsEnabled(void);
bool BQ76922_IsShutdownRequested(const BQ76922_Device_t *dev);
BQ76922_State_t BQ76922_GetState(const BQ76922_Device_t *dev);

void BQ76922_Task(BQ76922_Device_t *dev, uint32_t now_ms);
void BQ76922_AlertFromIsr(BQ76922_Device_t *dev);
void BQ76922_GetSnapshot(const BQ76922_Device_t *dev, BQ76922_Snapshot_t *out);
void BQ76922_ClearShutdownRequest(BQ76922_Device_t *dev);
/* True when RAM 4S config is applied (safe to skip host CFGUPDATE). */
bool BQ76922_IsConfiguredHealthy(const BQ76922_Device_t *dev);
/* Re-run CONFIG_UPDATE + 4S VCell Mode + FET_ENABLE + ALL_FETS_ON (no OTP).
 * While FETs are already on this can drop PACK, hold I2C4 (starve BQ/TPS), and
 * ALL_FETS_ON inrush may brown-out the MCU (PIN/POR). Prefer host BMS soft path. */
void BQ76922_RequestReinit(BQ76922_Device_t *dev);
/* Enter BQ769x2 SHUTDOWN: ALL_FETS_OFF then SHUTDOWN() twice (TI).
 * Keep balance harness connected; do not hold the TS2 button (soft-SHUTDOWN).
 * Wake later: TS2→VSS or charger on LD. Returns I2C status of last write. */
BQ76922_Status_t BQ76922_EnterShutdown(BQ76922_Device_t *dev);

#endif /* BQ76922_H */
