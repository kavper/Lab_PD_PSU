#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include "bq25731.h"
#include "main.h"
#include "tps25751.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    POWER_MANAGER_USER_AUTO = 0,
    POWER_MANAGER_USER_SINK_ONLY,
    POWER_MANAGER_USER_SOURCE_ONLY,
    POWER_MANAGER_USER_OFF
} PowerManager_UserMode_t;

typedef enum {
    POWER_MANAGER_INIT = 0,
    POWER_MANAGER_TPS_WAIT_APP,
    POWER_MANAGER_TPS_READY,
    POWER_MANAGER_BQ_PROBE,
    POWER_MANAGER_BQ_ADC_SETUP,
    POWER_MANAGER_RUN,
    POWER_MANAGER_DEGRADED,
    POWER_MANAGER_FAULT,

    /* Read-only compatibility names used by psu_gui_api.c. */
    POWER_MANAGER_BQ_MONITOR = POWER_MANAGER_RUN,
    POWER_MANAGER_SAFE_MONITORING = POWER_MANAGER_RUN,
    POWER_MANAGER_DEGRADED_BQ = POWER_MANAGER_DEGRADED,
    POWER_MANAGER_BQ_SAFE_START = POWER_MANAGER_BQ_ADC_SETUP
} PowerManager_State_t;

typedef enum {
    POWER_MANAGER_PD_ROLE_UNKNOWN = 0,
    POWER_MANAGER_PD_ROLE_SINK,
    POWER_MANAGER_PD_ROLE_SOURCE
} PowerManager_PdPowerRole_t;

typedef enum {
    POWER_MANAGER_POWER_NO_INPUT = 0,
    POWER_MANAGER_POWER_LOW,
    POWER_MANAGER_POWER_MEDIUM,
    POWER_MANAGER_POWER_HIGH,
    POWER_MANAGER_POWER_FULL
} PowerManager_PowerClass_t;

typedef struct {
    bool attached;
    PowerManager_PdPowerRole_t power_role;
    bool data_role_dfp;
    uint64_t active_pdo_raw;
    uint32_t active_rdo_raw;
    uint32_t contract_voltage_mv;
    uint32_t contract_current_ma;
    uint32_t contract_power_mw;
    PowerManager_PowerClass_t power_class;
} PowerManager_PdSnapshot_t;

typedef enum {
    POWER_MANAGER_ERROR_NONE = 0,
    POWER_MANAGER_ERROR_TPS,
    POWER_MANAGER_ERROR_BQ,
    POWER_MANAGER_ERROR_PROTOCOL
} PowerManager_ErrorSource_t;

typedef struct {
    PowerManager_ErrorSource_t source;
    uint32_t code;
    uint8_t reg;
    uint32_t tick_ms;
} PowerManager_Error_t;

typedef struct {
    PowerManager_State_t state;
    PowerManager_UserMode_t requested_mode;
    PowerManager_UserMode_t applied_mode;
    bool applied_mode_valid;
    bool otg_pin_high;
    bool source_fault_latched;
    TPS25751_Status_t tps_status;
    BQ25731_Status_t bq_status;
    TPS25751_Telemetry_t tps;
    BQ25731_Telemetry_t bq;
    PowerManager_PdSnapshot_t pd_snapshot;
    PowerManager_Error_t last_error;
} PowerManager_Status_t;

void PowerManager_Init(I2C_HandleTypeDef *hi2c);
void PowerManager_Task(void);
void PowerManager_GetStatus(PowerManager_Status_t *out);
bool PowerManager_SetUserMode(PowerManager_UserMode_t mode);

/* Compatibility wrappers required by the unchanged GUI adapter. */
PowerManager_State_t PowerManager_GetState(void);
BQ25731_Status_t PowerManager_GetBqStatus(void);
bool PowerManager_GetPdSnapshot(PowerManager_PdSnapshot_t *out);
bool PowerManager_IsPdCycleTestEnabled(void);

#endif
