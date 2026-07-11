#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include "bq25731.h"
#include "main.h"
#include "tps25751.h"

#include <stdint.h>

#ifndef POWER_MANAGER_PD_CYCLE_TEST
#define POWER_MANAGER_PD_CYCLE_TEST 0U
#endif

#ifndef POWER_MANAGER_PD_POLICY_DEBUG
#define POWER_MANAGER_PD_POLICY_DEBUG 0U
#endif
#ifndef POWER_MANAGER_PD_REQUEST_MAX_POWER
#define POWER_MANAGER_PD_REQUEST_MAX_POWER 1U
#endif

typedef enum {
    POWER_MANAGER_INIT = 0,
    POWER_MANAGER_TPS_WAIT_APP,
    POWER_MANAGER_TPS_READY,
    POWER_MANAGER_BQ_PROBE,
    POWER_MANAGER_BQ_SAFE_START,
    POWER_MANAGER_SAFE_MONITORING,
    POWER_MANAGER_BQ_MONITOR = POWER_MANAGER_SAFE_MONITORING,
    POWER_MANAGER_DEGRADED_BQ,
    POWER_MANAGER_FAULT
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

typedef struct {
    PowerManager_State_t state;
    TPS25751_Status_t tps_status;
    BQ25731_Status_t bq_status;
    TPS25751_Telemetry_t tps_telemetry;
    BQ25731_Telemetry_t bq_telemetry;
    BQ25731_MonitorSnapshot_t bq_monitor;
    BQ25731_SafeStartupResult_t bq_safe_start;
    PowerManager_PdSnapshot_t pd_snapshot;
    uint32_t last_error_reg;
    uint32_t last_error_code;
} PowerManager_Status_t;

void PowerManager_Init(I2C_HandleTypeDef *hi2c);
void PowerManager_Task(void);
PowerManager_State_t PowerManager_GetState(void);
BQ25731_Status_t PowerManager_GetBqStatus(void);
void PowerManager_GetStatus(PowerManager_Status_t *status);
bool PowerManager_GetPdSnapshot(PowerManager_PdSnapshot_t *out);
bool PowerManager_IsPdCycleTestEnabled(void);
void PowerManager_LogPdSinkPolicy(void);

#endif
