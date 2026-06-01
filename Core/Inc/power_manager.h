#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include "bq25731.h"
#include "main.h"
#include "tps25751.h"

#include <stdint.h>

typedef enum {
    POWER_MANAGER_INIT = 0,
    POWER_MANAGER_TPS_WAIT_APP,
    POWER_MANAGER_TPS_READY,
    POWER_MANAGER_BQ_PROBE,
    POWER_MANAGER_BQ_MONITOR,
    POWER_MANAGER_ERROR
} PowerManager_State_t;

typedef struct {
    PowerManager_State_t state;
    TPS25751_Status_t tps_status;
    BQ25731_Status_t bq_status;
    TPS25751_Telemetry_t tps_telemetry;
    BQ25731_Telemetry_t bq_telemetry;
    uint32_t last_error_reg;
    uint32_t last_error_code;
} PowerManager_Status_t;

void PowerManager_Init(I2C_HandleTypeDef *hi2c);
void PowerManager_Task(void);
PowerManager_State_t PowerManager_GetState(void);
void PowerManager_GetStatus(PowerManager_Status_t *status);

#endif
