#ifndef POWER_STAGE_H
#define POWER_STAGE_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    POWER_REGION_BUCK = 0,
    POWER_REGION_BOOST,
    POWER_REGION_BUCK_BOOST
} PowerStage_Region_t;

typedef enum {
    POWER_STAGE_ERR_NONE = 0,
    POWER_STAGE_ERR_NOT_INITIALIZED,
    POWER_STAGE_ERR_DRIVER_FAULT,
    POWER_STAGE_ERR_COUNTER_START,
    POWER_STAGE_ERR_OUTPUT_START
} PowerStage_Error_t;

void PowerStage_Init(HRTIM_HandleTypeDef *hhrtim);
bool PowerStage_Enable(void);
void PowerStage_Disable(void);
void PowerStage_ForceSafeState(void);
/* duty_a = D_A buck high-side TA1/HIN, duty_c = D_B boost low-side TC2/LIN. */
void PowerStage_SetDuty(float duty_a, float duty_c);
void PowerStage_SetDuty10k(uint32_t duty_a_10k, uint32_t duty_b_10k);
void PowerStage_SetBuckDuty(float duty_a);
void PowerStage_SetBoostDuty(float duty_b);
void PowerStage_SetBuckBoostDuty(float duty_a, float duty_b);
void PowerStage_SetAdcTriggerPoint(float trigger_point);
void PowerStage_SetAdcTriggerPoint10k(uint32_t trigger_point_10k);
void PowerStage_SetRegion(PowerStage_Region_t region);
PowerStage_Region_t PowerStage_GetRegion(void);
bool PowerStage_IsFaultActive(void);
bool PowerStage_IsEnabled(void);
uint8_t PowerStage_GetLastError(void);
float PowerStage_GetDutyA(void);
float PowerStage_GetDutyC(void);

#endif
