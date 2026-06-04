#ifndef POWER_STAGE_H
#define POWER_STAGE_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * ZMIENIAJ TYLKO TO, zeby ustawic czestotliwosc kluczowania PWM.
 * Przy 200 kHz kod automatycznie liczy HRTIM period = 27200 tickow
 * dla aktualnego zegara 170 MHz i HRTIM_PRESCALERRATIO_MUL32.
 * Przy tym preskalerze nie schodz ponizej ok. 83 kHz, bo HRTIM ma 16-bit period.
 */
#ifndef PSU_PWM_SWITCHING_FREQUENCY_HZ
#define PSU_PWM_SWITCHING_FREQUENCY_HZ             500000U
#endif

#ifndef POWER_STAGE_FSW_HZ
#define POWER_STAGE_FSW_HZ                         PSU_PWM_SWITCHING_FREQUENCY_HZ
#endif

#ifndef POWER_STAGE_HRTIM_PRESCALER_RATIO
#define POWER_STAGE_HRTIM_PRESCALER_RATIO          HRTIM_PRESCALERRATIO_MUL32
#endif

#ifndef POWER_STAGE_BOOTSTRAP_REFRESH_ENABLE
#define POWER_STAGE_BOOTSTRAP_REFRESH_ENABLE       1U
#endif

#ifndef POWER_STAGE_BOOTSTRAP_REFRESH_BUCK_ENABLE
#define POWER_STAGE_BOOTSTRAP_REFRESH_BUCK_ENABLE  1U
#endif

#ifndef POWER_STAGE_BOOTSTRAP_REFRESH_BOOST_ENABLE
#define POWER_STAGE_BOOTSTRAP_REFRESH_BOOST_ENABLE 0U
#endif

/* Bootstrap refresh frequency is set here; this is not PWM switching frequency. */
#ifndef POWER_STAGE_BOOTSTRAP_REFRESH_PERIOD_HZ
#define POWER_STAGE_BOOTSTRAP_REFRESH_PERIOD_HZ    50000U
#endif

#ifndef POWER_STAGE_BOOTSTRAP_REFRESH_PULSE_NS
#define POWER_STAGE_BOOTSTRAP_REFRESH_PULSE_NS     1000U
#endif

/*
 * HRTIM period, startup compare values and ADC trigger ticks are derived from
 * this value and POWER_STAGE_HRTIM_PRESCALER_RATIO by PowerStage_GetConfigured*
 * helpers; do not hardcode PWM period in main.c.
 */

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
void PowerStage_SuspendOutputsKeepDriverOn(void);
void PowerStage_ForceSafeState(void);
void PowerStage_SetBuckDischarge(uint32_t pulse_ns, uint32_t every_periods);
uint32_t PowerStage_GetConfiguredPeriodTicks(void);
uint32_t PowerStage_GetConfiguredAdcTriggerTicks(void);
uint32_t PowerStage_GetSafeInitCompareTicks(void);
uint32_t PowerStage_GetFswHz(void);
bool PowerStage_IsBootstrapRefreshActive(void);
uint32_t PowerStage_GetBootstrapRefreshHz(void);
uint32_t PowerStage_GetBootstrapRefreshPeriodTicks(void);
uint32_t PowerStage_GetBootstrapRefreshPulseTicks(void);

/* Konfiguracje pol-mostkow: static high lub klasyczne PWM. */
void PowerStage_ConfigHalfBridgeA_StaticHigh(bool enable_refresh);
void PowerStage_ConfigHalfBridgeC_StaticHigh(bool enable_refresh);
void PowerStage_ConfigHalfBridgeA_Pwm(float duty_a);
void PowerStage_ConfigHalfBridgeC_Pwm(float duty_c);

/* duty_a = D_A buck high-side TA1/HIN, duty_c = D_B boost low-side TC2/LIN. */
void PowerStage_SetDuty(float duty_a, float duty_c);
void PowerStage_SetDuty10k(uint32_t duty_a_10k, uint32_t duty_b_10k);
void PowerStage_SetBuckDuty(float duty_a);
/* Diagnostic-only helper for pure BOOST experiments (not used in normal auto-CV). */
void PowerStage_SetBoostDuty(float duty_b);
void PowerStage_SetBuckBoostDuty(float duty_a, float duty_b);
void PowerStage_SetAdcTriggerPoint(float trigger_point);
void PowerStage_SetAdcTriggerPoint10k(uint32_t trigger_point_10k);
void PowerStage_SetRegion(PowerStage_Region_t region);
PowerStage_Region_t PowerStage_GetRegion(void);
bool PowerStage_IsFaultActive(void);
bool PowerStage_IsEnabled(void);
bool PowerStage_IsDischarging(void);
uint8_t PowerStage_GetLastError(void);
float PowerStage_GetDutyA(void);
float PowerStage_GetDutyC(void);
uint32_t PowerStage_GetDutyA10k(void);
/* Zadane duty TC2/LIN_C (komenda aplikacji). */
uint32_t PowerStage_GetDutyCCmd10k(void);
/* Fizyczne duty TC2/LIN_C przeliczone z aktywnego CMP1. */
uint32_t PowerStage_GetDutyC10k(void);
uint32_t PowerStage_GetDutyCPhys10k(void);
uint32_t PowerStage_GetExpectedTc1Duty10k(void);
uint32_t PowerStage_GetExpectedTc2Duty10k(void);
uint32_t PowerStage_GetPeriodTicks(void);
uint32_t PowerStage_GetCmpA(void);
uint32_t PowerStage_GetCmpC(void);
float PowerStage_GetPwmStepPercent(void);

#endif
