#include "power_stage.h"

#include <stddef.h>

#define POWER_STAGE_DUTY_SCALE          10000U
#define POWER_STAGE_BOOTSTRAP_REFRESH_10K 100U
#define POWER_STAGE_STATIC_HIGH_10K     (POWER_STAGE_DUTY_SCALE - POWER_STAGE_BOOTSTRAP_REFRESH_10K)
#define POWER_STAGE_ENABLE_DELAY_MS     1U
#define POWER_STAGE_DEFAULT_PERIOD      5440U
#define POWER_STAGE_ADC_TRIGGER_10K     5000U
#define POWER_STAGE_TIMERS              (HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_C)
#define POWER_STAGE_OUTPUTS             (HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | \
                                         HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2)

typedef struct {
    HRTIM_HandleTypeDef *hhrtim;
    PowerStage_Region_t region;
    float duty_a;
    float duty_c;
    uint32_t duty_a_10k;
    uint32_t duty_b_10k;
    uint32_t adc_trigger_10k;
    uint32_t discharge_pulse_ticks;
    uint32_t discharge_every_periods;
    uint8_t last_error;
    bool enabled;
    bool initialized;
    bool discharge_active;
} PowerStage_Context_t;

static PowerStage_Context_t ps;

static float PowerStage_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint32_t PowerStage_PeriodTicks(void)
{
    uint32_t period;

    if ((ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return POWER_STAGE_DEFAULT_PERIOD;
    }

    period = ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR;
    if (period == 0U) {
        period = POWER_STAGE_DEFAULT_PERIOD;
    }

    return period;
}

static uint32_t PowerStage_Clamp10k(uint32_t value)
{
    if (value > POWER_STAGE_DUTY_SCALE) {
        value = POWER_STAGE_DUTY_SCALE;
    }

    return value;
}

static uint32_t PowerStage_FloatTo10k(float duty)
{
    duty = PowerStage_Clamp(duty, 0.0f, 1.0f);
    return (uint32_t)((duty * (float)POWER_STAGE_DUTY_SCALE) + 0.5f);
}

static float PowerStage_10kToFloat(uint32_t duty_10k)
{
    duty_10k = PowerStage_Clamp10k(duty_10k);
    return (float)duty_10k / (float)POWER_STAGE_DUTY_SCALE;
}

static void PowerStage_DisableBurstMode(void)
{
    if ((ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return;
    }

    (void)HAL_HRTIM_BurstModeCtl(ps.hhrtim, HRTIM_BURSTMODECTL_DISABLED);
}

static uint32_t PowerStage_NsToTicks(uint32_t pulse_ns)
{
    uint32_t period = PowerStage_PeriodTicks();
    uint32_t ticks;

    if (pulse_ns == 0U) {
        pulse_ns = 1U;
    }

    ticks = (period * pulse_ns) / 1000U;
    if (ticks == 0U) {
        ticks = 1U;
    } else if (ticks >= period) {
        ticks = period - 1U;
    }

    return ticks;
}

static uint32_t PowerStage_DutyAToCmp(uint32_t duty_a_10k)
{
    uint32_t period = PowerStage_PeriodTicks();

    duty_a_10k = PowerStage_Clamp10k(duty_a_10k);

    /*
     * D_A steruje wysokim tranzystorem mostka buck (TA1/HIN).
     * D_A = 100% w komendzie oznacza prawie-stale high z krotkim impulsem
     * TA2/LIN do odswiezania bootstrapu high-side drivera.
     */
    if (duty_a_10k >= POWER_STAGE_DUTY_SCALE) {
        duty_a_10k = POWER_STAGE_STATIC_HIGH_10K;
    }

    return (duty_a_10k * period) / POWER_STAGE_DUTY_SCALE;
}

static uint32_t PowerStage_DutyBToCmp(uint32_t duty_b_10k)
{
    uint32_t period = PowerStage_PeriodTicks();

    duty_b_10k = PowerStage_Clamp10k(duty_b_10k);

    /*
     * D_B opisuje duty boost-switcha, czyli niskiego tranzystora TC2/LIN.
     * HRTIM porownuje CMP1 dla TC1(HIN), wiec wpisujemy komplement:
     * D_B = 0%   -> TC1 prawie stale high, TC2 ma krotki bootstrap refresh
     * D_B > 0%   -> TC2 dostaje zadane duty, TC1 jego komplement
     */
    if (duty_b_10k == 0U) {
        duty_b_10k = POWER_STAGE_BOOTSTRAP_REFRESH_10K;
    }

    if (duty_b_10k >= POWER_STAGE_DUTY_SCALE) {
        return 0U;
    }

    return ((POWER_STAGE_DUTY_SCALE - duty_b_10k) * period) / POWER_STAGE_DUTY_SCALE;
}

static void PowerStage_ConfigureComplementaryOutputs(void)
{
    HRTIM_OutputCfgTypeDef out_cfg = {0};

    if (ps.hhrtim == NULL) {
        return;
    }

    out_cfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
    out_cfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
    out_cfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
    out_cfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
    out_cfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    out_cfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
    out_cfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
    out_cfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;

    (void)HAL_HRTIM_WaveformOutputConfig(ps.hhrtim,
                                         HRTIM_TIMERINDEX_TIMER_A,
                                         HRTIM_OUTPUT_TA1,
                                         &out_cfg);
    (void)HAL_HRTIM_WaveformOutputConfig(ps.hhrtim,
                                         HRTIM_TIMERINDEX_TIMER_C,
                                         HRTIM_OUTPUT_TC1,
                                         &out_cfg);

    out_cfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
    out_cfg.ResetSource = HRTIM_OUTPUTRESET_TIMPER;

    (void)HAL_HRTIM_WaveformOutputConfig(ps.hhrtim,
                                         HRTIM_TIMERINDEX_TIMER_A,
                                         HRTIM_OUTPUT_TA2,
                                         &out_cfg);
    (void)HAL_HRTIM_WaveformOutputConfig(ps.hhrtim,
                                         HRTIM_TIMERINDEX_TIMER_C,
                                         HRTIM_OUTPUT_TC2,
                                         &out_cfg);
}

static void PowerStage_ApplyBaseTiming(void)
{
    uint32_t period;

    if ((ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return;
    }

    period = PowerStage_PeriodTicks();
    ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR = period;
    ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].PERxR = period;
    PowerStage_SetAdcTriggerPoint10k(ps.adc_trigger_10k);

    (void)HAL_HRTIM_SoftwareUpdate(ps.hhrtim,
                                   HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_C);
}

static bool PowerStage_StartCounters(void)
{
    if ((!ps.initialized) || (ps.hhrtim == NULL)) {
        ps.last_error = POWER_STAGE_ERR_NOT_INITIALIZED;
        return false;
    }

    if (HAL_HRTIM_WaveformCounterStart(ps.hhrtim, POWER_STAGE_TIMERS) != HAL_OK) {
        ps.last_error = POWER_STAGE_ERR_COUNTER_START;
        return false;
    }

    return true;
}

void PowerStage_Init(HRTIM_HandleTypeDef *hhrtim)
{
    GPIO_InitTypeDef gpio_cfg = {0};

    ps.hhrtim = hhrtim;
    ps.region = POWER_REGION_BUCK;
    ps.duty_a = 0.0f;
    ps.duty_c = 0.0f;
    ps.duty_a_10k = 0U;
    ps.duty_b_10k = 0U;
    ps.adc_trigger_10k = POWER_STAGE_ADC_TRIGGER_10K;
    ps.discharge_pulse_ticks = 0U;
    ps.discharge_every_periods = 0U;
    ps.last_error = POWER_STAGE_ERR_NONE;
    ps.enabled = false;
    ps.initialized = true;
    ps.discharge_active = false;

    /* FLT ma byc wejsciem open-drain z zewnetrznym pull-up. */
    gpio_cfg.Pin = FLT_Pin;
    gpio_cfg.Mode = GPIO_MODE_INPUT;
    gpio_cfg.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(FLT_GPIO_Port, &gpio_cfg);

    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_RESET);

    PowerStage_ConfigureComplementaryOutputs();
    PowerStage_ApplyBaseTiming();
    PowerStage_SetDuty(0.0f, 0.0f);
    PowerStage_ForceSafeState();
    (void)PowerStage_StartCounters();
}

bool PowerStage_Enable(void)
{
    if ((!ps.initialized) || (ps.hhrtim == NULL)) {
        ps.last_error = POWER_STAGE_ERR_NOT_INITIALIZED;
        return false;
    }

    ps.last_error = POWER_STAGE_ERR_NONE;
    PowerStage_DisableBurstMode();

    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);
    HAL_Delay(POWER_STAGE_ENABLE_DELAY_MS);

    if (PowerStage_IsFaultActive()) {
        ps.last_error = POWER_STAGE_ERR_DRIVER_FAULT;
        PowerStage_ForceSafeState();
        return false;
    }

    PowerStage_ConfigureComplementaryOutputs();
    PowerStage_ApplyBaseTiming();
    PowerStage_SetDuty(ps.duty_a, ps.duty_c);

    if (!PowerStage_StartCounters()) {
        PowerStage_ForceSafeState();
        return false;
    }

    if (HAL_HRTIM_WaveformOutputStart(ps.hhrtim, POWER_STAGE_OUTPUTS) != HAL_OK) {
        ps.last_error = POWER_STAGE_ERR_OUTPUT_START;
        PowerStage_ForceSafeState();
        return false;
    }

    ps.enabled = true;
    ps.discharge_active = false;
    ps.last_error = POWER_STAGE_ERR_NONE;
    return true;
}

void PowerStage_Disable(void)
{
    if ((!ps.initialized) || (ps.hhrtim == NULL)) {
        ps.last_error = POWER_STAGE_ERR_NOT_INITIALIZED;
        return;
    }

    (void)HAL_HRTIM_WaveformOutputStop(ps.hhrtim, POWER_STAGE_OUTPUTS);
    PowerStage_DisableBurstMode();
    PowerStage_SetDuty(0.0f, 0.0f);

    HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
    ps.enabled = false;
    ps.discharge_active = false;
}

void PowerStage_SuspendOutputsKeepDriverOn(void)
{
    if ((!ps.initialized) || (ps.hhrtim == NULL)) {
        ps.last_error = POWER_STAGE_ERR_NOT_INITIALIZED;
        return;
    }

    (void)HAL_HRTIM_WaveformOutputStop(ps.hhrtim, POWER_STAGE_OUTPUTS);
    PowerStage_DisableBurstMode();
    PowerStage_SetDuty(0.0f, 0.0f);

    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);
    ps.enabled = false;
    ps.discharge_active = false;
    ps.last_error = POWER_STAGE_ERR_NONE;
}

void PowerStage_ForceSafeState(void)
{
    PowerStage_Disable();
}

void PowerStage_SetDuty10k(uint32_t duty_a_10k, uint32_t duty_b_10k)
{
    uint32_t cmp_a;
    uint32_t cmp_c;

    if ((!ps.initialized) || (ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return;
    }

    PowerStage_DisableBurstMode();

    duty_a_10k = PowerStage_Clamp10k(duty_a_10k);
    duty_b_10k = PowerStage_Clamp10k(duty_b_10k);

    cmp_a = PowerStage_DutyAToCmp(duty_a_10k);
    cmp_c = PowerStage_DutyBToCmp(duty_b_10k);

    ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = cmp_a;
    ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = cmp_c;

    (void)HAL_HRTIM_SoftwareUpdate(ps.hhrtim,
                                   HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_C);

    ps.duty_a_10k = duty_a_10k;
    ps.duty_b_10k = duty_b_10k;
    ps.duty_a = PowerStage_10kToFloat(duty_a_10k);
    ps.duty_c = PowerStage_10kToFloat(duty_b_10k);
    ps.discharge_active = false;
}

void PowerStage_SetBuckDischarge(uint32_t pulse_ns, uint32_t every_periods)
{
    HRTIM_OutputCfgTypeDef out_cfg = {0};
    HRTIM_BurstModeCfgTypeDef burst_cfg = {0};
    uint32_t pulse_ticks;
    uint32_t period;
    uint32_t cmp_c;

    if ((!ps.initialized) || (ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        ps.last_error = POWER_STAGE_ERR_NOT_INITIALIZED;
        return;
    }

    if (every_periods == 0U) {
        every_periods = 1U;
    }

    pulse_ticks = PowerStage_NsToTicks(pulse_ns);
    if (ps.discharge_active &&
        (ps.discharge_pulse_ticks == pulse_ticks) &&
        (ps.discharge_every_periods == every_periods)) {
        return;
    }

    period = PowerStage_PeriodTicks();
    cmp_c = PowerStage_DutyBToCmp(0U);

    (void)HAL_HRTIM_WaveformOutputStop(ps.hhrtim, POWER_STAGE_OUTPUTS);
    PowerStage_DisableBurstMode();

    out_cfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
    out_cfg.SetSource = HRTIM_OUTPUTSET_NONE;
    out_cfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
    out_cfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
    out_cfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    out_cfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
    out_cfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
    out_cfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;

    /* TA1 ma byc stale wylaczony, zeby nie pompowac energii z VIN. */
    (void)HAL_HRTIM_WaveformOutputConfig(ps.hhrtim,
                                         HRTIM_TIMERINDEX_TIMER_A,
                                         HRTIM_OUTPUT_TA1,
                                         &out_cfg);
    (void)HAL_HRTIM_WaveformSetOutputLevel(ps.hhrtim,
                                           HRTIM_TIMERINDEX_TIMER_A,
                                           HRTIM_OUTPUT_TA1,
                                           HRTIM_OUTPUTLEVEL_INACTIVE);

    /* TA2 daje krotki impuls low-side A: realny czas w ns, nie mikro-duty. */
    out_cfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
    out_cfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
    (void)HAL_HRTIM_WaveformOutputConfig(ps.hhrtim,
                                         HRTIM_TIMERINDEX_TIMER_A,
                                         HRTIM_OUTPUT_TA2,
                                         &out_cfg);

    /* Mostek C zostaje jak buck pass-through, z refresh bootstrapu. */
    (void)HAL_HRTIM_WaveformOutputConfig(ps.hhrtim,
                                         HRTIM_TIMERINDEX_TIMER_C,
                                         HRTIM_OUTPUT_TC1,
                                         &out_cfg);
    out_cfg.SetSource = HRTIM_OUTPUTSET_TIMCMP1;
    out_cfg.ResetSource = HRTIM_OUTPUTRESET_TIMPER;
    (void)HAL_HRTIM_WaveformOutputConfig(ps.hhrtim,
                                         HRTIM_TIMERINDEX_TIMER_C,
                                         HRTIM_OUTPUT_TC2,
                                         &out_cfg);

    ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = pulse_ticks;
    ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = cmp_c;
    ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR = period;
    ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].PERxR = period;

    (void)HAL_HRTIM_SoftwareUpdate(ps.hhrtim,
                                   HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_C);

    if (every_periods > 1U) {
        burst_cfg.Mode = HRTIM_BURSTMODE_CONTINOUS;
        burst_cfg.ClockSource = HRTIM_BURSTMODECLOCKSOURCE_TIMER_A;
        burst_cfg.Prescaler = HRTIM_BURSTMODEPRESCALER_DIV1;
        burst_cfg.PreloadEnable = HRIM_BURSTMODEPRELOAD_DISABLED;
        burst_cfg.Trigger = HRTIM_BURSTMODETRIGGER_NONE;
        burst_cfg.IdleDuration = every_periods - 1U;
        burst_cfg.Period = every_periods;
        (void)HAL_HRTIM_BurstModeConfig(ps.hhrtim, &burst_cfg);
        (void)HAL_HRTIM_BurstModeCtl(ps.hhrtim, HRTIM_BURSTMODECTL_ENABLED);
        (void)HAL_HRTIM_BurstModeSoftwareTrigger(ps.hhrtim);
    }

    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_SET);

    if (PowerStage_IsFaultActive()) {
        ps.last_error = POWER_STAGE_ERR_DRIVER_FAULT;
        PowerStage_ForceSafeState();
        return;
    }

    if (!PowerStage_StartCounters()) {
        PowerStage_ForceSafeState();
        return;
    }

    if (HAL_HRTIM_WaveformOutputStart(ps.hhrtim,
                                      HRTIM_OUTPUT_TA2 | HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2) != HAL_OK) {
        ps.last_error = POWER_STAGE_ERR_OUTPUT_START;
        PowerStage_ForceSafeState();
        return;
    }

    ps.region = POWER_REGION_BUCK;
    ps.duty_a_10k = 0U;
    ps.duty_b_10k = 0U;
    ps.duty_a = 0.0f;
    ps.duty_c = 0.0f;
    ps.discharge_pulse_ticks = pulse_ticks;
    ps.discharge_every_periods = every_periods;
    ps.enabled = true;
    ps.discharge_active = true;
    ps.last_error = POWER_STAGE_ERR_NONE;
}

void PowerStage_SetDuty(float duty_a, float duty_c)
{
    PowerStage_SetDuty10k(PowerStage_FloatTo10k(duty_a),
                          PowerStage_FloatTo10k(duty_c));
}

void PowerStage_SetBuckDuty(float duty_a)
{
    PowerStage_SetDuty(duty_a, 0.0f);
}

void PowerStage_SetBoostDuty(float duty_b)
{
    PowerStage_SetDuty(1.0f, duty_b);
}

void PowerStage_SetBuckBoostDuty(float duty_a, float duty_b)
{
    PowerStage_SetDuty(duty_a, duty_b);
}

void PowerStage_SetAdcTriggerPoint10k(uint32_t trigger_point_10k)
{
    uint32_t period;
    uint32_t cmp;

    trigger_point_10k = PowerStage_Clamp10k(trigger_point_10k);

    if ((!ps.initialized) || (ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        ps.adc_trigger_10k = trigger_point_10k;
        return;
    }

    period = PowerStage_PeriodTicks();
    cmp = (trigger_point_10k * period) / POWER_STAGE_DUTY_SCALE;

    if (cmp == 0U) {
        cmp = 1U;
    } else if (cmp >= period) {
        cmp = period - 1U;
    }

    ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP3xR = cmp;
    ps.adc_trigger_10k = trigger_point_10k;

    (void)HAL_HRTIM_SoftwareUpdate(ps.hhrtim, HRTIM_TIMERUPDATE_A);
}

void PowerStage_SetAdcTriggerPoint(float trigger_point)
{
    PowerStage_SetAdcTriggerPoint10k(PowerStage_FloatTo10k(trigger_point));
}

void PowerStage_SetRegion(PowerStage_Region_t region)
{
    ps.region = region;
}

PowerStage_Region_t PowerStage_GetRegion(void)
{
    return ps.region;
}

bool PowerStage_IsFaultActive(void)
{
    return (HAL_GPIO_ReadPin(FLT_GPIO_Port, FLT_Pin) == GPIO_PIN_RESET);
}

bool PowerStage_IsEnabled(void)
{
    return ps.enabled;
}

bool PowerStage_IsDischarging(void)
{
    return ps.discharge_active;
}

uint8_t PowerStage_GetLastError(void)
{
    return ps.last_error;
}

float PowerStage_GetDutyA(void)
{
    return ps.duty_a;
}

float PowerStage_GetDutyC(void)
{
    return ps.duty_c;
}
