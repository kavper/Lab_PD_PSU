#include "power_stage.h"

#include <stddef.h>

#define POWER_STAGE_DUTY_SCALE               10000U
#define POWER_STAGE_MASTER_REFRESH_PRESCALER HRTIM_PRESCALERRATIO_DIV4
#define POWER_STAGE_MASTER_MAX_PERIOD        0xFFFFU
#define POWER_STAGE_ENABLE_DELAY_MS          1U
#define POWER_STAGE_PERIOD_MIN_TICKS         64U
#define POWER_STAGE_PERIOD_MAX_TICKS         0xFFDFU
#define POWER_STAGE_ADC_TRIGGER_10K          5000U
#define POWER_STAGE_TIMERS                   (HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_C)
#define POWER_STAGE_OUTPUTS                  (HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 | \
                                              HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2)

typedef enum {
    POWER_STAGE_OUTPUT_NONE = 0,
    POWER_STAGE_OUTPUT_BUCK,
    POWER_STAGE_OUTPUT_BOOST,
    POWER_STAGE_OUTPUT_BUCK_BOOST,
    POWER_STAGE_OUTPUT_DISCHARGE
} PowerStage_OutputMode_t;

typedef struct {
    HRTIM_HandleTypeDef *hhrtim;
    PowerStage_Region_t region;
    PowerStage_OutputMode_t output_mode;
    float duty_a;
    float duty_c;
    uint32_t duty_a_10k;
    uint32_t duty_b_10k;
    uint32_t duty_c_cmd_10k;
    uint32_t duty_c_phys_10k;
    uint32_t tc1_expected_10k;
    uint32_t tc2_expected_10k;
    uint32_t adc_trigger_10k;
    uint32_t discharge_pulse_ticks;
    uint32_t discharge_every_periods;
    uint32_t refresh_period_ticks;
    uint32_t refresh_pulse_ticks;
    uint8_t last_error;
    bool enabled;
    bool initialized;
    bool discharge_active;
    bool refresh_master_active;
    bool refresh_a_active;
    bool refresh_c_active;
} PowerStage_Context_t;

static PowerStage_Context_t ps;

static float PowerStage_Clamp(float value, float min_value, float max_value)
{
    if (!(value == value)) {
        return min_value;
    }

    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint64_t PowerStage_HrtimBaseHz(void)
{
    uint64_t hrtim_hz = HAL_RCC_GetPCLK2Freq();

    if (hrtim_hz == 0ULL) {
        hrtim_hz = SystemCoreClock;
    }
    if (hrtim_hz == 0ULL) {
        hrtim_hz = 1ULL;
    }

    return hrtim_hz;
}

static uint64_t PowerStage_HrtimTickHzForPrescaler(uint32_t prescaler_ratio)
{
    uint64_t hrtim_hz = PowerStage_HrtimBaseHz();

    switch (prescaler_ratio) {
        case HRTIM_PRESCALERRATIO_MUL32:
            return hrtim_hz * 32ULL;
        case HRTIM_PRESCALERRATIO_MUL16:
            return hrtim_hz * 16ULL;
        case HRTIM_PRESCALERRATIO_MUL8:
            return hrtim_hz * 8ULL;
        case HRTIM_PRESCALERRATIO_MUL4:
            return hrtim_hz * 4ULL;
        case HRTIM_PRESCALERRATIO_MUL2:
            return hrtim_hz * 2ULL;
        case HRTIM_PRESCALERRATIO_DIV2:
            return hrtim_hz / 2ULL;
        case HRTIM_PRESCALERRATIO_DIV4:
            return hrtim_hz / 4ULL;
        case HRTIM_PRESCALERRATIO_DIV1:
        default:
            return hrtim_hz;
    }
}

static uint64_t PowerStage_HrtimTickHz(void)
{
    return PowerStage_HrtimTickHzForPrescaler(POWER_STAGE_HRTIM_PRESCALER_RATIO);
}

static uint32_t PowerStage_NsToTicksAtHz(uint32_t ns, uint64_t tick_hz)
{
    uint64_t ticks;
    uint64_t whole;
    uint64_t frac;

    if (ns == 0U) {
        ns = 1U;
    }
    if (tick_hz == 0ULL) {
        tick_hz = 1ULL;
    }

    whole = (tick_hz / 1000000000ULL) * (uint64_t)ns;
    frac = (((tick_hz % 1000000000ULL) * (uint64_t)ns) + 500000000ULL) /
           1000000000ULL;
    ticks = whole + frac;

    if (ticks == 0ULL) {
        ticks = 1ULL;
    } else if (ticks > UINT32_MAX) {
        ticks = UINT32_MAX;
    }

    return (uint32_t)ticks;
}

static uint32_t PowerStage_PeriodFromFsw(void)
{
    uint64_t ticks;
    uint64_t hrtim_tick_hz;
    uint32_t fsw_hz = POWER_STAGE_FSW_HZ;

    if (fsw_hz == 0U) {
        fsw_hz = 1U;
    }

    hrtim_tick_hz = PowerStage_HrtimTickHz();
    ticks = (hrtim_tick_hz + ((uint64_t)fsw_hz / 2ULL)) /
            (uint64_t)fsw_hz;

    if (ticks < (uint64_t)POWER_STAGE_PERIOD_MIN_TICKS) {
        ticks = POWER_STAGE_PERIOD_MIN_TICKS;
    } else if (ticks > (uint64_t)POWER_STAGE_PERIOD_MAX_TICKS) {
        ticks = POWER_STAGE_PERIOD_MAX_TICKS;
    }

    return (uint32_t)ticks;
}

static uint32_t PowerStage_PeriodTicksRaw(void)
{
    uint32_t period;

    if ((ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return PowerStage_PeriodFromFsw();
    }

    period = ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR;
    if (period == 0U) {
        period = PowerStage_PeriodFromFsw();
    }

    return period;
}

uint32_t PowerStage_GetConfiguredPeriodTicks(void)
{
    return PowerStage_PeriodFromFsw();
}

uint32_t PowerStage_GetFswHz(void)
{
    uint32_t period = PowerStage_PeriodTicksRaw();
    uint64_t hrtim_tick_hz;

    if (period == 0U) {
        return 0U;
    }

    hrtim_tick_hz = PowerStage_HrtimTickHz();
    return (uint32_t)((hrtim_tick_hz + ((uint64_t)period / 2ULL)) /
                      (uint64_t)period);
}

uint32_t PowerStage_GetPeriodTicks(void)
{
    return PowerStage_PeriodTicksRaw();
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

static uint32_t PowerStage_Duty10kToSafeCmp(uint32_t duty_10k)
{
    uint32_t period = PowerStage_PeriodFromFsw();
    uint32_t cmp;

    duty_10k = PowerStage_Clamp10k(duty_10k);
    cmp = (duty_10k * period) / POWER_STAGE_DUTY_SCALE;

    if (cmp == 0U) {
        cmp = 1U;
    } else if (cmp >= period) {
        cmp = period - 1U;
    }

    return cmp;
}

uint32_t PowerStage_GetConfiguredAdcTriggerTicks(void)
{
    return PowerStage_Duty10kToSafeCmp(POWER_STAGE_ADC_TRIGGER_10K);
}

uint32_t PowerStage_GetSafeInitCompareTicks(void)
{
    return PowerStage_Duty10kToSafeCmp(1U);
}

static float PowerStage_10kToFloat(uint32_t duty_10k)
{
    duty_10k = PowerStage_Clamp10k(duty_10k);
    return (float)duty_10k / (float)POWER_STAGE_DUTY_SCALE;
}

static bool PowerStage_IsBuckRefreshEnabled(void)
{
#if (POWER_STAGE_BOOTSTRAP_REFRESH_ENABLE != 0U) && \
    (POWER_STAGE_BOOTSTRAP_REFRESH_BUCK_ENABLE != 0U)
    return true;
#else
    return false;
#endif
}

static bool PowerStage_IsBoostRefreshEnabled(void)
{
#if (POWER_STAGE_BOOTSTRAP_REFRESH_ENABLE != 0U) && \
    (POWER_STAGE_BOOTSTRAP_REFRESH_BOOST_ENABLE != 0U)
    return true;
#else
    return false;
#endif
}

static void PowerStage_DisableHrtimBurst(void)
{
    if ((ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return;
    }

    (void)HAL_HRTIM_BurstModeCtl(ps.hhrtim, HRTIM_BURSTMODECTL_DISABLED);
}

static void PowerStage_DisableBurstMode(void)
{
    if ((ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return;
    }

    PowerStage_DisableHrtimBurst();
    (void)HAL_HRTIM_WaveformCounterStop(ps.hhrtim, HRTIM_TIMERID_MASTER);
    ps.refresh_master_active = false;
    ps.refresh_period_ticks = 0U;
    ps.refresh_pulse_ticks = 0U;
}

static uint32_t PowerStage_NsToTicks(uint32_t pulse_ns)
{
    uint32_t period = PowerStage_PeriodTicksRaw();
    uint32_t ticks;

    ticks = PowerStage_NsToTicksAtHz(pulse_ns, PowerStage_HrtimTickHz());
    if (ticks == 0U) {
        ticks = 1U;
    } else if (ticks >= period) {
        ticks = period - 1U;
    }

    return ticks;
}

static uint32_t PowerStage_DutyAToCmp(uint32_t duty_a_10k)
{
    uint32_t period = PowerStage_PeriodTicksRaw();

    duty_a_10k = PowerStage_Clamp10k(duty_a_10k);
    return (duty_a_10k * period) / POWER_STAGE_DUTY_SCALE;
}

static uint32_t PowerStage_DutyBToCmp(uint32_t duty_b_10k)
{
    uint32_t period = PowerStage_PeriodTicksRaw();

    duty_b_10k = PowerStage_Clamp10k(duty_b_10k);

    /*
     * D_B opisuje duty boost-switcha, czyli niskiego tranzystora TC2/LIN.
     * CMP1 steruje TC1(HIN), wiec wpisujemy komplement.
     */
    if (duty_b_10k >= POWER_STAGE_DUTY_SCALE) {
        return 0U;
    }

    return ((POWER_STAGE_DUTY_SCALE - duty_b_10k) * period) / POWER_STAGE_DUTY_SCALE;
}

static uint32_t PowerStage_CmpToDutyTc2_10k(uint32_t cmp)
{
    uint32_t period = PowerStage_PeriodTicksRaw();
    uint32_t duty;

    if (period == 0U) {
        return 0U;
    }

    if (cmp >= period) {
        return 0U;
    }

    duty = ((period - cmp) * POWER_STAGE_DUTY_SCALE) / period;
    return PowerStage_Clamp10k(duty);
}

static void PowerStage_ConfigOutput(uint32_t timer_index,
                                    uint32_t output,
                                    uint32_t set_source,
                                    uint32_t reset_source)
{
    HRTIM_OutputCfgTypeDef out_cfg = {0};

    if (ps.hhrtim == NULL) {
        return;
    }

    out_cfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
    out_cfg.SetSource = set_source;
    out_cfg.ResetSource = reset_source;
    out_cfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
    out_cfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    out_cfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
    out_cfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
    out_cfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;

    (void)HAL_HRTIM_WaveformOutputConfig(ps.hhrtim,
                                         timer_index,
                                         output,
                                         &out_cfg);
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

static uint32_t PowerStage_MasterRefreshPeriodTicks(void)
{
    uint32_t refresh_hz = POWER_STAGE_BOOTSTRAP_REFRESH_PERIOD_HZ;
    uint64_t refresh_tick_hz;
    uint64_t ticks_calc;
    uint32_t ticks;

    if (refresh_hz == 0U) {
        refresh_hz = 1U;
    }

    refresh_tick_hz = PowerStage_HrtimTickHzForPrescaler(POWER_STAGE_MASTER_REFRESH_PRESCALER);
    ticks_calc = (refresh_tick_hz + ((uint64_t)refresh_hz / 2ULL)) /
                 (uint64_t)refresh_hz;
    ticks = (ticks_calc > UINT32_MAX) ? UINT32_MAX : (uint32_t)ticks_calc;

    if (ticks < 4U) {
        ticks = 4U;
    } else if (ticks > POWER_STAGE_MASTER_MAX_PERIOD) {
        ticks = POWER_STAGE_MASTER_MAX_PERIOD;
    }

    return ticks;
}

static uint32_t PowerStage_MasterRefreshPulseTicks(void)
{
    uint32_t ticks;
    uint32_t max_ticks;

    ticks = PowerStage_NsToTicksAtHz(POWER_STAGE_BOOTSTRAP_REFRESH_PULSE_NS,
                                     PowerStage_HrtimTickHzForPrescaler(POWER_STAGE_MASTER_REFRESH_PRESCALER));
    if (ticks == 0U) {
        ticks = 1U;
    }

    max_ticks = PowerStage_MasterRefreshPeriodTicks() / 4U;
    if (max_ticks == 0U) {
        max_ticks = 1U;
    }
    if (ticks > max_ticks) {
        ticks = max_ticks;
    }

    return ticks;
}

uint32_t PowerStage_GetBootstrapRefreshHz(void)
{
    return POWER_STAGE_BOOTSTRAP_REFRESH_PERIOD_HZ;
}

uint32_t PowerStage_GetBootstrapRefreshPeriodTicks(void)
{
    if (ps.refresh_master_active) {
        return ps.refresh_period_ticks;
    }

    return PowerStage_MasterRefreshPeriodTicks();
}

uint32_t PowerStage_GetBootstrapRefreshPulseTicks(void)
{
    if (ps.refresh_master_active) {
        return ps.refresh_pulse_ticks;
    }

    return PowerStage_MasterRefreshPulseTicks();
}

bool PowerStage_IsBootstrapRefreshActive(void)
{
    return ps.refresh_master_active;
}

static void PowerStage_ConfigBootstrapRefreshMaster(void)
{
    uint32_t period_ticks;
    uint32_t pulse_ticks;
    uint32_t cmp1_ticks;
    uint32_t cmp2_ticks;

#if (POWER_STAGE_BOOTSTRAP_REFRESH_ENABLE == 0U)
    return;
#endif

    if ((ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return;
    }

    period_ticks = PowerStage_MasterRefreshPeriodTicks();
    pulse_ticks = PowerStage_MasterRefreshPulseTicks();
    cmp1_ticks = 1U;
    cmp2_ticks = cmp1_ticks + pulse_ticks;

    if (cmp2_ticks >= period_ticks) {
        cmp2_ticks = period_ticks - 1U;
    }

    if (ps.refresh_master_active &&
        (ps.refresh_period_ticks == period_ticks) &&
        (ps.refresh_pulse_ticks == pulse_ticks)) {
        return;
    }

    MODIFY_REG(ps.hhrtim->Instance->sMasterRegs.MCR,
               HRTIM_MCR_CK_PSC,
               POWER_STAGE_MASTER_REFRESH_PRESCALER);

    ps.hhrtim->Instance->sMasterRegs.MPER = period_ticks;
    ps.hhrtim->Instance->sMasterRegs.MCMP1R = cmp1_ticks;
    ps.hhrtim->Instance->sMasterRegs.MCMP2R = cmp2_ticks;
    ps.hhrtim->Instance->sMasterRegs.MCNTR = 0U;

    (void)HAL_HRTIM_SoftwareUpdate(ps.hhrtim, HRTIM_TIMERUPDATE_MASTER);
    (void)HAL_HRTIM_WaveformCounterStart(ps.hhrtim, HRTIM_TIMERID_MASTER);

    ps.refresh_master_active = true;
    ps.refresh_period_ticks = period_ticks;
    ps.refresh_pulse_ticks = pulse_ticks;
}

static void PowerStage_RefreshMasterSync(bool need_refresh)
{
    if (need_refresh) {
        PowerStage_ConfigBootstrapRefreshMaster();
    } else {
        PowerStage_DisableBurstMode();
    }
}

void PowerStage_ConfigHalfBridgeA_Pwm(float duty_a)
{
    uint32_t cmp;

    if ((ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return;
    }

    cmp = PowerStage_DutyAToCmp(PowerStage_FloatTo10k(duty_a));

    PowerStage_ConfigOutput(HRTIM_TIMERINDEX_TIMER_A,
                            HRTIM_OUTPUT_TA1,
                            HRTIM_OUTPUTSET_TIMPER,
                            HRTIM_OUTPUTRESET_TIMCMP1);
    PowerStage_ConfigOutput(HRTIM_TIMERINDEX_TIMER_A,
                            HRTIM_OUTPUT_TA2,
                            HRTIM_OUTPUTSET_TIMCMP1,
                            HRTIM_OUTPUTRESET_TIMPER);
    ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = cmp;
}

void PowerStage_ConfigHalfBridgeC_Pwm(float duty_c)
{
    uint32_t cmp;

    if ((ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return;
    }

    /*
     * duty_c oznacza fizyczny duty TC2/LIN_C.
     * TC1/HIN_C dostaje przebieg komplementarny (z dead-time) przez CMP1.
     */
    cmp = PowerStage_DutyBToCmp(PowerStage_FloatTo10k(duty_c));

    PowerStage_ConfigOutput(HRTIM_TIMERINDEX_TIMER_C,
                            HRTIM_OUTPUT_TC1,
                            HRTIM_OUTPUTSET_TIMPER,
                            HRTIM_OUTPUTRESET_TIMCMP1);
    PowerStage_ConfigOutput(HRTIM_TIMERINDEX_TIMER_C,
                            HRTIM_OUTPUT_TC2,
                            HRTIM_OUTPUTSET_TIMCMP1,
                            HRTIM_OUTPUTRESET_TIMPER);
    ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = cmp;
}

void PowerStage_ConfigHalfBridgeA_StaticHigh(bool enable_refresh)
{
    if ((ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return;
    }

    if (enable_refresh) {
        PowerStage_ConfigOutput(HRTIM_TIMERINDEX_TIMER_A,
                                HRTIM_OUTPUT_TA1,
                                HRTIM_OUTPUTSET_MASTERCMP2,
                                HRTIM_OUTPUTRESET_MASTERCMP1);
        PowerStage_ConfigOutput(HRTIM_TIMERINDEX_TIMER_A,
                                HRTIM_OUTPUT_TA2,
                                HRTIM_OUTPUTSET_MASTERCMP1,
                                HRTIM_OUTPUTRESET_MASTERCMP2);
        (void)HAL_HRTIM_WaveformSetOutputLevel(ps.hhrtim,
                                               HRTIM_TIMERINDEX_TIMER_A,
                                               HRTIM_OUTPUT_TA1,
                                               HRTIM_OUTPUTLEVEL_ACTIVE);
        (void)HAL_HRTIM_WaveformSetOutputLevel(ps.hhrtim,
                                               HRTIM_TIMERINDEX_TIMER_A,
                                               HRTIM_OUTPUT_TA2,
                                               HRTIM_OUTPUTLEVEL_INACTIVE);
    } else {
        /*
         * Static-high bez pseudo PWM: TA1 ustawiany na PER i bez resetu.
         * TA2 pozostaje stale nieaktywny.
         */
        PowerStage_ConfigOutput(HRTIM_TIMERINDEX_TIMER_A,
                                HRTIM_OUTPUT_TA1,
                                HRTIM_OUTPUTSET_TIMPER,
                                HRTIM_OUTPUTRESET_NONE);
        PowerStage_ConfigOutput(HRTIM_TIMERINDEX_TIMER_A,
                                HRTIM_OUTPUT_TA2,
                                HRTIM_OUTPUTSET_NONE,
                                HRTIM_OUTPUTRESET_TIMPER);
        (void)HAL_HRTIM_WaveformSetOutputLevel(ps.hhrtim,
                                               HRTIM_TIMERINDEX_TIMER_A,
                                               HRTIM_OUTPUT_TA1,
                                               HRTIM_OUTPUTLEVEL_ACTIVE);
        (void)HAL_HRTIM_WaveformSetOutputLevel(ps.hhrtim,
                                               HRTIM_TIMERINDEX_TIMER_A,
                                               HRTIM_OUTPUT_TA2,
                                               HRTIM_OUTPUTLEVEL_INACTIVE);
    }
}

void PowerStage_ConfigHalfBridgeC_StaticHigh(bool enable_refresh)
{
    if ((ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return;
    }

    if (enable_refresh) {
        PowerStage_ConfigOutput(HRTIM_TIMERINDEX_TIMER_C,
                                HRTIM_OUTPUT_TC1,
                                HRTIM_OUTPUTSET_MASTERCMP2,
                                HRTIM_OUTPUTRESET_MASTERCMP1);
        PowerStage_ConfigOutput(HRTIM_TIMERINDEX_TIMER_C,
                                HRTIM_OUTPUT_TC2,
                                HRTIM_OUTPUTSET_MASTERCMP1,
                                HRTIM_OUTPUTRESET_MASTERCMP2);
        (void)HAL_HRTIM_WaveformSetOutputLevel(ps.hhrtim,
                                               HRTIM_TIMERINDEX_TIMER_C,
                                               HRTIM_OUTPUT_TC1,
                                               HRTIM_OUTPUTLEVEL_ACTIVE);
        (void)HAL_HRTIM_WaveformSetOutputLevel(ps.hhrtim,
                                               HRTIM_TIMERINDEX_TIMER_C,
                                               HRTIM_OUTPUT_TC2,
                                               HRTIM_OUTPUTLEVEL_INACTIVE);
    } else {
        /*
         * Static-high bez pseudo PWM: TC1 ustawiany na PER i bez resetu.
         * TC2 pozostaje stale nieaktywny.
         */
        PowerStage_ConfigOutput(HRTIM_TIMERINDEX_TIMER_C,
                                HRTIM_OUTPUT_TC1,
                                HRTIM_OUTPUTSET_TIMPER,
                                HRTIM_OUTPUTRESET_NONE);
        PowerStage_ConfigOutput(HRTIM_TIMERINDEX_TIMER_C,
                                HRTIM_OUTPUT_TC2,
                                HRTIM_OUTPUTSET_NONE,
                                HRTIM_OUTPUTRESET_TIMPER);
        (void)HAL_HRTIM_WaveformSetOutputLevel(ps.hhrtim,
                                               HRTIM_TIMERINDEX_TIMER_C,
                                               HRTIM_OUTPUT_TC1,
                                               HRTIM_OUTPUTLEVEL_ACTIVE);
        (void)HAL_HRTIM_WaveformSetOutputLevel(ps.hhrtim,
                                               HRTIM_TIMERINDEX_TIMER_C,
                                               HRTIM_OUTPUT_TC2,
                                               HRTIM_OUTPUTLEVEL_INACTIVE);
    }
}

static void PowerStage_ApplyBaseTiming(void)
{
    uint32_t period;

    if ((ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return;
    }

    period = PowerStage_PeriodFromFsw();
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
    ps.output_mode = POWER_STAGE_OUTPUT_NONE;
    ps.duty_a = 0.0f;
    ps.duty_c = 0.0f;
    ps.duty_a_10k = 0U;
    ps.duty_b_10k = 0U;
    ps.duty_c_cmd_10k = 0U;
    ps.duty_c_phys_10k = 0U;
    ps.tc1_expected_10k = 0U;
    ps.tc2_expected_10k = 0U;
    ps.adc_trigger_10k = POWER_STAGE_ADC_TRIGGER_10K;
    ps.discharge_pulse_ticks = 0U;
    ps.discharge_every_periods = 0U;
    ps.refresh_period_ticks = 0U;
    ps.refresh_pulse_ticks = 0U;
    ps.last_error = POWER_STAGE_ERR_NONE;
    ps.enabled = false;
    ps.initialized = true;
    ps.discharge_active = false;
    ps.refresh_master_active = false;
    ps.refresh_a_active = false;
    ps.refresh_c_active = false;

    /* FLT ma byc wejsciem open-drain z zewnetrznym pull-up. */
    gpio_cfg.Pin = FLT_Pin;
    gpio_cfg.Mode = GPIO_MODE_INPUT;
    gpio_cfg.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(FLT_GPIO_Port, &gpio_cfg);

    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SD_GPIO_Port, SD_Pin, GPIO_PIN_RESET);

    PowerStage_ConfigureComplementaryOutputs();
    ps.output_mode = POWER_STAGE_OUTPUT_NONE;
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
    ps.output_mode = POWER_STAGE_OUTPUT_NONE;
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
    ps.output_mode = POWER_STAGE_OUTPUT_NONE;
    ps.duty_c_cmd_10k = 0U;
    ps.duty_c_phys_10k = 0U;
    ps.tc1_expected_10k = 0U;
    ps.tc2_expected_10k = 0U;
    ps.refresh_a_active = false;
    ps.refresh_c_active = false;
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
    ps.output_mode = POWER_STAGE_OUTPUT_NONE;
    ps.duty_c_cmd_10k = 0U;
    ps.duty_c_phys_10k = 0U;
    ps.tc1_expected_10k = 0U;
    ps.tc2_expected_10k = 0U;
    ps.refresh_a_active = false;
    ps.refresh_c_active = false;
    ps.last_error = POWER_STAGE_ERR_NONE;
}

void PowerStage_ForceSafeState(void)
{
    PowerStage_Disable();
}

void PowerStage_SetDuty10k(uint32_t duty_a_10k, uint32_t duty_b_10k)
{
    PowerStage_OutputMode_t desired_mode;
    bool buck_refresh = false;
    bool boost_refresh = false;

    if ((!ps.initialized) || (ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return;
    }

    duty_a_10k = PowerStage_Clamp10k(duty_a_10k);
    duty_b_10k = PowerStage_Clamp10k(duty_b_10k);

    ps.discharge_active = false;

    switch (ps.region) {
        case POWER_REGION_BUCK:
            desired_mode = POWER_STAGE_OUTPUT_BUCK;
            buck_refresh = PowerStage_IsBuckRefreshEnabled();

            ps.duty_a_10k = duty_a_10k;
            ps.duty_b_10k = 0U;
            ps.duty_c_cmd_10k = 0U;
            ps.duty_c_phys_10k = 0U;
            ps.tc1_expected_10k = POWER_STAGE_DUTY_SCALE;
            ps.tc2_expected_10k = 0U;
            ps.duty_a = PowerStage_10kToFloat(duty_a_10k);
            ps.duty_c = 0.0f;

            if ((ps.output_mode != desired_mode) ||
                (ps.refresh_c_active != buck_refresh) ||
                ps.refresh_a_active) {
                PowerStage_ConfigHalfBridgeA_Pwm(ps.duty_a);
                PowerStage_ConfigHalfBridgeC_StaticHigh(buck_refresh);
            } else {
                ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR =
                    PowerStage_DutyAToCmp(duty_a_10k);
            }

            ps.refresh_a_active = false;
            ps.refresh_c_active = buck_refresh;
            PowerStage_RefreshMasterSync(buck_refresh);
            break;

        case POWER_REGION_BOOST:
            desired_mode = POWER_STAGE_OUTPUT_BOOST;
            boost_refresh = PowerStage_IsBoostRefreshEnabled();

            ps.duty_a_10k = POWER_STAGE_DUTY_SCALE;
            ps.duty_b_10k = duty_b_10k;
            ps.duty_c_cmd_10k = duty_b_10k;
            ps.duty_c_phys_10k = duty_b_10k;
            ps.tc1_expected_10k = POWER_STAGE_DUTY_SCALE - duty_b_10k;
            ps.tc2_expected_10k = duty_b_10k;
            ps.duty_a = 1.0f;
            ps.duty_c = PowerStage_10kToFloat(duty_b_10k);

            if ((ps.output_mode != desired_mode) ||
                (ps.refresh_a_active != boost_refresh) ||
                ps.refresh_c_active) {
                /* Pure BOOST is kept only for diagnostics. */
                PowerStage_ConfigHalfBridgeA_StaticHigh(boost_refresh);
                PowerStage_ConfigHalfBridgeC_Pwm(ps.duty_c);
            } else {
                ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR =
                    PowerStage_DutyBToCmp(duty_b_10k);
            }

            ps.refresh_a_active = boost_refresh;
            ps.refresh_c_active = false;
            PowerStage_RefreshMasterSync(boost_refresh);
            break;

        case POWER_REGION_BUCK_BOOST:
        default:
            desired_mode = POWER_STAGE_OUTPUT_BUCK_BOOST;

            ps.duty_a_10k = duty_a_10k;
            ps.duty_b_10k = duty_b_10k;
            ps.duty_c_cmd_10k = duty_b_10k;
            ps.duty_c_phys_10k = duty_b_10k;
            ps.tc1_expected_10k = POWER_STAGE_DUTY_SCALE - duty_b_10k;
            ps.tc2_expected_10k = duty_b_10k;
            ps.duty_a = PowerStage_10kToFloat(duty_a_10k);
            ps.duty_c = PowerStage_10kToFloat(duty_b_10k);

            /* BUCK_BOOST: klasyczne PWM na obu pol-mostkach, bez global refresh. */
            if ((ps.output_mode != desired_mode) ||
                ps.refresh_a_active ||
                ps.refresh_c_active) {
                PowerStage_ConfigHalfBridgeA_Pwm(ps.duty_a);
                PowerStage_ConfigHalfBridgeC_Pwm(ps.duty_c);
            } else {
                ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR =
                    PowerStage_DutyAToCmp(duty_a_10k);
                ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR =
                    PowerStage_DutyBToCmp(duty_b_10k);
            }

            ps.refresh_a_active = false;
            ps.refresh_c_active = false;
            PowerStage_RefreshMasterSync(false);
            break;
    }

    ps.output_mode = desired_mode;

    (void)HAL_HRTIM_SoftwareUpdate(ps.hhrtim,
                                   HRTIM_TIMERUPDATE_A | HRTIM_TIMERUPDATE_C);

    if ((ps.output_mode == POWER_STAGE_OUTPUT_BOOST) ||
        (ps.output_mode == POWER_STAGE_OUTPUT_BUCK_BOOST)) {
        uint32_t cmp_c = ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR;

        /* Telemetria: fizyczny duty TC2/LIN_C wyznaczony z CMP1. */
        ps.duty_c_phys_10k = PowerStage_CmpToDutyTc2_10k(cmp_c);
    } else if (ps.output_mode == POWER_STAGE_OUTPUT_BUCK) {
        ps.duty_c_phys_10k = 0U;
    }

    ps.tc2_expected_10k = ps.duty_c_phys_10k;
    ps.tc1_expected_10k = POWER_STAGE_DUTY_SCALE - ps.tc2_expected_10k;
    ps.duty_c = PowerStage_10kToFloat(ps.duty_c_phys_10k);
}

void PowerStage_SetBuckDischarge(uint32_t pulse_ns, uint32_t every_periods)
{
    HRTIM_OutputCfgTypeDef out_cfg = {0};
    HRTIM_BurstModeCfgTypeDef burst_cfg = {0};
    uint32_t pulse_ticks;
    uint32_t period;

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

    period = PowerStage_PeriodTicksRaw();

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

    (void)HAL_HRTIM_WaveformOutputConfig(ps.hhrtim,
                                         HRTIM_TIMERINDEX_TIMER_A,
                                         HRTIM_OUTPUT_TA1,
                                         &out_cfg);
    (void)HAL_HRTIM_WaveformSetOutputLevel(ps.hhrtim,
                                           HRTIM_TIMERINDEX_TIMER_A,
                                           HRTIM_OUTPUT_TA1,
                                           HRTIM_OUTPUTLEVEL_INACTIVE);

    out_cfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
    out_cfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
    (void)HAL_HRTIM_WaveformOutputConfig(ps.hhrtim,
                                         HRTIM_TIMERINDEX_TIMER_A,
                                         HRTIM_OUTPUT_TA2,
                                         &out_cfg);

    PowerStage_ConfigHalfBridgeC_StaticHigh(PowerStage_IsBuckRefreshEnabled());
    PowerStage_RefreshMasterSync(PowerStage_IsBuckRefreshEnabled());

    ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = pulse_ticks;
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
    ps.duty_c_cmd_10k = 0U;
    ps.duty_c_phys_10k = 0U;
    ps.tc1_expected_10k = POWER_STAGE_DUTY_SCALE;
    ps.tc2_expected_10k = 0U;
    ps.duty_a = 0.0f;
    ps.duty_c = 0.0f;
    ps.discharge_pulse_ticks = pulse_ticks;
    ps.discharge_every_periods = every_periods;
    ps.enabled = true;
    ps.discharge_active = true;
    ps.output_mode = POWER_STAGE_OUTPUT_DISCHARGE;
    ps.last_error = POWER_STAGE_ERR_NONE;
}

void PowerStage_SetDuty(float duty_a, float duty_c)
{
    PowerStage_SetDuty10k(PowerStage_FloatTo10k(duty_a),
                          PowerStage_FloatTo10k(duty_c));
}

void PowerStage_SetBuckDuty(float duty_a)
{
    ps.region = POWER_REGION_BUCK;
    PowerStage_SetDuty(duty_a, 0.0f);
}

void PowerStage_SetBoostDuty(float duty_b)
{
    /* Diagnostic-only API for pure BOOST experiments. */
    ps.region = POWER_REGION_BOOST;
    PowerStage_SetDuty(1.0f, duty_b);
}

void PowerStage_SetBuckBoostDuty(float duty_a, float duty_b)
{
    ps.region = POWER_REGION_BUCK_BOOST;
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

    period = PowerStage_PeriodTicksRaw();
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

uint32_t PowerStage_GetDutyA10k(void)
{
    return ps.duty_a_10k;
}

uint32_t PowerStage_GetDutyCCmd10k(void)
{
    return ps.duty_c_cmd_10k;
}

uint32_t PowerStage_GetDutyC10k(void)
{
    return ps.duty_c_phys_10k;
}

uint32_t PowerStage_GetDutyCPhys10k(void)
{
    return ps.duty_c_phys_10k;
}

uint32_t PowerStage_GetExpectedTc1Duty10k(void)
{
    return ps.tc1_expected_10k;
}

uint32_t PowerStage_GetExpectedTc2Duty10k(void)
{
    return ps.tc2_expected_10k;
}

uint32_t PowerStage_GetCmpA(void)
{
    if ((ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return 0U;
    }

    return ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR;
}

uint32_t PowerStage_GetCmpC(void)
{
    if ((ps.hhrtim == NULL) || (ps.hhrtim->Instance == NULL)) {
        return 0U;
    }

    return ps.hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR;
}

float PowerStage_GetPwmStepPercent(void)
{
    uint32_t period = PowerStage_PeriodTicksRaw();

    if (period == 0U) {
        return 0.0f;
    }

    return 100.0f / (float)period;
}
