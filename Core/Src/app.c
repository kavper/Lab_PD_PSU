#include "app.h"

#include "control_cv.h"
#include "debug_uart.h"
#include "measurements.h"
#include "power_manager.h"
#include "power_stage.h"

#include <stdlib.h>
#include <string.h>

#define APP_CTRL_FREQ_HZ                     4000U
#define APP_CTRL_PERIOD_US                   (1000000U / APP_CTRL_FREQ_HZ)
/* UART debug print period is set here. */
#define APP_DEBUG_PERIOD_MS                  200U
#define APP_DEBUG_VERBOSE                    0U
#define APP_STARTUP_HOLD_MS                  2000U
#define APP_LED_BLINK_MS                     250U
#define APP_MAX_POWER_W                      100U

#define VOUT_SETPOINT_DEFAULT_V              10.00f
#define VOUT_OVP_LIMIT                       35.00f
#define IOUT_OCP_LIMIT                       4.00f
#define IOUT_LIMIT_MAX                       5.80f
#define VIN_UVLO_LIMIT                       7.00f

#define DUTY_MIN_ABS                         0.000f
#define DUTY_MAX_ABS                         1.000f

#define DUTY_BUCK_MIN                        0.020f
#define DUTY_BUCK_MAX                        0.950f

/* BUCK_BOOST mixed-mode duty limits are set here. */
#define DUTY_MIXED_A                         0.80f
#define DUTY_MIXED_B_MIN                     0.03f
#define DUTY_MIXED_B_MAX                     0.80f
#define BUCK_BOOST_DUTY_A_BASE               DUTY_MIXED_A
#define BUCK_BOOST_DUTY_C_MIN                DUTY_MIXED_B_MIN
#define BUCK_BOOST_DUTY_C_MAX                DUTY_MIXED_B_MAX
#define BUCK_BOOST_DUTY_C_INIT_MIN           0.05f
#define BUCK_BOOST_DUTY_C_INIT_MAX           0.30f
#define BUCK_BOOST_DUTY_C_SLEW_PER_CTRL      0.0008f
#define BUCK_BOOST_DUTY_C_SOFTSTART_MS       700U
#define BUCK_BOOST_CTRL_GAIN_NEAR_VIN        0.55f
#define BUCK_BOOST_CTRL_GAIN_ABOVE_VIN       0.80f

#define DUTY_BOOST_MIN                       0.050f
#define DUTY_BOOST_MAX                       0.900f
#define DUTY_STARTUP_MIN                     0.030f
#define DUTY_STARTUP_MAX                     0.100f

#ifndef BOOST_DUTY_MAX_BRINGUP
#define BOOST_DUTY_MAX_BRINGUP               DUTY_MIXED_B_MAX
#endif

#ifndef POWER_STAGE_TEST_BOOST_PWM_FIXED
#define POWER_STAGE_TEST_BOOST_PWM_FIXED     0U
#endif

#ifndef POWER_STAGE_TEST_BOOST_STEP_MS
#define POWER_STAGE_TEST_BOOST_STEP_MS       1000U
#endif

#define CV_KP                                0.040f
#define CV_KI                                18.0f
#define CV_SLEW_UP_V_PER_S                   25.0f
#define CV_SLEW_DOWN_V_PER_S                 25.0f

/* Low-setpoint startup threshold and first-duty step are set here. */
#define CV_LOW_SETPOINT_START_THRESHOLD_V    2.0f
#define CV_LOW_SETPOINT_START_DUTY           0.02f
#define CV_LOW_SETPOINT_START_TIME_MS        10U

#define CV_2P2Z_B0                           0.0500f
#define CV_2P2Z_B1                           0.0000f
#define CV_2P2Z_B2                           0.0000f
#define CV_2P2Z_A1                           0.0000f
#define CV_2P2Z_A2                           0.0000f

#define REGION_BUCK_ENTER_MARGIN_V           1.50f
#define REGION_BUCK_EXIT_MARGIN_V            0.80f
#define REGION_SWITCH_CONFIRM_COUNT          8U
#define REGION_MIN_DWELL_MS                  200U

#define OCP_ACTIVE_MIN_LIMIT_A               0.05f
#define OCP_HIT_COUNT_LIMIT                  20U

#define OFF_RAMP_DONE_V                      0.020f
#define OFF_DISABLE_HOLD_TICKS               80U
#define CTRL_DT_SANITY_MAX_US                100000U

typedef struct {
    Measurements_t meas;
    ControlCv_t cv;
    Control2p2z_t cv_2p2z;
    TIM_HandleTypeDef ctrl_tim;

    App_Mode_t requested_mode;
    App_Mode_t active_mode;

    uint32_t fault_flags;
    uint32_t latched_fault_flags;

    uint32_t startup_tick_ms;
    uint32_t last_debug_tick_ms;
    uint32_t last_led_tick_ms;

    uint32_t last_ctrl_tick_us;
    uint32_t adc_last_update_us;
    uint32_t last_adc_dma_updates;

    uint32_t ctrl_tick;
    uint32_t ctrl_overrun;
    uint32_t ctrl_dt_us;
    uint32_t ctrl_dt_max_us;
    uint32_t adc_age_us;
    uint32_t pwm_update_cnt;
    uint32_t region_trans_cnt;
    uint32_t timebase_glitch_cnt;

    uint32_t ocp_hit_count;
    uint32_t mode_top_hits;
    uint32_t mode_bottom_hits;

    uint32_t off_ramp_done_ticks;
    uint32_t region_hold_until_ms;
    uint32_t buck_boost_softstart_tick_ms;
    uint32_t low_setpoint_start_until_ms;

    PowerStage_Region_t region_candidate;
    uint32_t region_candidate_count;
    uint32_t region_last_confirm_count;

    PowerStage_Region_t region_old_debug;
    PowerStage_Region_t region_new_debug;
    uint8_t region_trans_debug;

    float duty_ff_a;
    float duty_ff_c;
    float duty_cmd_a;
    float duty_cmd_c;
    float buck_boost_softstart_start_duty_c;
    float buck_boost_duty_c_cap;
    uint8_t sat_hi_debug;
    uint8_t sat_lo_debug;

    float cv_user_setpoint;
    float current_limit_a;

    bool stage_enabled;
    bool pending_disable_request;
    bool ctrl_timer_ready;
    bool timebase_ready;
    bool fast_loop_running;
    bool timebase_glitch;
    bool led_blink_state;
    bool low_setpoint_start_active;
} App_Context_t;

static App_Context_t app;

static float App_Clamp(float value, float min_value, float max_value)
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

static float App_MaxFloat(float a, float b)
{
    return (a > b) ? a : b;
}

static float App_SlewLimit(float current, float target, float max_step)
{
    float delta;

    if (max_step <= 0.0f) {
        return target;
    }

    delta = target - current;
    if (delta > max_step) {
        delta = max_step;
    } else if (delta < -max_step) {
        delta = -max_step;
    }

    return current + delta;
}

static int32_t App_ToFixed(float value, int32_t scale)
{
    float scaled = value * (float)scale;
    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }
    return (int32_t)scaled;
}

static long App_IntPart(int32_t fixed, int32_t scale)
{
    return (long)(labs((long)fixed) / (long)scale);
}

static long App_FracPart(int32_t fixed, int32_t scale)
{
    return (long)(labs((long)fixed) % (long)scale);
}

static const char *App_ModeText(App_Mode_t mode)
{
    switch (mode) {
        case MODE_CV:
            return "CV";
        case MODE_CC:
            return "CC";
        case MODE_IDLE:
        default:
            return "IDLE";
    }
}

static const char *App_RegionText(PowerStage_Region_t region)
{
    switch (region) {
        case POWER_REGION_BUCK:
            return "BUCK";
        case POWER_REGION_BOOST:
            return "BOOST";
        case POWER_REGION_BUCK_BOOST:
        default:
            return "BUCK_BOOST";
    }
}

static void App_FaultText(uint32_t faults, char *text, size_t text_len)
{
    if ((text == NULL) || (text_len == 0U)) {
        return;
    }

    text[0] = '\0';

    if (faults == FAULT_NONE) {
        (void)strncpy(text, "NONE", text_len - 1U);
        text[text_len - 1U] = '\0';
        return;
    }

    if ((faults & FAULT_DRIVER) != 0U) {
        (void)strncat(text, "DRIVER|", text_len - strlen(text) - 1U);
    }
    if ((faults & FAULT_OVP) != 0U) {
        (void)strncat(text, "OVP|", text_len - strlen(text) - 1U);
    }
    if ((faults & FAULT_OCP) != 0U) {
        (void)strncat(text, "OCP|", text_len - strlen(text) - 1U);
    }
    if ((faults & FAULT_UVIN) != 0U) {
        (void)strncat(text, "UVIN|", text_len - strlen(text) - 1U);
    }
    if ((faults & FAULT_ADC) != 0U) {
        (void)strncat(text, "ADC|", text_len - strlen(text) - 1U);
    }

    if (strlen(text) > 0U) {
        text[strlen(text) - 1U] = '\0';
    }
}

static void App_TimebaseInit(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    app.timebase_ready = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);
}

static uint32_t App_Micros(void)
{
    if (app.timebase_ready) {
        uint32_t hclk_mhz = SystemCoreClock / 1000000U;

        if (hclk_mhz == 0U) {
            hclk_mhz = 1U;
        }

        return DWT->CYCCNT / hclk_mhz;
    }

    return HAL_GetTick() * 1000U;
}

static uint32_t App_StartupHoldRemainingMs(void)
{
    uint32_t elapsed_ms = HAL_GetTick() - app.startup_tick_ms;

    if (elapsed_ms >= APP_STARTUP_HOLD_MS) {
        return 0U;
    }

    return APP_STARTUP_HOLD_MS - elapsed_ms;
}

static void App_LedTask(void)
{
    uint32_t now_ms = HAL_GetTick();

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

    if ((uint32_t)(now_ms - app.last_led_tick_ms) >= APP_LED_BLINK_MS) {
        app.last_led_tick_ms = now_ms;
        app.led_blink_state = !app.led_blink_state;
        HAL_GPIO_WritePin(LED2_GPIO_Port,
                          LED2_Pin,
                          app.led_blink_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

static bool App_IsDriverAwake(void)
{
    return (HAL_GPIO_ReadPin(STBY_GPIO_Port, STBY_Pin) == GPIO_PIN_SET) &&
           (HAL_GPIO_ReadPin(SD_GPIO_Port, SD_Pin) == GPIO_PIN_SET);
}

static bool App_IsZeroSetpointTarget(void)
{
    return app.cv_user_setpoint <= OFF_RAMP_DONE_V;
}

static void App_GetDutyLimitsForRegion(PowerStage_Region_t region,
                                       float *min_duty,
                                       float *max_duty)
{
    float min_value = DUTY_BUCK_MIN;
    float max_value = DUTY_BUCK_MAX;

    if (region == POWER_REGION_BOOST) {
        min_value = DUTY_BOOST_MIN;
        max_value = App_Clamp(BOOST_DUTY_MAX_BRINGUP, DUTY_BOOST_MIN, DUTY_BOOST_MAX);
    } else if (region == POWER_REGION_BUCK_BOOST) {
        min_value = BUCK_BOOST_DUTY_C_MIN;
        max_value = App_Clamp(app.buck_boost_duty_c_cap,
                              BUCK_BOOST_DUTY_C_MIN,
                              BUCK_BOOST_DUTY_C_MAX);
    }

    if ((region == POWER_REGION_BUCK) && App_IsZeroSetpointTarget()) {
        min_value = 0.0f;
    }

    if (min_duty != NULL) {
        *min_duty = min_value;
    }
    if (max_duty != NULL) {
        *max_duty = max_value;
    }
}

static void App_SetCvLimitsForRegion(PowerStage_Region_t region)
{
    App_GetDutyLimitsForRegion(region, &app.cv.duty_min, &app.cv.duty_max);
}

static void App_ArmBuckBoostSoftstart(float start_duty_c)
{
    app.buck_boost_softstart_tick_ms = HAL_GetTick();
    app.buck_boost_softstart_start_duty_c = App_Clamp(start_duty_c,
                                                      BUCK_BOOST_DUTY_C_INIT_MIN,
                                                      BUCK_BOOST_DUTY_C_INIT_MAX);
    app.buck_boost_duty_c_cap = app.buck_boost_softstart_start_duty_c;
}

static void App_UpdateBuckBoostSoftstart(PowerStage_Region_t region)
{
    uint32_t elapsed_ms;
    float start_duty_c;
    float progress;

    if (region != POWER_REGION_BUCK_BOOST) {
        app.buck_boost_duty_c_cap = BUCK_BOOST_DUTY_C_MAX;
        return;
    }

    elapsed_ms = HAL_GetTick() - app.buck_boost_softstart_tick_ms;
    if (elapsed_ms >= BUCK_BOOST_DUTY_C_SOFTSTART_MS) {
        app.buck_boost_duty_c_cap = BUCK_BOOST_DUTY_C_MAX;
        return;
    }

    start_duty_c = App_Clamp(app.buck_boost_softstart_start_duty_c,
                             BUCK_BOOST_DUTY_C_MIN,
                             BUCK_BOOST_DUTY_C_MAX);
    progress = (float)elapsed_ms / (float)BUCK_BOOST_DUTY_C_SOFTSTART_MS;
    app.buck_boost_duty_c_cap = start_duty_c +
                                ((BUCK_BOOST_DUTY_C_MAX - start_duty_c) * progress);
    app.buck_boost_duty_c_cap = App_Clamp(app.buck_boost_duty_c_cap,
                                          BUCK_BOOST_DUTY_C_MIN,
                                          BUCK_BOOST_DUTY_C_MAX);
}

static bool App_IsLowSetpointTarget(void)
{
    if (App_IsZeroSetpointTarget()) {
        return false;
    }

    return app.cv_user_setpoint < CV_LOW_SETPOINT_START_THRESHOLD_V;
}

static bool App_IsLowSetpointStartupActive(void)
{
    if (!app.low_setpoint_start_active) {
        return false;
    }

    if ((int32_t)(HAL_GetTick() - app.low_setpoint_start_until_ms) >= 0) {
        app.low_setpoint_start_active = false;
        return false;
    }

    return true;
}

static bool App_ShouldForceLowSetpointBuck(float ramped_setpoint_v)
{
    if (App_IsZeroSetpointTarget()) {
        return true;
    }

    if (!App_IsLowSetpointTarget()) {
        return false;
    }

    return App_IsLowSetpointStartupActive() ||
           (!app.stage_enabled) ||
           (ramped_setpoint_v < CV_LOW_SETPOINT_START_THRESHOLD_V);
}

static PowerStage_Region_t App_SelectRegionBase(float vin, float setpoint_v)
{
    if (setpoint_v < CV_LOW_SETPOINT_START_THRESHOLD_V) {
        return POWER_REGION_BUCK;
    }

    if (setpoint_v < (vin - REGION_BUCK_ENTER_MARGIN_V)) {
        return POWER_REGION_BUCK;
    }

    return POWER_REGION_BUCK_BOOST;
}

static PowerStage_Region_t App_SelectRegionWithHysteresis(PowerStage_Region_t current,
                                                           float vin,
                                                           float setpoint_v)
{
    float buck_enter_threshold = vin - REGION_BUCK_ENTER_MARGIN_V;
    float buck_exit_threshold = vin - REGION_BUCK_EXIT_MARGIN_V;

    switch (current) {
        case POWER_REGION_BUCK:
            if (setpoint_v >= buck_exit_threshold) {
                return POWER_REGION_BUCK_BOOST;
            }
            return POWER_REGION_BUCK;

        case POWER_REGION_BOOST:
            return POWER_REGION_BUCK_BOOST;

        case POWER_REGION_BUCK_BOOST:
        default:
            if (setpoint_v <= buck_enter_threshold) {
                return POWER_REGION_BUCK;
            }
            return POWER_REGION_BUCK_BOOST;
    }
}

static PowerStage_Region_t App_LimitRegionStep(PowerStage_Region_t current,
                                               PowerStage_Region_t candidate)
{
    if (candidate == POWER_REGION_BOOST) {
        return POWER_REGION_BUCK_BOOST;
    }

    if ((current == POWER_REGION_BOOST) && (candidate == POWER_REGION_BUCK)) {
        return POWER_REGION_BUCK_BOOST;
    }

    return candidate;
}

static void App_EstimateDutyForRegion(PowerStage_Region_t region,
                                      float vin,
                                      float setpoint_v,
                                      float *duty_a,
                                      float *duty_c)
{
    float local_a = 0.0f;
    float local_c = 0.0f;
    float effective_setpoint_v;
    float denom;

    vin = App_MaxFloat(vin, 0.10f);
    effective_setpoint_v = App_MaxFloat(setpoint_v, 0.01f);

    switch (region) {
        case POWER_REGION_BUCK:
            if (setpoint_v <= OFF_RAMP_DONE_V) {
                local_a = 0.0f;
            } else {
                local_a = App_Clamp(effective_setpoint_v / vin, DUTY_BUCK_MIN, DUTY_BUCK_MAX);
            }
            local_c = 0.0f;
            break;

        case POWER_REGION_BOOST:
            local_a = 1.0f;
            denom = App_MaxFloat(effective_setpoint_v, vin + 0.10f);
            local_c = 1.0f - (vin / denom);
            local_c = App_Clamp(local_c,
                                DUTY_BOOST_MIN,
                                App_Clamp(BOOST_DUTY_MAX_BRINGUP, DUTY_BOOST_MIN, DUTY_BOOST_MAX));
            break;

        case POWER_REGION_BUCK_BOOST:
        default:
            local_a = BUCK_BOOST_DUTY_A_BASE;
            denom = App_MaxFloat(effective_setpoint_v, (BUCK_BOOST_DUTY_A_BASE * vin) + 0.10f);
            local_c = 1.0f - ((BUCK_BOOST_DUTY_A_BASE * vin) / denom);
            local_c = App_Clamp(local_c, BUCK_BOOST_DUTY_C_MIN, BUCK_BOOST_DUTY_C_MAX);
            break;
    }

    if (duty_a != NULL) {
        *duty_a = local_a;
    }
    if (duty_c != NULL) {
        *duty_c = local_c;
    }
}

static float App_GetRegionControlFeedForward(PowerStage_Region_t region,
                                             float duty_ff_a,
                                             float duty_ff_c)
{
    if (region == POWER_REGION_BUCK) {
        return duty_ff_a;
    }
    return duty_ff_c;
}

static void App_ApplyDuty(PowerStage_Region_t region,
                          float duty_cmd_a,
                          float duty_cmd_c)
{
    float adc_trigger;

    duty_cmd_a = App_Clamp(duty_cmd_a, DUTY_MIN_ABS, DUTY_MAX_ABS);
    duty_cmd_c = App_Clamp(duty_cmd_c, DUTY_MIN_ABS, DUTY_MAX_ABS);

    PowerStage_SetRegion(region);

    switch (region) {
        case POWER_REGION_BUCK:
            PowerStage_SetDuty(duty_cmd_a, 0.0f);
            adc_trigger = App_Clamp(duty_cmd_a + 0.20f, 0.15f, 0.85f);
            PowerStage_SetAdcTriggerPoint(adc_trigger);
            break;

        case POWER_REGION_BOOST:
            /* Diagnostic-only path: not selected by automatic CV region logic. */
            PowerStage_SetDuty(1.0f, duty_cmd_c);
            adc_trigger = App_Clamp(duty_cmd_c + 0.20f, 0.15f, 0.85f);
            PowerStage_SetAdcTriggerPoint(adc_trigger);
            break;

        case POWER_REGION_BUCK_BOOST:
        default:
            PowerStage_SetDuty(duty_cmd_a, duty_cmd_c);
            PowerStage_SetAdcTriggerPoint(0.60f);
            break;
    }

    app.duty_cmd_a = duty_cmd_a;
    app.duty_cmd_c = duty_cmd_c;
    app.pwm_update_cnt++;
}

static void App_OnRegionChange(PowerStage_Region_t old_region,
                               PowerStage_Region_t new_region,
                               float vin,
                               float setpoint_v)
{
    float duty_ff_a;
    float duty_ff_c;
    float control_ff;

    if (old_region == new_region) {
        return;
    }

    App_EstimateDutyForRegion(new_region, vin, setpoint_v, &duty_ff_a, &duty_ff_c);
    if ((new_region == POWER_REGION_BUCK_BOOST) && (old_region == POWER_REGION_BUCK)) {
        duty_ff_c = App_Clamp(duty_ff_c,
                              BUCK_BOOST_DUTY_C_INIT_MIN,
                              BUCK_BOOST_DUTY_C_INIT_MAX);
    }

    if (new_region == POWER_REGION_BUCK_BOOST) {
        App_ArmBuckBoostSoftstart(duty_ff_c);
    } else {
        app.buck_boost_duty_c_cap = BUCK_BOOST_DUTY_C_MAX;
    }

    control_ff = App_GetRegionControlFeedForward(new_region, duty_ff_a, duty_ff_c);

    app.region_old_debug = old_region;
    app.region_new_debug = new_region;
    app.region_trans_debug = 1U;
    app.region_trans_cnt++;
    app.region_hold_until_ms = HAL_GetTick() + REGION_MIN_DWELL_MS;

    app.duty_ff_a = duty_ff_a;
    app.duty_ff_c = duty_ff_c;

    App_SetCvLimitsForRegion(new_region);
    ControlCv_Reset(&app.cv, control_ff);
#if (CONTROL_CV_USE_2P2Z != 0)
    Control2p2z_Reset(&app.cv_2p2z, control_ff);
#endif

    App_ApplyDuty(new_region, duty_ff_a, duty_ff_c);
}

static PowerStage_Region_t App_UpdateRegionHysteresis(float vin,
                                                       float setpoint_v)
{
    PowerStage_Region_t current = PowerStage_GetRegion();
    PowerStage_Region_t candidate;
    uint32_t now_ms = HAL_GetTick();

    if (App_ShouldForceLowSetpointBuck(setpoint_v)) {
        candidate = POWER_REGION_BUCK;
    } else {
        candidate = App_SelectRegionWithHysteresis(current, vin, setpoint_v);
    }

    if ((candidate == POWER_REGION_BUCK) &&
        App_ShouldForceLowSetpointBuck(setpoint_v) &&
        (current != POWER_REGION_BUCK)) {
        App_OnRegionChange(current, POWER_REGION_BUCK, vin, setpoint_v);
        app.region_candidate = POWER_REGION_BUCK;
        app.region_candidate_count = 0U;
        return POWER_REGION_BUCK;
    }

    if (current == POWER_REGION_BOOST) {
        App_OnRegionChange(current, POWER_REGION_BUCK_BOOST, vin, setpoint_v);
        app.region_candidate = POWER_REGION_BUCK_BOOST;
        app.region_candidate_count = 0U;
        return POWER_REGION_BUCK_BOOST;
    }

    candidate = App_LimitRegionStep(current, candidate);

    if ((candidate != current) &&
        ((int32_t)(now_ms - app.region_hold_until_ms) < 0)) {
        return current;
    }

    if (candidate == current) {
        app.region_candidate = current;
        app.region_candidate_count = 0U;
        return current;
    }

    if (candidate != app.region_candidate) {
        app.region_candidate = candidate;
        app.region_candidate_count = 1U;
    } else if (app.region_candidate_count < REGION_SWITCH_CONFIRM_COUNT) {
        app.region_candidate_count++;
    }

    if (app.region_candidate_count >= REGION_SWITCH_CONFIRM_COUNT) {
        app.region_last_confirm_count = app.region_candidate_count;
        App_OnRegionChange(current,
                           candidate,
                           app.meas.vin,
                           ControlCv_GetRampedSetpoint(&app.cv));
        app.region_candidate_count = 0U;
        return candidate;
    }

    return current;
}

static void App_SaturateForRegion(PowerStage_Region_t region,
                                  float ctrl_cmd,
                                  float *duty_cmd_a,
                                  float *duty_cmd_c,
                                  bool *sat_hi,
                                  bool *sat_lo)
{
    float min_duty;
    float max_duty;

    if (sat_hi != NULL) {
        *sat_hi = false;
    }
    if (sat_lo != NULL) {
        *sat_lo = false;
    }

    App_GetDutyLimitsForRegion(region, &min_duty, &max_duty);

    if (region == POWER_REGION_BUCK) {
        float duty_a = ctrl_cmd;

        if (duty_a > max_duty) {
            duty_a = max_duty;
            if (sat_hi != NULL) {
                *sat_hi = true;
            }
        } else if (duty_a < min_duty) {
            duty_a = min_duty;
            if (sat_lo != NULL) {
                *sat_lo = true;
            }
        }

        if (duty_cmd_a != NULL) {
            *duty_cmd_a = duty_a;
        }
        if (duty_cmd_c != NULL) {
            *duty_cmd_c = 0.0f;
        }
        return;
    }

    if (region == POWER_REGION_BOOST) {
        /* Pure BOOST remains diagnostic-only; not selected in normal CV operation. */
        float duty_c = ctrl_cmd;

        if (duty_c > max_duty) {
            duty_c = max_duty;
            if (sat_hi != NULL) {
                *sat_hi = true;
            }
        } else if (duty_c < min_duty) {
            duty_c = min_duty;
            if (sat_lo != NULL) {
                *sat_lo = true;
            }
        }

        if (duty_cmd_a != NULL) {
            *duty_cmd_a = 1.0f;
        }
        if (duty_cmd_c != NULL) {
            *duty_cmd_c = duty_c;
        }
        return;
    }

    {
        float duty_c = ctrl_cmd;

        if (duty_c > max_duty) {
            duty_c = max_duty;
            if (sat_hi != NULL) {
                *sat_hi = true;
            }
        } else if (duty_c < min_duty) {
            duty_c = min_duty;
            if (sat_lo != NULL) {
                *sat_lo = true;
            }
        }

        if (duty_cmd_a != NULL) {
            *duty_cmd_a = BUCK_BOOST_DUTY_A_BASE;
        }
        if (duty_cmd_c != NULL) {
            *duty_cmd_c = duty_c;
        }
    }
}

static void App_FaultShutdown(uint32_t reason)
{
    if (reason == FAULT_NONE) {
        reason = FAULT_DRIVER;
    }

    app.latched_fault_flags |= reason;
    app.fault_flags = app.latched_fault_flags;

    if (app.stage_enabled || PowerStage_IsEnabled() || App_IsDriverAwake()) {
        PowerStage_Disable();
    }

    app.stage_enabled = false;
    app.pending_disable_request = false;
    app.active_mode = MODE_IDLE;
    app.low_setpoint_start_active = false;

    ControlCv_SetTarget(&app.cv, 0.0f);
    app.cv.ramped_setpoint = 0.0f;
    ControlCv_Reset(&app.cv, DUTY_STARTUP_MIN);
#if (CONTROL_CV_USE_2P2Z != 0)
    Control2p2z_Reset(&app.cv_2p2z, DUTY_STARTUP_MIN);
#endif
}

static void App_UpdateFaultFlagsFast(bool adc_ok)
{
    uint32_t flags = app.latched_fault_flags;
    bool protection_active = (app.requested_mode != MODE_IDLE) || app.stage_enabled;

    if (!adc_ok) {
        flags |= FAULT_ADC;
    }

    if ((app.stage_enabled || App_IsDriverAwake()) && PowerStage_IsFaultActive()) {
        flags |= FAULT_DRIVER;
    }

    if (adc_ok && protection_active) {
        if (app.meas.vin < VIN_UVLO_LIMIT) {
            flags |= FAULT_UVIN;
        }
        if (app.meas.vout > VOUT_OVP_LIMIT) {
            flags |= FAULT_OVP;
        }

        if (app.stage_enabled && (app.current_limit_a >= OCP_ACTIVE_MIN_LIMIT_A)) {
            if (app.meas.iout > app.current_limit_a) {
                if (app.ocp_hit_count < OCP_HIT_COUNT_LIMIT) {
                    app.ocp_hit_count++;
                }
                if (app.ocp_hit_count >= OCP_HIT_COUNT_LIMIT) {
                    flags |= FAULT_OCP;
                }
            } else if (app.ocp_hit_count > 0U) {
                app.ocp_hit_count--;
            }
        } else {
            app.ocp_hit_count = 0U;
        }
    }

    app.fault_flags = flags;
}

static void App_RunCvLoopFast(float dt_s)
{
    PowerStage_Region_t region;
    float ctrl_cmd;
    float duty_cmd_a;
    float duty_cmd_c;
    bool sat_hi;
    bool sat_lo;
    float ramped_setpoint;

    ramped_setpoint = ControlCv_GetRampedSetpoint(&app.cv);
    region = App_UpdateRegionHysteresis(app.meas.vin, ramped_setpoint);

    App_UpdateBuckBoostSoftstart(region);
    App_SetCvLimitsForRegion(region);

#if (CONTROL_CV_USE_2P2Z != 0)
    ctrl_cmd = Control2p2z_RunImmediate(&app.cv_2p2z, ramped_setpoint - app.meas.vout);
#else
    ctrl_cmd = ControlCv_RunPreClamp(&app.cv, app.meas.vout, dt_s);
#endif

    if (region == POWER_REGION_BUCK_BOOST) {
        float ff_duty_a = 0.0f;
        float ff_duty_c = 0.0f;
        float ctrl_gain = (ramped_setpoint < app.meas.vin) ?
                          BUCK_BOOST_CTRL_GAIN_NEAR_VIN :
                          BUCK_BOOST_CTRL_GAIN_ABOVE_VIN;

        App_EstimateDutyForRegion(region,
                                  app.meas.vin,
                                  ramped_setpoint,
                                  &ff_duty_a,
                                  &ff_duty_c);
        (void)ff_duty_a;

        app.duty_ff_a = ff_duty_a;
        app.duty_ff_c = ff_duty_c;
        ctrl_cmd = ff_duty_c + ((ctrl_cmd - ff_duty_c) * ctrl_gain);
    }

    App_SaturateForRegion(region,
                          ctrl_cmd,
                          &duty_cmd_a,
                          &duty_cmd_c,
                          &sat_hi,
                          &sat_lo);

    if ((region == POWER_REGION_BOOST) ||
        (region == POWER_REGION_BUCK_BOOST)) {
        duty_cmd_c = App_SlewLimit(app.duty_cmd_c,
                                   duty_cmd_c,
                                   BUCK_BOOST_DUTY_C_SLEW_PER_CTRL);
    }

#if (CONTROL_CV_USE_2P2Z != 0)
    Control2p2z_Commit(&app.cv_2p2z, sat_hi, sat_lo);
#else
    ControlCv_CommitIntegrator(&app.cv, sat_hi, sat_lo);
#endif

    app.sat_hi_debug = sat_hi ? 1U : 0U;
    app.sat_lo_debug = sat_lo ? 1U : 0U;
    app.mode_top_hits = sat_hi ? 1U : 0U;
    app.mode_bottom_hits = sat_lo ? 1U : 0U;

    App_ApplyDuty(region, duty_cmd_a, duty_cmd_c);
}

#if (POWER_STAGE_TEST_BOOST_PWM_FIXED != 0U)
static void App_RunBoostFixedTest(void)
{
    static const float duty_steps[] = {0.10f, 0.30f, 0.50f};
    static uint32_t last_step_ms = 0U;
    static uint32_t step_idx = 0U;
    uint32_t now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - last_step_ms) >= POWER_STAGE_TEST_BOOST_STEP_MS) {
        last_step_ms = now_ms;
        step_idx++;
        if (step_idx >= (sizeof(duty_steps) / sizeof(duty_steps[0]))) {
            step_idx = 0U;
        }
    }

    app.region_old_debug = PowerStage_GetRegion();
    app.region_new_debug = POWER_REGION_BOOST;
    app.region_trans_debug = (app.region_old_debug != POWER_REGION_BOOST) ? 1U : 0U;
    if (app.region_trans_debug != 0U) {
        app.region_trans_cnt++;
    }

    app.duty_ff_a = 1.0f;
    app.duty_ff_c = duty_steps[step_idx];
    app.sat_hi_debug = 0U;
    app.sat_lo_debug = 0U;

    App_ApplyDuty(POWER_REGION_BOOST, 1.0f, duty_steps[step_idx]);
    app.active_mode = MODE_CV;
}
#endif

static void App_CheckDisableCondition(void)
{
    float ramped = ControlCv_GetRampedSetpoint(&app.cv);

    if (ramped <= OFF_RAMP_DONE_V) {
        if (app.off_ramp_done_ticks < OFF_DISABLE_HOLD_TICKS) {
            app.off_ramp_done_ticks++;
        }
    } else {
        app.off_ramp_done_ticks = 0U;
    }

    if (app.off_ramp_done_ticks >= OFF_DISABLE_HOLD_TICKS) {
        app.pending_disable_request = true;
    }
}

static void App_FastControlTask(void)
{
    bool adc_ok;
    uint32_t now_us;
    float dt_s;

    if (app.fast_loop_running) {
        app.ctrl_overrun++;
        return;
    }

    app.fast_loop_running = true;
    app.timebase_glitch = false;

    now_us = App_Micros();

    if (app.last_ctrl_tick_us != 0U) {
        uint32_t dt_us = now_us - app.last_ctrl_tick_us;

        if (dt_us > CTRL_DT_SANITY_MAX_US) {
            app.ctrl_dt_us = 0U;
            app.timebase_glitch = true;
            app.timebase_glitch_cnt++;
        } else {
            app.ctrl_dt_us = dt_us;

            if (app.ctrl_dt_us > app.ctrl_dt_max_us) {
                app.ctrl_dt_max_us = app.ctrl_dt_us;
            }
            if (app.ctrl_dt_us > (APP_CTRL_PERIOD_US + (APP_CTRL_PERIOD_US / 2U))) {
                app.ctrl_overrun++;
            }
        }
    }

    app.last_ctrl_tick_us = now_us;
    app.ctrl_tick++;

    adc_ok = Measurements_Update(&app.meas);

    if (adc_ok) {
        uint32_t dma_updates = Measurements_GetDmaUpdateCount();

        if (dma_updates != app.last_adc_dma_updates) {
            app.last_adc_dma_updates = dma_updates;
            app.adc_last_update_us = now_us;
        }

        app.adc_age_us = now_us - app.adc_last_update_us;
    }

    App_UpdateFaultFlagsFast(adc_ok);

    if (app.fault_flags != FAULT_NONE) {
        App_FaultShutdown(app.fault_flags);
        app.fast_loop_running = false;
        return;
    }

    dt_s = 1.0f / (float)APP_CTRL_FREQ_HZ;
    ControlCv_UpdateRampedSetpoint(&app.cv, dt_s);

    app.region_trans_debug = 0U;

    if (!app.stage_enabled) {
        app.active_mode = MODE_IDLE;
        app.fast_loop_running = false;
        return;
    }

#if (POWER_STAGE_TEST_BOOST_PWM_FIXED != 0U)
    if (app.requested_mode == MODE_CV) {
        App_RunBoostFixedTest();
        app.fast_loop_running = false;
        return;
    }
#endif

    if (app.requested_mode == MODE_CV) {
        app.active_mode = MODE_CV;
        app.off_ramp_done_ticks = 0U;
        App_RunCvLoopFast(dt_s);
    } else if (app.requested_mode == MODE_IDLE) {
        app.active_mode = MODE_CV;
        App_RunCvLoopFast(dt_s);
        App_CheckDisableCondition();
    } else {
        app.pending_disable_request = true;
        app.active_mode = MODE_CC;
    }

    app.fast_loop_running = false;
}

static bool App_EnableStageSlow(void)
{
    PowerStage_Region_t region;
    float setpoint_v;
    float duty_ff_a;
    float duty_ff_c;
    float control_ff;
    bool zero_setpoint;
    bool low_setpoint_start;
    uint32_t now_ms;

    if (app.stage_enabled) {
        return true;
    }

    zero_setpoint = App_IsZeroSetpointTarget();
    setpoint_v = ControlCv_GetRampedSetpoint(&app.cv);
    if (zero_setpoint) {
        setpoint_v = 0.0f;
    } else if (setpoint_v < 0.10f) {
        setpoint_v = App_MaxFloat(app.cv_user_setpoint * 0.2f, 0.10f);
    }

    low_setpoint_start = App_IsLowSetpointTarget();
    now_ms = HAL_GetTick();
    if (!low_setpoint_start) {
        app.low_setpoint_start_active = false;
    }

    region = zero_setpoint ?
             POWER_REGION_BUCK :
             low_setpoint_start ?
             POWER_REGION_BUCK :
             App_SelectRegionBase(app.meas.vin, setpoint_v);
    App_EstimateDutyForRegion(region, app.meas.vin, setpoint_v, &duty_ff_a, &duty_ff_c);

    if (zero_setpoint) {
        duty_ff_a = 0.0f;
        duty_ff_c = 0.0f;
        app.buck_boost_duty_c_cap = BUCK_BOOST_DUTY_C_MAX;
        app.low_setpoint_start_active = false;
    } else if (low_setpoint_start) {
        duty_ff_a = CV_LOW_SETPOINT_START_DUTY;
        duty_ff_c = 0.0f;
        app.buck_boost_duty_c_cap = BUCK_BOOST_DUTY_C_MAX;
        app.low_setpoint_start_active = true;
        app.low_setpoint_start_until_ms = now_ms + CV_LOW_SETPOINT_START_TIME_MS;
    } else if (region == POWER_REGION_BUCK) {
        duty_ff_a = App_Clamp(duty_ff_a, DUTY_STARTUP_MIN, DUTY_STARTUP_MAX);
        app.buck_boost_duty_c_cap = BUCK_BOOST_DUTY_C_MAX;
    } else if (region == POWER_REGION_BUCK_BOOST) {
        duty_ff_c = App_Clamp(duty_ff_c,
                              BUCK_BOOST_DUTY_C_INIT_MIN,
                              BUCK_BOOST_DUTY_C_INIT_MAX);
        App_ArmBuckBoostSoftstart(duty_ff_c);
    } else {
        duty_ff_c = App_Clamp(duty_ff_c, DUTY_STARTUP_MIN, DUTY_STARTUP_MAX);
        app.buck_boost_duty_c_cap = BUCK_BOOST_DUTY_C_MAX;
    }

    app.duty_ff_a = duty_ff_a;
    app.duty_ff_c = duty_ff_c;

    control_ff = App_GetRegionControlFeedForward(region, duty_ff_a, duty_ff_c);

    app.region_candidate = region;
    app.region_candidate_count = 0U;
    app.region_last_confirm_count = 0U;
    app.region_old_debug = region;
    app.region_new_debug = region;
    app.region_hold_until_ms = now_ms + REGION_MIN_DWELL_MS;

    App_SetCvLimitsForRegion(region);
    ControlCv_Reset(&app.cv, control_ff);
#if (CONTROL_CV_USE_2P2Z != 0)
    Control2p2z_Reset(&app.cv_2p2z, control_ff);
#endif

    App_ApplyDuty(region, duty_ff_a, duty_ff_c);

    if (!PowerStage_Enable()) {
        app.fault_flags |= FAULT_DRIVER;
        App_FaultShutdown(app.fault_flags);
        return false;
    }

    app.stage_enabled = true;
    app.pending_disable_request = false;
    app.off_ramp_done_ticks = 0U;
    return true;
}

static void App_DisableStageSlow(void)
{
    if (app.stage_enabled || PowerStage_IsEnabled() || App_IsDriverAwake()) {
        PowerStage_Disable();
    }

    app.stage_enabled = false;
    app.pending_disable_request = false;
    app.off_ramp_done_ticks = 0U;
    app.active_mode = MODE_IDLE;
    app.low_setpoint_start_active = false;

    ControlCv_Reset(&app.cv, DUTY_STARTUP_MIN);
#if (CONTROL_CV_USE_2P2Z != 0)
    Control2p2z_Reset(&app.cv_2p2z, DUTY_STARTUP_MIN);
#endif
}

static void App_ControlSlowTask(void)
{
    if (app.fault_flags != FAULT_NONE) {
        return;
    }

    if (app.requested_mode == MODE_CC) {
        app.pending_disable_request = true;
    }

    if (app.requested_mode == MODE_CV) {
        if (App_StartupHoldRemainingMs() == 0U) {
            if (!app.stage_enabled) {
                (void)App_EnableStageSlow();
            }
        }
    } else {
        if (app.stage_enabled && app.pending_disable_request) {
            App_DisableStageSlow();
        }

        if (!app.stage_enabled) {
            app.active_mode = MODE_IDLE;
        }
    }
}

static void App_DebugTask(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t primask;
    uint32_t fault_flags;
    uint32_t ctrl_overrun;
    uint32_t ctrl_dt_us;
    uint32_t ctrl_dt_max_us;
    uint32_t adc_age_us;
    uint32_t pwm_update_cnt;
    uint32_t fsw_hz;
    uint32_t hrtim_period;
    uint32_t refresh_hz;
    uint32_t refresh_period;
    uint32_t refresh_pulse;
    uint32_t cmp_a;
    uint32_t cmp_c;
    uint32_t duty_a_10k;
    uint32_t duty_c_phys_10k;
    uint32_t uart_buf_used;
    uint32_t uart_dropped;
    bool stage_enabled;
    bool refresh_active;
    App_Mode_t active_mode;
    App_Mode_t requested_mode;
    PowerStage_Region_t active_region;
    float vin;
    float vout;
    float iout;
    float setpoint_v;
    float ramped_setpoint_v;
    float duty_cmd_c;
    float duty_ff_c;
    float duty_c_limit_min;
    float duty_c_limit_max;
    float cv_error_v;
    float pi_integrator;
    uint8_t sat_hi_debug;
    uint8_t sat_lo_debug;
    int32_t vin_x1000;
    int32_t vout_x1000;
    int32_t iout_x1000;
    int32_t set_x1000;
    int32_t rset_x1000;
    int32_t err_x1000;
    int32_t duty_a_hw_x10;
    int32_t duty_c_hw_x10;
    int32_t cmd_c_x10;
    int32_t ff_c_x10;
    int32_t duty_c_limit_min_x10;
    int32_t duty_c_limit_max_x10;
    int32_t pi_x1000;
    char fault_text[48];

    if ((uint32_t)(now_ms - app.last_debug_tick_ms) < APP_DEBUG_PERIOD_MS) {
        return;
    }

    app.last_debug_tick_ms = now_ms;

    primask = __get_PRIMASK();
    __disable_irq();

    vin = app.meas.vin;
    vout = app.meas.vout;
    iout = app.meas.iout;
    setpoint_v = app.cv_user_setpoint;
    ramped_setpoint_v = ControlCv_GetRampedSetpoint(&app.cv);
    duty_cmd_c = app.duty_cmd_c;
    duty_ff_c = app.duty_ff_c;
    pi_integrator = app.cv.integrator;
    active_mode = app.active_mode;
    requested_mode = app.requested_mode;
    stage_enabled = app.stage_enabled;
    fault_flags = app.fault_flags;
    ctrl_overrun = app.ctrl_overrun;
    ctrl_dt_us = app.ctrl_dt_us;
    ctrl_dt_max_us = app.ctrl_dt_max_us;
    adc_age_us = app.adc_age_us;
    pwm_update_cnt = app.pwm_update_cnt;
    sat_hi_debug = app.sat_hi_debug;
    sat_lo_debug = app.sat_lo_debug;

    active_region = PowerStage_GetRegion();
    fsw_hz = PowerStage_GetFswHz();
    hrtim_period = PowerStage_GetPeriodTicks();
    refresh_active = PowerStage_IsBootstrapRefreshActive();
    refresh_hz = PowerStage_GetBootstrapRefreshHz();
    refresh_period = PowerStage_GetBootstrapRefreshPeriodTicks();
    refresh_pulse = PowerStage_GetBootstrapRefreshPulseTicks();
    cmp_a = PowerStage_GetCmpA();
    cmp_c = PowerStage_GetCmpC();
    duty_a_10k = PowerStage_GetDutyA10k();
    duty_c_phys_10k = PowerStage_GetDutyCPhys10k();

    if (primask == 0U) {
        __enable_irq();
    }

    if ((active_region == POWER_REGION_BUCK_BOOST) ||
        (active_region == POWER_REGION_BOOST)) {
        App_GetDutyLimitsForRegion(active_region,
                                   &duty_c_limit_min,
                                   &duty_c_limit_max);
    } else {
        duty_c_limit_min = 0.0f;
        duty_c_limit_max = 0.0f;
    }

    cv_error_v = ramped_setpoint_v - vout;
    vin_x1000 = App_ToFixed(vin, 1000);
    vout_x1000 = App_ToFixed(vout, 1000);
    iout_x1000 = App_ToFixed(iout, 1000);
    set_x1000 = App_ToFixed(setpoint_v, 1000);
    rset_x1000 = App_ToFixed(ramped_setpoint_v, 1000);
    err_x1000 = App_ToFixed(cv_error_v, 1000);
    duty_a_hw_x10 = App_ToFixed((float)duty_a_10k / 100.0f, 10);
    duty_c_hw_x10 = App_ToFixed((float)duty_c_phys_10k / 100.0f, 10);
    cmd_c_x10 = App_ToFixed(duty_cmd_c * 100.0f, 10);
    ff_c_x10 = App_ToFixed(duty_ff_c * 100.0f, 10);
    duty_c_limit_min_x10 = App_ToFixed(duty_c_limit_min * 100.0f, 10);
    duty_c_limit_max_x10 = App_ToFixed(duty_c_limit_max * 100.0f, 10);
    pi_x1000 = App_ToFixed(pi_integrator, 1000);

    App_FaultText(fault_flags, fault_text, sizeof(fault_text));

    Debug_Printf("[APP] mode=%s req=%s region=%s vin=%s%ld.%03ldV vout=%s%ld.%03ldV set=%s%ld.%03ldV rset=%s%ld.%03ldV err=%s%ld.%03ldV iout=%s%ld.%03ldA fault=%s en=%u",
                 App_ModeText(active_mode),
                 App_ModeText(requested_mode),
                 App_RegionText(active_region),
                 (vin_x1000 < 0) ? "-" : "",
                 App_IntPart(vin_x1000, 1000),
                 App_FracPart(vin_x1000, 1000),
                 (vout_x1000 < 0) ? "-" : "",
                 App_IntPart(vout_x1000, 1000),
                 App_FracPart(vout_x1000, 1000),
                 (set_x1000 < 0) ? "-" : "",
                 App_IntPart(set_x1000, 1000),
                 App_FracPart(set_x1000, 1000),
                 (rset_x1000 < 0) ? "-" : "",
                 App_IntPart(rset_x1000, 1000),
                 App_FracPart(rset_x1000, 1000),
                 (err_x1000 < 0) ? "-" : "",
                 App_IntPart(err_x1000, 1000),
                 App_FracPart(err_x1000, 1000),
                 (iout_x1000 < 0) ? "-" : "",
                 App_IntPart(iout_x1000, 1000),
                 App_FracPart(iout_x1000, 1000),
                 fault_text,
                 stage_enabled ? 1U : 0U);

    Debug_Printf("[CTRL] dt=%luus max=%luus ov=%lu adc_age=%luus SAT_HI=%u SAT_LO=%u CV_ERR=%s%ld.%03ldV PI_INT=%s%ld.%03ld pwm_updates=%lu",
                 (unsigned long)ctrl_dt_us,
                 (unsigned long)ctrl_dt_max_us,
                 (unsigned long)ctrl_overrun,
                 (unsigned long)adc_age_us,
                 (unsigned int)sat_hi_debug,
                 (unsigned int)sat_lo_debug,
                 (err_x1000 < 0) ? "-" : "",
                 App_IntPart(err_x1000, 1000),
                 App_FracPart(err_x1000, 1000),
                 (pi_x1000 < 0) ? "-" : "",
                 App_IntPart(pi_x1000, 1000),
                 App_FracPart(pi_x1000, 1000),
                 (unsigned long)pwm_update_cnt);

    Debug_Printf("[PWM] fsw=%luHz per=%lu refresh=%luHz ref_act=%u ref_per=%lu ref_pulse=%lu A=%ld.%01ld%% C=%ld.%01ld%% cmpA=%lu cmpC=%lu DUTY_CMD_C=%s%ld.%01ld%% DUTY_FF_C=%s%ld.%01ld%% DUTY_C_LIMIT_MIN=%s%ld.%01ld%% DUTY_C_LIMIT_MAX=%s%ld.%01ld%%",
                 (unsigned long)fsw_hz,
                 (unsigned long)hrtim_period,
                 (unsigned long)refresh_hz,
                 refresh_active ? 1U : 0U,
                 (unsigned long)refresh_period,
                 (unsigned long)refresh_pulse,
                 App_IntPart(duty_a_hw_x10, 10),
                 App_FracPart(duty_a_hw_x10, 10),
                 App_IntPart(duty_c_hw_x10, 10),
                 App_FracPart(duty_c_hw_x10, 10),
                 (unsigned long)cmp_a,
                 (unsigned long)cmp_c,
                 (cmd_c_x10 < 0) ? "-" : "",
                 App_IntPart(cmd_c_x10, 10),
                 App_FracPart(cmd_c_x10, 10),
                 (ff_c_x10 < 0) ? "-" : "",
                 App_IntPart(ff_c_x10, 10),
                 App_FracPart(ff_c_x10, 10),
                 (duty_c_limit_min_x10 < 0) ? "-" : "",
                 App_IntPart(duty_c_limit_min_x10, 10),
                 App_FracPart(duty_c_limit_min_x10, 10),
                 (duty_c_limit_max_x10 < 0) ? "-" : "",
                 App_IntPart(duty_c_limit_max_x10, 10),
                 App_FracPart(duty_c_limit_max_x10, 10));

    uart_buf_used = Debug_GetTxBufferUsed();
    uart_dropped = Debug_GetDroppedCount();
    Debug_Printf("[UART] UART_BUSY=%u UART_BUF_USED=%lu UART_DROPPED=%lu",
                 Debug_IsTxBusy() ? 1U : 0U,
                 (unsigned long)uart_buf_used,
                 (unsigned long)uart_dropped);
}

static void App_ControlTimerInit(void)
{
    uint32_t timer_clk_hz = 1000000U;
    uint32_t psc;
    uint32_t period;

    __HAL_RCC_TIM6_CLK_ENABLE();

    psc = (SystemCoreClock / timer_clk_hz);
    if (psc == 0U) {
        psc = 1U;
    }
    psc -= 1U;

    period = (timer_clk_hz / APP_CTRL_FREQ_HZ);
    if (period == 0U) {
        period = 1U;
    }
    period -= 1U;

    app.ctrl_tim.Instance = TIM6;
    app.ctrl_tim.Init.Prescaler = (uint16_t)psc;
    app.ctrl_tim.Init.CounterMode = TIM_COUNTERMODE_UP;
    app.ctrl_tim.Init.Period = period;
    app.ctrl_tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&app.ctrl_tim) != HAL_OK) {
        app.ctrl_timer_ready = false;
        return;
    }

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    if (HAL_TIM_Base_Start_IT(&app.ctrl_tim) != HAL_OK) {
        app.ctrl_timer_ready = false;
        return;
    }

    app.ctrl_timer_ready = true;
}

void App_Init(HRTIM_HandleTypeDef *hhrtim,
              ADC_HandleTypeDef *hadc1,
              ADC_HandleTypeDef *hadc2,
              UART_HandleTypeDef *huart_debug,
              I2C_HandleTypeDef *hi2c_pd)
{
    int32_t set_x100;
    int32_t ovp_x100;
    int32_t ocp_x100;
    int32_t uvlo_x100;
    int32_t buck_enter_margin_x100;
    int32_t buck_exit_margin_x100;
    uint32_t reset_flags;

    memset(&app, 0, sizeof(app));

    Debug_Init(huart_debug);
    App_TimebaseInit();

    PowerStage_Init(hhrtim);
    Measurements_Init(hadc1, hadc2);

    ControlCv_Init(&app.cv,
                   CV_KP,
                   CV_KI,
                   DUTY_BUCK_MIN,
                   DUTY_BUCK_MAX,
                   CV_SLEW_UP_V_PER_S);
    ControlCv_SetSlewRates(&app.cv, CV_SLEW_UP_V_PER_S, CV_SLEW_DOWN_V_PER_S);

#if (CONTROL_CV_USE_2P2Z != 0)
    Control2p2z_Init(&app.cv_2p2z,
                     CV_2P2Z_B0,
                     CV_2P2Z_B1,
                     CV_2P2Z_B2,
                     CV_2P2Z_A1,
                     CV_2P2Z_A2);
#endif

    app.cv_user_setpoint = VOUT_SETPOINT_DEFAULT_V;
    ControlCv_SetTarget(&app.cv, 0.0f);

    app.requested_mode = MODE_IDLE;
    app.active_mode = MODE_IDLE;
    app.fault_flags = FAULT_NONE;
    app.latched_fault_flags = FAULT_NONE;
    app.current_limit_a = IOUT_OCP_LIMIT;

    app.startup_tick_ms = HAL_GetTick();
    app.last_debug_tick_ms = app.startup_tick_ms;
    app.last_led_tick_ms = app.startup_tick_ms;
    app.last_ctrl_tick_us = App_Micros();
    app.adc_last_update_us = app.last_ctrl_tick_us;
    app.last_adc_dma_updates = Measurements_GetDmaUpdateCount();

    app.region_candidate = POWER_REGION_BUCK;
    app.region_old_debug = POWER_REGION_BUCK;
    app.region_new_debug = POWER_REGION_BUCK;
    app.buck_boost_softstart_start_duty_c = BUCK_BOOST_DUTY_C_INIT_MIN;
    app.buck_boost_duty_c_cap = BUCK_BOOST_DUTY_C_MAX;

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    PowerManager_Init(hi2c_pd);
    App_ControlTimerInit();

    PowerStage_ForceSafeState();
    reset_flags = RCC->CSR;

    set_x100 = App_ToFixed(VOUT_SETPOINT_DEFAULT_V, 100);
    ovp_x100 = App_ToFixed(VOUT_OVP_LIMIT, 100);
    ocp_x100 = App_ToFixed(app.current_limit_a, 100);
    uvlo_x100 = App_ToFixed(VIN_UVLO_LIMIT, 100);
    buck_enter_margin_x100 = App_ToFixed(REGION_BUCK_ENTER_MARGIN_V, 100);
    buck_exit_margin_x100 = App_ToFixed(REGION_BUCK_EXIT_MARGIN_V, 100);

    Debug_Printf("\r\n[APP] Start. ctrl=%lu Hz hold=%lu ms reset=0x%08lX",
                 (unsigned long)APP_CTRL_FREQ_HZ,
                 (unsigned long)APP_STARTUP_HOLD_MS,
                 (unsigned long)reset_flags);
    Debug_Printf("[APP] SET=%ld.%02ldV OVP=%ld.%02ldV OCP=%ld.%02ldA UVLO=%ld.%02ldV",
                 App_IntPart(set_x100, 100), App_FracPart(set_x100, 100),
                 App_IntPart(ovp_x100, 100), App_FracPart(ovp_x100, 100),
                 App_IntPart(ocp_x100, 100), App_FracPart(ocp_x100, 100),
                 App_IntPart(uvlo_x100, 100), App_FracPart(uvlo_x100, 100));

    if (!app.ctrl_timer_ready) {
        Debug_Printf("[APP] WARN: TIM6 ctrl loop start failed");
    }

    Debug_Printf("[APP] Region strategy: BUCK if RSET < VIN-%ld.%02ldV, else BUCK_BOOST (BOOST disabled in auto CV)",
                 App_IntPart(buck_enter_margin_x100, 100),
                 App_FracPart(buck_enter_margin_x100, 100));
    Debug_Printf("[APP] BUCK exit threshold: RSET >= VIN-%ld.%02ldV; debug=%lums verbose=%u",
                 App_IntPart(buck_exit_margin_x100, 100),
                 App_FracPart(buck_exit_margin_x100, 100),
                 (unsigned long)APP_DEBUG_PERIOD_MS,
                 (unsigned int)APP_DEBUG_VERBOSE);
#if (POWER_STAGE_TEST_BOOST_PWM_FIXED != 0U)
    Debug_Printf("[APP] DIAG: pure BOOST fixed PWM test ENABLED (10%% -> 30%% -> 50%%)");
#endif

    __HAL_RCC_CLEAR_RESET_FLAGS();
}

void App_Run(void)
{
    App_LedTask();
    App_ControlSlowTask();
    PowerManager_Task();
    App_DebugTask();
}

void App_SetRequestedMode(App_Mode_t mode)
{
    if (mode == MODE_IDLE) {
        app.requested_mode = MODE_IDLE;
        ControlCv_SetTarget(&app.cv, 0.0f);
        return;
    }

    if (mode == MODE_CC) {
        app.requested_mode = MODE_CC;
        ControlCv_SetTarget(&app.cv, 0.0f);
        return;
    }

    app.requested_mode = MODE_CV;
    ControlCv_SetTarget(&app.cv, app.cv_user_setpoint);
}

App_Mode_t App_GetMode(void)
{
    return app.active_mode;
}

App_Mode_t App_GetRequestedMode(void)
{
    return app.requested_mode;
}

uint32_t App_GetFaultFlags(void)
{
    return app.fault_flags;
}

void App_ClearFaults(void)
{
    app.latched_fault_flags = FAULT_NONE;
    app.fault_flags = FAULT_NONE;
    app.ocp_hit_count = 0U;
    app.pending_disable_request = false;

    if (app.requested_mode == MODE_CV) {
        ControlCv_SetTarget(&app.cv, app.cv_user_setpoint);
    }
}

void App_SetCvSetpoint(float setpoint_v)
{
    if (setpoint_v < 0.0f) {
        setpoint_v = 0.0f;
    }

    app.cv_user_setpoint = setpoint_v;

    if (app.requested_mode == MODE_CV) {
        ControlCv_SetTarget(&app.cv, app.cv_user_setpoint);
    }
}

float App_GetCvSetpoint(void)
{
    return app.cv_user_setpoint;
}

float App_GetCvRampedSetpoint(void)
{
    return ControlCv_GetRampedSetpoint(&app.cv);
}

void App_SetCurrentLimit(float current_limit_a)
{
    if (current_limit_a < 0.0f) {
        current_limit_a = 0.0f;
    }
    if (current_limit_a > IOUT_LIMIT_MAX) {
        current_limit_a = IOUT_LIMIT_MAX;
    }

    app.current_limit_a = current_limit_a;
}

float App_GetCurrentLimit(void)
{
    return app.current_limit_a;
}

float App_GetInputVoltage(void)
{
    return app.meas.vin;
}

float App_GetOutputVoltage(void)
{
    return app.meas.vout;
}

float App_GetOutputCurrent(void)
{
    return app.meas.iout;
}

uint32_t APP_GetPower(void)
{
    return APP_MAX_POWER_W;
}

void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&app.ctrl_tim);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &app.ctrl_tim) {
        App_FastControlTask();
    }
}
