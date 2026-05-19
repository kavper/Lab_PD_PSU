#include "app.h"

#include "control_cv.h"
#include "debug_uart.h"
#include "measurements.h"
#include "power_stage.h"

#include <stdlib.h>
#include <string.h>

#define APP_CTRL_FREQ_HZ                       4000U
#define APP_CTRL_PERIOD_US                     (1000000U / APP_CTRL_FREQ_HZ)
#define APP_DEBUG_PERIOD_MS                    200U
#define APP_STARTUP_HOLD_MS                    2000U
#define APP_LED_BLINK_MS                       250U
#define APP_MAX_POWER_W                        100U

/*
 * GLOWNE NASTAWY BRING-UP (CV + zabezpieczenia):
 * TODO: skalibrowac pod realny hardware i dzielniki pomiarowe.
 */
#define VOUT_SETPOINT                          10.00f
#define VOUT_OVP_LIMIT                         35.00f
#define IOUT_OCP_LIMIT                         4.00f
#define VIN_UVLO_LIMIT                         8.00f

#define DUTY_MIN                               0.001f
#define DUTY_MAX                               0.95f
#define DUTY_BUCK_MIN                          0.001f
#define DUTY_BUCK_MAX                          0.95f
#define DUTY_BOOST_MIN                         0.05f
#define DUTY_BOOST_MAX                         0.90f
#define DUTY_MIXED_B_MIN                       0.05f
#define DUTY_MIXED_B_MAX                       0.35f
#define CV_STARTUP_DUTY                        0.005f
#define CV_STARTUP_DUTY_MAX                    0.05f
#define DUTY_MIXED_A                           0.80f
#define DUTY_MIXED_B_START                     0.05f
#define DUTY_BOOST_B_START                     0.05f
#define DUTY_BUCK_MIN_EFFECTIVE                0.02f
#define DUTY_BOOST_MIN_EFFECTIVE               0.02f
#define MODE_SWITCH_HIT_COUNT                  1000U

#define CV_KP                                  0.040f
#define CV_KI                                  18.0f
#define CV_SOFTSTART_SLEW_V_PER_S              25.0f

#define REGION_VIN_MARGIN_V                    1.20f
#define REGION_SWITCH_CONFIRM_COUNT            8U
#define OCP_ACTIVE_MIN_LIMIT_A                 0.05f
#define OCP_HIT_COUNT_LIMIT                    20U
#define ZERO_HOLD_SETPOINT_V                   0.03f
#define DISCHARGE_ENABLE_ERROR_V               2.00f
#define DISCHARGE_ENABLE_RATIO                 1.20f
#define DISCHARGE_DISABLE_ERROR_V              0.30f
#define DISCHARGE_MIN_SETPOINT_V               0.50f
#define NORMAL_CV_SINK_ERROR_V                 0.50f
#define DISCHARGE_PULSE_NS                     100U
#define DISCHARGE_EVERY_PERIODS                32U

typedef struct {
    Measurements_t meas;
    ControlCv_t cv;
    App_Mode_t requested_mode;
    App_Mode_t active_mode;
    uint32_t fault_flags;
    uint32_t startup_tick_ms;
    uint32_t last_debug_tick_ms;
    uint32_t last_led_tick_ms;
    uint32_t last_ctrl_tick_us;
    uint32_t mode_top_hits;
    uint32_t mode_bottom_hits;
    uint32_t ocp_hit_count;
    uint32_t discharge_start_tick_ms;
    uint32_t region_candidate_count;
    uint32_t region_last_confirm_count;
    uint32_t region_transition_count;
    uint32_t latched_fault_flags;
    PowerStage_Region_t region_candidate;
    PowerStage_Region_t region_old_debug;
    PowerStage_Region_t region_new_debug;
    float current_limit_a;
    float last_est_duty_a;
    float last_est_duty_c;
    bool enable_attempted;
    bool shutdown_latched;
    bool stage_enabled;
    bool timebase_ready;
    bool led_blink_state;
    bool coast_active;
    bool discharge_active;
} App_Context_t;

static App_Context_t app;

static bool App_IsDriverAwake(void);

static uint32_t App_StartupHoldRemainingMs(void)
{
    uint32_t elapsed_ms = HAL_GetTick() - app.startup_tick_ms;

    if (elapsed_ms >= APP_STARTUP_HOLD_MS) {
        return 0U;
    }

    return APP_STARTUP_HOLD_MS - elapsed_ms;
}

static float App_Clamp(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
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

static const char *App_StateText(void)
{
    if (app.fault_flags != FAULT_NONE) {
        return "FAULT";
    }
    if (app.requested_mode == MODE_IDLE) {
        return "OFF";
    }
    if (App_StartupHoldRemainingMs() > 0U) {
        return "HOLD";
    }
    if (app.shutdown_latched) {
        return "LATCH";
    }
    if (app.discharge_active) {
        return "DISCHARGE";
    }
    if (app.coast_active) {
        return "COAST";
    }
    if (!app.stage_enabled) {
        return "ARM";
    }
    return "RUN";
}

static const char *App_WhyText(void)
{
    if (app.fault_flags != FAULT_NONE) {
        return "fault";
    }
    if (app.requested_mode == MODE_IDLE) {
        return "user_off";
    }
    if (App_StartupHoldRemainingMs() > 0U) {
        return "startup_hold";
    }
    if (app.shutdown_latched) {
        return "latched";
    }
    if (app.discharge_active) {
        if (app.cv.target_setpoint <= ZERO_HOLD_SETPOINT_V) {
            return "zero_hold_discharge";
        }
        return "vout_above_set_discharge";
    }
    if (app.coast_active) {
        return "vout_above_set";
    }
    if (!app.stage_enabled) {
        return "waiting_enable";
    }
    if (app.active_mode == MODE_CV) {
        return "cv_loop";
    }
    if (app.active_mode == MODE_CC) {
        return "cc_stub";
    }
    return "unknown";
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
        return (DWT->CYCCNT / hclk_mhz);
    }

    return HAL_GetTick() * 1000U;
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

static void App_EnterIdle(void)
{
    if (app.stage_enabled || App_IsDriverAwake() || PowerStage_IsEnabled() || PowerStage_IsDischarging()) {
        PowerStage_Disable();
    }

    app.stage_enabled = false;
    ControlCv_Reset(&app.cv, DUTY_MIN);
    app.cv.ramped_setpoint = 0.0f;
    app.mode_top_hits = 0U;
    app.mode_bottom_hits = 0U;
    app.ocp_hit_count = 0U;
    app.region_candidate = POWER_REGION_BUCK;
    app.region_candidate_count = 0U;
    app.region_last_confirm_count = 0U;
    app.region_transition_count = 0U;
    app.region_old_debug = POWER_REGION_BUCK;
    app.region_new_debug = POWER_REGION_BUCK;
    app.last_est_duty_a = 0.0f;
    app.last_est_duty_c = 0.0f;
    app.coast_active = false;
    app.discharge_active = false;
    PowerStage_SetRegion(POWER_REGION_BUCK);
    app.active_mode = MODE_IDLE;
}

static void App_LatchShutdown(uint32_t reason_flags)
{
    if (reason_flags == FAULT_NONE) {
        reason_flags = FAULT_DRIVER;
    }

    app.latched_fault_flags |= reason_flags;
    app.shutdown_latched = true;
    App_EnterIdle();
}

static PowerStage_Region_t App_SelectRegion(float vin, float setpoint_v)
{
    if (setpoint_v < (vin - REGION_VIN_MARGIN_V)) {
        return POWER_REGION_BUCK;
    }
    if (setpoint_v > (vin + REGION_VIN_MARGIN_V)) {
        return POWER_REGION_BOOST;
    }
    return POWER_REGION_BUCK_BOOST;
}

static float App_MaxFloat(float a, float b)
{
    return (a > b) ? a : b;
}

static float App_EstimateBuckDuty(float vin, float rset_v)
{
    float duty;

    vin = App_MaxFloat(vin, 0.10f);
    duty = rset_v / vin;
    return App_Clamp(duty, DUTY_BUCK_MIN_EFFECTIVE, DUTY_BUCK_MAX);
}

static float App_EstimateBoostDuty(float vin, float rset_v)
{
    float denom = App_MaxFloat(rset_v, vin + 0.10f);
    float duty = 1.0f - (vin / denom);

    return App_Clamp(duty, DUTY_BOOST_MIN, DUTY_BOOST_MAX);
}

static void App_EstimateDutyNow(PowerStage_Region_t region,
                                float vin,
                                float rset_v,
                                float *est_duty_a,
                                float *est_duty_c)
{
    float duty_a = 0.0f;
    float duty_c = 0.0f;

    if (rset_v > ZERO_HOLD_SETPOINT_V) {
        switch (region) {
            case POWER_REGION_BUCK:
                duty_a = App_EstimateBuckDuty(vin, rset_v);
                break;

            case POWER_REGION_BOOST:
                duty_a = 1.0f;
                duty_c = App_EstimateBoostDuty(vin, rset_v);
                break;

            case POWER_REGION_BUCK_BOOST:
            default:
                duty_a = DUTY_MIXED_A;
                duty_c = App_Clamp(App_EstimateBoostDuty(vin, rset_v),
                                   DUTY_MIXED_B_MIN,
                                   DUTY_MIXED_B_MAX);
                break;
        }
    }

    if (est_duty_a != NULL) {
        *est_duty_a = duty_a;
    }
    if (est_duty_c != NULL) {
        *est_duty_c = duty_c;
    }
}

static PowerStage_Region_t App_LimitRegionStep(PowerStage_Region_t current_region,
                                               PowerStage_Region_t candidate_region)
{
    if (((current_region == POWER_REGION_BUCK) && (candidate_region == POWER_REGION_BOOST)) ||
        ((current_region == POWER_REGION_BOOST) && (candidate_region == POWER_REGION_BUCK))) {
        return POWER_REGION_BUCK_BOOST;
    }

    return candidate_region;
}

static PowerStage_Region_t App_SelectRegionCandidate(PowerStage_Region_t current_region,
                                                     float vin,
                                                     float rset_v)
{
    PowerStage_Region_t candidate_region = App_SelectRegion(vin, rset_v);

    return App_LimitRegionStep(current_region, candidate_region);
}

static float App_EstimateStartupDuty(float vin, float setpoint_v)
{
    PowerStage_Region_t region = App_SelectRegion(vin, setpoint_v);
    float duty = CV_STARTUP_DUTY;

    if ((region == POWER_REGION_BUCK) && (vin > 1.0f)) {
        duty = setpoint_v / vin;
        return App_Clamp(duty, DUTY_MIN, CV_STARTUP_DUTY_MAX);
    }

    if ((region == POWER_REGION_BOOST) && (setpoint_v > 1.0f)) {
        duty = 1.0f - (vin / setpoint_v);
        return App_Clamp(duty, DUTY_BOOST_MIN, CV_STARTUP_DUTY_MAX);
    }

    return App_Clamp(duty, DUTY_MIN, CV_STARTUP_DUTY_MAX);
}

static float App_AdcTriggerFromDuty(float duty)
{
    duty = App_Clamp(duty, 0.0f, 1.0f);

    if (duty > 0.5f) {
        return duty * 0.5f;
    }

    return duty + ((1.0f - duty) * 0.5f);
}

static void App_GetDutyLimitsForRegion(PowerStage_Region_t region,
                                       float *min_duty,
                                       float *max_duty)
{
    float min_value = DUTY_BUCK_MIN_EFFECTIVE;
    float max_value = DUTY_BUCK_MAX;

    if (region == POWER_REGION_BUCK_BOOST) {
        min_value = DUTY_MIXED_B_MIN;
        max_value = DUTY_MIXED_B_MAX;
    } else if (region == POWER_REGION_BOOST) {
        min_value = (DUTY_BOOST_MIN > DUTY_BOOST_MIN_EFFECTIVE) ?
                    DUTY_BOOST_MIN :
                    DUTY_BOOST_MIN_EFFECTIVE;
        max_value = DUTY_BOOST_MAX;
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

static float App_LimitDutyForRegion(PowerStage_Region_t region,
                                    float duty_cmd,
                                    bool *top_hit,
                                    bool *bottom_hit)
{
    float min_duty;
    float max_duty;

    if (top_hit != NULL) {
        *top_hit = false;
    }
    if (bottom_hit != NULL) {
        *bottom_hit = false;
    }

    App_GetDutyLimitsForRegion(region, &min_duty, &max_duty);

    if (duty_cmd >= max_duty) {
        duty_cmd = max_duty;
        if (top_hit != NULL) {
            *top_hit = true;
        }
    } else if (duty_cmd <= min_duty) {
        duty_cmd = min_duty;
        if (bottom_hit != NULL) {
            *bottom_hit = true;
        }
    }

    return duty_cmd;
}

static float App_RegionStartupDuty(PowerStage_Region_t region)
{
    switch (region) {
        case POWER_REGION_BUCK:
            return CV_STARTUP_DUTY;
        case POWER_REGION_BOOST:
            return DUTY_BOOST_B_START;
        case POWER_REGION_BUCK_BOOST:
        default:
            return DUTY_MIXED_B_START;
    }
}

static void App_ApplyRegionDuty(PowerStage_Region_t region, float duty_cmd)
{
    duty_cmd = App_Clamp(duty_cmd, DUTY_MIN, DUTY_MAX);

    switch (region) {
        case POWER_REGION_BUCK:
            PowerStage_SetBuckDuty(duty_cmd);
            PowerStage_SetAdcTriggerPoint(App_AdcTriggerFromDuty(duty_cmd));
            break;

        case POWER_REGION_BOOST:
            PowerStage_SetBoostDuty(duty_cmd);
            PowerStage_SetAdcTriggerPoint(App_AdcTriggerFromDuty(duty_cmd));
            break;

        case POWER_REGION_BUCK_BOOST:
        default:
            PowerStage_SetBuckBoostDuty(DUTY_MIXED_A, duty_cmd);
            PowerStage_SetAdcTriggerPoint(0.60f);
            break;
    }
}

static float App_EstimateDutyForRegionChange(PowerStage_Region_t old_region,
                                             PowerStage_Region_t new_region,
                                             float vin,
                                             float vout,
                                             float rset_v)
{
    float duty_cmd;
    float current_duty_c;

    (void)vout;

    app.last_est_duty_a = 0.0f;
    app.last_est_duty_c = 0.0f;

    switch (new_region) {
        case POWER_REGION_BUCK:
            duty_cmd = App_EstimateBuckDuty(vin, rset_v);
            app.last_est_duty_a = duty_cmd;
            app.last_est_duty_c = 0.0f;
            return duty_cmd;

        case POWER_REGION_BOOST:
            duty_cmd = App_EstimateBoostDuty(vin, rset_v);
            app.last_est_duty_a = 1.0f;
            app.last_est_duty_c = duty_cmd;
            return duty_cmd;

        case POWER_REGION_BUCK_BOOST:
        default:
            current_duty_c = PowerStage_GetDutyC();
            if (old_region == POWER_REGION_BUCK) {
                duty_cmd = App_Clamp(App_EstimateBoostDuty(vin, rset_v), DUTY_MIXED_B_MIN, 0.15f);
            } else if (current_duty_c > 0.0f) {
                duty_cmd = App_Clamp(current_duty_c, DUTY_MIXED_B_MIN, DUTY_MIXED_B_MAX);
            } else {
                duty_cmd = App_Clamp(App_EstimateBoostDuty(vin, rset_v),
                                     DUTY_MIXED_B_MIN,
                                     DUTY_MIXED_B_MAX);
            }

            app.last_est_duty_a = DUTY_MIXED_A;
            app.last_est_duty_c = duty_cmd;
            return duty_cmd;
    }
}

static void App_OnRegionChange(PowerStage_Region_t old_region,
                               PowerStage_Region_t new_region,
                               float vin,
                               float vout,
                               float rset_v)
{
    float duty_init;

    if (old_region == new_region) {
        return;
    }

    duty_init = App_EstimateDutyForRegionChange(old_region, new_region, vin, vout, rset_v);

    app.region_old_debug = old_region;
    app.region_new_debug = new_region;
    app.region_transition_count = 1U;
    app.coast_active = false;
    app.discharge_active = false;

    PowerStage_SetRegion(new_region);
    App_SetCvLimitsForRegion(new_region);
    ControlCv_Reset(&app.cv, duty_init);
    App_ApplyRegionDuty(new_region, duty_init);
}

static PowerStage_Region_t App_UpdateRegionHysteresis(float vin, float vout, float rset_v)
{
    PowerStage_Region_t current_region = PowerStage_GetRegion();
    PowerStage_Region_t candidate_region = App_SelectRegionCandidate(current_region, vin, rset_v);

    if (candidate_region == current_region) {
        app.region_candidate = current_region;
        app.region_candidate_count = 0U;
        return current_region;
    }

    if (candidate_region != app.region_candidate) {
        app.region_candidate = candidate_region;
        app.region_candidate_count = 1U;
    } else if (app.region_candidate_count < REGION_SWITCH_CONFIRM_COUNT) {
        app.region_candidate_count++;
    }

    if (app.region_candidate_count >= REGION_SWITCH_CONFIRM_COUNT) {
        app.region_last_confirm_count = app.region_candidate_count;
        App_OnRegionChange(current_region, candidate_region, vin, vout, rset_v);
        app.region_candidate_count = 0U;
        return candidate_region;
    }

    return current_region;
}

static bool App_IsDriverAwake(void)
{
    return (HAL_GPIO_ReadPin(STBY_GPIO_Port, STBY_Pin) == GPIO_PIN_SET) &&
           (HAL_GPIO_ReadPin(SD_GPIO_Port, SD_Pin) == GPIO_PIN_SET);
}

static bool App_ShouldDischargeCv(float rset_v)
{
    float vout_error;

    if (app.requested_mode != MODE_CV) {
        return false;
    }

    if ((app.cv.target_setpoint <= ZERO_HOLD_SETPOINT_V) && (rset_v <= ZERO_HOLD_SETPOINT_V)) {
        return true;
    }

    if (rset_v < DISCHARGE_MIN_SETPOINT_V) {
        return false;
    }

    vout_error = app.meas.vout - rset_v;
    if (vout_error <= NORMAL_CV_SINK_ERROR_V) {
        return false;
    }
    if (vout_error <= DISCHARGE_ENABLE_ERROR_V) {
        return false;
    }

    return (app.meas.vout > (rset_v * DISCHARGE_ENABLE_RATIO));
}

static void App_UpdateFaultFlags(bool adc_ok)
{
    uint32_t flags = app.latched_fault_flags;
    bool ocp_enabled;

    if (!adc_ok) {
        flags |= FAULT_ADC;
    }

    if ((app.stage_enabled || App_IsDriverAwake()) && PowerStage_IsFaultActive()) {
        flags |= FAULT_DRIVER;
    }

    if (adc_ok) {
        if (app.meas.vin < VIN_UVLO_LIMIT) {
            flags |= FAULT_UVIN;
        }
        if (app.meas.vout > VOUT_OVP_LIMIT) {
            flags |= FAULT_OVP;
        }
        ocp_enabled = app.stage_enabled && (app.current_limit_a >= OCP_ACTIVE_MIN_LIMIT_A);
        if (ocp_enabled && (app.meas.iout > app.current_limit_a)) {
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

    app.fault_flags = flags;
}

static bool App_ResumeCvFromDischarge(float rset_v)
{
    PowerStage_Region_t old_region = PowerStage_GetRegion();
    PowerStage_Region_t new_region = App_SelectRegion(app.meas.vin, rset_v);
    float duty_init;

    new_region = App_LimitRegionStep(old_region, new_region);
    duty_init = App_EstimateDutyForRegionChange(old_region,
                                                new_region,
                                                app.meas.vin,
                                                app.meas.vout,
                                                rset_v);

    app.discharge_active = false;
    app.coast_active = false;
    app.region_candidate = new_region;
    app.region_candidate_count = 0U;
    app.region_old_debug = old_region;
    app.region_new_debug = new_region;
    app.region_transition_count = 1U;

    PowerStage_SetRegion(new_region);
    App_SetCvLimitsForRegion(new_region);
    ControlCv_Reset(&app.cv, duty_init);
    App_ApplyRegionDuty(new_region, duty_init);

    app.enable_attempted = true;
    if (!PowerStage_Enable()) {
        app.fault_flags |= FAULT_DRIVER;
        App_LatchShutdown(app.fault_flags);
        return true;
    }

    app.stage_enabled = true;
    return false;
}

static bool App_HandleActiveDischarge(void)
{
    uint32_t now_ms = HAL_GetTick();
    float rset_v = ControlCv_GetRampedSetpoint(&app.cv);
    bool zero_hold;

    if (app.cv.target_setpoint <= ZERO_HOLD_SETPOINT_V) {
        rset_v = 0.0f;
        app.cv.ramped_setpoint = 0.0f;
    }

    zero_hold = (app.cv.target_setpoint <= ZERO_HOLD_SETPOINT_V);

    if (app.discharge_active) {
        if ((!zero_hold) && (app.meas.vout <= (rset_v + DISCHARGE_DISABLE_ERROR_V))) {
            return App_ResumeCvFromDischarge(rset_v);
        }
    }

    if (!App_ShouldDischargeCv(rset_v)) {
        if (app.discharge_active) {
            return App_ResumeCvFromDischarge(rset_v);
        }
        app.discharge_active = false;
        return false;
    }

    app.active_mode = MODE_CV;
    app.coast_active = false;
    if (!app.discharge_active) {
        app.discharge_start_tick_ms = now_ms;
    }

    app.discharge_active = true;
    app.mode_top_hits = 0U;
    app.mode_bottom_hits = 0U;
    app.cv.ramped_setpoint = rset_v;
    ControlCv_Reset(&app.cv, DUTY_MIN);
    PowerStage_SetRegion(POWER_REGION_BUCK);
    PowerStage_SetBuckDischarge(DISCHARGE_PULSE_NS, DISCHARGE_EVERY_PERIODS);
    app.stage_enabled = PowerStage_IsEnabled();
    return true;
}

static void App_RunCvControl(void)
{
    const float dt_s = 1.0f / (float)APP_CTRL_FREQ_HZ;
    float duty_cmd;
    float region_setpoint;
    bool top_hit;
    bool bottom_hit;
    PowerStage_Region_t region;
    PowerStage_Region_t old_region;

    region = PowerStage_GetRegion();
    App_SetCvLimitsForRegion(region);
    duty_cmd = ControlCv_Run(&app.cv, app.meas.vout, dt_s);

    region_setpoint = ControlCv_GetRampedSetpoint(&app.cv);
    old_region = region;
    region = App_UpdateRegionHysteresis(app.meas.vin, app.meas.vout, region_setpoint);
    if (region != old_region) {
        duty_cmd = app.cv.integrator;
    }

    App_SetCvLimitsForRegion(region);
    duty_cmd = App_LimitDutyForRegion(region, duty_cmd, &top_hit, &bottom_hit);
    PowerStage_SetRegion(region);
    app.mode_top_hits = top_hit ? 1U : 0U;
    app.mode_bottom_hits = bottom_hit ? 1U : 0U;

    App_ApplyRegionDuty(region, duty_cmd);

}

static void App_ControlTask(void)
{
    bool adc_ok;
    uint32_t uptime_ms;

    app.region_transition_count = 0U;

    adc_ok = Measurements_Update(&app.meas);
    App_UpdateFaultFlags(adc_ok);

    if (app.fault_flags != FAULT_NONE) {
        if (app.stage_enabled || app.enable_attempted || App_IsDriverAwake()) {
            App_LatchShutdown(app.fault_flags);
        } else {
            App_EnterIdle();
        }
        return;
    }

    if (app.requested_mode == MODE_IDLE) {
        App_EnterIdle();
        return;
    }

    uptime_ms = App_StartupHoldRemainingMs();
    if (uptime_ms > 0U) {
        PowerStage_SuspendOutputsKeepDriverOn();
        app.stage_enabled = false;
        app.active_mode = MODE_IDLE;
        return;
    }

    if (app.shutdown_latched) {
        App_EnterIdle();
        return;
    }

    if (App_HandleActiveDischarge()) {
        return;
    }

    if (app.coast_active) {
        app.coast_active = false;
        app.enable_attempted = false;
    }

    if (!app.stage_enabled) {
        float startup_duty = App_EstimateStartupDuty(app.meas.vin, app.cv.target_setpoint);
        PowerStage_Region_t startup_region = App_SelectRegion(app.meas.vin, app.cv.target_setpoint);

        if (app.enable_attempted) {
            App_EnterIdle();
            return;
        }

        if (startup_region != POWER_REGION_BUCK) {
            startup_duty = App_RegionStartupDuty(startup_region);
        }

        app.mode_top_hits = 0U;
        app.mode_bottom_hits = 0U;
        app.region_candidate = startup_region;
        app.region_candidate_count = 0U;
        app.region_old_debug = startup_region;
        app.region_new_debug = startup_region;
        (void)App_EstimateDutyForRegionChange(startup_region,
                                              startup_region,
                                              app.meas.vin,
                                              app.meas.vout,
                                              app.cv.target_setpoint);
        PowerStage_SetRegion(startup_region);
        App_SetCvLimitsForRegion(startup_region);
        App_ApplyRegionDuty(startup_region, startup_duty);
        ControlCv_Reset(&app.cv, startup_duty);

        app.enable_attempted = true;
        if (!PowerStage_Enable()) {
            app.fault_flags |= FAULT_DRIVER;
            App_LatchShutdown(app.fault_flags);
            return;
        }

        app.stage_enabled = true;
    }

    if (app.requested_mode == MODE_CV) {
        app.active_mode = MODE_CV;
        App_RunCvControl();
        return;
    }

    if (app.requested_mode == MODE_CC) {
        app.active_mode = MODE_CC;
        /* Stub pod przyszly regulator CC: na razie bezpiecznie bez wyjsc PWM. */
        if (app.stage_enabled) {
            PowerStage_Disable();
            app.stage_enabled = false;
        }
        return;
    }

    App_EnterIdle();
}

static void App_DebugTask(void)
{
    uint32_t now_ms = HAL_GetTick();
    int32_t vin_x100;
    int32_t vout_x100;
    int32_t iout_x100;
    int32_t set_x100;
    int32_t rset_x100;
    int32_t ilim_x100;
    int32_t duty_a_x10;
    int32_t duty_c_x10;
    int32_t est_duty_a_now_x10;
    int32_t est_duty_c_now_x10;
    int32_t last_est_duty_a_x10;
    int32_t last_est_duty_c_x10;
    int32_t pi_int_x10;
    float rset_v;
    float est_duty_a_now;
    float est_duty_c_now;
    uint8_t flt_level;
    uint8_t stby_level;
    uint8_t sd_level;
    uint8_t driver_awake;
    uint8_t adc_err;
    uint8_t ps_err;
    uint8_t ps_enabled;
    uint8_t ps_discharge;
    uint8_t enable_attempted;
    uint8_t shutdown_latched;
    uint8_t coast_active;
    uint8_t discharge_active;
    uint8_t hold_active;
    uint8_t transition_active;
    uint8_t adc_dma_running;
    uint32_t hold_ms;
    uint32_t region_cnt_debug;
    uint32_t adc_dma_updates;
    HAL_StatusTypeDef adc_hal_status;
    char fault_text[48];

    if ((uint32_t)(now_ms - app.last_debug_tick_ms) < APP_DEBUG_PERIOD_MS) {
        return;
    }

    app.last_debug_tick_ms = now_ms;

    vin_x100 = App_ToFixed(app.meas.vin, 100);
    vout_x100 = App_ToFixed(app.meas.vout, 100);
    iout_x100 = App_ToFixed(app.meas.iout, 100);
    set_x100 = App_ToFixed(app.cv.target_setpoint, 100);
    rset_v = ControlCv_GetRampedSetpoint(&app.cv);
    rset_x100 = App_ToFixed(rset_v, 100);
    ilim_x100 = App_ToFixed(app.current_limit_a, 100);
    duty_a_x10 = App_ToFixed(PowerStage_GetDutyA() * 100.0f, 10);
    duty_c_x10 = App_ToFixed(PowerStage_GetDutyC() * 100.0f, 10);
    App_EstimateDutyNow(PowerStage_GetRegion(), app.meas.vin, rset_v, &est_duty_a_now, &est_duty_c_now);
    est_duty_a_now_x10 = App_ToFixed(est_duty_a_now * 100.0f, 10);
    est_duty_c_now_x10 = App_ToFixed(est_duty_c_now * 100.0f, 10);
    last_est_duty_a_x10 = App_ToFixed(app.last_est_duty_a * 100.0f, 10);
    last_est_duty_c_x10 = App_ToFixed(app.last_est_duty_c * 100.0f, 10);
    pi_int_x10 = App_ToFixed(app.cv.integrator * 100.0f, 10);
    flt_level = (uint8_t)HAL_GPIO_ReadPin(FLT_GPIO_Port, FLT_Pin);
    stby_level = (uint8_t)HAL_GPIO_ReadPin(STBY_GPIO_Port, STBY_Pin);
    sd_level = (uint8_t)HAL_GPIO_ReadPin(SD_GPIO_Port, SD_Pin);
    driver_awake = ((stby_level != 0U) && (sd_level != 0U)) ? 1U : 0U;
    adc_err = Measurements_GetLastError();
    ps_err = PowerStage_GetLastError();
    ps_enabled = (uint8_t)PowerStage_IsEnabled();
    ps_discharge = (uint8_t)PowerStage_IsDischarging();
    enable_attempted = (uint8_t)app.enable_attempted;
    shutdown_latched = (uint8_t)app.shutdown_latched;
    coast_active = (uint8_t)app.coast_active;
    discharge_active = (uint8_t)app.discharge_active;
    adc_dma_running = (uint8_t)Measurements_IsDmaRunning();
    hold_ms = App_StartupHoldRemainingMs();
    hold_active = (hold_ms > 0U) ? 1U : 0U;
    transition_active = (app.region_transition_count > 0U) ? 1U : 0U;
    region_cnt_debug = (transition_active != 0U) ? app.region_last_confirm_count : app.region_candidate_count;
    adc_dma_updates = Measurements_GetDmaUpdateCount();
    adc_hal_status = Measurements_GetLastHalStatus();

    App_FaultText(app.fault_flags, fault_text, sizeof(fault_text));

    Debug_Printf("MODE=%s REQ=%s STATE=%s WHY=%s REGION=%s VIN=%s%ld.%02ld VOUT=%s%ld.%02ld IOUT=%s%ld.%02ld "
                 "DUTY_A=%s%ld.%01ld DUTY_C=%s%ld.%01ld SET=%s%ld.%02ld RSET=%s%ld.%02ld "
                 "ILIM=%s%ld.%02ld FAULT=%s FLT=%u STBY=%u SD=%u DRV=%u EN=%u OUT=%u TRY=%u LATCH=%u PS_ERR=%u "
                 "REGION_CAND=%s REGION_CNT=%lu REGION_LIVE_CNT=%lu REGION_LAST_CNT=%lu TRANS=%u ROLD=%s RNEW=%s "
                 "EST_A_NOW=%s%ld.%01ld EST_C_NOW=%s%ld.%01ld LAST_EST_A=%s%ld.%01ld LAST_EST_C=%s%ld.%01ld PI_INT=%s%ld.%01ld "
                 "COAST=%u DISCH=%u HOLD=%u HOLD_MS=%lu HIT[%lu,%lu] OCP_CNT=%lu IDLE_CAP=%u RAW[%u,%u,%u] ADC_ERR=%u ADC_ST=%u "
                 "ADC_DMA=%u ADC_UPD=%lu\r\n",
                 App_ModeText(app.active_mode),
                 App_ModeText(app.requested_mode),
                 App_StateText(),
                 App_WhyText(),
                 App_RegionText(PowerStage_GetRegion()),
                 (vin_x100 < 0) ? "-" : "",
                 App_IntPart(vin_x100, 100),
                 App_FracPart(vin_x100, 100),
                 (vout_x100 < 0) ? "-" : "",
                 App_IntPart(vout_x100, 100),
                 App_FracPart(vout_x100, 100),
                 (iout_x100 < 0) ? "-" : "",
                 App_IntPart(iout_x100, 100),
                 App_FracPart(iout_x100, 100),
                 (duty_a_x10 < 0) ? "-" : "",
                 App_IntPart(duty_a_x10, 10),
                 App_FracPart(duty_a_x10, 10),
                 (duty_c_x10 < 0) ? "-" : "",
                 App_IntPart(duty_c_x10, 10),
                 App_FracPart(duty_c_x10, 10),
                 (set_x100 < 0) ? "-" : "",
                 App_IntPart(set_x100, 100),
                 App_FracPart(set_x100, 100),
                 (rset_x100 < 0) ? "-" : "",
                 App_IntPart(rset_x100, 100),
                 App_FracPart(rset_x100, 100),
                 (ilim_x100 < 0) ? "-" : "",
                 App_IntPart(ilim_x100, 100),
                 App_FracPart(ilim_x100, 100),
                 fault_text,
                 (unsigned int)flt_level,
                 (unsigned int)stby_level,
                 (unsigned int)sd_level,
                 (unsigned int)driver_awake,
                 (unsigned int)ps_enabled,
                 (unsigned int)ps_enabled,
                 (unsigned int)enable_attempted,
                 (unsigned int)shutdown_latched,
                 (unsigned int)ps_err,
                 App_RegionText(app.region_candidate),
                 (unsigned long)region_cnt_debug,
                 (unsigned long)app.region_candidate_count,
                 (unsigned long)app.region_last_confirm_count,
                 (unsigned int)transition_active,
                 App_RegionText(app.region_old_debug),
                 App_RegionText(app.region_new_debug),
                 (est_duty_a_now_x10 < 0) ? "-" : "",
                 App_IntPart(est_duty_a_now_x10, 10),
                 App_FracPart(est_duty_a_now_x10, 10),
                 (est_duty_c_now_x10 < 0) ? "-" : "",
                 App_IntPart(est_duty_c_now_x10, 10),
                 App_FracPart(est_duty_c_now_x10, 10),
                 (last_est_duty_a_x10 < 0) ? "-" : "",
                 App_IntPart(last_est_duty_a_x10, 10),
                 App_FracPart(last_est_duty_a_x10, 10),
                 (last_est_duty_c_x10 < 0) ? "-" : "",
                 App_IntPart(last_est_duty_c_x10, 10),
                 App_FracPart(last_est_duty_c_x10, 10),
                 (pi_int_x10 < 0) ? "-" : "",
                 App_IntPart(pi_int_x10, 10),
                 App_FracPart(pi_int_x10, 10),
                 (unsigned int)coast_active,
                 (unsigned int)((discharge_active != 0U) || (ps_discharge != 0U)),
                 (unsigned int)hold_active,
                 (unsigned long)hold_ms,
                 (unsigned long)app.mode_top_hits,
                 (unsigned long)app.mode_bottom_hits,
                 (unsigned long)app.ocp_hit_count,
                 (unsigned int)((app.active_mode == MODE_IDLE) && (app.meas.vout > 0.5f)),
                 (unsigned int)app.meas.raw_vin,
                 (unsigned int)app.meas.raw_vout,
                 (unsigned int)app.meas.raw_iout,
                 (unsigned int)adc_err,
                 (unsigned int)adc_hal_status,
                 (unsigned int)adc_dma_running,
                 (unsigned long)adc_dma_updates);
}

void App_Init(HRTIM_HandleTypeDef *hhrtim,
              ADC_HandleTypeDef *hadc1,
              ADC_HandleTypeDef *hadc2,
              UART_HandleTypeDef *huart_debug)
{
    int32_t set_x100;
    int32_t ovp_x100;
    int32_t ocp_x100;
    int32_t uvlo_x100;
    int32_t duty_min_x100;
    int32_t duty_max_x100;
    uint32_t reset_flags;

    memset(&app, 0, sizeof(app));

    Debug_Init(huart_debug);
    App_TimebaseInit();

    PowerStage_Init(hhrtim);
    Measurements_Init(hadc1, hadc2);

    ControlCv_Init(&app.cv,
                   CV_KP,
                   CV_KI,
                   DUTY_MIN,
                   DUTY_MAX,
                   CV_SOFTSTART_SLEW_V_PER_S);
    ControlCv_SetTarget(&app.cv, VOUT_SETPOINT);

    app.requested_mode = MODE_IDLE;
    app.active_mode = MODE_IDLE;
    app.fault_flags = FAULT_NONE;
    app.current_limit_a = IOUT_OCP_LIMIT;
    app.startup_tick_ms = HAL_GetTick();
    app.last_debug_tick_ms = app.startup_tick_ms;
    app.last_led_tick_ms = app.startup_tick_ms;
    app.last_ctrl_tick_us = App_Micros();
    app.latched_fault_flags = FAULT_NONE;
    app.ocp_hit_count = 0U;
    app.discharge_start_tick_ms = 0U;
    app.region_candidate = POWER_REGION_BUCK;
    app.region_candidate_count = 0U;
    app.region_last_confirm_count = 0U;
    app.region_transition_count = 0U;
    app.region_old_debug = POWER_REGION_BUCK;
    app.region_new_debug = POWER_REGION_BUCK;
    app.last_est_duty_a = 0.0f;
    app.last_est_duty_c = 0.0f;
    app.enable_attempted = false;
    app.shutdown_latched = false;
    app.led_blink_state = false;
    app.coast_active = false;
    app.discharge_active = false;

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

    PowerStage_ForceSafeState();
    reset_flags = RCC->CSR;

    set_x100 = App_ToFixed(VOUT_SETPOINT, 100);
    ovp_x100 = App_ToFixed(VOUT_OVP_LIMIT, 100);
    ocp_x100 = App_ToFixed(app.current_limit_a, 100);
    uvlo_x100 = App_ToFixed(VIN_UVLO_LIMIT, 100);
    duty_min_x100 = App_ToFixed(DUTY_MIN, 100);
    duty_max_x100 = App_ToFixed(DUTY_MAX, 100);

    Debug_Printf("\r\n[APP] PSU bring-up start. Mode=IDLE, request=IDLE, ctrl=%lu Hz, enable_delay=%lu ms\r\n",
                 (unsigned long)APP_CTRL_FREQ_HZ,
                 (unsigned long)APP_STARTUP_HOLD_MS);
    Debug_Printf("[APP] RESET_FLAGS=0x%08lX\r\n", (unsigned long)reset_flags);
    Debug_Printf("[APP] SET=%ld.%02ldV OVP=%ld.%02ldV OCP=%ld.%02ldA UVLO=%ld.%02ldV "
                 "DUTY=[%ld.%02ld..%ld.%02ld]\r\n",
                 App_IntPart(set_x100, 100), App_FracPart(set_x100, 100),
                 App_IntPart(ovp_x100, 100), App_FracPart(ovp_x100, 100),
                 App_IntPart(ocp_x100, 100), App_FracPart(ocp_x100, 100),
                 App_IntPart(uvlo_x100, 100), App_FracPart(uvlo_x100, 100),
                 App_IntPart(duty_min_x100, 100), App_FracPart(duty_min_x100, 100),
                 App_IntPart(duty_max_x100, 100), App_FracPart(duty_max_x100, 100));
    Debug_Printf("[APP] NOTE: in IDLE VOUT can stay charged without load; this is not PWM activity.\r\n");
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

void App_Run(void)
{
    uint32_t now_us = App_Micros();

    App_LedTask();

    if ((uint32_t)(now_us - app.last_ctrl_tick_us) >= APP_CTRL_PERIOD_US) {
        app.last_ctrl_tick_us = now_us;
        App_ControlTask();
    }

    App_DebugTask();
}

void App_SetRequestedMode(App_Mode_t mode)
{
    if (mode == MODE_IDLE) {
        app.requested_mode = MODE_IDLE;
        app.enable_attempted = false;
        App_EnterIdle();
        return;
    }

    app.enable_attempted = false;
    app.requested_mode = mode;
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
    app.coast_active = false;
    app.discharge_active = false;
    app.shutdown_latched = false;
    app.enable_attempted = false;
}

void App_SetCvSetpoint(float setpoint_v)
{
    if (setpoint_v < 0.0f) {
        setpoint_v = 0.0f;
    }

    ControlCv_SetTarget(&app.cv, setpoint_v);
}

float App_GetCvSetpoint(void)
{
    return app.cv.target_setpoint;
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
