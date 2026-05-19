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
#define DUTY_MAX                               0.999f
#define DUTY_BUCK_MIN                          0.001f
#define DUTY_BUCK_MAX                          0.95f
#define DUTY_BOOST_MIN                         0.05f
#define DUTY_BOOST_MAX                         0.95f
#define DUTY_MIXED_B_MIN                       0.001f
#define DUTY_MIXED_B_MAX                       0.45f
#define CV_STARTUP_DUTY                        0.15f
#define CV_STARTUP_DUTY_MAX                    0.30f
#define DUTY_MIXED_A                           0.80f
#define DUTY_MIXED_B_START                     0.20f
#define DUTY_BOOST_B_START                     0.12f
#define MODE_SWITCH_HIT_COUNT                  1000U

#define CV_KP                                  0.040f
#define CV_KI                                  18.0f
#define CV_SOFTSTART_SLEW_V_PER_S              25.0f

#define REGION_BUCK_MARGIN_V                   0.80f
#define REGION_BOOST_MARGIN_V                  0.80f

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
    uint32_t latched_fault_flags;
    float current_limit_a;
    bool enable_attempted;
    bool shutdown_latched;
    bool stage_enabled;
    bool timebase_ready;
    bool led_blink_state;
} App_Context_t;

static App_Context_t app;

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
    if (app.stage_enabled) {
        PowerStage_Disable();
        app.stage_enabled = false;
    }

    ControlCv_Reset(&app.cv, DUTY_MIN);
    app.cv.ramped_setpoint = 0.0f;
    app.mode_top_hits = 0U;
    app.mode_bottom_hits = 0U;
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
    if (vin > (setpoint_v + REGION_BUCK_MARGIN_V)) {
        return POWER_REGION_BUCK;
    }
    if (vin < (setpoint_v - REGION_BOOST_MARGIN_V)) {
        return POWER_REGION_BOOST;
    }
    return POWER_REGION_BUCK_BOOST;
}

static float App_EstimateStartupDuty(float vin, float setpoint_v)
{
    float duty = CV_STARTUP_DUTY;

    if ((vin > 1.0f) && (setpoint_v > 1.0f)) {
        if (vin > (setpoint_v + REGION_BUCK_MARGIN_V)) {
            duty = setpoint_v / vin;
        } else if (vin < (setpoint_v - REGION_BOOST_MARGIN_V)) {
            duty = 1.0f - (vin / setpoint_v);
        }
    }

    return App_Clamp(duty, CV_STARTUP_DUTY, CV_STARTUP_DUTY_MAX);
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
    float min_value = DUTY_BUCK_MIN;
    float max_value = DUTY_BUCK_MAX;

    if (region == POWER_REGION_BUCK_BOOST) {
        min_value = DUTY_MIXED_B_MIN;
        max_value = DUTY_MIXED_B_MAX;
    } else if (region == POWER_REGION_BOOST) {
        min_value = DUTY_BOOST_MIN;
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

static void App_UpdateModeHitCounters(bool top_hit, bool bottom_hit)
{
    if (top_hit) {
        if (app.mode_top_hits < MODE_SWITCH_HIT_COUNT) {
            app.mode_top_hits++;
        }
    } else if (app.mode_top_hits > 0U) {
        app.mode_top_hits--;
    }

    if (bottom_hit) {
        if (app.mode_bottom_hits < MODE_SWITCH_HIT_COUNT) {
            app.mode_bottom_hits++;
        }
    } else if (app.mode_bottom_hits > 0U) {
        app.mode_bottom_hits--;
    }
}

static bool App_EvalRegionSwitch(PowerStage_Region_t *region)
{
    bool switched = false;

    if (region == NULL) {
        return false;
    }

    switch (*region) {
        case POWER_REGION_BUCK:
            if (app.mode_top_hits >= MODE_SWITCH_HIT_COUNT) {
                *region = POWER_REGION_BUCK_BOOST;
                switched = true;
            }
            break;

        case POWER_REGION_BUCK_BOOST:
            if (app.mode_top_hits >= MODE_SWITCH_HIT_COUNT) {
                *region = POWER_REGION_BOOST;
                switched = true;
            } else if (app.mode_bottom_hits >= MODE_SWITCH_HIT_COUNT) {
                *region = POWER_REGION_BUCK;
                switched = true;
            }
            break;

        case POWER_REGION_BOOST:
        default:
            if (app.mode_bottom_hits >= MODE_SWITCH_HIT_COUNT) {
                *region = POWER_REGION_BUCK_BOOST;
                switched = true;
            }
            break;
    }

    if (switched) {
        app.mode_top_hits = 0U;
        app.mode_bottom_hits = 0U;
    }

    return switched;
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

static void App_UpdateFaultFlags(bool adc_ok)
{
    uint32_t flags = app.latched_fault_flags;

    if (!adc_ok) {
        flags |= FAULT_ADC;
    }

    if (app.stage_enabled && PowerStage_IsFaultActive()) {
        flags |= FAULT_DRIVER;
    }

    if (adc_ok) {
        if (app.meas.vin < VIN_UVLO_LIMIT) {
            flags |= FAULT_UVIN;
        }
        if (app.meas.vout > VOUT_OVP_LIMIT) {
            flags |= FAULT_OVP;
        }
        if (app.meas.iout > app.current_limit_a) {
            flags |= FAULT_OCP;
        }
    }

    app.fault_flags = flags;
}

static void App_RunCvControl(void)
{
    float duty_cmd;
    bool top_hit;
    bool bottom_hit;
    bool switched;
    PowerStage_Region_t region;

    region = PowerStage_GetRegion();
    App_SetCvLimitsForRegion(region);
    duty_cmd = ControlCv_Run(&app.cv, app.meas.vout, 1.0f / (float)APP_CTRL_FREQ_HZ);
    duty_cmd = App_LimitDutyForRegion(region, duty_cmd, &top_hit, &bottom_hit);
    App_UpdateModeHitCounters(top_hit, bottom_hit);
    switched = App_EvalRegionSwitch(&region);
    PowerStage_SetRegion(region);

    if (switched) {
        App_SetCvLimitsForRegion(region);
        duty_cmd = App_RegionStartupDuty(region);
        ControlCv_Reset(&app.cv, duty_cmd);
    }

    App_ApplyRegionDuty(region, duty_cmd);
}

static void App_ControlTask(void)
{
    bool adc_ok;
    uint32_t uptime_ms;

    adc_ok = Measurements_Update(&app.meas);
    App_UpdateFaultFlags(adc_ok);

    if (app.fault_flags != FAULT_NONE) {
        if (app.stage_enabled || app.enable_attempted) {
            App_LatchShutdown(app.fault_flags);
        } else {
            App_EnterIdle();
        }
        return;
    }

    uptime_ms = HAL_GetTick() - app.startup_tick_ms;
    if (uptime_ms < APP_STARTUP_HOLD_MS) {
        App_EnterIdle();
        return;
    }

    if (app.requested_mode == MODE_IDLE) {
        App_EnterIdle();
        return;
    }

    if (app.shutdown_latched) {
        App_EnterIdle();
        return;
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
    int32_t duty_a_x10;
    int32_t duty_c_x10;
    uint8_t flt_level;
    uint8_t adc_err;
    uint8_t ps_err;
    uint8_t ps_enabled;
    uint8_t enable_attempted;
    uint8_t shutdown_latched;
    uint8_t adc_dma_running;
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
    duty_a_x10 = App_ToFixed(PowerStage_GetDutyA() * 100.0f, 10);
    duty_c_x10 = App_ToFixed(PowerStage_GetDutyC() * 100.0f, 10);
    flt_level = (uint8_t)HAL_GPIO_ReadPin(FLT_GPIO_Port, FLT_Pin);
    adc_err = Measurements_GetLastError();
    ps_err = PowerStage_GetLastError();
    ps_enabled = (uint8_t)PowerStage_IsEnabled();
    enable_attempted = (uint8_t)app.enable_attempted;
    shutdown_latched = (uint8_t)app.shutdown_latched;
    adc_dma_running = (uint8_t)Measurements_IsDmaRunning();
    adc_dma_updates = Measurements_GetDmaUpdateCount();
    adc_hal_status = Measurements_GetLastHalStatus();

    App_FaultText(app.fault_flags, fault_text, sizeof(fault_text));

    Debug_Printf("MODE=%s REGION=%s VIN=%s%ld.%02ld VOUT=%s%ld.%02ld IOUT=%s%ld.%02ld "
                 "DUTY_A=%s%ld.%01ld DUTY_C=%s%ld.%01ld SET=%s%ld.%02ld FAULT=%s FLT=%u "
                 "EN=%u TRY=%u LATCH=%u PS_ERR=%u HIT[%lu,%lu] RAW[%u,%u,%u] ADC_ERR=%u ADC_ST=%u "
                 "ADC_DMA=%u ADC_UPD=%lu\r\n",
                 App_ModeText(app.active_mode),
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
                 fault_text,
                 (unsigned int)flt_level,
                 (unsigned int)ps_enabled,
                 (unsigned int)enable_attempted,
                 (unsigned int)shutdown_latched,
                 (unsigned int)ps_err,
                 (unsigned long)app.mode_top_hits,
                 (unsigned long)app.mode_bottom_hits,
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
    app.enable_attempted = false;
    app.shutdown_latched = false;
    app.led_blink_state = false;

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

    if (app.requested_mode == MODE_IDLE) {
        app.startup_tick_ms = HAL_GetTick();
        app.enable_attempted = false;
    }

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
