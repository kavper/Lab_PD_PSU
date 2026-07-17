#include "power_manager.h"

#include "debug_uart.h"
#include "tps_int_event.h"

#include <string.h>

#define PM_MODE_POLL_MS              100U
#define PM_BOOT_FLAGS_POLL_MS       1000U
#define PM_TPS_STEP_MS                20U
#define PM_TPS_STATUS_STALE_MS      1500U
#define PM_PD_LOG_MS                1000U
#define PM_BQ_START_DELAY_MS        1000U
#define PM_BQ_INIT_STEP_MS           200U
#define PM_BQ_RETRY_MS              2000U
#define PM_BQ_TELEMETRY_MS          1000U
#define PM_BQ_CONFIG_MS             5000U
#define PM_HARD_RESET_HOLDOFF_MS     250U
#define PM_ERROR_LOG_MS             1000U
#define PM_POLICY_SETTLE_MS          500U
#define PM_POLICY_STEP_MS             20U
#define PM_POLICY_CAP_RETRY_MS       1500U
#define PM_POLICY_ROLE_DRIFT_MS       750U
#define PM_POLICY_SWAP_RETRY_MS      3000U
#define PM_POLICY_MAX_CAP_ATTEMPTS      3U
typedef enum {
    PM_JOB_NONE = 0,
    PM_JOB_READ_MODE,
    PM_JOB_READ_BOOT_FLAGS,
    PM_JOB_READ_PORT_CONFIG,
    PM_JOB_WRITE_PORT_CONFIG,
    PM_JOB_READ_INT_MASK,
    PM_JOB_WRITE_INT_MASK,
    PM_JOB_READ_STATUS,
    PM_JOB_READ_POWER_PATH,
    PM_JOB_READ_POWER_STATUS,
    PM_JOB_READ_PD_STATUS,
    PM_JOB_READ_ADC,
    PM_JOB_READ_ACTIVE_PDO,
    PM_JOB_READ_ACTIVE_RDO,
    PM_JOB_READ_EVENT,
    PM_JOB_CLEAR_EVENT,
    PM_JOB_GET_SINK_CAPS,
    PM_JOB_READ_SINK_CAPS,
    PM_JOB_READ_SOURCE_CAPS,
    PM_JOB_SWAP_TO_SOURCE,
    PM_JOB_SWAP_TO_SINK,
    PM_JOB_BQ_READ_OPTION0,
    PM_JOB_BQ_WRITE_STARTUP_OPTION0,
    PM_JOB_BQ_READ_OPTION4,
    PM_JOB_BQ_WRITE_STARTUP_OPTION4,
    PM_JOB_BQ_WRITE_ADC,
    PM_JOB_BQ_VERIFY_ADC,
    PM_JOB_BQ_READ_ID,
    PM_JOB_BQ_READ_CONFIG_BLOCK,
    PM_JOB_BQ_READ_STATUS_BLOCK,
    PM_JOB_BQ_READ_ADC_BLOCK
} PowerManager_Job_t;

typedef enum {
    PM_BQ_INIT_WAIT = 0,
    PM_BQ_INIT_READ_ID,
    PM_BQ_INIT_READ_OPTION0,
    PM_BQ_INIT_WRITE_OPTION0,
    PM_BQ_INIT_VERIFY_OPTION0,
    PM_BQ_INIT_READ_OPTION4,
    PM_BQ_INIT_WRITE_OPTION4,
    PM_BQ_INIT_VERIFY_OPTION4,
    PM_BQ_INIT_WRITE_ADC,
    PM_BQ_INIT_VERIFY_ADC,
    PM_BQ_INIT_DONE
} PowerManager_BqInit_t;

typedef enum {
    PM_POLICY_IDLE = 0,
    PM_POLICY_WAIT_SETTLE,
    PM_POLICY_GET_SINK_CAPS,
    PM_POLICY_READ_SINK_CAPS,
    PM_POLICY_READ_SOURCE_CAPS,
    PM_POLICY_DECIDE,
    PM_POLICY_SWAP_TO_SOURCE,
    PM_POLICY_SWAP_TO_SINK,
    PM_POLICY_DONE
} PowerManager_PolicyPhase_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    TPS25751_Device_t tps;
    BQ25731_Device_t bq;
    PowerManager_Status_t status;

    PowerManager_Job_t job;
    PowerManager_Job_t last_job;
    PowerManager_BqInit_t bq_init;
    PowerManager_PolicyPhase_t policy_phase;
    uint8_t telemetry_phase;
    uint8_t bq_telemetry_phase;
    uint8_t port_config[TPS25751_PORT_CONFIG_LEN];
    uint8_t int_mask[TPS_INT_EVENT_BYTES];
    uint8_t event_to_clear[TPS_INT_EVENT_BYTES];

    bool initialized;
    bool mode_update_pending;
    bool port_write_pending;
    bool event_mask_ready;
    bool event_mask_write_pending;
    bool event_clear_pending;

    uint32_t next_mode_ms;
    uint32_t next_boot_flags_ms;
    uint32_t next_tps_step_ms;
    uint32_t next_bq_action_ms;
    uint32_t next_bq_telemetry_ms;
    uint32_t next_bq_config_ms;
    uint32_t policy_next_ms;
    uint32_t app_seen_ms;
    uint32_t status_updated_ms;
    uint32_t hard_reset_holdoff_until_ms;
    uint32_t last_pd_log_ms;
    uint32_t last_error_log_ms;
    uint32_t tps_error_count;
    uint32_t bq_error_count;
    uint32_t policy_role_mismatch_since_ms;
    uint16_t bq_startup_option0_target;
    uint16_t bq_startup_option4_target;
    uint8_t previous_hard_reset_reason;
    uint8_t policy_cap_attempts;
    bool previous_attached;
    bool policy_swap_attempted;
    bool partner_source_caps_current;
    bool partner_sink_observed;
    TPS25751_PowerRole_t policy_desired_role;
    TPS25751_Capabilities_t partner_sink_caps;
    TPS25751_Capabilities_t partner_source_caps;
} PowerManager_Context_t;

static PowerManager_Context_t g_pm;

static const char *PowerManager_RoleToString(TPS25751_PowerRole_t role);

static bool PowerManager_TickReached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static const char *PowerManager_StateToString(PowerManager_State_t state)
{
    switch (state) {
        case POWER_MANAGER_INIT: return "INIT";
        case POWER_MANAGER_TPS_WAIT_APP: return "TPS_WAIT_APP";
        case POWER_MANAGER_TPS_READY: return "TPS_READY";
        case POWER_MANAGER_BQ_PROBE: return "BQ_PROBE";
        case POWER_MANAGER_BQ_ADC_SETUP: return "BQ_ADC_SETUP";
        case POWER_MANAGER_RUN: return "RUN";
        case POWER_MANAGER_DEGRADED: return "DEGRADED";
        default: return "FAULT";
    }
}

static const char *PowerManager_UserModeToString(PowerManager_UserMode_t mode)
{
    switch (mode) {
        case POWER_MANAGER_USER_AUTO: return "AUTO";
        case POWER_MANAGER_USER_SINK_ONLY: return "SINK_ONLY";
        case POWER_MANAGER_USER_SOURCE_ONLY: return "SOURCE_ONLY";
        default: return "OFF";
    }
}

static const char *PowerManager_JobToString(PowerManager_Job_t job)
{
    switch (job) {
        case PM_JOB_READ_MODE: return "READ_MODE";
        case PM_JOB_READ_BOOT_FLAGS: return "READ_BOOT_FLAGS";
        case PM_JOB_READ_PORT_CONFIG: return "READ_PORT_CONFIG";
        case PM_JOB_WRITE_PORT_CONFIG: return "WRITE_PORT_CONFIG";
        case PM_JOB_READ_INT_MASK: return "READ_INT_MASK";
        case PM_JOB_WRITE_INT_MASK: return "WRITE_INT_MASK";
        case PM_JOB_READ_STATUS: return "READ_STATUS";
        case PM_JOB_READ_POWER_PATH: return "READ_POWER_PATH";
        case PM_JOB_READ_POWER_STATUS: return "READ_POWER_STATUS";
        case PM_JOB_READ_PD_STATUS: return "READ_PD_STATUS";
        case PM_JOB_READ_ADC: return "READ_TPS_ADC";
        case PM_JOB_READ_ACTIVE_PDO: return "READ_ACTIVE_PDO";
        case PM_JOB_READ_ACTIVE_RDO: return "READ_ACTIVE_RDO";
        case PM_JOB_READ_EVENT: return "READ_EVENT";
        case PM_JOB_CLEAR_EVENT: return "CLEAR_EVENT";
        case PM_JOB_GET_SINK_CAPS: return "GET_SINK_CAPS";
        case PM_JOB_READ_SINK_CAPS: return "READ_SINK_CAPS";
        case PM_JOB_READ_SOURCE_CAPS: return "READ_SOURCE_CAPS";
        case PM_JOB_SWAP_TO_SOURCE: return "SWAP_TO_SOURCE";
        case PM_JOB_SWAP_TO_SINK: return "SWAP_TO_SINK";
        case PM_JOB_BQ_READ_OPTION0: return "BQ_READ_OPTION0";
        case PM_JOB_BQ_WRITE_STARTUP_OPTION0: return "BQ_WRITE_QUIET_OPTION0";
        case PM_JOB_BQ_READ_OPTION4: return "BQ_READ_OPTION4";
        case PM_JOB_BQ_WRITE_STARTUP_OPTION4: return "BQ_WRITE_DITHER_OPTION4";
        case PM_JOB_BQ_WRITE_ADC: return "BQ_WRITE_ADC_ONLY";
        case PM_JOB_BQ_VERIFY_ADC: return "BQ_VERIFY_ADC";
        case PM_JOB_BQ_READ_ID: return "BQ_READ_ID";
        case PM_JOB_BQ_READ_CONFIG_BLOCK: return "BQ_READ_CONFIG";
        case PM_JOB_BQ_READ_STATUS_BLOCK: return "BQ_READ_STATUS";
        case PM_JOB_BQ_READ_ADC_BLOCK: return "BQ_READ_ADC_BLOCK";
        default: return "NONE";
    }
}

static void PowerManager_LogError(const char *source,
                                  TPS25751_Status_t status,
                                  uint32_t count,
                                  uint32_t now_ms)
{
    if ((count != 1U) &&
        !PowerManager_TickReached(now_ms,
                                  g_pm.last_error_log_ms + PM_ERROR_LOG_MS)) {
        return;
    }
    g_pm.last_error_log_ms = now_ms;
    Debug_Printf("[PM-ERR] src=%s job=%s status=%s reg=0x%02X req=%u got=%u task=%u hal=0x%08lX i2c_state=0x%08lX count=%lu",
                 source,
                 PowerManager_JobToString(g_pm.last_job),
                 TPS25751_StatusToString(status),
                 g_pm.tps.register_address,
                 g_pm.tps.requested_length,
                 g_pm.tps.reported_length,
                 g_pm.tps.task_return_code,
                 (unsigned long)g_pm.tps.hal_error,
                 (unsigned long)HAL_I2C_GetState(g_pm.hi2c),
                 (unsigned long)count);
}

static void PowerManager_SetState(PowerManager_State_t state)
{
    if (g_pm.status.state != state) {
        g_pm.status.state = state;
        Debug_Printf("[PM] state=%s", PowerManager_StateToString(state));
    }
}

static void PowerManager_RecordError(PowerManager_ErrorSource_t source,
                                     uint32_t code,
                                     uint8_t reg,
                                     uint32_t now_ms)
{
    g_pm.status.last_error.source = source;
    g_pm.status.last_error.code = code;
    g_pm.status.last_error.reg = reg;
    g_pm.status.last_error.tick_ms = now_ms;
}

static TPS25751_PortMode_t PowerManager_MapPortMode(
    PowerManager_UserMode_t mode)
{
    switch (mode) {
        case POWER_MANAGER_USER_SINK_ONLY: return TPS25751_PORT_SINK_ONLY;
        case POWER_MANAGER_USER_SOURCE_ONLY: return TPS25751_PORT_SOURCE_ONLY;
        case POWER_MANAGER_USER_OFF: return TPS25751_PORT_DISABLED;
        default: return TPS25751_PORT_DRP;
    }
}

static void PowerManager_SetOtgPin(bool high)
{
    if (g_pm.status.otg_pin_high != high) {
        HAL_GPIO_WritePin(OTG_EN_GPIO_Port, OTG_EN_Pin,
                          high ? GPIO_PIN_SET : GPIO_PIN_RESET);
        g_pm.status.otg_pin_high = high;
    }
}

static void PowerManager_UpdateOtgGate(uint32_t now_ms)
{
    bool mode_allows_source;
    bool status_is_fresh;
    bool allow;

    if ((!g_pm.status.tps.attached) ||
        (g_pm.status.tps.role != TPS25751_ROLE_SOURCE) ||
        (g_pm.status.requested_mode == POWER_MANAGER_USER_SINK_ONLY) ||
        (g_pm.status.requested_mode == POWER_MANAGER_USER_OFF)) {
        g_pm.status.source_fault_latched = false;
    }

    mode_allows_source =
        (g_pm.status.applied_mode == POWER_MANAGER_USER_AUTO) ||
        (g_pm.status.applied_mode == POWER_MANAGER_USER_SOURCE_ONLY);
    status_is_fresh = (g_pm.status_updated_ms != 0U) &&
        ((uint32_t)(now_ms - g_pm.status_updated_ms) <=
         PM_TPS_STATUS_STALE_MS);

    allow = (g_pm.status.tps.mode == TPS25751_MODE_APP) &&
            g_pm.status.applied_mode_valid && !g_pm.mode_update_pending &&
            mode_allows_source && status_is_fresh &&
            g_pm.status.tps.attached &&
            (g_pm.status.tps.connection_state >= 6U) &&
            (g_pm.status.tps.role == TPS25751_ROLE_SOURCE) &&
            !g_pm.status.source_fault_latched &&
            PowerManager_TickReached(now_ms,
                                     g_pm.hard_reset_holdoff_until_ms);
    if (allow != g_pm.status.otg_pin_high) {
        Debug_Printf("[PD-OTG] pin=%s attached=%u conn=%u role=%s fresh=%u fault=%u holdoff=%u",
                     allow ? "HIGH" : "LOW",
                     g_pm.status.tps.attached ? 1U : 0U,
                     g_pm.status.tps.connection_state,
                     PowerManager_RoleToString(g_pm.status.tps.role),
                     status_is_fresh ? 1U : 0U,
                     g_pm.status.source_fault_latched ? 1U : 0U,
                     PowerManager_TickReached(
                         now_ms, g_pm.hard_reset_holdoff_until_ms) ? 0U : 1U);
    }
    PowerManager_SetOtgPin(allow);
}

static void PowerManager_UpdatePdSnapshot(void)
{
    PowerManager_PdSnapshot_t *snapshot = &g_pm.status.pd_snapshot;
    const TPS25751_Telemetry_t *t = &g_pm.status.tps;
    uint32_t current_ma = 0U;

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->attached = t->attached;
    snapshot->data_role_dfp = t->data_role_dfp;
    snapshot->active_pdo_raw = t->active_pdo_raw;
    snapshot->active_rdo_raw = t->active_rdo_raw;

    if (t->role == TPS25751_ROLE_SOURCE) {
        snapshot->power_role = POWER_MANAGER_PD_ROLE_SOURCE;
    } else if (t->role == TPS25751_ROLE_SINK) {
        snapshot->power_role = POWER_MANAGER_PD_ROLE_SINK;
    } else {
        snapshot->power_role = POWER_MANAGER_PD_ROLE_UNKNOWN;
    }

    if (t->active_pdo.valid) {
        snapshot->contract_voltage_mv = t->active_pdo.voltage_mv;
        current_ma = t->active_pdo.current_ma;
    }
    if (t->active_rdo.valid &&
        (t->active_rdo.operating_current_ma != 0U)) {
        current_ma = t->active_rdo.operating_current_ma;
    }
    snapshot->contract_current_ma = current_ma;
    snapshot->contract_power_mw =
        snapshot->contract_voltage_mv * current_ma / 1000U;

    if (!snapshot->attached || (snapshot->contract_power_mw == 0U)) {
        snapshot->power_class = POWER_MANAGER_POWER_NO_INPUT;
    } else if (snapshot->contract_power_mw <= 15000U) {
        snapshot->power_class = POWER_MANAGER_POWER_LOW;
    } else if (snapshot->contract_power_mw <= 30000U) {
        snapshot->power_class = POWER_MANAGER_POWER_MEDIUM;
    } else if (snapshot->contract_power_mw <= 60000U) {
        snapshot->power_class = POWER_MANAGER_POWER_HIGH;
    } else {
        snapshot->power_class = POWER_MANAGER_POWER_FULL;
    }
}

static bool PowerManager_HasSinkContract(void)
{
    return g_pm.status.tps.attached &&
           (g_pm.status.tps.connection_state >= 6U) &&
           (g_pm.status.tps.role == TPS25751_ROLE_SINK) &&
           g_pm.status.tps.active_pdo.valid &&
           g_pm.status.tps.active_rdo.valid;
}

static void PowerManager_LogPd(uint32_t now_ms)
{
    const char *role = "NONE";

    if (!PowerManager_TickReached(now_ms,
                                  g_pm.last_pd_log_ms + PM_PD_LOG_MS)) {
        return;
    }
    g_pm.last_pd_log_ms = now_ms;

    if (g_pm.status.tps.attached &&
        (g_pm.status.tps.role == TPS25751_ROLE_SOURCE)) {
        role = "SOURCE";
    } else if (g_pm.status.tps.attached &&
               (g_pm.status.tps.role == TPS25751_ROLE_SINK)) {
        role = "SINK";
    }
    Debug_Printf("[PD] mode=%s plug=%u conn=%u role=%s desired=%s sink_seen=%u pdo_owner=TPS legacy=%lu OTG_EN=%u VBUS=%lumV contract=%lumV/%lumA/%lumW PDO=0x%08lX RDO=0x%08lX PP5V=%u PPHV=%u HR=%u STATUS=0x%02lX%08lX",
                 PowerManager_UserModeToString(g_pm.status.requested_mode),
                 g_pm.status.tps.attached ? 1U : 0U,
                 g_pm.status.tps.connection_state,
                 role,
                 PowerManager_RoleToString(g_pm.policy_desired_role),
                 g_pm.partner_sink_observed ? 1U : 0U,
                 (unsigned long)((g_pm.status.tps.status_raw >> 24) & 0x03U),
                 g_pm.status.otg_pin_high ? 1U : 0U,
                 (unsigned long)g_pm.status.tps.vbus_mv,
                 (unsigned long)g_pm.status.pd_snapshot.contract_voltage_mv,
                 (unsigned long)g_pm.status.pd_snapshot.contract_current_ma,
                 (unsigned long)g_pm.status.pd_snapshot.contract_power_mw,
                 (unsigned long)g_pm.status.tps.active_pdo_raw,
                 (unsigned long)g_pm.status.tps.active_rdo_raw,
                 g_pm.status.tps.pp5v_state,
                 g_pm.status.tps.pphv_state,
                 g_pm.status.tps.hard_reset_reason,
                 (unsigned long)((g_pm.status.tps.status_raw >> 32) & 0xFFU),
                 (unsigned long)g_pm.status.tps.status_raw);
}

static void PowerManager_LogBq(void)
{
    const BQ25731_Telemetry_t *bq = &g_pm.status.bq;

    Debug_Printf("[BQ] id=%02X/%02X Option0=0x%04X Option4=0x%04X WDT=%s OOA=%u FSW=%lukHz DITHER=+/-%lu%% ADCOption=0x%04X VREG=%lumV OTG_SET=%lumV ICHG_SET=%lumA IIN_HOST_READ=%lumA IIN_EFFECTIVE=%lumA",
                 bq->manufacturer_id, bq->device_id, bq->charge_option0,
                 bq->charge_option4,
                 ((bq->charge_option0 &
                   BQ25731_CHARGE_OPTION0_WDT_MASK) == 0U) ? "OFF" : "ON",
                 (bq->charge_option0 &
                  BQ25731_CHARGE_OPTION0_EN_OOA) != 0U ? 1U : 0U,
                 (unsigned long)BQ25731_DecodePwmFrequencyKhz(
                     bq->charge_option0),
                 (unsigned long)BQ25731_DecodeDitherPercent(
                     bq->charge_option4),
                 bq->adc_option,
                 (unsigned long)bq->charge_voltage_mv,
                 (unsigned long)bq->otg_voltage_mv,
                 (unsigned long)bq->charge_current_ma,
                 (unsigned long)bq->input_current_ma,
                 (unsigned long)bq->iin_dpm_ma);
}

static void PowerManager_LogBqMonitor(void)
{
    const BQ25731_Telemetry_t *bq = &g_pm.status.bq;
    uint8_t raw_vbat = (uint8_t)bq->adc_vsys_vbat;
    const char *power_flow = "IDLE";
    const char *power_meter = "NONE";
    uint32_t selected_voltage_mv = 0U;
    int32_t selected_current_ma = 0;
    int32_t selected_power_mw = 0;
    bool power_available = false;
    bool sink_contract = PowerManager_HasSinkContract();
    bool source_path = g_pm.status.tps.attached &&
        (g_pm.status.tps.connection_state >= 6U) &&
        (g_pm.status.tps.role == TPS25751_ROLE_SOURCE) &&
        g_pm.status.otg_pin_high;

    if (source_path && bq->in_otg) {
        power_flow = "BAT_TO_VBUS";
        power_meter = "VBUS_ADC";
        selected_voltage_mv = bq->adc_vbus_mv;
        selected_current_ma = -(int32_t)bq->adc_iin_ma;
        selected_power_mw = -(int32_t)bq->input_power_mw;
        power_available = true;
    } else if (sink_contract && (!bq->in_otg) &&
               (bq->battery_power_mw > 0)) {
        power_flow = "VBUS_TO_BAT";
        power_meter = "BAT_ADC";
        selected_voltage_mv = bq->adc_vbat_mv;
        selected_current_ma = bq->battery_current_ma;
        selected_power_mw = bq->battery_power_mw;
        power_available = true;
    } else if (g_pm.status.tps.attached) {
        power_flow = "TRANSITION";
    }

    /* Expose only the two rails relevant to power flow: the USB/adapter-side
     * VBUS and the battery-side VBAT. VSYS is a separate BQ node, not VBUS. */
    Debug_Printf("[BQ-ADC] rawVBAT=0x%02X VBUS=%lumV VBAT=%lumV IVBUS=%lumA IBAT=%ldmA",
                 raw_vbat,
                 (unsigned long)bq->adc_vbus_mv,
                 (unsigned long)bq->adc_vbat_mv,
                 (unsigned long)bq->adc_iin_ma,
                 (long)bq->battery_current_ma);
    if (power_available) {
        Debug_Printf("[BQ-PWR] flow=%s meter=%s V=%lumV I=%ldmA P=%ldmW",
                     power_flow,
                     power_meter,
                     (unsigned long)selected_voltage_mv,
                     (long)selected_current_ma,
                     (long)selected_power_mw);
    } else {
        Debug_Printf("[BQ-PWR] flow=%s meter=%s P=--",
                     power_flow, power_meter);
    }
    Debug_Printf("[BQ-STAT] input=%s precharge=%u fast=%u iindpm=%u vindpm=%u otg=%u faults=0x%02X status=0x%04X IIN_DPM=%lumA",
                 bq->input_present ? "YES" : "NO",
                 bq->in_precharge ? 1U : 0U,
                 bq->in_fast_charge ? 1U : 0U,
                 bq->in_iin_dpm ? 1U : 0U,
                 bq->in_vindpm ? 1U : 0U,
                 bq->in_otg ? 1U : 0U,
                 bq->fault_flags,
                 bq->charger_status,
                 (unsigned long)bq->iin_dpm_ma);
    if (bq->in_precharge) {
        Debug_Printf("[BQ-LIMIT] state=PRECHARGE hw_cells=5S VBAT=%lumV VSYS_MIN=15400mV ICHG_SET=%lumA actual_IBAT=%ldmA limiter=BQ_PROTECTION",
                     (unsigned long)bq->adc_vbat_mv,
                     (unsigned long)bq->charge_current_ma,
                     (long)bq->battery_current_ma);
    }
}

static const char *PowerManager_RoleToString(TPS25751_PowerRole_t role)
{
    if (role == TPS25751_ROLE_SOURCE) {
        return "SOURCE";
    }
    if (role == TPS25751_ROLE_SINK) {
        return "SINK";
    }
    return "UNKNOWN";
}

static bool PowerManager_HasTypecPowerConnection(void)
{
    return g_pm.status.tps.attached &&
           (g_pm.status.tps.connection_state >= 6U);
}

static void PowerManager_LogCapabilities(
    const char *name,
    const TPS25751_Capabilities_t *caps)
{
    uint8_t i;

    Debug_Printf("[PD-CAPS] %s count=%u max=%lumV drp=%u",
                 name, caps->count,
                 (unsigned long)caps->max_voltage_mv,
                 caps->first_pdo_dual_role_power ? 1U : 0U);
    for (i = 0U; i < caps->count; ++i) {
        const TPS25751_Pdo_t *pdo = &caps->pdo[i];
        Debug_Printf("[PD-CAPS] %s PDO%u=0x%08lX %lu-%lumV %lumA %lumW",
                     name, (unsigned int)(i + 1U),
                     (unsigned long)pdo->raw,
                     (unsigned long)pdo->min_voltage_mv,
                     (unsigned long)pdo->max_voltage_mv,
                     (unsigned long)pdo->current_ma,
                     (unsigned long)pdo->power_mw);
    }
}

static void PowerManager_ResetPolicy(uint32_t now_ms, bool attached)
{
    memset(&g_pm.partner_sink_caps, 0, sizeof(g_pm.partner_sink_caps));
    memset(&g_pm.partner_source_caps, 0, sizeof(g_pm.partner_source_caps));
    g_pm.policy_swap_attempted = false;
    g_pm.partner_source_caps_current = false;
    g_pm.partner_sink_observed = false;
    g_pm.policy_cap_attempts = 0U;
    g_pm.policy_role_mismatch_since_ms = 0U;
    g_pm.policy_desired_role = TPS25751_ROLE_UNKNOWN;

    if (attached &&
        (g_pm.status.requested_mode == POWER_MANAGER_USER_AUTO)) {
        if ((g_pm.status.tps.role == TPS25751_ROLE_SOURCE) &&
            (g_pm.status.tps.connection_state >= 6U)) {
            /* Attached.SRC means that TPS detected Rd on CC.  That is direct
             * Type-C evidence that the partner is a power sink; a PD
             * Get_Sink_Cap response is optional and must never gate VBUS. */
            g_pm.partner_sink_observed = true;
            g_pm.policy_desired_role = TPS25751_ROLE_SOURCE;
            /* TPS25751 Source Policy automatically advertises the PDOs from
             * TX_SOURCE_CAPS. SSrC is only required after the host changes
             * that register. Issuing SSrC on every attach can occupy the 4CC
             * interface until its timeout and starve STATUS telemetry long
             * enough for the external OTG safety gate to drop VBUS. */
            g_pm.policy_phase = PM_POLICY_DONE;
            g_pm.policy_next_ms = 0U;
            Debug_Printf("[PD-POLICY] attach role=SOURCE partner=Sink(Rd); VBUS enabled, TPS Source Policy owns automatic PDO advertisement");
        } else {
            g_pm.policy_phase = PM_POLICY_WAIT_SETTLE;
            g_pm.policy_next_ms = now_ms + PM_POLICY_SETTLE_MS;
            Debug_Printf("[PD-POLICY] attach role=%s; probing partner Sink/Source PDOs",
                         PowerManager_RoleToString(g_pm.status.tps.role));
        }
    } else {
        g_pm.policy_phase = PM_POLICY_IDLE;
        g_pm.policy_next_ms = 0U;
    }
}

static void PowerManager_UpdateAttachPolicy(uint32_t now_ms)
{
    if (g_pm.status.tps.attached != g_pm.previous_attached) {
        g_pm.previous_attached = g_pm.status.tps.attached;
        PowerManager_ResetPolicy(now_ms, g_pm.status.tps.attached);
        if (!g_pm.status.tps.attached) {
            Debug_Printf("[PD-POLICY] detach; decision state cleared");
        }
    }
}

static void PowerManager_DecidePolicy(uint32_t now_ms)
{
    TPS25751_PowerRole_t current = g_pm.status.tps.role;
    TPS25751_PowerRole_t desired = current;
    uint32_t source_max_mv = 0U;
    bool current_sink_contract =
        (current == TPS25751_ROLE_SINK) &&
        g_pm.status.tps.active_pdo.valid &&
        g_pm.status.tps.active_rdo.valid;
    const char *action = "KEEP_CURRENT_ROLE";
    const char *reason = "INSUFFICIENT_VALID_PARTNER_CAPS";

    /* RX_SOURCE_CAPS is usable only when this physical attach produced it.
     * Otherwise the TPS register may still describe a previous partner. */
    if (g_pm.partner_source_caps_current || current_sink_contract) {
        source_max_mv = g_pm.partner_source_caps.max_voltage_mv;
        if (current_sink_contract &&
            (g_pm.status.tps.active_pdo.max_voltage_mv > source_max_mv)) {
            source_max_mv = g_pm.status.tps.active_pdo.max_voltage_mv;
        }
    }

    if (source_max_mv > 5000U) {
        desired = TPS25751_ROLE_SINK;
        action = "DRAW_FROM_PARTNER";
        reason = "PARTNER_SOURCE_ABOVE_5V";
    } else if ((g_pm.partner_sink_caps.count > 0U) ||
               g_pm.partner_sink_observed) {
        desired = TPS25751_ROLE_SOURCE;
        action = "CHARGE_PARTNER";
        reason = g_pm.partner_sink_observed ?
                 "PARTNER_SINK_CONFIRMED_IN_THIS_ATTACH" :
                 "PARTNER_DRP_SINK_WITH_ONLY_5V_SOURCE";
    } else if ((g_pm.partner_source_caps.count > 0U) ||
               current_sink_contract) {
        desired = TPS25751_ROLE_SINK;
        action = "DRAW_FROM_PARTNER";
        reason = "PARTNER_SOURCE_ONLY";
    }

    g_pm.policy_desired_role = desired;
    Debug_Printf("[PD-POLICY] sink_pdos=%u sink_seen=%u source_pdos=%u rx_source_max=%lumV source_evidence_max=%lumV current=%s decision=%s reason=%s cap_try=%u/%u",
                 g_pm.partner_sink_caps.count,
                 g_pm.partner_sink_observed ? 1U : 0U,
                 g_pm.partner_source_caps.count,
                 (unsigned long)g_pm.partner_source_caps.max_voltage_mv,
                 (unsigned long)source_max_mv,
                 PowerManager_RoleToString(current), action, reason,
                 g_pm.policy_cap_attempts,
                 PM_POLICY_MAX_CAP_ATTEMPTS);

    if (!g_pm.partner_sink_observed && current_sink_contract &&
        (source_max_mv <= 5000U) &&
        (g_pm.policy_cap_attempts < PM_POLICY_MAX_CAP_ATTEMPTS)) {
        g_pm.policy_desired_role = desired;
        g_pm.policy_swap_attempted = false;
        g_pm.policy_phase = PM_POLICY_WAIT_SETTLE;
        g_pm.policy_next_ms = now_ms + PM_POLICY_CAP_RETRY_MS;
        Debug_Printf("[PD-POLICY] Sink capability not confirmed at 5V; retrying probe in %lums",
                     (unsigned long)PM_POLICY_CAP_RETRY_MS);
        return;
    }

    if ((desired == current) || (desired == TPS25751_ROLE_UNKNOWN) ||
        g_pm.policy_swap_attempted) {
        g_pm.policy_phase = PM_POLICY_DONE;
    } else if (desired == TPS25751_ROLE_SOURCE) {
        g_pm.policy_phase = PM_POLICY_SWAP_TO_SOURCE;
    } else {
        g_pm.policy_phase = PM_POLICY_SWAP_TO_SINK;
    }
}

static void PowerManager_MaintainPolicy(uint32_t now_ms)
{
    TPS25751_PowerRole_t current;
    bool contract_valid;
    bool active_source_pdo_is_drp;

    if ((g_pm.status.requested_mode != POWER_MANAGER_USER_AUTO) ||
        !g_pm.status.applied_mode_valid ||
        (g_pm.status.applied_mode != POWER_MANAGER_USER_AUTO) ||
        !PowerManager_HasTypecPowerConnection()) {
        g_pm.policy_role_mismatch_since_ms = 0U;
        return;
    }

    current = g_pm.status.tps.role;
    contract_valid = g_pm.status.tps.active_pdo.valid &&
                     g_pm.status.tps.active_rdo.valid;
    active_source_pdo_is_drp = contract_valid &&
        (current == TPS25751_ROLE_SINK) &&
        (((g_pm.status.tps.active_pdo_raw >> 30) & 0x03U) == 0U) &&
        ((g_pm.status.tps.active_pdo_raw & (1UL << 29)) != 0U);

    if ((current == TPS25751_ROLE_SOURCE) && !contract_valid &&
        (g_pm.policy_desired_role == TPS25751_ROLE_UNKNOWN)) {
        g_pm.partner_sink_observed = true;
        g_pm.policy_desired_role = TPS25751_ROLE_SOURCE;
        Debug_Printf("[PD-POLICY] desired=SOURCE established from Type-C Rd attach");
    }

    if (active_source_pdo_is_drp && !g_pm.partner_sink_observed) {
        g_pm.partner_sink_observed = true;
        Debug_Printf("[PD-POLICY] partner Sink capability confirmed by DRP bit in active Source PDO=0x%08lX",
                     (unsigned long)g_pm.status.tps.active_pdo_raw);
    }

    if (contract_valid && (current == TPS25751_ROLE_SOURCE)) {
        if (!g_pm.partner_sink_observed) {
            g_pm.partner_sink_observed = true;
            Debug_Printf("[PD-POLICY] partner Sink confirmed by active Source contract PDO=0x%08lX RDO=0x%08lX",
                         (unsigned long)g_pm.status.tps.active_pdo_raw,
                         (unsigned long)g_pm.status.tps.active_rdo_raw);
        }
        if (g_pm.policy_desired_role == TPS25751_ROLE_UNKNOWN) {
            g_pm.policy_desired_role = TPS25751_ROLE_SOURCE;
            Debug_Printf("[PD-POLICY] desired=SOURCE established from active partner Sink contract");
        }
    } else if (contract_valid && (current == TPS25751_ROLE_SINK)) {
        uint32_t source_mv = g_pm.status.tps.active_pdo.max_voltage_mv;

        if (source_mv > 5000U) {
            if (g_pm.policy_desired_role != TPS25751_ROLE_SINK) {
                Debug_Printf("[PD-POLICY] desired=SINK updated: partner contract is %lumV (>5V)",
                             (unsigned long)source_mv);
            }
            g_pm.policy_desired_role = TPS25751_ROLE_SINK;
        } else if (g_pm.partner_sink_observed &&
                   (g_pm.policy_desired_role != TPS25751_ROLE_SOURCE)) {
            g_pm.policy_desired_role = TPS25751_ROLE_SOURCE;
            Debug_Printf("[PD-POLICY] desired=SOURCE restored: partner Sink was confirmed and now offers only %lumV",
                         (unsigned long)source_mv);
        }
    }

    if (!contract_valid ||
        (g_pm.policy_desired_role == TPS25751_ROLE_UNKNOWN) ||
        (current == g_pm.policy_desired_role)) {
        g_pm.policy_role_mismatch_since_ms = 0U;
        return;
    }

    if (g_pm.policy_role_mismatch_since_ms == 0U) {
        g_pm.policy_role_mismatch_since_ms = now_ms;
        Debug_Printf("[PD-POLICY] role drift current=%s desired=%s; waiting %lums for stable contract",
                     PowerManager_RoleToString(current),
                     PowerManager_RoleToString(g_pm.policy_desired_role),
                     (unsigned long)PM_POLICY_ROLE_DRIFT_MS);
        return;
    }

    if (((uint32_t)(now_ms - g_pm.policy_role_mismatch_since_ms) <
         PM_POLICY_ROLE_DRIFT_MS) ||
        !PowerManager_TickReached(now_ms, g_pm.policy_next_ms) ||
        (g_pm.policy_phase != PM_POLICY_DONE)) {
        return;
    }

    g_pm.policy_swap_attempted = false;
    g_pm.policy_phase =
        (g_pm.policy_desired_role == TPS25751_ROLE_SOURCE) ?
        PM_POLICY_SWAP_TO_SOURCE : PM_POLICY_SWAP_TO_SINK;
    g_pm.policy_next_ms = now_ms;
    g_pm.policy_role_mismatch_since_ms = 0U;
    Debug_Printf("[PD-POLICY] enforcing maintained role target=%s",
                 PowerManager_RoleToString(g_pm.policy_desired_role));
}

static void PowerManager_HandleTpsError(TPS25751_Status_t status,
                                        uint32_t now_ms)
{
    ++g_pm.tps_error_count;
    g_pm.status.tps_status = status;
    PowerManager_RecordError(POWER_MANAGER_ERROR_TPS,
                             (uint32_t)status,
                             g_pm.tps.register_address,
                             now_ms);
    PowerManager_LogError("TPS", status, g_pm.tps_error_count, now_ms);
    g_pm.next_tps_step_ms = now_ms + PM_MODE_POLL_MS;
    if (g_pm.status.tps.mode != TPS25751_MODE_APP) {
        PowerManager_SetState(POWER_MANAGER_TPS_WAIT_APP);
    }
}

static void PowerManager_HandleBqError(TPS25751_Status_t tps_status,
                                       uint32_t now_ms)
{
    ++g_pm.bq_error_count;
    g_pm.status.tps_status = tps_status;
    g_pm.status.bq_status = BQ25731_MapTpsStatus(tps_status);
    g_pm.status.bq.online = false;
    PowerManager_RecordError(POWER_MANAGER_ERROR_BQ,
                             (uint32_t)tps_status,
                             g_pm.tps.register_address,
                             now_ms);
    PowerManager_LogError("BQ", tps_status, g_pm.bq_error_count, now_ms);
    g_pm.bq_init = PM_BQ_INIT_READ_OPTION0;
    g_pm.next_bq_action_ms = now_ms + PM_BQ_RETRY_MS;
    PowerManager_SetState(POWER_MANAGER_DEGRADED);
}

static void PowerManager_HandleBqTelemetryError(
    TPS25751_Status_t tps_status,
    uint32_t now_ms)
{
    ++g_pm.bq_error_count;
    g_pm.status.tps_status = tps_status;
    g_pm.status.bq_status = BQ25731_MapTpsStatus(tps_status);
    g_pm.status.bq.online = false;
    PowerManager_RecordError(POWER_MANAGER_ERROR_BQ,
                             (uint32_t)tps_status,
                             g_pm.tps.register_address,
                             now_ms);
    PowerManager_LogError("BQ-TELEM", tps_status,
                          g_pm.bq_error_count, now_ms);
    g_pm.next_bq_telemetry_ms = now_ms + PM_BQ_RETRY_MS;
    g_pm.next_bq_config_ms = now_ms + PM_BQ_RETRY_MS;
    /* A telemetry read failure must not trigger repeated writes to BQ. */
    PowerManager_SetState(POWER_MANAGER_DEGRADED);
}

static uint16_t PowerManager_ResultLe16(bool *valid)
{
    uint8_t length = 0U;
    const uint8_t *data = TPS25751_GetResult(&g_pm.tps, &length);

    *valid = (data != NULL) && (length >= 2U);
    return *valid ? TPS25751_ReadLe16(data) : 0U;
}

static void PowerManager_HandleEvent(const uint8_t *data, uint32_t now_ms)
{
    TPS_IntEvent_t event;

    TPS_IntEventDecode(&event, data);
    if (!event.any) {
        return;
    }

    memcpy(g_pm.event_to_clear, event.raw, sizeof(g_pm.event_to_clear));
    g_pm.event_clear_pending = true;

    if (event.hard_reset || event.plug_changed ||
        event.power_swap_complete || event.new_contract_consumer ||
        event.new_contract_provider || event.source_caps_received ||
        event.power_swap_requested ||
        event.unable_to_source || event.power_event_error) {
        Debug_Printf("[PD-EVENT] hard_reset=%u plug_change=%u swap_request=%u swap_done=%u contract_consumer=%u contract_provider=%u source_caps=%u unable_source=%u power_error=%u raw0=0x%02X raw1=0x%02X raw2=0x%02X raw5=0x%02X",
                     event.hard_reset ? 1U : 0U,
                     event.plug_changed ? 1U : 0U,
                     event.power_swap_requested ? 1U : 0U,
                     event.power_swap_complete ? 1U : 0U,
                     event.new_contract_consumer ? 1U : 0U,
                     event.new_contract_provider ? 1U : 0U,
                     event.source_caps_received ? 1U : 0U,
                     event.unable_to_source ? 1U : 0U,
                     event.power_event_error ? 1U : 0U,
                     event.raw[0], event.raw[1], event.raw[2], event.raw[5]);
    }

    if (event.hard_reset) {
        g_pm.hard_reset_holdoff_until_ms = now_ms +
                                           PM_HARD_RESET_HOLDOFF_MS;
        /* A pre-reset role sample is no longer proof that Source is safe. */
        g_pm.status_updated_ms = 0U;
        PowerManager_SetOtgPin(false);
    }
    if (event.overcurrent || event.power_event_error ||
        event.unable_to_source) {
        g_pm.status.source_fault_latched = true;
        PowerManager_SetOtgPin(false);
    }
    if (event.ext_source_safe_state) {
        g_pm.hard_reset_holdoff_until_ms = now_ms +
                                           PM_HARD_RESET_HOLDOFF_MS;
        g_pm.status_updated_ms = 0U;
        PowerManager_SetOtgPin(false);
    }
    if (event.i2c_controller_nack) {
        PowerManager_RecordError(POWER_MANAGER_ERROR_BQ,
                                 TPS25751_COMMAND_ERROR,
                                 TPS25751_REG_INT_EVENT,
                                 now_ms);
    }
    if (event.source_caps_received || event.new_contract_consumer) {
        g_pm.partner_source_caps_current = true;
    }
}

static void PowerManager_ProcessCompletedJob(TPS25751_Status_t operation_status,
                                             uint32_t now_ms)
{
    PowerManager_Job_t completed_job = g_pm.job;
    const uint8_t *data;
    uint8_t length = 0U;
    bool valid;
    uint16_t raw16;

    g_pm.job = PM_JOB_NONE;
    g_pm.last_job = completed_job;
    data = TPS25751_GetResult(&g_pm.tps, &length);

    if (operation_status != TPS25751_OK) {
        if (completed_job == PM_JOB_READ_BOOT_FLAGS) {
            g_pm.next_boot_flags_ms = now_ms + PM_BOOT_FLAGS_POLL_MS;
            PowerManager_HandleTpsError(operation_status, now_ms);
        } else if (completed_job == PM_JOB_GET_SINK_CAPS) {
            memset(&g_pm.partner_sink_caps, 0,
                   sizeof(g_pm.partner_sink_caps));
            g_pm.policy_phase = PM_POLICY_READ_SOURCE_CAPS;
            g_pm.policy_next_ms = now_ms + PM_POLICY_STEP_MS;
            Debug_Printf("[PD-POLICY] partner did not provide Sink PDOs task=%u status=%s",
                         g_pm.tps.task_return_code,
                         TPS25751_StatusToString(operation_status));
            if (operation_status != TPS25751_COMMAND_ERROR) {
                PowerManager_HandleTpsError(operation_status, now_ms);
            }
        } else if (completed_job == PM_JOB_READ_SINK_CAPS) {
            memset(&g_pm.partner_sink_caps, 0,
                   sizeof(g_pm.partner_sink_caps));
            g_pm.policy_phase = PM_POLICY_READ_SOURCE_CAPS;
            g_pm.policy_next_ms = now_ms + PM_POLICY_STEP_MS;
            PowerManager_HandleTpsError(operation_status, now_ms);
        } else if (completed_job == PM_JOB_READ_SOURCE_CAPS) {
            memset(&g_pm.partner_source_caps, 0,
                   sizeof(g_pm.partner_source_caps));
            g_pm.policy_phase = PM_POLICY_DECIDE;
            g_pm.policy_next_ms = now_ms + PM_POLICY_STEP_MS;
            PowerManager_HandleTpsError(operation_status, now_ms);
        } else if ((completed_job == PM_JOB_SWAP_TO_SOURCE) ||
                   (completed_job == PM_JOB_SWAP_TO_SINK)) {
            g_pm.policy_phase = PM_POLICY_DONE;
            g_pm.policy_next_ms = now_ms + PM_POLICY_SWAP_RETRY_MS;
            Debug_Printf("[PD-POLICY] role swap rejected target=%s task=%u status=%s; maintained policy retries after %lums",
                         PowerManager_RoleToString(g_pm.policy_desired_role),
                         g_pm.tps.task_return_code,
                         TPS25751_StatusToString(operation_status),
                         (unsigned long)PM_POLICY_SWAP_RETRY_MS);
            if (operation_status != TPS25751_COMMAND_ERROR) {
                PowerManager_HandleTpsError(operation_status, now_ms);
            }
        } else if ((completed_job == PM_JOB_BQ_READ_CONFIG_BLOCK) ||
                   (completed_job == PM_JOB_BQ_READ_STATUS_BLOCK) ||
                   (completed_job == PM_JOB_BQ_READ_ADC_BLOCK) ||
                   (completed_job == PM_JOB_BQ_READ_ID)) {
            PowerManager_HandleBqTelemetryError(operation_status, now_ms);
        } else if (completed_job >= PM_JOB_BQ_READ_OPTION0) {
            PowerManager_HandleBqError(operation_status, now_ms);
        } else {
            PowerManager_HandleTpsError(operation_status, now_ms);
        }
        return;
    }
    g_pm.status.tps_status = TPS25751_OK;

    switch (completed_job) {
        case PM_JOB_READ_MODE:
            if ((data == NULL) || (length != TPS25751_MODE_LEN)) {
                PowerManager_HandleTpsError(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.status.tps.mode = TPS25751_DecodeMode(data);
            memcpy(g_pm.status.tps.mode_ascii, data, TPS25751_MODE_LEN);
            g_pm.status.tps.mode_ascii[TPS25751_MODE_LEN] = '\0';
            g_pm.next_mode_ms = now_ms + PM_MODE_POLL_MS;
            if (g_pm.status.tps.mode == TPS25751_MODE_APP) {
                if (g_pm.app_seen_ms == 0U) {
                    g_pm.app_seen_ms = now_ms;
                    g_pm.next_bq_action_ms = now_ms + PM_BQ_START_DELAY_MS;
                    g_pm.next_tps_step_ms = now_ms;
                    g_pm.mode_update_pending = true;
                    g_pm.bq_init = PM_BQ_INIT_WAIT;
                }
                PowerManager_SetState(POWER_MANAGER_TPS_READY);
            } else {
                g_pm.app_seen_ms = 0U;
                g_pm.status.applied_mode_valid = false;
                g_pm.mode_update_pending = true;
                PowerManager_SetOtgPin(false);
                PowerManager_SetState(POWER_MANAGER_TPS_WAIT_APP);
            }
            break;

        case PM_JOB_READ_BOOT_FLAGS:
            if ((data == NULL) || (length != TPS25751_BOOT_FLAGS_LEN)) {
                PowerManager_HandleTpsError(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.status.tps.boot_flags_raw =
                (uint64_t)TPS25751_ReadLe32(data) |
                ((uint64_t)data[4] << 32);
            g_pm.next_boot_flags_ms = now_ms + PM_BOOT_FLAGS_POLL_MS;
            Debug_Printf("[TPS-BOOT] MODE=PTCH flags=0x%02lX%08lX cfg_src=%lu eeprom_present=%lu r0_try=%lu r1_try=%lu r0_invalid=%lu r1_invalid=%lu r0_ioerr=%lu r1_ioerr=%lu r0_crc=%lu r1_crc=%lu patch_err=%lu header_err=%lu",
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 32) & 0xFFU),
                         (unsigned long)g_pm.status.tps.boot_flags_raw,
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 29) & 0x07U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 3) & 0x01U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 4) & 0x01U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 5) & 0x01U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 6) & 0x01U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 7) & 0x01U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 8) & 0x01U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 9) & 0x01U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 12) & 0x01U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 13) & 0x01U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 10) & 0x01U),
                         (unsigned long)(g_pm.status.tps.boot_flags_raw & 0x01U));
            break;

        case PM_JOB_READ_PORT_CONFIG:
            if ((data == NULL) || (length != TPS25751_PORT_CONFIG_LEN)) {
                PowerManager_HandleTpsError(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            memcpy(g_pm.port_config, data, sizeof(g_pm.port_config));
            g_pm.port_write_pending = TPS25751_PatchPortMode(
                g_pm.port_config,
                PowerManager_MapPortMode(g_pm.status.requested_mode));
            if (!g_pm.port_write_pending) {
                g_pm.status.applied_mode = g_pm.status.requested_mode;
                g_pm.status.applied_mode_valid = true;
                g_pm.mode_update_pending = false;
            }
            break;

        case PM_JOB_WRITE_PORT_CONFIG:
            g_pm.port_write_pending = false;
            g_pm.mode_update_pending = false;
            g_pm.status.applied_mode = g_pm.status.requested_mode;
            g_pm.status.applied_mode_valid = true;
            g_pm.status.source_fault_latched = false;
            g_pm.status_updated_ms = 0U;
            g_pm.next_tps_step_ms = now_ms + PM_TPS_STEP_MS;
            break;

        case PM_JOB_READ_INT_MASK:
            if ((data == NULL) || (length != TPS_INT_EVENT_BYTES)) {
                PowerManager_HandleTpsError(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            memcpy(g_pm.int_mask, data, sizeof(g_pm.int_mask));
            TPS_IntEventEnableRequiredBits(g_pm.int_mask);
            g_pm.event_mask_write_pending = true;
            break;

        case PM_JOB_WRITE_INT_MASK:
            g_pm.event_mask_write_pending = false;
            g_pm.event_mask_ready = true;
            break;

        case PM_JOB_READ_STATUS:
        {
            TPS25751_PowerRole_t old_role = g_pm.status.tps.role;
            TPS25751_DecodeStatus(&g_pm.status.tps, data);
            g_pm.status.tps.updated_ms = now_ms;
            g_pm.status_updated_ms = now_ms;
            if (g_pm.status.tps.attached &&
                (g_pm.status.tps.role != old_role)) {
                Debug_Printf("[PD-ROLE] %s -> %s conn=%u STATUS=0x%02lX%08lX",
                             PowerManager_RoleToString(old_role),
                             PowerManager_RoleToString(g_pm.status.tps.role),
                             g_pm.status.tps.connection_state,
                             (unsigned long)((g_pm.status.tps.status_raw >> 32) & 0xFFU),
                             (unsigned long)g_pm.status.tps.status_raw);
            }
            PowerManager_UpdateAttachPolicy(now_ms);
            g_pm.telemetry_phase = 1U;
            break;
        }

        case PM_JOB_READ_POWER_PATH:
            TPS25751_DecodePowerPath(&g_pm.status.tps, data);
            if (g_pm.status.tps.pp5v_overcurrent ||
                g_pm.status.tps.ppcable_overcurrent) {
                g_pm.status.source_fault_latched = true;
                PowerManager_SetOtgPin(false);
            }
            g_pm.telemetry_phase = 2U;
            break;

        case PM_JOB_READ_POWER_STATUS:
            TPS25751_DecodePowerStatus(&g_pm.status.tps, data);
            g_pm.telemetry_phase = 3U;
            break;

        case PM_JOB_READ_PD_STATUS:
            TPS25751_DecodePdStatus(&g_pm.status.tps, data);
            if ((g_pm.status.tps.hard_reset_reason != 0U) &&
                (g_pm.status.tps.hard_reset_reason !=
                 g_pm.previous_hard_reset_reason)) {
                g_pm.hard_reset_holdoff_until_ms = now_ms +
                                                   PM_HARD_RESET_HOLDOFF_MS;
                g_pm.status_updated_ms = 0U;
                PowerManager_SetOtgPin(false);
            }
            g_pm.previous_hard_reset_reason =
                g_pm.status.tps.hard_reset_reason;
            g_pm.telemetry_phase = 4U;
            break;

        case PM_JOB_READ_ADC:
            TPS25751_DecodeAdcResults(&g_pm.status.tps, data);
            g_pm.telemetry_phase = 5U;
            break;

        case PM_JOB_READ_ACTIVE_PDO:
            g_pm.status.tps.active_pdo_raw = TPS25751_ReadLe32(data);
            g_pm.status.tps.active_pdo = TPS25751_DecodePdo(
                g_pm.status.tps.active_pdo_raw);
            g_pm.telemetry_phase = 6U;
            PowerManager_UpdatePdSnapshot();
            break;

        case PM_JOB_READ_ACTIVE_RDO:
            g_pm.status.tps.active_rdo_raw = TPS25751_ReadLe32(data);
            g_pm.status.tps.active_rdo = TPS25751_DecodeRdo(
                g_pm.status.tps.active_rdo_raw);
            g_pm.telemetry_phase = 7U;
            PowerManager_UpdatePdSnapshot();
            break;

        case PM_JOB_READ_EVENT:
            PowerManager_HandleEvent(data, now_ms);
            g_pm.telemetry_phase = 0U;
            PowerManager_UpdatePdSnapshot();
            PowerManager_LogPd(now_ms);
            break;

        case PM_JOB_CLEAR_EVENT:
            g_pm.event_clear_pending = false;
            memset(g_pm.event_to_clear, 0,
                   sizeof(g_pm.event_to_clear));
            break;

        case PM_JOB_GET_SINK_CAPS:
            g_pm.policy_phase = PM_POLICY_READ_SINK_CAPS;
            g_pm.policy_next_ms = now_ms + PM_POLICY_STEP_MS;
            break;

        case PM_JOB_READ_SINK_CAPS:
            if (!TPS25751_DecodeCapabilities(&g_pm.partner_sink_caps,
                                             data, length)) {
                memset(&g_pm.partner_sink_caps, 0,
                       sizeof(g_pm.partner_sink_caps));
                Debug_Printf("[PD-POLICY] invalid Sink PDO payload; ignoring it");
                g_pm.policy_phase = PM_POLICY_READ_SOURCE_CAPS;
                g_pm.policy_next_ms = now_ms + PM_POLICY_STEP_MS;
                break;
            }
            if (g_pm.partner_sink_caps.count > 0U) {
                g_pm.partner_sink_observed = true;
            }
            PowerManager_LogCapabilities("SINK", &g_pm.partner_sink_caps);
            g_pm.policy_phase = PM_POLICY_READ_SOURCE_CAPS;
            g_pm.policy_next_ms = now_ms + PM_POLICY_STEP_MS;
            break;

        case PM_JOB_READ_SOURCE_CAPS:
            if (!TPS25751_DecodeCapabilities(&g_pm.partner_source_caps,
                                             data, length)) {
                memset(&g_pm.partner_source_caps, 0,
                       sizeof(g_pm.partner_source_caps));
                Debug_Printf("[PD-POLICY] invalid Source PDO payload; ignoring it");
                g_pm.policy_phase = PM_POLICY_DECIDE;
                g_pm.policy_next_ms = now_ms + PM_POLICY_STEP_MS;
                break;
            }
            if ((g_pm.status.tps.role == TPS25751_ROLE_SINK) &&
                g_pm.status.tps.active_pdo.valid) {
                g_pm.partner_source_caps_current = true;
            }
            PowerManager_LogCapabilities("SOURCE",
                                         &g_pm.partner_source_caps);
            g_pm.policy_phase = PM_POLICY_DECIDE;
            g_pm.policy_next_ms = now_ms + PM_POLICY_STEP_MS;
            break;

        case PM_JOB_SWAP_TO_SOURCE:
        case PM_JOB_SWAP_TO_SINK:
            g_pm.policy_phase = PM_POLICY_DONE;
            g_pm.policy_next_ms = now_ms + PM_POLICY_SWAP_RETRY_MS;
            Debug_Printf("[PD-POLICY] role swap accepted target=%s; maintained policy verifies actual role in [PD]",
                         PowerManager_RoleToString(g_pm.policy_desired_role));
            break;

        case PM_JOB_BQ_READ_OPTION0:
            raw16 = PowerManager_ResultLe16(&valid);
            if (!valid) {
                PowerManager_HandleBqError(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.status.bq.charge_option0 = raw16;
            g_pm.status.bq.online = true;
            g_pm.status.bq_status = BQ25731_OK;
            if (g_pm.bq_init == PM_BQ_INIT_VERIFY_OPTION0) {
                if ((raw16 & (BQ25731_CHARGE_OPTION0_EN_OOA |
                              BQ25731_CHARGE_OPTION0_PWM_FREQ)) !=
                    (g_pm.bq_startup_option0_target &
                     (BQ25731_CHARGE_OPTION0_EN_OOA |
                      BQ25731_CHARGE_OPTION0_PWM_FREQ))) {
                    PowerManager_HandleBqError(
                        TPS25751_COMMAND_ERROR, now_ms);
                    break;
                }
                g_pm.bq_init = PM_BQ_INIT_READ_OPTION4;
            } else {
                g_pm.bq_startup_option0_target =
                    BQ25731_BuildStartupOption0(raw16);
                g_pm.bq_init =
                    (g_pm.bq_startup_option0_target == raw16) ?
                    PM_BQ_INIT_READ_OPTION4 :
                    PM_BQ_INIT_WRITE_OPTION0;
            }
            g_pm.next_bq_action_ms = now_ms + PM_BQ_INIT_STEP_MS;
            PowerManager_SetState(POWER_MANAGER_BQ_ADC_SETUP);
            break;

        case PM_JOB_BQ_WRITE_STARTUP_OPTION0:
            g_pm.bq_init = PM_BQ_INIT_VERIFY_OPTION0;
            g_pm.next_bq_action_ms = now_ms + PM_BQ_INIT_STEP_MS;
            break;

        case PM_JOB_BQ_READ_OPTION4:
            raw16 = PowerManager_ResultLe16(&valid);
            if (!valid) {
                PowerManager_HandleBqError(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.status.bq.charge_option4 = raw16;
            if (g_pm.bq_init == PM_BQ_INIT_VERIFY_OPTION4) {
                if ((raw16 & BQ25731_CHARGE_OPTION4_DITHER_MASK) !=
                    (g_pm.bq_startup_option4_target &
                     BQ25731_CHARGE_OPTION4_DITHER_MASK)) {
                    PowerManager_HandleBqError(
                        TPS25751_COMMAND_ERROR, now_ms);
                    break;
                }
                g_pm.bq_init = PM_BQ_INIT_WRITE_ADC;
                Debug_Printf("[BQ-INIT] OOA=%u FSW=%lukHz DITHER=+/-%lu%% Option0=0x%04X Option4=0x%04X verified",
                             (g_pm.status.bq.charge_option0 &
                              BQ25731_CHARGE_OPTION0_EN_OOA) != 0U ? 1U : 0U,
                             (unsigned long)BQ25731_DecodePwmFrequencyKhz(
                                 g_pm.status.bq.charge_option0),
                             (unsigned long)BQ25731_DecodeDitherPercent(
                                 g_pm.status.bq.charge_option4),
                             g_pm.status.bq.charge_option0,
                             g_pm.status.bq.charge_option4);
            } else {
                g_pm.bq_startup_option4_target =
                    BQ25731_BuildStartupOption4(raw16);
                g_pm.bq_init =
                    (g_pm.bq_startup_option4_target == raw16) ?
                    PM_BQ_INIT_WRITE_ADC :
                    PM_BQ_INIT_WRITE_OPTION4;
            }
            g_pm.next_bq_action_ms = now_ms + PM_BQ_INIT_STEP_MS;
            break;

        case PM_JOB_BQ_WRITE_STARTUP_OPTION4:
            g_pm.bq_init = PM_BQ_INIT_VERIFY_OPTION4;
            g_pm.next_bq_action_ms = now_ms + PM_BQ_INIT_STEP_MS;
            break;

        case PM_JOB_BQ_WRITE_ADC:
            g_pm.bq_init = PM_BQ_INIT_VERIFY_ADC;
            g_pm.next_bq_action_ms = now_ms + PM_BQ_TELEMETRY_MS;
            break;

        case PM_JOB_BQ_VERIFY_ADC:
            raw16 = PowerManager_ResultLe16(&valid);
            if (!valid ||
                ((raw16 & BQ25731_ADC_OPTION_VERIFY_MASK) !=
                 (BQ25731_ADC_OPTION_MONITORING &
                  BQ25731_ADC_OPTION_VERIFY_MASK))) {
                PowerManager_HandleBqError(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.status.bq.adc_option = raw16;
            g_pm.status.bq.adc_configured = true;
            g_pm.status.bq.online = true;
            g_pm.status.bq_status = BQ25731_OK;
            g_pm.bq_init = PM_BQ_INIT_DONE;
            g_pm.next_bq_telemetry_ms = now_ms;
            g_pm.next_bq_config_ms = now_ms;
            PowerManager_SetState(POWER_MANAGER_RUN);
            Debug_Printf("[BQ] TPS EEPROM owns power limits; STM startup writes only OOA/PWM_FREQ/DITHER and ADC_OPTION");
            break;

        case PM_JOB_BQ_READ_ID:
            if ((data == NULL) || (length < 2U)) {
                PowerManager_HandleBqError(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.status.bq.manufacturer_id = data[0];
            g_pm.status.bq.device_id = data[1];
            g_pm.status.bq.id_valid = (data[0] == 0x40U) &&
                                      (data[1] == 0xD6U);
            g_pm.status.bq_status = g_pm.status.bq.id_valid ?
                BQ25731_OK : BQ25731_DEVICE_ID_MISMATCH;
            if (g_pm.bq_init == PM_BQ_INIT_READ_ID) {
                g_pm.bq_init = PM_BQ_INIT_READ_OPTION0;
                g_pm.next_bq_action_ms = now_ms + PM_BQ_INIT_STEP_MS;
            }
            break;

        case PM_JOB_BQ_READ_CONFIG_BLOCK:
            if (!BQ25731_DecodeConfigBlock(&g_pm.status.bq,
                                           data, length)) {
                PowerManager_HandleBqTelemetryError(
                    TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.status.bq.online = true;
            g_pm.status.bq_status = BQ25731_OK;
            g_pm.next_bq_config_ms = now_ms + PM_BQ_CONFIG_MS;
            PowerManager_LogBq();
            break;

        case PM_JOB_BQ_READ_STATUS_BLOCK:
            if (!BQ25731_DecodeStatusBlock(&g_pm.status.bq,
                                           data, length)) {
                PowerManager_HandleBqTelemetryError(
                    TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.bq_telemetry_phase = 1U;
            g_pm.next_bq_telemetry_ms = now_ms;
            break;

        case PM_JOB_BQ_READ_ADC_BLOCK:
            if (!BQ25731_DecodeAdcBlock(&g_pm.status.bq,
                                        data, length)) {
                PowerManager_HandleBqTelemetryError(
                    TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.status.bq_status = g_pm.status.bq.id_valid ?
                BQ25731_OK : BQ25731_DEVICE_ID_MISMATCH;
            g_pm.status.bq.updated_ms = now_ms;
            g_pm.bq_telemetry_phase = 0U;
            g_pm.next_bq_telemetry_ms = now_ms + PM_BQ_TELEMETRY_MS;
            if ((g_pm.status.state == POWER_MANAGER_DEGRADED) &&
                g_pm.status.bq.id_valid) {
                PowerManager_SetState(POWER_MANAGER_RUN);
            }
            PowerManager_LogBqMonitor();
            break;

        default:
            break;
    }
}

static TPS25751_Status_t PowerManager_StartJob(PowerManager_Job_t job)
{
    TPS25751_Status_t status = TPS25751_INVALID_ARG;
    BQ25731_Status_t bq_status;

    switch (job) {
        case PM_JOB_READ_MODE:
            status = TPS25751_StartReadRegister(&g_pm.tps,
                TPS25751_REG_MODE, TPS25751_MODE_LEN);
            break;
        case PM_JOB_READ_BOOT_FLAGS:
            status = TPS25751_StartReadRegister(&g_pm.tps,
                TPS25751_REG_BOOT_FLAGS, TPS25751_BOOT_FLAGS_LEN);
            break;
        case PM_JOB_READ_PORT_CONFIG:
            status = TPS25751_StartReadRegister(&g_pm.tps,
                TPS25751_REG_PORT_CONFIG, TPS25751_PORT_CONFIG_LEN);
            break;
        case PM_JOB_WRITE_PORT_CONFIG:
            status = TPS25751_StartWriteRegister(&g_pm.tps,
                TPS25751_REG_PORT_CONFIG, g_pm.port_config,
                sizeof(g_pm.port_config));
            break;
        case PM_JOB_READ_INT_MASK:
            status = TPS25751_StartReadRegister(&g_pm.tps,
                TPS25751_REG_INT_MASK, TPS_INT_EVENT_BYTES);
            break;
        case PM_JOB_WRITE_INT_MASK:
            status = TPS25751_StartWriteRegister(&g_pm.tps,
                TPS25751_REG_INT_MASK, g_pm.int_mask,
                sizeof(g_pm.int_mask));
            break;
        case PM_JOB_READ_STATUS:
            status = TPS25751_StartReadRegister(&g_pm.tps,
                TPS25751_REG_STATUS, TPS25751_STATUS_LEN);
            break;
        case PM_JOB_READ_POWER_PATH:
            status = TPS25751_StartReadRegister(&g_pm.tps,
                TPS25751_REG_POWER_PATH_STATUS, TPS25751_POWER_PATH_LEN);
            break;
        case PM_JOB_READ_POWER_STATUS:
            status = TPS25751_StartReadRegister(&g_pm.tps,
                TPS25751_REG_POWER_STATUS, TPS25751_POWER_STATUS_LEN);
            break;
        case PM_JOB_READ_PD_STATUS:
            status = TPS25751_StartReadRegister(&g_pm.tps,
                TPS25751_REG_PD_STATUS, TPS25751_PD_STATUS_LEN);
            break;
        case PM_JOB_READ_ADC:
            status = TPS25751_StartReadRegister(&g_pm.tps,
                TPS25751_REG_ADC_RESULTS, TPS25751_ADC_RESULTS_LEN);
            break;
        case PM_JOB_READ_ACTIVE_PDO:
            status = TPS25751_StartReadRegister(&g_pm.tps,
                TPS25751_REG_ACTIVE_PDO, TPS25751_ACTIVE_PDO_LEN);
            break;
        case PM_JOB_READ_ACTIVE_RDO:
            status = TPS25751_StartReadRegister(&g_pm.tps,
                TPS25751_REG_ACTIVE_RDO, TPS25751_ACTIVE_RDO_LEN);
            break;
        case PM_JOB_READ_EVENT:
            status = TPS_IntEventStartRead(&g_pm.tps);
            break;
        case PM_JOB_CLEAR_EVENT:
            status = TPS_IntEventStartClear(&g_pm.tps,
                                            g_pm.event_to_clear);
            break;
        case PM_JOB_GET_SINK_CAPS:
            status = TPS25751_StartCommand(&g_pm.tps, "GSkC",
                                           NULL, 0U, 1U);
            break;
        case PM_JOB_READ_SINK_CAPS:
            status = TPS25751_StartReadRegister(
                &g_pm.tps, TPS25751_REG_RX_SINK_CAPS,
                TPS25751_RX_CAPS_LEN);
            break;
        case PM_JOB_READ_SOURCE_CAPS:
            status = TPS25751_StartReadRegister(
                &g_pm.tps, TPS25751_REG_RX_SOURCE_CAPS,
                TPS25751_RX_CAPS_LEN);
            break;
        case PM_JOB_SWAP_TO_SOURCE:
            status = TPS25751_StartCommand(&g_pm.tps, "SWSr",
                                           NULL, 0U, 1U);
            break;
        case PM_JOB_SWAP_TO_SINK:
            status = TPS25751_StartCommand(&g_pm.tps, "SWSk",
                                           NULL, 0U, 1U);
            break;
        case PM_JOB_BQ_READ_OPTION0:
            bq_status = BQ25731_StartRead16(&g_pm.bq,
                                            BQ25731_REG_CHARGE_OPTION0);
            status = (bq_status == BQ25731_OK) ? TPS25751_OK :
                                                TPS25751_BUSY;
            break;
        case PM_JOB_BQ_WRITE_STARTUP_OPTION0:
            bq_status = BQ25731_StartWriteStartupOption0(
                &g_pm.bq, g_pm.bq_startup_option0_target);
            status = (bq_status == BQ25731_OK) ? TPS25751_OK :
                                                TPS25751_BUSY;
            break;
        case PM_JOB_BQ_READ_OPTION4:
            bq_status = BQ25731_StartRead16(&g_pm.bq,
                                            BQ25731_REG_CHARGE_OPTION4);
            status = (bq_status == BQ25731_OK) ? TPS25751_OK :
                                                TPS25751_BUSY;
            break;
        case PM_JOB_BQ_WRITE_STARTUP_OPTION4:
            bq_status = BQ25731_StartWriteStartupOption4(
                &g_pm.bq, g_pm.bq_startup_option4_target);
            status = (bq_status == BQ25731_OK) ? TPS25751_OK :
                                                TPS25751_BUSY;
            break;
        case PM_JOB_BQ_WRITE_ADC:
            bq_status = BQ25731_StartConfigureMonitoringAdc(&g_pm.bq);
            status = (bq_status == BQ25731_OK) ? TPS25751_OK :
                                                TPS25751_BUSY;
            break;
        case PM_JOB_BQ_VERIFY_ADC:
            bq_status = BQ25731_StartRead16(&g_pm.bq,
                                            BQ25731_REG_ADC_OPTION);
            status = (bq_status == BQ25731_OK) ? TPS25751_OK :
                                                TPS25751_BUSY;
            break;
        case PM_JOB_BQ_READ_ID:
            bq_status = BQ25731_StartReadId(&g_pm.bq);
            status = (bq_status == BQ25731_OK) ? TPS25751_OK :
                                                TPS25751_BUSY;
            break;
        case PM_JOB_BQ_READ_CONFIG_BLOCK:
            bq_status = BQ25731_StartReadConfigBlock(&g_pm.bq);
            status = (bq_status == BQ25731_OK) ? TPS25751_OK :
                                                TPS25751_BUSY;
            break;
        case PM_JOB_BQ_READ_STATUS_BLOCK:
            bq_status = BQ25731_StartReadStatusBlock(&g_pm.bq);
            status = (bq_status == BQ25731_OK) ? TPS25751_OK :
                                                TPS25751_BUSY;
            break;
        case PM_JOB_BQ_READ_ADC_BLOCK:
            bq_status = BQ25731_StartReadAdcBlock(&g_pm.bq);
            status = (bq_status == BQ25731_OK) ? TPS25751_OK :
                                                TPS25751_BUSY;
            break;
        default:
            break;
    }

    if (status == TPS25751_OK) {
        g_pm.job = job;
        if (job == PM_JOB_GET_SINK_CAPS) {
            if (g_pm.policy_cap_attempts < 0xFFU) {
                ++g_pm.policy_cap_attempts;
            }
            Debug_Printf("[PD-POLICY] requesting partner Sink PDOs attempt=%u/%u",
                         g_pm.policy_cap_attempts,
                         PM_POLICY_MAX_CAP_ATTEMPTS);
        }
        if ((job == PM_JOB_SWAP_TO_SOURCE) ||
            (job == PM_JOB_SWAP_TO_SINK)) {
            g_pm.policy_swap_attempted = true;
        }
    }
    return status;
}

static PowerManager_Job_t PowerManager_SelectTelemetryJob(void)
{
    switch (g_pm.telemetry_phase) {
        case 0U: return PM_JOB_READ_STATUS;
        case 1U: return PM_JOB_READ_POWER_PATH;
        case 2U: return PM_JOB_READ_POWER_STATUS;
        case 3U: return PM_JOB_READ_PD_STATUS;
        case 4U: return PM_JOB_READ_ADC;
        case 5U: return PM_JOB_READ_ACTIVE_PDO;
        case 6U: return PM_JOB_READ_ACTIVE_RDO;
        default: return PM_JOB_READ_EVENT;
    }
}

static PowerManager_Job_t PowerManager_SelectBqTelemetryJob(void)
{
    return (g_pm.bq_telemetry_phase == 0U) ?
           PM_JOB_BQ_READ_STATUS_BLOCK : PM_JOB_BQ_READ_ADC_BLOCK;
}

static PowerManager_Job_t PowerManager_SelectPolicyJob(uint32_t now_ms)
{
    if ((g_pm.status.requested_mode != POWER_MANAGER_USER_AUTO) ||
        !g_pm.status.applied_mode_valid ||
        (g_pm.status.applied_mode != POWER_MANAGER_USER_AUTO) ||
        !PowerManager_HasTypecPowerConnection()) {
        return PM_JOB_NONE;
    }

    if ((g_pm.policy_phase == PM_POLICY_WAIT_SETTLE) &&
        PowerManager_TickReached(now_ms, g_pm.policy_next_ms)) {
        g_pm.policy_phase = PM_POLICY_GET_SINK_CAPS;
    }
    if (!PowerManager_TickReached(now_ms, g_pm.policy_next_ms)) {
        return PM_JOB_NONE;
    }
    if (g_pm.policy_phase == PM_POLICY_DECIDE) {
        PowerManager_DecidePolicy(now_ms);
    }

    switch (g_pm.policy_phase) {
        case PM_POLICY_GET_SINK_CAPS: return PM_JOB_GET_SINK_CAPS;
        case PM_POLICY_READ_SINK_CAPS: return PM_JOB_READ_SINK_CAPS;
        case PM_POLICY_READ_SOURCE_CAPS: return PM_JOB_READ_SOURCE_CAPS;
        case PM_POLICY_SWAP_TO_SOURCE: return PM_JOB_SWAP_TO_SOURCE;
        case PM_POLICY_SWAP_TO_SINK: return PM_JOB_SWAP_TO_SINK;
        default: return PM_JOB_NONE;
    }
}

static PowerManager_Job_t PowerManager_SelectJob(uint32_t now_ms)
{
    PowerManager_Job_t policy_job;

    if (g_pm.status.tps.mode != TPS25751_MODE_APP) {
        if ((g_pm.status.tps.mode == TPS25751_MODE_PTCH) &&
            PowerManager_TickReached(now_ms, g_pm.next_boot_flags_ms)) {
            return PM_JOB_READ_BOOT_FLAGS;
        }
        return PowerManager_TickReached(now_ms, g_pm.next_mode_ms) ?
               PM_JOB_READ_MODE : PM_JOB_NONE;
    }

    if (PowerManager_TickReached(now_ms, g_pm.next_mode_ms)) {
        return PM_JOB_READ_MODE;
    }
    if (g_pm.port_write_pending) {
        return PM_JOB_WRITE_PORT_CONFIG;
    }
    if (g_pm.mode_update_pending) {
        return PM_JOB_READ_PORT_CONFIG;
    }
    if (g_pm.event_mask_write_pending) {
        return PM_JOB_WRITE_INT_MASK;
    }
    if (!g_pm.event_mask_ready) {
        return PM_JOB_READ_INT_MASK;
    }
    if (g_pm.event_clear_pending) {
        return PM_JOB_CLEAR_EVENT;
    }

    policy_job = PowerManager_SelectPolicyJob(now_ms);
    if (policy_job != PM_JOB_NONE) {
        return policy_job;
    }

    if (g_pm.bq_init == PM_BQ_INIT_WAIT &&
        PowerManager_TickReached(now_ms, g_pm.next_bq_action_ms)) {
        g_pm.bq_init = PM_BQ_INIT_READ_ID;
        PowerManager_SetState(POWER_MANAGER_BQ_PROBE);
    }
    if (PowerManager_TickReached(now_ms, g_pm.next_bq_action_ms)) {
        switch (g_pm.bq_init) {
            case PM_BQ_INIT_READ_ID: return PM_JOB_BQ_READ_ID;
            case PM_BQ_INIT_READ_OPTION0: return PM_JOB_BQ_READ_OPTION0;
            case PM_BQ_INIT_WRITE_OPTION0:
                return PM_JOB_BQ_WRITE_STARTUP_OPTION0;
            case PM_BQ_INIT_VERIFY_OPTION0:
                return PM_JOB_BQ_READ_OPTION0;
            case PM_BQ_INIT_READ_OPTION4:
                return PM_JOB_BQ_READ_OPTION4;
            case PM_BQ_INIT_WRITE_OPTION4:
                return PM_JOB_BQ_WRITE_STARTUP_OPTION4;
            case PM_BQ_INIT_VERIFY_OPTION4:
                return PM_JOB_BQ_READ_OPTION4;
            case PM_BQ_INIT_WRITE_ADC: return PM_JOB_BQ_WRITE_ADC;
            case PM_BQ_INIT_VERIFY_ADC: return PM_JOB_BQ_VERIFY_ADC;
            default: break;
        }
    }
    if (PowerManager_TickReached(now_ms, g_pm.next_tps_step_ms)) {
        g_pm.next_tps_step_ms = now_ms + PM_TPS_STEP_MS;
        return PowerManager_SelectTelemetryJob();
    }

    if ((g_pm.bq_init == PM_BQ_INIT_DONE) &&
        PowerManager_TickReached(now_ms, g_pm.next_bq_telemetry_ms)) {
        return PowerManager_SelectBqTelemetryJob();
    }
    if ((g_pm.bq_init == PM_BQ_INIT_DONE) &&
        PowerManager_TickReached(now_ms, g_pm.next_bq_config_ms)) {
        return PM_JOB_BQ_READ_CONFIG_BLOCK;
    }
    return PM_JOB_NONE;
}

void PowerManager_Init(I2C_HandleTypeDef *hi2c)
{
    uint32_t now_ms = HAL_GetTick();

    memset(&g_pm, 0, sizeof(g_pm));
    g_pm.hi2c = hi2c;
    g_pm.status.state = POWER_MANAGER_INIT;
    g_pm.status.requested_mode = POWER_MANAGER_USER_AUTO;
    g_pm.status.applied_mode = POWER_MANAGER_USER_OFF;
    g_pm.status.tps_status = TPS25751_INVALID_ARG;
    g_pm.status.bq_status = BQ25731_NOT_READY;
    g_pm.mode_update_pending = true;
    g_pm.next_mode_ms = now_ms;
    g_pm.next_boot_flags_ms = now_ms;
    g_pm.next_tps_step_ms = now_ms;
    g_pm.next_bq_action_ms = now_ms + PM_BQ_START_DELAY_MS;
    g_pm.hard_reset_holdoff_until_ms = now_ms;

    HAL_GPIO_WritePin(OTG_EN_GPIO_Port, OTG_EN_Pin, GPIO_PIN_RESET);
    g_pm.status.otg_pin_high = false;

    if ((TPS25751_Init(&g_pm.tps, hi2c,
                       TPS25751_I2C_ADDR_DEFAULT) != TPS25751_OK) ||
        (BQ25731_Init(&g_pm.bq, &g_pm.tps,
                      BQ25731_I2C_ADDR_7BIT) != BQ25731_OK)) {
        PowerManager_SetState(POWER_MANAGER_FAULT);
        return;
    }
    g_pm.initialized = true;
    Debug_Printf("[PM] transport=I2C4-IT TPS=0x%02X BQ=0x%02X port_config_len=%u",
                 TPS25751_I2C_ADDR_DEFAULT,
                 BQ25731_I2C_ADDR_7BIT,
                 TPS25751_PORT_CONFIG_LEN);
    PowerManager_SetState(POWER_MANAGER_TPS_WAIT_APP);
}

void PowerManager_Task(void)
{
    uint32_t now_ms;
    TPS25751_Status_t operation_status;
    PowerManager_Job_t next_job;

    if (!g_pm.initialized) {
        return;
    }
    now_ms = HAL_GetTick();
    PowerManager_UpdateOtgGate(now_ms);

    if (g_pm.job != PM_JOB_NONE) {
        operation_status = TPS25751_Task(&g_pm.tps, now_ms);
        if (operation_status != TPS25751_BUSY) {
            PowerManager_ProcessCompletedJob(operation_status, now_ms);
            PowerManager_UpdateOtgGate(now_ms);
        }
        return;
    }

    PowerManager_MaintainPolicy(now_ms);
    next_job = PowerManager_SelectJob(now_ms);
    if (next_job == PM_JOB_NONE) {
        return;
    }
    if (PowerManager_StartJob(next_job) != TPS25751_OK) {
        return;
    }

    /* Starting an operation only prepares it. This call starts exactly one
     * short interrupt-driven physical I2C transfer and returns immediately. */
    operation_status = TPS25751_Task(&g_pm.tps, now_ms);
    if (operation_status != TPS25751_BUSY) {
        PowerManager_ProcessCompletedJob(operation_status, now_ms);
    }
}

void PowerManager_GetStatus(PowerManager_Status_t *out)
{
    if (out != NULL) {
        *out = g_pm.status;
    }
}

bool PowerManager_SetUserMode(PowerManager_UserMode_t mode)
{
    if (mode > POWER_MANAGER_USER_OFF) {
        return false;
    }
    if (g_pm.status.requested_mode != mode) {
        g_pm.status.requested_mode = mode;
        g_pm.mode_update_pending = true;
        g_pm.port_write_pending = false;
        g_pm.status.applied_mode_valid = false;
        g_pm.status.source_fault_latched = false;
        PowerManager_SetOtgPin(false);
        PowerManager_ResetPolicy(HAL_GetTick(),
                                 g_pm.status.tps.attached);
    }
    return true;
}

PowerManager_State_t PowerManager_GetState(void)
{
    return g_pm.status.state;
}

BQ25731_Status_t PowerManager_GetBqStatus(void)
{
    return g_pm.status.bq_status;
}

bool PowerManager_GetPdSnapshot(PowerManager_PdSnapshot_t *out)
{
    if (out != NULL) {
        *out = g_pm.status.pd_snapshot;
    }
    return g_pm.status.pd_snapshot.attached &&
           (g_pm.status.pd_snapshot.active_rdo_raw != 0U) &&
           (g_pm.status.pd_snapshot.contract_voltage_mv != 0U) &&
           (g_pm.status.pd_snapshot.contract_power_mw != 0U);
}
