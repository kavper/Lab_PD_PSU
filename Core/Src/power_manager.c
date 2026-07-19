#include "power_manager.h"

#include "debug_uart.h"
#include "tps_int_event.h"

#include <stdio.h>
#include <string.h>

#define PM_MODE_POLL_MS              100U
#define PM_BOOT_FLAGS_POLL_MS       1000U
#define PM_TPS_STEP_MS                20U
#define PM_TPS_STATUS_STALE_MS      1500U
#define PM_PD_LOG_MS                2000U
#define PM_BQ_START_DELAY_MS        1000U
#define PM_BQ_INIT_STEP_MS           200U
#define PM_BQ_RETRY_MS              2000U
#define PM_BQ_TELEMETRY_MS          2000U
#define PM_BQ_CONFIG_MS            10000U
#define PM_BQ_MONITOR_HOLDOFF_MS    1500U
#define PM_HARD_RESET_HOLDOFF_MS     250U
#define PM_ERROR_LOG_MS             1000U
#define PM_POLICY_STEP_MS             20U
#define PM_POLICY_CAP_RETRY_MS       1500U
#define PM_POLICY_ROLE_DRIFT_MS       750U
#define PM_POLICY_SWAP_RETRY_MS      3000U
#define PM_POLICY_MAX_CAP_ATTEMPTS      3U
#define PM_ENABLE_BQ_EC_ACCESS          1U
#define PM_VERBOSE_TRANSITION_LOGS       0U
typedef enum {
    PM_JOB_NONE = 0,
    PM_JOB_READ_MODE,
    PM_JOB_READ_BOOT_FLAGS,
    PM_JOB_READ_PORT_CONFIG,
    PM_JOB_WRITE_PORT_CONFIG,
    PM_JOB_READ_INT_MASK,
    PM_JOB_WRITE_INT_MASK,
    PM_JOB_READ_STATUS,
    PM_JOB_READ_TYPEC_STATE,
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
    PM_JOB_READ_LOCAL_SOURCE_CAPS,
    PM_JOB_TRACE_SOURCE_CAPS,
    PM_JOB_SWAP_TO_SOURCE,
    PM_JOB_SWAP_TO_SINK,
    PM_JOB_BQ_TRACE_IIN_HOST,
    PM_JOB_BQ_READ_OPTION0,
    PM_JOB_BQ_WRITE_STARTUP_OPTION0,
    PM_JOB_BQ_READ_OPTION4,
    PM_JOB_BQ_WRITE_STARTUP_OPTION4,
    PM_JOB_BQ_READ_OPTION1,
    PM_JOB_BQ_WRITE_STARTUP_OPTION1,
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
    PM_BQ_INIT_READ_OPTION1,
    PM_BQ_INIT_WRITE_OPTION1,
    PM_BQ_INIT_VERIFY_OPTION1,
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
    uint32_t bq_monitor_holdoff_until_ms;
    uint32_t policy_next_ms;
    uint32_t app_seen_ms;
    uint32_t status_updated_ms;
    uint32_t hard_reset_holdoff_until_ms;
    uint32_t last_pd_log_ms;
    uint32_t last_error_log_ms;
    uint32_t tps_error_count;
    uint32_t bq_error_count;
    uint32_t attach_count;
    uint32_t detach_count;
    uint32_t plug_change_count;
    uint32_t contract_count;
    uint32_t power_swap_count;
    uint32_t hard_reset_count;
    uint32_t power_error_count;
    uint32_t i2c_nack_count;
    uint32_t unable_source_count;
    uint32_t overcurrent_count;
    uint32_t policy_role_mismatch_since_ms;
    uint16_t bq_startup_option0_target;
    uint16_t bq_startup_option4_target;
    uint16_t bq_startup_option1_target;
    uint8_t previous_hard_reset_reason;
    uint8_t policy_cap_attempts;
    bool previous_attached;
    bool policy_swap_attempted;
    bool partner_source_caps_current;
    bool partner_sink_observed;
    bool pdo_report_pending;
    bool local_source_caps_pending;
    bool local_source_caps_valid;
    bool source_caps_trace_pending;
    bool bq_iin_trace_pending;
    bool typec_trace_valid;
    TPS25751_PowerRole_t policy_desired_role;
    TPS25751_Capabilities_t local_source_caps;
    TPS25751_Capabilities_t partner_sink_caps;
    TPS25751_Capabilities_t partner_source_caps;
} PowerManager_Context_t;

static PowerManager_Context_t g_pm;

static const char *PowerManager_RoleToString(TPS25751_PowerRole_t role);
static const char *PowerManager_TypecStateToString(uint8_t state);
static const char *PowerManager_CcStateToString(uint8_t state);

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
        case PM_JOB_READ_TYPEC_STATE: return "READ_TYPEC_STATE";
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
        case PM_JOB_READ_LOCAL_SOURCE_CAPS: return "READ_LOCAL_SOURCE_CAPS";
        case PM_JOB_TRACE_SOURCE_CAPS: return "TRACE_SOURCE_CAPS";
        case PM_JOB_SWAP_TO_SOURCE: return "SWAP_TO_SOURCE";
        case PM_JOB_SWAP_TO_SINK: return "SWAP_TO_SINK";
        case PM_JOB_BQ_TRACE_IIN_HOST: return "BQ_TRACE_IIN_HOST";
        case PM_JOB_BQ_READ_OPTION0: return "BQ_READ_OPTION0";
        case PM_JOB_BQ_WRITE_STARTUP_OPTION0: return "BQ_WRITE_QUIET_OPTION0";
        case PM_JOB_BQ_READ_OPTION4: return "BQ_READ_OPTION4";
        case PM_JOB_BQ_WRITE_STARTUP_OPTION4: return "BQ_WRITE_DITHER_OPTION4";
        case PM_JOB_BQ_READ_OPTION1: return "BQ_READ_5MOHM_OPTION1";
        case PM_JOB_BQ_WRITE_STARTUP_OPTION1: return "BQ_WRITE_5MOHM_OPTION1";
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
    if ((PM_VERBOSE_TRANSITION_LOGS != 0U) &&
        (allow != g_pm.status.otg_pin_high)) {
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
        (t->active_rdo.requested_voltage_mv != 0U)) {
        snapshot->contract_voltage_mv =
            t->active_rdo.requested_voltage_mv;
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

static void PowerManager_LogMonitorRow(const char *item,
                                       const char *value,
                                       const char *details)
{
    Debug_Printf("[MON] | %-12.12s | %-36.36s | %-40.40s |",
                 item, value, details);
}

static void PowerManager_LogPd(uint32_t now_ms)
{
    const TPS25751_Telemetry_t *tps = &g_pm.status.tps;
    const BQ25731_Telemetry_t *bq = &g_pm.status.bq;
    const char *role = "NONE";
    const char *charge_state = "IDLE";
    const char *power_flow = "IDLE";
    char value[64];
    char details[64];
    uint32_t uart_used;
    uint32_t uart_dropped;

    if (!PowerManager_TickReached(now_ms,
                                  g_pm.last_pd_log_ms + PM_PD_LOG_MS)) {
        return;
    }
    g_pm.last_pd_log_ms = now_ms;

    if (tps->attached && (tps->role == TPS25751_ROLE_SOURCE)) {
        role = "SOURCE";
    } else if (tps->attached && (tps->role == TPS25751_ROLE_SINK)) {
        role = "SINK";
    }

    if (bq->in_otg) {
        charge_state = "OTG";
    } else if (bq->in_fast_charge) {
        charge_state = "FAST_CHARGE";
    } else if (bq->in_precharge) {
        charge_state = "PRECHARGE";
    }
    if (PowerManager_HasSinkContract() && !bq->in_otg) {
        power_flow = "VBUS -> BAT";
    } else if (tps->attached &&
               (tps->role == TPS25751_ROLE_SOURCE) &&
               g_pm.status.otg_pin_high) {
        power_flow = "BAT -> VBUS";
    } else if (tps->attached) {
        power_flow = "TRANSITION";
    }

    uart_used = Debug_GetTxBufferUsed();
    uart_dropped = Debug_GetDroppedCount();

    Debug_Printf("[MON] ");
    Debug_Printf("[MON] ============================ POWER / USB-C LIVE MONITOR =============================");
    PowerManager_LogMonitorRow("ITEM", "VALUE", "DETAILS");
    Debug_Printf("[MON] +--------------+--------------------------------------+------------------------------------------+");

    (void)snprintf(value, sizeof(value), "%s  t=%lu ms",
                   PowerManager_StateToString(g_pm.status.state),
                   (unsigned long)now_ms);
    (void)snprintf(details, sizeof(details), "TPS=%s  BQ=%s",
                   TPS25751_ModeToString(tps->mode),
                   BQ25731_StatusToString(g_pm.status.bq_status));
    PowerManager_LogMonitorRow("SYSTEM", value, details);

    (void)snprintf(value, sizeof(value), "%s",
                   PowerManager_TypecStateToString(tps->typec_port_state));
    (void)snprintf(details, sizeof(details), "CC1=%s  CC2=%s  PD_CC=%u",
                   PowerManager_CcStateToString(tps->cc1_state),
                   PowerManager_CcStateToString(tps->cc2_state),
                   tps->pd_cc_pin);
    PowerManager_LogMonitorRow("TYPE-C", value, details);

    (void)snprintf(value, sizeof(value), "%s  desired=%s", role,
                   PowerManager_RoleToString(g_pm.policy_desired_role));
    (void)snprintf(details, sizeof(details),
                   "mode=%s  attach=%u  conn=%u  OTG=%u",
                   PowerManager_UserModeToString(g_pm.status.requested_mode),
                   tps->attached ? 1U : 0U, tps->connection_state,
                   g_pm.status.otg_pin_high ? 1U : 0U);
    PowerManager_LogMonitorRow("ROLE", value, details);

    (void)snprintf(value, sizeof(value), "PDO%u  %lu mV  %lu mA",
                   tps->active_rdo.object_position,
                   (unsigned long)g_pm.status.pd_snapshot.contract_voltage_mv,
                   (unsigned long)g_pm.status.pd_snapshot.contract_current_ma);
    (void)snprintf(details, sizeof(details), "%lu mW  mismatch=%u",
                   (unsigned long)g_pm.status.pd_snapshot.contract_power_mw,
                   tps->active_rdo.capability_mismatch ? 1U : 0U);
    PowerManager_LogMonitorRow("CONTRACT", value, details);

    (void)snprintf(value, sizeof(value), "PDO=0x%08lX",
                   (unsigned long)tps->active_pdo_raw);
    (void)snprintf(details, sizeof(details), "RDO=0x%08lX",
                   (unsigned long)tps->active_rdo_raw);
    PowerManager_LogMonitorRow("PD RAW", value, details);

    (void)snprintf(value, sizeof(value), "VBUS=%lu mV",
                   (unsigned long)tps->vbus_mv);
    (void)snprintf(details, sizeof(details),
                   "PP5V=%u PPHV=%u HR=%u OC=%u/%u",
                   tps->pp5v_state, tps->pphv_state,
                   tps->hard_reset_reason,
                   tps->pp5v_overcurrent ? 1U : 0U,
                   tps->ppcable_overcurrent ? 1U : 0U);
    PowerManager_LogMonitorRow("TPS PATH", value, details);

    (void)snprintf(value, sizeof(value), "%s", power_flow);
    (void)snprintf(details, sizeof(details), "input=%s  phase=%s",
                   bq->input_present ? "YES" : "NO", charge_state);
    PowerManager_LogMonitorRow("POWER FLOW", value, details);

    (void)snprintf(value, sizeof(value), "VBUS=%lu mV  I=%lu mA",
                   (unsigned long)bq->adc_vbus_mv,
                   (unsigned long)bq->adc_iin_ma);
    (void)snprintf(details, sizeof(details),
                   "IIN_SET=%lu mA  IINDPM=%u  VINDPM=%u",
                   (unsigned long)bq->iin_dpm_ma,
                   bq->in_iin_dpm ? 1U : 0U,
                   bq->in_vindpm ? 1U : 0U);
    PowerManager_LogMonitorRow("BQ INPUT", value, details);

    (void)snprintf(value, sizeof(value), "VBAT=%lu mV  I=%ld mA",
                   (unsigned long)bq->adc_vbat_mv,
                   (long)bq->battery_current_ma);
    (void)snprintf(details, sizeof(details), "P=%ld mW",
                   (long)bq->battery_power_mw);
    PowerManager_LogMonitorRow("BATTERY", value, details);

    (void)snprintf(value, sizeof(value), "VREG=%lu mV  ICHG=%lu mA",
                   (unsigned long)bq->charge_voltage_mv,
                   (unsigned long)bq->charge_current_ma);
    (void)snprintf(details, sizeof(details),
                   "faults=0x%02X  status=0x%04X",
                   bq->fault_flags, bq->charger_status);
    PowerManager_LogMonitorRow("BQ LIMITS", value, details);

    (void)snprintf(value, sizeof(value), "attach=%lu detach=%lu HR=%lu",
                   (unsigned long)g_pm.attach_count,
                   (unsigned long)g_pm.detach_count,
                   (unsigned long)g_pm.hard_reset_count);
    (void)snprintf(details, sizeof(details),
                   "plug=%lu contract=%lu swap=%lu",
                   (unsigned long)g_pm.plug_change_count,
                   (unsigned long)g_pm.contract_count,
                   (unsigned long)g_pm.power_swap_count);
    PowerManager_LogMonitorRow("EVENTS", value, details);

    (void)snprintf(value, sizeof(value), "%s",
                   (g_pm.status.source_fault_latched ||
                    (bq->fault_flags != 0U)) ? "FAULT" :
                   ((g_pm.power_error_count != 0U) ||
                    (g_pm.i2c_nack_count != 0U) ||
                    (g_pm.unable_source_count != 0U) ||
                    (g_pm.overcurrent_count != 0U)) ? "WARN" : "OK");
    (void)snprintf(details, sizeof(details),
                   "pwr=%lu nack=%lu unable=%lu OC=%lu",
                   (unsigned long)g_pm.power_error_count,
                   (unsigned long)g_pm.i2c_nack_count,
                   (unsigned long)g_pm.unable_source_count,
                   (unsigned long)g_pm.overcurrent_count);
    PowerManager_LogMonitorRow("HEALTH", value, details);

    (void)snprintf(value, sizeof(value), "queue=%lu B  dropped=%lu",
                   (unsigned long)uart_used,
                   (unsigned long)uart_dropped);
    (void)snprintf(details, sizeof(details), "busy=%u",
                   Debug_IsTxBusy() ? 1U : 0U);
    PowerManager_LogMonitorRow("UART", value, details);

    Debug_Printf("[MON] =======================================================================================");
    Debug_Printf("[MON] ");
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

static const char *PowerManager_TypecStateToString(uint8_t state)
{
    switch (state) {
        case 0x00U: return "Disabled";
        case 0x05U: return "ErrorRecovery";
        case 0x24U: return "Unattached.Accessory";
        case 0x2BU: return "AttachWait.Accessory";
        case 0x45U: return "Try.SRC";
        case 0x4EU: return "TryWait.SNK";
        case 0x4FU: return "Try.SNK";
        case 0x50U: return "TryWait.SRC";
        case 0x60U: return "Attached.SRC";
        case 0x61U: return "Attached.SNK";
        case 0x62U: return "AudioAccessory";
        case 0x63U: return "DebugAccessory";
        case 0x64U: return "AttachWait.SRC";
        case 0x65U: return "AttachWait.SNK";
        case 0x66U: return "Unattached.SNK";
        case 0x67U: return "Unattached.SRC";
        default: return "Unknown";
    }
}

static const char *PowerManager_CcStateToString(uint8_t state)
{
    switch (state) {
        case 0U: return "open";
        case 1U: return "Ra";
        case 2U: return "Rd";
        case 3U: return "Rp-default";
        case 4U: return "Rp-1.5A";
        case 5U: return "Rp-3A";
        default: return "invalid";
    }
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

static void PowerManager_TryLogContractPdos(void)
{
    const TPS25751_Capabilities_t *caps;
    const char *list_name;
    uint8_t selected_index;
    uint8_t count;
    uint8_t i;

    if (!g_pm.pdo_report_pending ||
        !PowerManager_HasTypecPowerConnection() ||
        !g_pm.status.tps.active_pdo.valid ||
        !g_pm.status.tps.active_rdo.valid) {
        return;
    }

    if (g_pm.status.tps.role == TPS25751_ROLE_SOURCE) {
        if (!g_pm.local_source_caps_valid) {
            return;
        }
        caps = &g_pm.local_source_caps;
        count = caps->count;
        list_name = "TPS_TX_SOURCE";
    } else if (g_pm.status.tps.role == TPS25751_ROLE_SINK) {
        if (!g_pm.partner_source_caps_current) {
            return;
        }
        caps = &g_pm.partner_source_caps;
        count = caps->count;
        list_name = "PARTNER_SOURCE";
    } else {
        return;
    }

    if (count == 0U) {
        return;
    }

    selected_index = g_pm.status.tps.active_rdo.object_position;

    Debug_Printf("[PD-PDO] ");
    Debug_Printf("[PD-PDO] =========================== USB-C PD PDO TABLE ===========================");
    Debug_Printf("[PD-PDO] Role: %-6s | List: %-14s | Entries: %u",
                 PowerManager_RoleToString(g_pm.status.tps.role),
                 list_name, count);
    Debug_Printf("[PD-PDO] +-----+---------+-------------------+-------------------+-----------+-----------------------+");
    Debug_Printf("[PD-PDO] | PDO | TYPE    | VOLTAGE           | CURRENT LIMIT     | MAX POWER | STATE                 |");
    Debug_Printf("[PD-PDO] +-----+---------+-------------------+-------------------+-----------+-----------------------+");
    for (i = 0U; i < count; ++i) {
        char voltage_text[32];
        char current_text[32];
        char power_text[20];
        uint8_t index = (uint8_t)(i + 1U);
        uint32_t raw = caps->pdo[i].raw;
        TPS25751_Pdo_t pdo = TPS25751_DecodePdo(raw);
        const char *state = (index == selected_index) ?
                            "<-- SELECTED + ACTIVE" : "";
        const char *type;
        uint32_t supply_type = (raw >> 30) & 0x03U;
        uint32_t apdo_type = (raw >> 28) & 0x03U;

        (void)snprintf(voltage_text, sizeof(voltage_text),
                       "%lu.%03lu-%lu.%03lu V",
                       (unsigned long)(pdo.min_voltage_mv / 1000U),
                       (unsigned long)(pdo.min_voltage_mv % 1000U),
                       (unsigned long)(pdo.max_voltage_mv / 1000U),
                       (unsigned long)(pdo.max_voltage_mv % 1000U));
        (void)snprintf(power_text, sizeof(power_text), "%lu.%03lu W",
                       (unsigned long)(pdo.power_mw / 1000U),
                       (unsigned long)(pdo.power_mw % 1000U));

        if ((supply_type == 3U) && (apdo_type == 2U)) {
            type = "SPR AVS";
            (void)snprintf(current_text, sizeof(current_text),
                           "%lu.%03lu/%lu.%03lu A",
                           (unsigned long)((((raw >> 10) & 0x3FFU) * 10U) / 1000U),
                           (unsigned long)((((raw >> 10) & 0x3FFU) * 10U) % 1000U),
                           (unsigned long)(((raw & 0x3FFU) * 10U) / 1000U),
                           (unsigned long)(((raw & 0x3FFU) * 10U) % 1000U));
        } else {
            type = (supply_type == 0U) ? "FIXED" :
                   ((supply_type == 1U) ? "BATTERY" :
                   ((supply_type == 2U) ? "VARIABLE" : "PPS"));
            if (pdo.current_ma != 0U) {
                (void)snprintf(current_text, sizeof(current_text),
                               "%lu.%03lu A",
                               (unsigned long)(pdo.current_ma / 1000U),
                               (unsigned long)(pdo.current_ma % 1000U));
            } else {
                (void)snprintf(current_text, sizeof(current_text), "power based");
            }
        }

        if (pdo.min_voltage_mv == pdo.max_voltage_mv) {
            (void)snprintf(voltage_text, sizeof(voltage_text), "%lu.%03lu V",
                           (unsigned long)(pdo.voltage_mv / 1000U),
                           (unsigned long)(pdo.voltage_mv % 1000U));
        }

        Debug_Printf("[PD-PDO] | %3u | %-7s | %-17s | %-17s | %-9s | %-21s |",
                     index, type, voltage_text, current_text, power_text, state);
    }

    Debug_Printf("[PD-PDO] +-----+---------+-------------------+-------------------+-----------+-----------------------+");
    Debug_Printf("[PD-PDO] ACTIVE CONTRACT: PDO%u -> %lu.%03lu V x %lu.%03lu A = %lu.%03lu W",
                 selected_index,
                 (unsigned long)(g_pm.status.pd_snapshot.contract_voltage_mv / 1000U),
                 (unsigned long)(g_pm.status.pd_snapshot.contract_voltage_mv % 1000U),
                 (unsigned long)(g_pm.status.pd_snapshot.contract_current_ma / 1000U),
                 (unsigned long)(g_pm.status.pd_snapshot.contract_current_ma % 1000U),
                 (unsigned long)(g_pm.status.pd_snapshot.contract_power_mw / 1000U),
                 (unsigned long)(g_pm.status.pd_snapshot.contract_power_mw % 1000U));
    Debug_Printf("[PD-PDO] ========================================================================");
    Debug_Printf("[PD-PDO] ");

    g_pm.pdo_report_pending = false;
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
    g_pm.pdo_report_pending = attached;
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
            if (PM_VERBOSE_TRANSITION_LOGS != 0U) {
                Debug_Printf("[PD-POLICY] attach role=SOURCE partner=Sink(Rd); VBUS enabled, TPS Source Policy owns automatic PDO advertisement");
            }
        } else {
            /* Do not issue Get_Sink_Cap or a role swap while TPS is still
             * establishing the first contract.  A strict source may reject
             * that optional message, and the 4CC timeout would delay ACTIVE
             * PDO/RDO telemetry and the matching BQ input-current update.
             * Once a contract is valid, MaintainPolicy uses its voltage and
             * the standard Dual-Role Power bit from the active Source PDO. */
            g_pm.policy_phase = PM_POLICY_DONE;
            g_pm.policy_next_ms = 0U;
            if (PM_VERBOSE_TRANSITION_LOGS != 0U) {
                Debug_Printf("[PD-POLICY] attach role=%s; TPS owns initial negotiation, policy waits for stable PDO/RDO",
                             PowerManager_RoleToString(g_pm.status.tps.role));
            }
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
        if (g_pm.status.tps.attached) {
            g_pm.attach_count++;
        } else {
            g_pm.detach_count++;
        }
        PowerManager_ResetPolicy(now_ms, g_pm.status.tps.attached);
        if ((PM_VERBOSE_TRANSITION_LOGS != 0U) &&
            !g_pm.status.tps.attached) {
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

    if (event.plug_changed) {
        g_pm.plug_change_count++;
    }
    if (event.new_contract_consumer || event.new_contract_provider) {
        g_pm.contract_count++;
    }
    if (event.power_swap_complete) {
        g_pm.power_swap_count++;
    }
    if (event.hard_reset) {
        g_pm.hard_reset_count++;
    }
    if (event.power_event_error) {
        g_pm.power_error_count++;
    }
    if (event.i2c_controller_nack) {
        g_pm.i2c_nack_count++;
    }
    if (event.unable_to_source) {
        g_pm.unable_source_count++;
    }
    if (event.overcurrent) {
        g_pm.overcurrent_count++;
    }

    if (event.hard_reset || event.unable_to_source ||
        event.power_event_error || event.i2c_controller_nack ||
        event.overcurrent) {
        Debug_Printf("[PD-WARN t=%lu] hard_reset=%u unable_source=%u power_error=%u i2cc_nack=%u overcurrent=%u raw0=0x%02X raw1=0x%02X raw2=0x%02X raw5=0x%02X raw10=0x%02X",
                     (unsigned long)now_ms,
                     event.hard_reset ? 1U : 0U,
                     event.unable_to_source ? 1U : 0U,
                     event.power_event_error ? 1U : 0U,
                     event.i2c_controller_nack ? 1U : 0U,
                     event.overcurrent ? 1U : 0U,
                     event.raw[0], event.raw[1], event.raw[2], event.raw[5],
                     event.raw[10]);
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
    if (event.source_caps_received) {
        g_pm.partner_source_caps_current = true;
        g_pm.source_caps_trace_pending = true;
    }
    if (event.plug_changed || event.source_caps_received ||
        event.new_contract_consumer || event.hard_reset) {
        /* Full BQ status/ADC reads use the TPS embedded-controller path.
         * Keep that low-priority traffic out of the time-critical attach and
         * contract window.  TPS remains the sole owner of BQ power limits. */
        g_pm.bq_monitor_holdoff_until_ms =
            now_ms + PM_BQ_MONITOR_HOLDOFF_MS;
    }
    if (event.plug_changed || event.power_swap_complete ||
        event.source_caps_received) {
        g_pm.pdo_report_pending = g_pm.status.tps.attached;
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
        } else if (completed_job == PM_JOB_READ_LOCAL_SOURCE_CAPS) {
            memset(&g_pm.local_source_caps, 0,
                   sizeof(g_pm.local_source_caps));
            g_pm.local_source_caps_pending = false;
            g_pm.local_source_caps_valid = false;
            Debug_Printf("[PD-CAPS] TPS TX_SOURCE_CAPS read failed status=%s len=%u",
                         TPS25751_StatusToString(operation_status),
                         g_pm.tps.reported_length);
            PowerManager_HandleTpsError(operation_status, now_ms);
        } else if (completed_job == PM_JOB_TRACE_SOURCE_CAPS) {
            memset(&g_pm.partner_source_caps, 0,
                   sizeof(g_pm.partner_source_caps));
            g_pm.partner_source_caps_current = false;
            g_pm.source_caps_trace_pending = false;
            PowerManager_HandleTpsError(operation_status, now_ms);
        } else if (completed_job == PM_JOB_BQ_TRACE_IIN_HOST) {
            g_pm.bq_iin_trace_pending = false;
            Debug_Printf("[BQ-TRACE t=%lu] IIN_HOST read failed status=%s task=%u",
                         (unsigned long)now_ms,
                         TPS25751_StatusToString(operation_status),
                         g_pm.tps.task_return_code);
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
                    g_pm.local_source_caps_pending = true;
                    g_pm.local_source_caps_valid = false;
                }
                PowerManager_SetState(POWER_MANAGER_TPS_READY);
            } else {
                g_pm.app_seen_ms = 0U;
                g_pm.status.applied_mode_valid = false;
                g_pm.mode_update_pending = true;
                g_pm.local_source_caps_pending = false;
                g_pm.local_source_caps_valid = false;
                memset(&g_pm.local_source_caps, 0,
                       sizeof(g_pm.local_source_caps));
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
                g_pm.pdo_report_pending = true;
                if (PM_VERBOSE_TRANSITION_LOGS != 0U) {
                    Debug_Printf("[PD-ROLE] %s -> %s conn=%u STATUS=0x%02lX%08lX",
                                 PowerManager_RoleToString(old_role),
                                 PowerManager_RoleToString(g_pm.status.tps.role),
                                 g_pm.status.tps.connection_state,
                                 (unsigned long)((g_pm.status.tps.status_raw >> 32) & 0xFFU),
                                 (unsigned long)g_pm.status.tps.status_raw);
                }
            }
            PowerManager_UpdateAttachPolicy(now_ms);
            g_pm.telemetry_phase = 1U;
            break;
        }

        case PM_JOB_READ_TYPEC_STATE:
        {
            uint32_t old_raw = g_pm.status.tps.typec_state_raw;

            TPS25751_DecodeTypecState(&g_pm.status.tps, data);
            if ((PM_VERBOSE_TRANSITION_LOGS != 0U) &&
                ((!g_pm.typec_trace_valid) ||
                 (g_pm.status.tps.typec_state_raw != old_raw))) {
                Debug_Printf("[PD-TYPEC t=%lu] state=0x%02X(%s) CC1=%u(%s) CC2=%u(%s) PD_CC=%u raw=0x%08lX STATUS=0x%02lX%08lX",
                             (unsigned long)now_ms,
                             g_pm.status.tps.typec_port_state,
                             PowerManager_TypecStateToString(
                                 g_pm.status.tps.typec_port_state),
                             g_pm.status.tps.cc1_state,
                             PowerManager_CcStateToString(
                                 g_pm.status.tps.cc1_state),
                             g_pm.status.tps.cc2_state,
                             PowerManager_CcStateToString(
                                 g_pm.status.tps.cc2_state),
                             g_pm.status.tps.pd_cc_pin,
                             (unsigned long)g_pm.status.tps.typec_state_raw,
                             (unsigned long)((g_pm.status.tps.status_raw >> 32) & 0xFFU),
                             (unsigned long)g_pm.status.tps.status_raw);
            }
            g_pm.typec_trace_valid = true;
            g_pm.telemetry_phase = 2U;
            break;
        }

        case PM_JOB_READ_POWER_PATH:
            TPS25751_DecodePowerPath(&g_pm.status.tps, data);
            if (g_pm.status.tps.pp5v_overcurrent ||
                g_pm.status.tps.ppcable_overcurrent) {
                g_pm.status.source_fault_latched = true;
                PowerManager_SetOtgPin(false);
            }
            g_pm.telemetry_phase = 3U;
            break;

        case PM_JOB_READ_POWER_STATUS:
            TPS25751_DecodePowerStatus(&g_pm.status.tps, data);
            g_pm.telemetry_phase = 4U;
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
            g_pm.telemetry_phase = 5U;
            break;

        case PM_JOB_READ_ADC:
            TPS25751_DecodeAdcResults(&g_pm.status.tps, data);
            g_pm.telemetry_phase = 6U;
            break;

        case PM_JOB_READ_ACTIVE_PDO:
            if (g_pm.status.tps.active_pdo_raw !=
                TPS25751_ReadLe32(data)) {
                g_pm.pdo_report_pending = g_pm.status.tps.attached;
            }
            g_pm.status.tps.active_pdo_raw = TPS25751_ReadLe32(data);
            g_pm.status.tps.active_pdo = TPS25751_DecodePdo(
                g_pm.status.tps.active_pdo_raw);
            g_pm.telemetry_phase = 7U;
            PowerManager_UpdatePdSnapshot();
            break;

        case PM_JOB_READ_ACTIVE_RDO:
            if (g_pm.status.tps.active_rdo_raw !=
                TPS25751_ReadLe32(data)) {
                g_pm.pdo_report_pending = g_pm.status.tps.attached;
            }
            g_pm.status.tps.active_rdo_raw = TPS25751_ReadLe32(data);
            g_pm.status.tps.active_rdo = TPS25751_DecodeRdo(
                g_pm.status.tps.active_rdo_raw,
                &g_pm.status.tps.active_pdo);
            g_pm.telemetry_phase = 8U;
            PowerManager_UpdatePdSnapshot();
            PowerManager_TryLogContractPdos();
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
            PowerManager_TryLogContractPdos();
            g_pm.policy_phase = PM_POLICY_DECIDE;
            g_pm.policy_next_ms = now_ms + PM_POLICY_STEP_MS;
            break;

        case PM_JOB_READ_LOCAL_SOURCE_CAPS:
            g_pm.local_source_caps_pending = false;
            if (!TPS25751_DecodeTxSourceCapabilities(
                    &g_pm.local_source_caps, data, length) ||
                (g_pm.local_source_caps.count == 0U)) {
                memset(&g_pm.local_source_caps, 0,
                       sizeof(g_pm.local_source_caps));
                g_pm.local_source_caps_valid = false;
                Debug_Printf("[PD-CAPS] invalid TPS TX_SOURCE_CAPS payload len=%u",
                             length);
                break;
            }
            g_pm.local_source_caps_valid = true;
            PowerManager_LogCapabilities("TPS_TX_SOURCE",
                                         &g_pm.local_source_caps);
            PowerManager_TryLogContractPdos();
            break;

        case PM_JOB_TRACE_SOURCE_CAPS:
            g_pm.source_caps_trace_pending = false;
            if (!TPS25751_DecodeCapabilities(&g_pm.partner_source_caps,
                                             data, length)) {
                memset(&g_pm.partner_source_caps, 0,
                       sizeof(g_pm.partner_source_caps));
                g_pm.partner_source_caps_current = false;
                Debug_Printf("[PD-TRACE t=%lu] Source PDO register invalid len=%u",
                             (unsigned long)now_ms, length);
                break;
            }
            g_pm.partner_source_caps_current = true;
            Debug_Printf("[PD-TRACE t=%lu] Source_Capabilities captured immediately",
                         (unsigned long)now_ms);
            PowerManager_LogCapabilities("SOURCE",
                                         &g_pm.partner_source_caps);
            PowerManager_TryLogContractPdos();
            break;

        case PM_JOB_SWAP_TO_SOURCE:
        case PM_JOB_SWAP_TO_SINK:
            g_pm.policy_phase = PM_POLICY_DONE;
            g_pm.policy_next_ms = now_ms + PM_POLICY_SWAP_RETRY_MS;
            Debug_Printf("[PD-POLICY] role swap accepted target=%s; maintained policy verifies actual role in [PD]",
                         PowerManager_RoleToString(g_pm.policy_desired_role));
            break;

        case PM_JOB_BQ_TRACE_IIN_HOST:
            g_pm.bq_iin_trace_pending = false;
            raw16 = PowerManager_ResultLe16(&valid);
            if (!valid) {
                Debug_Printf("[BQ-TRACE t=%lu] IIN_HOST invalid response len=%u",
                             (unsigned long)now_ms, length);
                break;
            }
            if (PM_VERBOSE_TRANSITION_LOGS != 0U) {
                Debug_Printf("[BQ-TRACE t=%lu] attached=%u role=%s IIN_HOST=0x%04X (%lumA)",
                             (unsigned long)now_ms,
                             g_pm.status.tps.attached ? 1U : 0U,
                             PowerManager_RoleToString(g_pm.status.tps.role),
                             raw16,
                             (unsigned long)BQ25731_DecodeInputCurrentMa(raw16));
            }
            g_pm.status.bq.iin_host = raw16;
            g_pm.status.bq.input_current_ma =
                BQ25731_DecodeInputCurrentMa(raw16);
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
                g_pm.bq_init = PM_BQ_INIT_READ_OPTION1;
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
                    PM_BQ_INIT_READ_OPTION1 :
                    PM_BQ_INIT_WRITE_OPTION4;
            }
            g_pm.next_bq_action_ms = now_ms + PM_BQ_INIT_STEP_MS;
            break;

        case PM_JOB_BQ_WRITE_STARTUP_OPTION4:
            g_pm.bq_init = PM_BQ_INIT_VERIFY_OPTION4;
            g_pm.next_bq_action_ms = now_ms + PM_BQ_INIT_STEP_MS;
            break;

        case PM_JOB_BQ_READ_OPTION1:
            raw16 = PowerManager_ResultLe16(&valid);
            if (!valid) {
                PowerManager_HandleBqError(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            if (g_pm.bq_init == PM_BQ_INIT_READ_OPTION1) {
                g_pm.bq_startup_option1_target =
                    BQ25731_BuildStartupOption1(raw16);
                g_pm.bq_init =
                    (g_pm.bq_startup_option1_target == raw16) ?
                    PM_BQ_INIT_WRITE_ADC : PM_BQ_INIT_WRITE_OPTION1;
            } else {
                if ((raw16 & BQ25731_CHARGE_OPTION1_5MOHM_MASK) !=
                    BQ25731_CHARGE_OPTION1_5MOHM_MASK) {
                    PowerManager_HandleBqError(
                        TPS25751_COMMAND_ERROR, now_ms);
                    break;
                }
                g_pm.bq_init = PM_BQ_INIT_WRITE_ADC;
            }
            Debug_Printf("[BQ-INIT] RAC=5mOhm RSR=5mOhm FAST_5MOHM=1 Option1=0x%04X%s",
                         raw16,
                         (g_pm.bq_init == PM_BQ_INIT_WRITE_OPTION1) ?
                         " -> programming" : " verified");
            g_pm.next_bq_action_ms = now_ms + PM_BQ_INIT_STEP_MS;
            break;

        case PM_JOB_BQ_WRITE_STARTUP_OPTION1:
            g_pm.bq_init = PM_BQ_INIT_VERIFY_OPTION1;
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
            Debug_Printf("[BQ] TPS owns PD and runtime limits; STM only applies startup RAC=5mOhm, RSR=5mOhm, fast compensation, quiet switching and ADC fields");
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
            /* IIN_HOST sits outside the compact configuration block and is
             * volatile across adapter transitions.  Read it separately so
             * the human-readable line never reports a stale/zero value. */
            g_pm.bq_iin_trace_pending = true;
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
        case PM_JOB_READ_TYPEC_STATE:
            status = TPS25751_StartReadRegister(
                &g_pm.tps, TPS25751_REG_TYPE_C_STATE,
                TPS25751_TYPE_C_STATE_LEN);
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
                TPS25751_REG_ACTIVE_PDO,
                TPS25751_ACTIVE_PDO_PREFIX_LEN);
            break;
        case PM_JOB_READ_ACTIVE_RDO:
            status = TPS25751_StartReadRegister(&g_pm.tps,
                TPS25751_REG_ACTIVE_RDO,
                TPS25751_ACTIVE_RDO_PREFIX_LEN);
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
        case PM_JOB_TRACE_SOURCE_CAPS:
            status = TPS25751_StartReadRegister(
                &g_pm.tps, TPS25751_REG_RX_SOURCE_CAPS,
                TPS25751_RX_CAPS_LEN);
            break;
        case PM_JOB_READ_LOCAL_SOURCE_CAPS:
            status = TPS25751_StartReadRegister(
                &g_pm.tps, TPS25751_REG_TX_SOURCE_CAPS,
                TPS25751_TX_SOURCE_CAPS_LEN);
            break;
        case PM_JOB_SWAP_TO_SOURCE:
            status = TPS25751_StartCommand(&g_pm.tps, "SWSr",
                                           NULL, 0U, 1U);
            break;
        case PM_JOB_SWAP_TO_SINK:
            status = TPS25751_StartCommand(&g_pm.tps, "SWSk",
                                           NULL, 0U, 1U);
            break;
        case PM_JOB_BQ_TRACE_IIN_HOST:
            bq_status = BQ25731_StartRead16(&g_pm.bq,
                                            BQ25731_REG_IIN_HOST);
            status = (bq_status == BQ25731_OK) ? TPS25751_OK :
                                                TPS25751_BUSY;
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
        case PM_JOB_BQ_READ_OPTION1:
            bq_status = BQ25731_StartRead16(
                &g_pm.bq, BQ25731_REG_CHARGE_OPTION1);
            status = (bq_status == BQ25731_OK) ? TPS25751_OK :
                                                TPS25751_BUSY;
            break;
        case PM_JOB_BQ_WRITE_STARTUP_OPTION1:
            bq_status = BQ25731_StartWriteStartupOption1(
                &g_pm.bq, g_pm.bq_startup_option1_target);
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
        case 1U: return PM_JOB_READ_TYPEC_STATE;
        case 2U: return PM_JOB_READ_POWER_PATH;
        case 3U: return PM_JOB_READ_POWER_STATUS;
        case 4U: return PM_JOB_READ_PD_STATUS;
        case 5U: return PM_JOB_READ_ADC;
        case 6U: return PM_JOB_READ_ACTIVE_PDO;
        case 7U: return PM_JOB_READ_ACTIVE_RDO;
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
    if (g_pm.local_source_caps_pending) {
        return PM_JOB_READ_LOCAL_SOURCE_CAPS;
    }
    if (g_pm.source_caps_trace_pending) {
        return PM_JOB_TRACE_SOURCE_CAPS;
    }
    if (g_pm.bq_iin_trace_pending) {
        return PM_JOB_BQ_TRACE_IIN_HOST;
    }

    policy_job = PowerManager_SelectPolicyJob(now_ms);
    if (policy_job != PM_JOB_NONE) {
        return policy_job;
    }

    if (g_pm.bq_init == PM_BQ_INIT_WAIT &&
        (PM_ENABLE_BQ_EC_ACCESS != 0U) &&
        PowerManager_TickReached(now_ms,
                                 g_pm.bq_monitor_holdoff_until_ms) &&
        PowerManager_TickReached(now_ms, g_pm.next_bq_action_ms)) {
        g_pm.bq_init = PM_BQ_INIT_READ_ID;
        PowerManager_SetState(POWER_MANAGER_BQ_PROBE);
    }
    if ((PM_ENABLE_BQ_EC_ACCESS != 0U) &&
        PowerManager_TickReached(now_ms,
                                 g_pm.bq_monitor_holdoff_until_ms) &&
        PowerManager_TickReached(now_ms, g_pm.next_bq_action_ms)) {
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
            case PM_BQ_INIT_READ_OPTION1:
            case PM_BQ_INIT_VERIFY_OPTION1:
                return PM_JOB_BQ_READ_OPTION1;
            case PM_BQ_INIT_WRITE_OPTION1:
                return PM_JOB_BQ_WRITE_STARTUP_OPTION1;
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
        (PM_ENABLE_BQ_EC_ACCESS != 0U) &&
        PowerManager_TickReached(now_ms,
                                 g_pm.bq_monitor_holdoff_until_ms) &&
        PowerManager_TickReached(now_ms, g_pm.next_bq_telemetry_ms)) {
        return PowerManager_SelectBqTelemetryJob();
    }
    if ((g_pm.bq_init == PM_BQ_INIT_DONE) &&
        (PM_ENABLE_BQ_EC_ACCESS != 0U) &&
        PowerManager_TickReached(now_ms,
                                 g_pm.bq_monitor_holdoff_until_ms) &&
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
    g_pm.bq_monitor_holdoff_until_ms =
        now_ms + PM_BQ_MONITOR_HOLDOFF_MS;
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
