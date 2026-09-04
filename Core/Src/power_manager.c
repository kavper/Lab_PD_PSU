/*
 * USB-C power-role manager for TPS25751 (PD controller) + BQ25731 (charger).
 *
 * Division of work:
 *   - The TPS25751 owns Type-C attach, PD negotiation and all BQ25731 power
 *     control (charge limits and OTG) over its own I2C controller port.
 *   - This module only selects the port role and reads telemetry. It never
 *     writes a BQ register that the TPS25751 uses for power control at
 *     runtime, and it never touches the PD state machine mid-contract.
 *
 * AUTO role rule requested by the application:
 *   - partner can supply more than 5 V  -> we sink from it,
 *   - partner is 5 V only or has no PD  -> we source into it.
 * The decision is taken once per attach and then locked in by disabling the
 * matching automatic PR_Swap accept bit in PORT_CONTROL, so two dual-role
 * ports can never ping-pong the power role.
 */

#include "power_manager.h"

#include "debug_uart.h"
#include "tps_int_event.h"

#include <stdio.h>
#include <string.h>

#define PM_MODE_POLL_MS               250U
#define PM_BOOT_FLAGS_POLL_MS        1000U
#define PM_TELEMETRY_STEP_MS           10U
#define PM_LOG_PERIOD_MS             2000U
#define PM_ERROR_LOG_MS              1000U
#define PM_ERROR_BACKOFF_MS           100U
#define PM_SETUP_RETRY_MS            2000U

/* Role policy. */
#define PM_SOURCE_5V_MV              5000U
#define PM_CAPS_GRACE_MS             1500U
#define PM_CAPS_RETRY_MS              300U
#define PM_SWAP_MAX_ATTEMPTS            2U
#define PM_SWAP_BACKOFF_MS           5000U

/* BQ25731 access through the TPS25751 I2C controller tunnel. */
#define PM_ENABLE_BQ_ACCESS             1U
#define PM_BQ_START_DELAY_MS         1000U
#define PM_BQ_STEP_MS                 200U
#define PM_BQ_TELEMETRY_MS           1000U
#define PM_BQ_CONFIG_MS             10000U
#define PM_BQ_PD_QUIET_MS            2000U
#define PM_BQ_MAX_ATTEMPTS              3U

typedef enum {
    PM_JOB_NONE = 0,
    PM_JOB_READ_MODE,
    PM_JOB_READ_BOOT_FLAGS,
    PM_JOB_READ_PORT_CONFIG,
    PM_JOB_WRITE_PORT_CONFIG,
    PM_JOB_READ_PORT_CONTROL,
    PM_JOB_WRITE_PORT_CONTROL,
    PM_JOB_READ_INT_MASK,
    PM_JOB_WRITE_INT_MASK,
    PM_JOB_CLEAR_EVENT,
    PM_JOB_READ_STATUS,
    PM_JOB_READ_TYPEC_STATE,
    PM_JOB_READ_POWER_PATH,
    PM_JOB_READ_PD_STATUS,
    PM_JOB_READ_ADC,
    PM_JOB_READ_ACTIVE_PDO,
    PM_JOB_READ_ACTIVE_RDO,
    PM_JOB_READ_EVENT,
    PM_JOB_READ_PARTNER_CAPS,
    PM_JOB_READ_LOCAL_CAPS,
    PM_JOB_READ_SINK_CAPS,
    PM_JOB_WRITE_SINK_CAPS,
    PM_JOB_SWAP_TO_SOURCE,
    PM_JOB_SWAP_TO_SINK,
    PM_JOB_BQ_READ_ID,
    PM_JOB_BQ_READ_OPTION0,
    PM_JOB_BQ_WRITE_OPTION0,
    PM_JOB_BQ_READ_OPTION4,
    PM_JOB_BQ_WRITE_OPTION4,
    PM_JOB_BQ_READ_OPTION1,
    PM_JOB_BQ_WRITE_OPTION1,
    PM_JOB_BQ_WRITE_ADC,
    PM_JOB_BQ_READ_STATUS_BLOCK,
    PM_JOB_BQ_READ_ADC_BLOCK,
    PM_JOB_BQ_READ_CONFIG_BLOCK
} PowerManager_Job_t;

typedef enum {
    PM_BQ_WAIT = 0,
    PM_BQ_ID,
    PM_BQ_OPTION0,
    PM_BQ_OPTION4,
    PM_BQ_OPTION1,
    PM_BQ_ADC,
    PM_BQ_READY,
    PM_BQ_DISABLED
} PowerManager_BqStage_t;

typedef enum {
    PM_ROLE_UNDECIDED = 0,
    PM_ROLE_WANT_SOURCE,
    PM_ROLE_WANT_SINK
} PowerManager_RoleTarget_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    TPS25751_Device_t tps;
    BQ25731_Device_t bq;
    PowerManager_Status_t status;

    PowerManager_Job_t job;
    PowerManager_Job_t last_job;
    bool initialized;

    /* TPS register maintenance. */
    uint8_t port_config[TPS25751_PORT_CONFIG_LEN];
    uint8_t port_control[TPS25751_PORT_CONTROL_LEN];
    uint8_t port_control_length;
    uint8_t int_mask[TPS_INT_EVENT_BYTES];
    uint8_t event_to_clear[TPS_INT_EVENT_BYTES];
    uint8_t event_seen[TPS_INT_EVENT_BYTES];
    uint8_t sink_caps[TPS25751_TX_SINK_CAPS_LEN];
    uint8_t sink_caps_length;
    uint8_t telemetry_phase;
    bool port_config_read_pending;
    bool port_config_write_pending;
    bool port_control_read_pending;
    bool port_control_write_pending;
    bool int_mask_read_pending;
    bool int_mask_write_pending;
    bool event_clear_pending;
    bool local_caps_pending;
    bool sink_caps_read_pending;
    bool sink_caps_write_pending;
    bool partner_caps_pending;
    bool int_mask_ready;
    bool sink_caps_checked;
    uint32_t setup_retry_ms;

    /* Role policy, valid for one attach session. */
    PowerManager_RoleTarget_t role_target;
    bool swap_job_pending;
    bool swap_budget_logged;
    bool accept_swap_to_source;
    bool accept_swap_to_sink;
    bool swap_policy_applied;
    uint8_t swap_attempts;
    uint32_t attach_ms;
    uint32_t swap_next_ms;
    uint32_t partner_caps_next_ms;
    bool partner_caps_valid;
    bool local_caps_valid;
    TPS25751_Capabilities_t partner_caps;
    TPS25751_Capabilities_t local_caps;

    /* BQ bring-up and telemetry. */
    PowerManager_BqStage_t bq_stage;
    bool bq_write_pending;
    uint8_t bq_attempts;
    uint8_t bq_telemetry_phase;
    uint16_t bq_option_target;
    uint32_t bq_next_action_ms;
    uint32_t bq_next_telemetry_ms;
    uint32_t bq_next_config_ms;
    uint32_t pd_quiet_until_ms;

    /* Timers and counters. */
    uint32_t next_mode_ms;
    uint32_t next_boot_flags_ms;
    uint32_t next_telemetry_ms;
    uint32_t next_log_ms;
    uint32_t last_error_log_ms;
    uint32_t retry_hold_ms;
    uint32_t app_seen_ms;
    uint32_t tps_error_count;
    uint32_t bq_error_count;
    uint32_t attach_count;
    uint32_t detach_count;
    uint32_t hard_reset_count;
    uint32_t swap_count;
    uint32_t overcurrent_count;
    uint32_t power_error_count;
    uint32_t unable_source_count;
    uint32_t i2c_nack_count;
} PowerManager_Context_t;

static PowerManager_Context_t g_pm;

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

static const char *PowerManager_RoleToString(TPS25751_PowerRole_t role)
{
    switch (role) {
        case TPS25751_ROLE_SOURCE: return "SOURCE";
        case TPS25751_ROLE_SINK: return "SINK";
        default: return "NONE";
    }
}

static const char *PowerManager_TargetToString(
    PowerManager_RoleTarget_t target)
{
    switch (target) {
        case PM_ROLE_WANT_SOURCE: return "SOURCE";
        case PM_ROLE_WANT_SINK: return "SINK";
        default: return "PENDING";
    }
}

static const char *PowerManager_TypecStateToString(uint8_t state)
{
    switch (state) {
        case 0x00U: return "Disabled";
        case 0x05U: return "ErrorRecovery";
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
        default: return "Other";
    }
}

static const char *PowerManager_JobToString(PowerManager_Job_t job)
{
    switch (job) {
        case PM_JOB_READ_MODE: return "READ_MODE";
        case PM_JOB_READ_BOOT_FLAGS: return "READ_BOOT_FLAGS";
        case PM_JOB_READ_PORT_CONFIG: return "READ_PORT_CONFIG";
        case PM_JOB_WRITE_PORT_CONFIG: return "WRITE_PORT_CONFIG";
        case PM_JOB_READ_PORT_CONTROL: return "READ_PORT_CONTROL";
        case PM_JOB_WRITE_PORT_CONTROL: return "WRITE_PORT_CONTROL";
        case PM_JOB_READ_INT_MASK: return "READ_INT_MASK";
        case PM_JOB_WRITE_INT_MASK: return "WRITE_INT_MASK";
        case PM_JOB_CLEAR_EVENT: return "CLEAR_EVENT";
        case PM_JOB_READ_STATUS: return "READ_STATUS";
        case PM_JOB_READ_TYPEC_STATE: return "READ_TYPEC_STATE";
        case PM_JOB_READ_POWER_PATH: return "READ_POWER_PATH";
        case PM_JOB_READ_PD_STATUS: return "READ_PD_STATUS";
        case PM_JOB_READ_ADC: return "READ_ADC";
        case PM_JOB_READ_ACTIVE_PDO: return "READ_ACTIVE_PDO";
        case PM_JOB_READ_ACTIVE_RDO: return "READ_ACTIVE_RDO";
        case PM_JOB_READ_EVENT: return "READ_EVENT";
        case PM_JOB_READ_PARTNER_CAPS: return "READ_PARTNER_CAPS";
        case PM_JOB_READ_LOCAL_CAPS: return "READ_LOCAL_CAPS";
        case PM_JOB_READ_SINK_CAPS: return "READ_SINK_CAPS";
        case PM_JOB_WRITE_SINK_CAPS: return "WRITE_SINK_CAPS";
        case PM_JOB_SWAP_TO_SOURCE: return "SWAP_TO_SOURCE";
        case PM_JOB_SWAP_TO_SINK: return "SWAP_TO_SINK";
        case PM_JOB_BQ_READ_ID: return "BQ_READ_ID";
        case PM_JOB_BQ_READ_OPTION0: return "BQ_READ_OPTION0";
        case PM_JOB_BQ_WRITE_OPTION0: return "BQ_WRITE_OPTION0";
        case PM_JOB_BQ_READ_OPTION4: return "BQ_READ_OPTION4";
        case PM_JOB_BQ_WRITE_OPTION4: return "BQ_WRITE_OPTION4";
        case PM_JOB_BQ_READ_OPTION1: return "BQ_READ_OPTION1";
        case PM_JOB_BQ_WRITE_OPTION1: return "BQ_WRITE_OPTION1";
        case PM_JOB_BQ_WRITE_ADC: return "BQ_WRITE_ADC";
        case PM_JOB_BQ_READ_STATUS_BLOCK: return "BQ_READ_STATUS";
        case PM_JOB_BQ_READ_ADC_BLOCK: return "BQ_READ_ADC";
        case PM_JOB_BQ_READ_CONFIG_BLOCK: return "BQ_READ_CONFIG";
        default: return "NONE";
    }
}

static void PowerManager_SetState(PowerManager_State_t state)
{
    if (g_pm.status.state != state) {
        g_pm.status.state = state;
        Debug_Printf("[PM] state=%s", PowerManager_StateToString(state));
    }
}

static bool PowerManager_PortConnected(void)
{
    return g_pm.status.tps.attached &&
           (g_pm.status.tps.connection_state >= 6U);
}

static bool PowerManager_ContractValid(void)
{
    return g_pm.status.tps.active_pdo.valid &&
           g_pm.status.tps.active_rdo.valid;
}

static void PowerManager_ClearPendingJobs(void)
{
    g_pm.port_config_read_pending = false;
    g_pm.port_config_write_pending = false;
    g_pm.port_control_read_pending = false;
    g_pm.port_control_write_pending = false;
    g_pm.int_mask_read_pending = false;
    g_pm.int_mask_write_pending = false;
    g_pm.sink_caps_read_pending = false;
    g_pm.sink_caps_write_pending = false;
    g_pm.local_caps_pending = false;
    g_pm.partner_caps_pending = false;
    g_pm.event_clear_pending = false;
    g_pm.swap_job_pending = false;
}

/* Everything the port needs once, re-armed slowly for as long as it has not
 * succeeded. This replaces per-request retry counters: a step that keeps
 * failing costs one transaction per period instead of blocking the queue. */
static void PowerManager_RearmSetup(uint32_t now_ms)
{
    if (!PowerManager_TickReached(now_ms, g_pm.setup_retry_ms)) {
        return;
    }
    g_pm.setup_retry_ms = now_ms + PM_SETUP_RETRY_MS;

    if (!g_pm.status.applied_mode_valid && !g_pm.port_config_write_pending) {
        g_pm.port_config_read_pending = true;
    }
    if (!g_pm.int_mask_ready && !g_pm.int_mask_write_pending) {
        g_pm.int_mask_read_pending = true;
    }
    if (!g_pm.sink_caps_checked && !g_pm.sink_caps_write_pending) {
        g_pm.sink_caps_read_pending = true;
    }
    if (!g_pm.swap_policy_applied && !g_pm.port_control_write_pending) {
        g_pm.port_control_read_pending = true;
    }
    if (!g_pm.local_caps_valid) {
        g_pm.local_caps_pending = true;
    }
}

static void PowerManager_RecordError(PowerManager_ErrorSource_t source,
                                     TPS25751_Status_t status,
                                     const char *what,
                                     uint32_t now_ms)
{
    uint32_t *counter = (source == POWER_MANAGER_ERROR_BQ) ?
                        &g_pm.bq_error_count : &g_pm.tps_error_count;

    ++(*counter);
    /* One shared backoff for every failed transaction. Without it a register
     * the device refuses to serve is retried on every main-loop pass, and the
     * resulting I2C storm is itself enough to disturb PD. */
    g_pm.retry_hold_ms = now_ms + PM_ERROR_BACKOFF_MS;
    g_pm.status.tps_status = status;
    g_pm.status.last_error.source = source;
    g_pm.status.last_error.code = (uint32_t)status;
    g_pm.status.last_error.reg = g_pm.tps.register_address;
    g_pm.status.last_error.tick_ms = now_ms;

    if ((*counter == 1U) ||
        PowerManager_TickReached(now_ms,
                                 g_pm.last_error_log_ms + PM_ERROR_LOG_MS)) {
        g_pm.last_error_log_ms = now_ms;
        Debug_Printf("[PM-ERR] %s job=%s status=%s reg=0x%02X task=%u len=%u/%u hal=0x%08lX count=%lu",
                     what,
                     PowerManager_JobToString(g_pm.last_job),
                     TPS25751_StatusToString(status),
                     g_pm.tps.register_address,
                     g_pm.tps.task_return_code,
                     g_pm.tps.reported_length,
                     g_pm.tps.requested_length,
                     (unsigned long)g_pm.tps.hal_error,
                     (unsigned long)(*counter));
    }
}

/* ------------------------------------------------------------------------
 * PD snapshot for the GUI
 * --------------------------------------------------------------------- */

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
    snapshot->power_role = (t->role == TPS25751_ROLE_SOURCE) ?
                           POWER_MANAGER_PD_ROLE_SOURCE :
                           ((t->role == TPS25751_ROLE_SINK) ?
                            POWER_MANAGER_PD_ROLE_SINK :
                            POWER_MANAGER_PD_ROLE_UNKNOWN);

    if (t->active_pdo.valid) {
        snapshot->contract_voltage_mv = t->active_pdo.voltage_mv;
        current_ma = t->active_pdo.current_ma;
    }
    if (t->active_rdo.valid && (t->active_rdo.requested_voltage_mv != 0U)) {
        snapshot->contract_voltage_mv = t->active_rdo.requested_voltage_mv;
    }
    if (t->active_rdo.valid && (t->active_rdo.operating_current_ma != 0U)) {
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

/* ------------------------------------------------------------------------
 * Role policy
 * --------------------------------------------------------------------- */

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

/* The automatic PR_Swap accept bits in PORT_CONTROL are the lock that keeps
 * the negotiated role stable. Once a role is chosen, the opposite direction
 * is refused, so a dual-role partner cannot drag the port back and forth. */
static void PowerManager_UpdateSwapPolicy(void)
{
    bool accept_source = true;
    bool accept_sink = true;

    switch (g_pm.status.requested_mode) {
        case POWER_MANAGER_USER_SINK_ONLY:
            accept_source = false;
            break;
        case POWER_MANAGER_USER_SOURCE_ONLY:
            accept_sink = false;
            break;
        case POWER_MANAGER_USER_OFF:
            accept_source = false;
            accept_sink = false;
            break;
        default:
            if (g_pm.role_target == PM_ROLE_WANT_SOURCE) {
                accept_sink = false;
            } else if (g_pm.role_target == PM_ROLE_WANT_SINK) {
                accept_source = false;
            }
            break;
    }

    if ((accept_source != g_pm.accept_swap_to_source) ||
        (accept_sink != g_pm.accept_swap_to_sink)) {
        g_pm.accept_swap_to_source = accept_source;
        g_pm.accept_swap_to_sink = accept_sink;
        g_pm.swap_policy_applied = false;
        g_pm.port_control_read_pending = true;
    }
}

static void PowerManager_ResetSession(uint32_t now_ms)
{
    g_pm.role_target = PM_ROLE_UNDECIDED;
    g_pm.swap_job_pending = false;
    g_pm.swap_budget_logged = false;
    g_pm.swap_attempts = 0U;
    g_pm.swap_next_ms = now_ms;
    g_pm.attach_ms = now_ms;
    g_pm.partner_caps_valid = false;
    g_pm.partner_caps_pending = false;
    g_pm.partner_caps_next_ms = now_ms;
    memset(&g_pm.partner_caps, 0, sizeof(g_pm.partner_caps));
    g_pm.status.source_fault_latched = false;
    PowerManager_UpdateSwapPolicy();
}

static void PowerManager_SetRoleTarget(PowerManager_RoleTarget_t target,
                                       const char *reason)
{
    g_pm.role_target = target;
    Debug_Printf("[PD] decision=%s reason=%s current=%s",
                 PowerManager_TargetToString(target), reason,
                 PowerManager_RoleToString(g_pm.status.tps.role));
    PowerManager_UpdateSwapPolicy();
}

/* Highest voltage the attached partner is able to supply, taken from its
 * Source_Capabilities when this attach produced them and otherwise from the
 * active contract. Zero means "nothing observed yet". */
static uint32_t PowerManager_PartnerSourceMaxMv(void)
{
    uint32_t max_mv = g_pm.partner_caps_valid ?
                      g_pm.partner_caps.max_voltage_mv : 0U;

    if ((g_pm.status.tps.role == TPS25751_ROLE_SINK) &&
        g_pm.status.tps.active_pdo.valid &&
        (g_pm.status.tps.active_pdo.max_voltage_mv > max_mv)) {
        max_mv = g_pm.status.tps.active_pdo.max_voltage_mv;
    }
    return max_mv;
}

static void PowerManager_DecideRole(uint32_t now_ms)
{
    uint32_t partner_mv;

    if ((g_pm.status.requested_mode != POWER_MANAGER_USER_AUTO) ||
        !g_pm.status.applied_mode_valid ||
        (g_pm.role_target != PM_ROLE_UNDECIDED) ||
        !PowerManager_PortConnected()) {
        return;
    }

    /* RX_SOURCE_CAPS is normally pulled in by the Source_Capabilities event,
     * but the event can be missed if it lands between two INT_EVENT reads.
     * Re-read it inside the decision window so the AUTO rule always sees the
     * partner's full offer and not just the 5 V PDO of the first contract. */
    if (!g_pm.partner_caps_valid && !g_pm.partner_caps_pending &&
        (g_pm.status.tps.role == TPS25751_ROLE_SINK) &&
        PowerManager_TickReached(now_ms, g_pm.partner_caps_next_ms)) {
        g_pm.partner_caps_next_ms = now_ms + PM_CAPS_RETRY_MS;
        g_pm.partner_caps_pending = true;
    }

    if (g_pm.status.tps.role == TPS25751_ROLE_SOURCE) {
        /* Attached.SRC means the partner presents Rd, which is direct
         * Type-C evidence that it consumes power. No swap is needed and
         * asking for its Sink_Capabilities would only add PD traffic. */
        PowerManager_SetRoleTarget(PM_ROLE_WANT_SOURCE,
                                   "partner presents Rd");
        return;
    }

    partner_mv = PowerManager_PartnerSourceMaxMv();
    if (partner_mv > PM_SOURCE_5V_MV) {
        PowerManager_SetRoleTarget(PM_ROLE_WANT_SINK, "partner offers >5V");
        return;
    }

    if ((partner_mv == 0U) && !PowerManager_ContractValid()) {
        /* Give the partner time to send Source_Capabilities before
         * concluding that it is a plain 5 V Type-C source. */
        if (!PowerManager_TickReached(now_ms,
                                      g_pm.attach_ms + PM_CAPS_GRACE_MS)) {
            return;
        }
        PowerManager_SetRoleTarget(PM_ROLE_WANT_SOURCE, "no PD source caps");
        return;
    }

    PowerManager_SetRoleTarget(PM_ROLE_WANT_SOURCE, "partner is 5V only");
}

static void PowerManager_MaintainRole(uint32_t now_ms)
{
    TPS25751_PowerRole_t wanted;

    if ((g_pm.role_target == PM_ROLE_UNDECIDED) ||
        !PowerManager_PortConnected() || g_pm.swap_job_pending) {
        return;
    }

    wanted = (g_pm.role_target == PM_ROLE_WANT_SOURCE) ?
             TPS25751_ROLE_SOURCE : TPS25751_ROLE_SINK;
    if (g_pm.status.tps.role == wanted) {
        return;
    }

    /* A PR_Swap is a request, not a command. Try a bounded number of times
     * and then keep whatever role the partner insists on: fighting a
     * dual-role partner forever is what produced the attach/detach loops. */
    if (g_pm.swap_attempts >= PM_SWAP_MAX_ATTEMPTS) {
        if (!g_pm.swap_budget_logged) {
            g_pm.swap_budget_logged = true;
            Debug_Printf("[PD] partner keeps role %s after %u swap requests; accepting it",
                         PowerManager_RoleToString(g_pm.status.tps.role),
                         g_pm.swap_attempts);
        }
        return;
    }
    /* Ask only once the refusal of the opposite direction is in place,
     * otherwise the partner can immediately swap us back. */
    if (!g_pm.swap_policy_applied ||
        !PowerManager_TickReached(now_ms, g_pm.swap_next_ms)) {
        return;
    }
    g_pm.swap_job_pending = true;
}

/* ------------------------------------------------------------------------
 * Logging
 * --------------------------------------------------------------------- */

static void PowerManager_LogCapabilities(const char *name,
                                         const TPS25751_Capabilities_t *caps)
{
    uint8_t i;

    Debug_Printf("[PD-CAPS] %s count=%u max=%lumV drp=%u", name, caps->count,
                 (unsigned long)caps->max_voltage_mv,
                 caps->first_pdo_dual_role_power ? 1U : 0U);
    for (i = 0U; i < caps->count; ++i) {
        Debug_Printf("[PD-CAPS] %s PDO%u=0x%08lX %lu-%lumV %lumA %lumW",
                     name, (unsigned int)(i + 1U),
                     (unsigned long)caps->pdo[i].raw,
                     (unsigned long)caps->pdo[i].min_voltage_mv,
                     (unsigned long)caps->pdo[i].max_voltage_mv,
                     (unsigned long)caps->pdo[i].current_ma,
                     (unsigned long)caps->pdo[i].power_mw);
    }
}

static void PowerManager_LogStatus(uint32_t now_ms)
{
    const TPS25751_Telemetry_t *tps = &g_pm.status.tps;
    const BQ25731_Telemetry_t *bq = &g_pm.status.bq;

    if (!PowerManager_TickReached(now_ms, g_pm.next_log_ms)) {
        return;
    }
    g_pm.next_log_ms = now_ms + PM_LOG_PERIOD_MS;

    Debug_Printf("[PM] %s mode=%s role=%s target=%s typec=%s conn=%u swaps=%u/%u",
                 PowerManager_StateToString(g_pm.status.state),
                 PowerManager_UserModeToString(g_pm.status.requested_mode),
                 PowerManager_RoleToString(tps->role),
                 PowerManager_TargetToString(g_pm.role_target),
                 PowerManager_TypecStateToString(tps->typec_port_state),
                 tps->connection_state,
                 g_pm.swap_attempts, PM_SWAP_MAX_ATTEMPTS);
    Debug_Printf("[PD] contract PDO%u %lumV %lumA %lumW vbus=%lumV pdo=0x%08lX rdo=0x%08lX",
                 tps->active_rdo.object_position,
                 (unsigned long)g_pm.status.pd_snapshot.contract_voltage_mv,
                 (unsigned long)g_pm.status.pd_snapshot.contract_current_ma,
                 (unsigned long)g_pm.status.pd_snapshot.contract_power_mw,
                 (unsigned long)tps->vbus_mv,
                 (unsigned long)tps->active_pdo_raw,
                 (unsigned long)tps->active_rdo_raw);
    Debug_Printf("[BQ] otg=%u chg=%u vin=%u vbus=%lumV iin=%lumA vbat=%lumV ibat=%ldmA faults=0x%02X",
                 bq->in_otg ? 1U : 0U,
                 bq->in_fast_charge ? 1U : 0U,
                 bq->input_present ? 1U : 0U,
                 (unsigned long)bq->adc_vbus_mv,
                 (unsigned long)bq->adc_iin_ma,
                 (unsigned long)bq->adc_vbat_mv,
                 (long)bq->battery_current_ma,
                 bq->fault_flags);
    Debug_Printf("[PM] events att=%lu det=%lu hr=%lu swap=%lu oc=%lu unable=%lu pwrerr=%lu nack=%lu err_tps=%lu err_bq=%lu",
                 (unsigned long)g_pm.attach_count,
                 (unsigned long)g_pm.detach_count,
                 (unsigned long)g_pm.hard_reset_count,
                 (unsigned long)g_pm.swap_count,
                 (unsigned long)g_pm.overcurrent_count,
                 (unsigned long)g_pm.unable_source_count,
                 (unsigned long)g_pm.power_error_count,
                 (unsigned long)g_pm.i2c_nack_count,
                 (unsigned long)g_pm.tps_error_count,
                 (unsigned long)g_pm.bq_error_count);
}

/* ------------------------------------------------------------------------
 * Event handling
 * --------------------------------------------------------------------- */

static void PowerManager_HandleEvent(const uint8_t *data, uint32_t now_ms)
{
    uint8_t fresh[TPS_INT_EVENT_BYTES];
    TPS_IntEvent_t event;
    uint8_t i;

    /* Act only on bits that were not already handled. A failed INT_CLEAR
     * would otherwise replay the same hard reset or attach on every poll and
     * keep restarting the role decision. */
    for (i = 0U; i < TPS_INT_EVENT_BYTES; ++i) {
        fresh[i] = (uint8_t)(data[i] & (uint8_t)~g_pm.event_seen[i]);
        g_pm.event_seen[i] = data[i];
    }

    TPS_IntEventDecode(&event, fresh);
    if (!event.any) {
        return;
    }

    memcpy(g_pm.event_to_clear, event.raw, sizeof(g_pm.event_to_clear));
    g_pm.event_clear_pending = true;

    if (event.hard_reset) {
        g_pm.hard_reset_count++;
    }
    if (event.power_swap_complete) {
        g_pm.swap_count++;
    }
    if (event.overcurrent) {
        g_pm.overcurrent_count++;
    }
    if (event.power_event_error) {
        g_pm.power_error_count++;
    }
    if (event.unable_to_source) {
        g_pm.unable_source_count++;
    }
    if (event.i2c_controller_nack) {
        g_pm.i2c_nack_count++;
    }
    if (event.overcurrent || event.power_event_error ||
        event.unable_to_source) {
        g_pm.status.source_fault_latched = true;
    }

    if (event.source_caps_received) {
        g_pm.partner_caps_pending = true;
    }
    if (event.hard_reset) {
        /* A hard reset restarts PD from scratch, so the previous role
         * decision is no longer backed by anything we observed. */
        PowerManager_ResetSession(now_ms);
    }

    if (event.plug_changed || event.hard_reset || event.source_caps_received ||
        event.new_contract_consumer || event.new_contract_provider ||
        event.power_swap_complete) {
        /* Every BQ access travels through the TPS 4CC tunnel and shares the
         * I2C controller the TPS uses to program the charger. Stay off that
         * bus while PD is negotiating. */
        g_pm.pd_quiet_until_ms = now_ms + PM_BQ_PD_QUIET_MS;
    }

    if (event.hard_reset || event.overcurrent || event.unable_to_source ||
        event.power_event_error || event.i2c_controller_nack) {
        Debug_Printf("[PD] warn hard_reset=%u oc=%u unable_source=%u power_error=%u i2c_nack=%u",
                     event.hard_reset ? 1U : 0U,
                     event.overcurrent ? 1U : 0U,
                     event.unable_to_source ? 1U : 0U,
                     event.power_event_error ? 1U : 0U,
                     event.i2c_controller_nack ? 1U : 0U);
    }
}

static void PowerManager_HandleStatus(const uint8_t *data,
                                      uint8_t length,
                                      uint32_t now_ms)
{
    bool was_attached = g_pm.status.tps.attached;
    TPS25751_PowerRole_t old_role = g_pm.status.tps.role;

    if (length < TPS25751_STATUS_LEN) {
        return;
    }
    TPS25751_DecodeStatus(&g_pm.status.tps, data);
    g_pm.status.tps.updated_ms = now_ms;
    PowerManager_UpdatePdSnapshot();

    if (g_pm.status.tps.attached != was_attached) {
        if (g_pm.status.tps.attached) {
            g_pm.attach_count++;
        } else {
            g_pm.detach_count++;
        }
        PowerManager_ResetSession(now_ms);
        Debug_Printf("[PD] %s", g_pm.status.tps.attached ? "attach" : "detach");
    } else if (g_pm.status.tps.attached &&
               (g_pm.status.tps.role != old_role)) {
        Debug_Printf("[PD] role %s -> %s",
                     PowerManager_RoleToString(old_role),
                     PowerManager_RoleToString(g_pm.status.tps.role));
    }
}

/* ------------------------------------------------------------------------
 * Job completion
 * --------------------------------------------------------------------- */

static uint16_t PowerManager_ResultLe16(const uint8_t *data,
                                        uint8_t length,
                                        bool *valid)
{
    *valid = (data != NULL) && (length >= 2U);
    return *valid ? TPS25751_ReadLe16(data) : 0U;
}

static void PowerManager_BqFailed(TPS25751_Status_t status, uint32_t now_ms)
{
    g_pm.status.bq_status = BQ25731_MapTpsStatus(status);
    g_pm.status.bq.online = false;
    PowerManager_RecordError(POWER_MANAGER_ERROR_BQ, status, "BQ", now_ms);

    g_pm.bq_write_pending = false;
    if (g_pm.bq_stage < PM_BQ_READY) {
        ++g_pm.bq_attempts;
        if (g_pm.bq_attempts >= PM_BQ_MAX_ATTEMPTS) {
            g_pm.bq_stage = PM_BQ_DISABLED;
            Debug_Printf("[BQ] startup configuration abandoned after %u attempts; telemetry only",
                         g_pm.bq_attempts);
        } else {
            g_pm.bq_stage = PM_BQ_ID;
        }
    }
    g_pm.bq_next_action_ms = now_ms + PM_BQ_TELEMETRY_MS;
    g_pm.bq_next_telemetry_ms = now_ms + PM_BQ_TELEMETRY_MS;
    g_pm.bq_next_config_ms = now_ms + PM_BQ_CONFIG_MS;
    PowerManager_SetState(POWER_MANAGER_DEGRADED);
}

static void PowerManager_ApplyPortMode(void)
{
    g_pm.status.applied_mode = g_pm.status.requested_mode;
    g_pm.status.applied_mode_valid = true;
    g_pm.port_config_read_pending = false;
    g_pm.port_config_write_pending = false;
}

static void PowerManager_ProcessJob(PowerManager_Job_t job,
                                    TPS25751_Status_t status,
                                    uint32_t now_ms)
{
    const uint8_t *data;
    uint8_t length = 0U;
    uint16_t value;
    uint8_t index;
    bool valid;
    bool dual_role;

    data = TPS25751_GetResult(&g_pm.tps, &length);

    if (status != TPS25751_OK) {
        if (job >= PM_JOB_BQ_READ_ID) {
            PowerManager_BqFailed(status, now_ms);
            return;
        }
        if ((job == PM_JOB_SWAP_TO_SOURCE) || (job == PM_JOB_SWAP_TO_SINK)) {
            /* Reject or Wait from the partner is a normal PD answer. */
            Debug_Printf("[PD] swap to %s refused (task=%u %s); attempt %u/%u",
                         (job == PM_JOB_SWAP_TO_SOURCE) ? "SOURCE" : "SINK",
                         g_pm.tps.task_return_code,
                         TPS25751_StatusToString(status),
                         g_pm.swap_attempts, PM_SWAP_MAX_ATTEMPTS);
            return;
        }
        /* Drop every one-shot request. None of them may hold the job queue:
         * telemetry has to keep running, and PowerManager_RearmSetup re-arms
         * whatever did not complete, at a rate that cannot flood the bus. */
        PowerManager_ClearPendingJobs();
        if (job == PM_JOB_READ_PARTNER_CAPS) {
            g_pm.partner_caps_valid = false;
        }
        PowerManager_RecordError(POWER_MANAGER_ERROR_TPS, status, "TPS",
                                 now_ms);
        if (g_pm.status.tps.mode != TPS25751_MODE_APP) {
            PowerManager_SetState(POWER_MANAGER_TPS_WAIT_APP);
        }
        return;
    }

    g_pm.status.tps_status = TPS25751_OK;

    switch (job) {
        case PM_JOB_READ_MODE:
            if (length < TPS25751_MODE_LEN) {
                break;
            }
            g_pm.status.tps.mode = TPS25751_DecodeMode(data);
            memcpy(g_pm.status.tps.mode_ascii, data, TPS25751_MODE_LEN);
            g_pm.status.tps.mode_ascii[TPS25751_MODE_LEN] = '\0';
            g_pm.next_mode_ms = now_ms + PM_MODE_POLL_MS;
            if (g_pm.status.tps.mode == TPS25751_MODE_APP) {
                if (g_pm.app_seen_ms == 0U) {
                    g_pm.app_seen_ms = now_ms;
                    g_pm.setup_retry_ms = now_ms;
                    g_pm.bq_next_action_ms = now_ms + PM_BQ_START_DELAY_MS;
                    PowerManager_SetState(POWER_MANAGER_TPS_READY);
                }
            } else if (g_pm.app_seen_ms != 0U) {
                g_pm.app_seen_ms = 0U;
                g_pm.status.applied_mode_valid = false;
                g_pm.local_caps_valid = false;
                g_pm.int_mask_ready = false;
                g_pm.sink_caps_checked = false;
                g_pm.swap_policy_applied = false;
                memset(g_pm.event_seen, 0, sizeof(g_pm.event_seen));
                PowerManager_SetState(POWER_MANAGER_TPS_WAIT_APP);
            }
            break;

        case PM_JOB_READ_BOOT_FLAGS:
            if (length < TPS25751_BOOT_FLAGS_LEN) {
                break;
            }
            g_pm.status.tps.boot_flags_raw =
                (uint64_t)TPS25751_ReadLe32(data) | ((uint64_t)data[4] << 32);
            g_pm.next_boot_flags_ms = now_ms + PM_BOOT_FLAGS_POLL_MS;
            Debug_Printf("[TPS] stuck in PTCH: boot_flags=0x%02lX%08lX cfg_src=%lu eeprom=%lu crc_err=%lu/%lu io_err=%lu/%lu header_err=%lu",
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 32) & 0xFFU),
                         (unsigned long)g_pm.status.tps.boot_flags_raw,
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 29) & 0x07U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 3) & 0x01U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 12) & 0x01U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 13) & 0x01U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 8) & 0x01U),
                         (unsigned long)((g_pm.status.tps.boot_flags_raw >> 9) & 0x01U),
                         (unsigned long)(g_pm.status.tps.boot_flags_raw & 0x01U));
            break;

        case PM_JOB_READ_PORT_CONFIG:
            if (length < TPS25751_PORT_CONFIG_LEN) {
                break;
            }
            memcpy(g_pm.port_config, data, sizeof(g_pm.port_config));
            if (TPS25751_PatchPortMode(
                    g_pm.port_config,
                    PowerManager_MapPortMode(g_pm.status.requested_mode))) {
                g_pm.port_config_write_pending = true;
                Debug_Printf("[PD] PORT_CONFIG state machine -> %s (forces a port reconnect)",
                             PowerManager_UserModeToString(
                                 g_pm.status.requested_mode));
            } else {
                PowerManager_ApplyPortMode();
            }
            break;

        case PM_JOB_WRITE_PORT_CONFIG:
            PowerManager_ApplyPortMode();
            PowerManager_ResetSession(now_ms);
            break;

        case PM_JOB_READ_PORT_CONTROL:
            if (length < 4U) {
                g_pm.port_control_read_pending = false;
                break;
            }
            g_pm.port_control_length =
                (length > TPS25751_PORT_CONTROL_LEN) ?
                TPS25751_PORT_CONTROL_LEN : length;
            memcpy(g_pm.port_control, data, g_pm.port_control_length);
            g_pm.port_control_read_pending = false;
            g_pm.port_control_write_pending = TPS25751_PatchSwapPolicy(
                g_pm.port_control, g_pm.accept_swap_to_source,
                g_pm.accept_swap_to_sink);
            g_pm.swap_policy_applied = !g_pm.port_control_write_pending;
            break;

        case PM_JOB_WRITE_PORT_CONTROL:
            g_pm.port_control_write_pending = false;
            g_pm.swap_policy_applied = true;
            Debug_Printf("[PD] PR_Swap accept: to_source=%u to_sink=%u",
                         g_pm.accept_swap_to_source ? 1U : 0U,
                         g_pm.accept_swap_to_sink ? 1U : 0U);
            break;

        case PM_JOB_READ_INT_MASK:
            if (length < TPS_INT_EVENT_BYTES) {
                g_pm.int_mask_read_pending = false;
                break;
            }
            memcpy(g_pm.int_mask, data, sizeof(g_pm.int_mask));
            TPS_IntEventEnableRequiredBits(g_pm.int_mask);
            g_pm.int_mask_read_pending = false;
            g_pm.int_mask_write_pending = true;
            break;

        case PM_JOB_WRITE_INT_MASK:
            g_pm.int_mask_write_pending = false;
            g_pm.int_mask_ready = true;
            break;

        case PM_JOB_CLEAR_EVENT:
            g_pm.event_clear_pending = false;
            for (index = 0U; index < TPS_INT_EVENT_BYTES; ++index) {
                g_pm.event_seen[index] &=
                    (uint8_t)~g_pm.event_to_clear[index];
            }
            memset(g_pm.event_to_clear, 0, sizeof(g_pm.event_to_clear));
            break;

        case PM_JOB_READ_STATUS:
            PowerManager_HandleStatus(data, length, now_ms);
            break;

        case PM_JOB_READ_TYPEC_STATE:
            if (length >= TPS25751_TYPE_C_STATE_LEN) {
                TPS25751_DecodeTypecState(&g_pm.status.tps, data);
            }
            break;

        case PM_JOB_READ_POWER_PATH:
            if (length >= TPS25751_POWER_PATH_LEN) {
                TPS25751_DecodePowerPath(&g_pm.status.tps, data);
                if (g_pm.status.tps.pp5v_overcurrent ||
                    g_pm.status.tps.ppcable_overcurrent) {
                    g_pm.status.source_fault_latched = true;
                }
            }
            break;

        case PM_JOB_READ_PD_STATUS:
            if (length >= TPS25751_PD_STATUS_LEN) {
                TPS25751_DecodePdStatus(&g_pm.status.tps, data);
            }
            break;

        case PM_JOB_READ_ADC:
            if (length >= TPS25751_ADC_RESULTS_LEN) {
                TPS25751_DecodeAdcResults(&g_pm.status.tps, data);
            }
            break;

        case PM_JOB_READ_ACTIVE_PDO:
            if (length >= 4U) {
                g_pm.status.tps.active_pdo_raw = TPS25751_ReadLe32(data);
                g_pm.status.tps.active_pdo =
                    TPS25751_DecodePdo(g_pm.status.tps.active_pdo_raw);
                PowerManager_UpdatePdSnapshot();
            }
            break;

        case PM_JOB_READ_ACTIVE_RDO:
            if (length >= 4U) {
                g_pm.status.tps.active_rdo_raw = TPS25751_ReadLe32(data);
                g_pm.status.tps.active_rdo = TPS25751_DecodeRdo(
                    g_pm.status.tps.active_rdo_raw,
                    &g_pm.status.tps.active_pdo);
                PowerManager_UpdatePdSnapshot();
            }
            break;

        case PM_JOB_READ_EVENT:
            PowerManager_HandleEvent(data, now_ms);
            break;

        case PM_JOB_READ_PARTNER_CAPS:
            g_pm.partner_caps_pending = false;
            g_pm.partner_caps_valid =
                TPS25751_DecodeCapabilities(&g_pm.partner_caps, data, length);
            if (g_pm.partner_caps_valid) {
                PowerManager_LogCapabilities("PARTNER_SOURCE",
                                             &g_pm.partner_caps);
            } else {
                memset(&g_pm.partner_caps, 0, sizeof(g_pm.partner_caps));
            }
            break;

        case PM_JOB_READ_LOCAL_CAPS:
            g_pm.local_caps_pending = false;
            g_pm.local_caps_valid = TPS25751_DecodeTxSourceCapabilities(
                &g_pm.local_caps, data, length);
            if (g_pm.local_caps_valid) {
                PowerManager_LogCapabilities("OUR_SOURCE", &g_pm.local_caps);
            }
            break;

        case PM_JOB_READ_SINK_CAPS:
            g_pm.sink_caps_read_pending = false;
            if (length < 5U) {
                g_pm.sink_caps_checked = true;
                break;
            }
            g_pm.sink_caps_length = (length > sizeof(g_pm.sink_caps)) ?
                                    (uint8_t)sizeof(g_pm.sink_caps) : length;
            memcpy(g_pm.sink_caps, data, g_pm.sink_caps_length);
            dual_role = TPS25751_CapsFirstPdoDualRole(g_pm.sink_caps,
                                                      g_pm.sink_caps_length,
                                                      1U);
            g_pm.sink_caps_write_pending = TPS25751_PatchSinkCapsDualRole(
                g_pm.sink_caps, g_pm.sink_caps_length);
            g_pm.sink_caps_checked = !g_pm.sink_caps_write_pending;
            Debug_Printf("[PD-CAPS] OUR_SINK count=%u PDO1=0x%08lX drp=%u%s",
                         g_pm.sink_caps[0] & 0x07U,
                         (unsigned long)TPS25751_ReadLe32(&g_pm.sink_caps[1]),
                         dual_role ? 1U : 0U,
                         g_pm.sink_caps_write_pending ?
                         " (EEPROM advertises sink-only; correcting)" : "");
            break;

        case PM_JOB_WRITE_SINK_CAPS:
            g_pm.sink_caps_write_pending = false;
            g_pm.sink_caps_checked = true;
            Debug_Printf("[PD-CAPS] OUR_SINK DualRolePower set; partners may now offer PR_Swap");
            break;

        case PM_JOB_SWAP_TO_SOURCE:
        case PM_JOB_SWAP_TO_SINK:
            Debug_Printf("[PD] swap to %s accepted",
                         (job == PM_JOB_SWAP_TO_SOURCE) ? "SOURCE" : "SINK");
            break;

        case PM_JOB_BQ_READ_ID:
            if (length < 2U) {
                PowerManager_BqFailed(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.status.bq.manufacturer_id = data[0];
            g_pm.status.bq.device_id = data[1];
            g_pm.status.bq.id_valid = (data[0] == 0x40U) && (data[1] == 0xD6U);
            g_pm.status.bq.online = true;
            g_pm.status.bq_status = g_pm.status.bq.id_valid ?
                                    BQ25731_OK : BQ25731_DEVICE_ID_MISMATCH;
            g_pm.bq_stage = PM_BQ_OPTION0;
            g_pm.bq_next_action_ms = now_ms + PM_BQ_STEP_MS;
            PowerManager_SetState(POWER_MANAGER_BQ_ADC_SETUP);
            break;

        case PM_JOB_BQ_READ_OPTION0:
            value = PowerManager_ResultLe16(data, length, &valid);
            if (!valid) {
                PowerManager_BqFailed(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.status.bq.charge_option0 = value;
            g_pm.bq_option_target = BQ25731_BuildStartupOption0(value);
            g_pm.bq_write_pending = (g_pm.bq_option_target != value);
            if (!g_pm.bq_write_pending) {
                g_pm.bq_stage = PM_BQ_OPTION4;
            }
            g_pm.bq_next_action_ms = now_ms + PM_BQ_STEP_MS;
            break;

        case PM_JOB_BQ_WRITE_OPTION0:
            g_pm.bq_write_pending = false;
            g_pm.bq_stage = PM_BQ_OPTION4;
            g_pm.bq_next_action_ms = now_ms + PM_BQ_STEP_MS;
            break;

        case PM_JOB_BQ_READ_OPTION4:
            value = PowerManager_ResultLe16(data, length, &valid);
            if (!valid) {
                PowerManager_BqFailed(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.status.bq.charge_option4 = value;
            g_pm.bq_option_target = BQ25731_BuildStartupOption4(value);
            g_pm.bq_write_pending = (g_pm.bq_option_target != value);
            if (!g_pm.bq_write_pending) {
                g_pm.bq_stage = PM_BQ_OPTION1;
            }
            g_pm.bq_next_action_ms = now_ms + PM_BQ_STEP_MS;
            break;

        case PM_JOB_BQ_WRITE_OPTION4:
            g_pm.bq_write_pending = false;
            g_pm.bq_stage = PM_BQ_OPTION1;
            g_pm.bq_next_action_ms = now_ms + PM_BQ_STEP_MS;
            break;

        case PM_JOB_BQ_READ_OPTION1:
            value = PowerManager_ResultLe16(data, length, &valid);
            if (!valid) {
                PowerManager_BqFailed(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.bq_option_target = BQ25731_BuildStartupOption1(value);
            g_pm.bq_write_pending = (g_pm.bq_option_target != value);
            if (!g_pm.bq_write_pending) {
                g_pm.bq_stage = PM_BQ_ADC;
            }
            g_pm.bq_next_action_ms = now_ms + PM_BQ_STEP_MS;
            break;

        case PM_JOB_BQ_WRITE_OPTION1:
            g_pm.bq_write_pending = false;
            g_pm.bq_stage = PM_BQ_ADC;
            g_pm.bq_next_action_ms = now_ms + PM_BQ_STEP_MS;
            break;

        case PM_JOB_BQ_WRITE_ADC:
            g_pm.status.bq.adc_configured = true;
            g_pm.bq_stage = PM_BQ_READY;
            g_pm.bq_next_telemetry_ms = now_ms;
            g_pm.bq_next_config_ms = now_ms;
            PowerManager_SetState(POWER_MANAGER_RUN);
            Debug_Printf("[BQ] startup done: OOA/PWM=0x%04X dither=0x%04X ADC monitoring on; TPS owns charge and OTG control",
                         g_pm.status.bq.charge_option0,
                         g_pm.status.bq.charge_option4);
            break;

        case PM_JOB_BQ_READ_STATUS_BLOCK:
            if (!BQ25731_DecodeStatusBlock(&g_pm.status.bq, data, length)) {
                PowerManager_BqFailed(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.status.bq_status = BQ25731_OK;
            g_pm.bq_telemetry_phase = 1U;
            g_pm.bq_next_telemetry_ms = now_ms;
            break;

        case PM_JOB_BQ_READ_ADC_BLOCK:
            if (!BQ25731_DecodeAdcBlock(&g_pm.status.bq, data, length)) {
                PowerManager_BqFailed(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.status.bq.updated_ms = now_ms;
            g_pm.bq_telemetry_phase = 0U;
            g_pm.bq_next_telemetry_ms = now_ms + PM_BQ_TELEMETRY_MS;
            if ((g_pm.status.state == POWER_MANAGER_DEGRADED) &&
                (g_pm.bq_stage >= PM_BQ_READY)) {
                PowerManager_SetState(POWER_MANAGER_RUN);
            }
            break;

        case PM_JOB_BQ_READ_CONFIG_BLOCK:
            if (!BQ25731_DecodeConfigBlock(&g_pm.status.bq, data, length)) {
                PowerManager_BqFailed(TPS25751_BAD_LENGTH, now_ms);
                break;
            }
            g_pm.bq_next_config_ms = now_ms + PM_BQ_CONFIG_MS;
            break;

        default:
            break;
    }
}

/* ------------------------------------------------------------------------
 * Job dispatch
 * --------------------------------------------------------------------- */

static TPS25751_Status_t PowerManager_MapBq(BQ25731_Status_t status)
{
    return (status == BQ25731_OK) ? TPS25751_OK : TPS25751_BUSY;
}

static TPS25751_Status_t PowerManager_StartJob(PowerManager_Job_t job)
{
    switch (job) {
        case PM_JOB_READ_MODE:
            return TPS25751_StartReadRegister(&g_pm.tps, TPS25751_REG_MODE,
                                              TPS25751_MODE_LEN);
        case PM_JOB_READ_BOOT_FLAGS:
            return TPS25751_StartReadRegister(&g_pm.tps,
                                              TPS25751_REG_BOOT_FLAGS,
                                              TPS25751_BOOT_FLAGS_LEN);
        case PM_JOB_READ_PORT_CONFIG:
            return TPS25751_StartReadRegister(&g_pm.tps,
                                              TPS25751_REG_PORT_CONFIG,
                                              TPS25751_PORT_CONFIG_LEN);
        case PM_JOB_WRITE_PORT_CONFIG:
            return TPS25751_StartWriteRegister(&g_pm.tps,
                                               TPS25751_REG_PORT_CONFIG,
                                               g_pm.port_config,
                                               sizeof(g_pm.port_config));
        case PM_JOB_READ_PORT_CONTROL:
            return TPS25751_StartReadRegister(&g_pm.tps,
                                              TPS25751_REG_PORT_CONTROL,
                                              TPS25751_PORT_CONTROL_LEN);
        case PM_JOB_WRITE_PORT_CONTROL:
            return TPS25751_StartWriteRegister(&g_pm.tps,
                                               TPS25751_REG_PORT_CONTROL,
                                               g_pm.port_control,
                                               g_pm.port_control_length);
        case PM_JOB_READ_INT_MASK:
            return TPS25751_StartReadRegister(&g_pm.tps,
                                              TPS25751_REG_INT_MASK,
                                              TPS_INT_EVENT_BYTES);
        case PM_JOB_WRITE_INT_MASK:
            return TPS25751_StartWriteRegister(&g_pm.tps,
                                               TPS25751_REG_INT_MASK,
                                               g_pm.int_mask,
                                               sizeof(g_pm.int_mask));
        case PM_JOB_CLEAR_EVENT:
            return TPS_IntEventStartClear(&g_pm.tps, g_pm.event_to_clear);
        case PM_JOB_READ_STATUS:
            return TPS25751_StartReadRegister(&g_pm.tps, TPS25751_REG_STATUS,
                                              TPS25751_STATUS_LEN);
        case PM_JOB_READ_TYPEC_STATE:
            return TPS25751_StartReadRegister(&g_pm.tps,
                                              TPS25751_REG_TYPE_C_STATE,
                                              TPS25751_TYPE_C_STATE_LEN);
        case PM_JOB_READ_POWER_PATH:
            return TPS25751_StartReadRegister(&g_pm.tps,
                                              TPS25751_REG_POWER_PATH_STATUS,
                                              TPS25751_POWER_PATH_LEN);
        case PM_JOB_READ_PD_STATUS:
            return TPS25751_StartReadRegister(&g_pm.tps,
                                              TPS25751_REG_PD_STATUS,
                                              TPS25751_PD_STATUS_LEN);
        case PM_JOB_READ_ADC:
            return TPS25751_StartReadRegister(&g_pm.tps,
                                              TPS25751_REG_ADC_RESULTS,
                                              TPS25751_ADC_RESULTS_LEN);
        case PM_JOB_READ_ACTIVE_PDO:
            return TPS25751_StartReadRegister(&g_pm.tps,
                                              TPS25751_REG_ACTIVE_PDO,
                                              TPS25751_ACTIVE_PDO_PREFIX_LEN);
        case PM_JOB_READ_ACTIVE_RDO:
            return TPS25751_StartReadRegister(&g_pm.tps,
                                              TPS25751_REG_ACTIVE_RDO,
                                              TPS25751_ACTIVE_RDO_PREFIX_LEN);
        case PM_JOB_READ_EVENT:
            return TPS_IntEventStartRead(&g_pm.tps);
        case PM_JOB_READ_PARTNER_CAPS:
            return TPS25751_StartReadRegister(&g_pm.tps,
                                              TPS25751_REG_RX_SOURCE_CAPS,
                                              TPS25751_RX_CAPS_LEN);
        case PM_JOB_READ_LOCAL_CAPS:
            return TPS25751_StartReadRegister(&g_pm.tps,
                                              TPS25751_REG_TX_SOURCE_CAPS,
                                              TPS25751_TX_SOURCE_CAPS_LEN);
        case PM_JOB_READ_SINK_CAPS:
            return TPS25751_StartReadRegister(&g_pm.tps,
                                              TPS25751_REG_TX_SINK_CAPS,
                                              TPS25751_TX_SINK_CAPS_LEN);
        case PM_JOB_WRITE_SINK_CAPS:
            return TPS25751_StartWriteRegister(&g_pm.tps,
                                               TPS25751_REG_TX_SINK_CAPS,
                                               g_pm.sink_caps,
                                               g_pm.sink_caps_length);
        case PM_JOB_SWAP_TO_SOURCE:
            return TPS25751_StartCommand(&g_pm.tps, "SWSr", NULL, 0U, 1U);
        case PM_JOB_SWAP_TO_SINK:
            return TPS25751_StartCommand(&g_pm.tps, "SWSk", NULL, 0U, 1U);
        case PM_JOB_BQ_READ_ID:
            return PowerManager_MapBq(BQ25731_StartReadId(&g_pm.bq));
        case PM_JOB_BQ_READ_OPTION0:
            return PowerManager_MapBq(BQ25731_StartRead16(
                &g_pm.bq, BQ25731_REG_CHARGE_OPTION0));
        case PM_JOB_BQ_WRITE_OPTION0:
            return PowerManager_MapBq(BQ25731_StartWriteStartupOption0(
                &g_pm.bq, g_pm.bq_option_target));
        case PM_JOB_BQ_READ_OPTION4:
            return PowerManager_MapBq(BQ25731_StartRead16(
                &g_pm.bq, BQ25731_REG_CHARGE_OPTION4));
        case PM_JOB_BQ_WRITE_OPTION4:
            return PowerManager_MapBq(BQ25731_StartWriteStartupOption4(
                &g_pm.bq, g_pm.bq_option_target));
        case PM_JOB_BQ_READ_OPTION1:
            return PowerManager_MapBq(BQ25731_StartRead16(
                &g_pm.bq, BQ25731_REG_CHARGE_OPTION1));
        case PM_JOB_BQ_WRITE_OPTION1:
            return PowerManager_MapBq(BQ25731_StartWriteStartupOption1(
                &g_pm.bq, g_pm.bq_option_target));
        case PM_JOB_BQ_WRITE_ADC:
            return PowerManager_MapBq(
                BQ25731_StartConfigureMonitoringAdc(&g_pm.bq));
        case PM_JOB_BQ_READ_STATUS_BLOCK:
            return PowerManager_MapBq(BQ25731_StartReadStatusBlock(&g_pm.bq));
        case PM_JOB_BQ_READ_ADC_BLOCK:
            return PowerManager_MapBq(BQ25731_StartReadAdcBlock(&g_pm.bq));
        case PM_JOB_BQ_READ_CONFIG_BLOCK:
            return PowerManager_MapBq(BQ25731_StartReadConfigBlock(&g_pm.bq));
        default:
            return TPS25751_INVALID_ARG;
    }
}

static PowerManager_Job_t PowerManager_SelectTelemetryJob(void)
{
    static const PowerManager_Job_t sequence[8] = {
        PM_JOB_READ_EVENT,
        PM_JOB_READ_STATUS,
        PM_JOB_READ_ACTIVE_PDO,
        PM_JOB_READ_ACTIVE_RDO,
        PM_JOB_READ_TYPEC_STATE,
        PM_JOB_READ_POWER_PATH,
        PM_JOB_READ_PD_STATUS,
        PM_JOB_READ_ADC
    };
    PowerManager_Job_t job = sequence[g_pm.telemetry_phase & 0x07U];

    g_pm.telemetry_phase = (uint8_t)((g_pm.telemetry_phase + 1U) & 0x07U);
    return job;
}

static PowerManager_Job_t PowerManager_SelectBqJob(uint32_t now_ms)
{
    if ((PM_ENABLE_BQ_ACCESS == 0U) || (g_pm.app_seen_ms == 0U) ||
        !PowerManager_TickReached(now_ms, g_pm.pd_quiet_until_ms)) {
        return PM_JOB_NONE;
    }

    if (g_pm.bq_stage < PM_BQ_READY) {
        /* The startup writes are read-modify-write on registers the TPS
         * also drives. Only touch them while nothing is attached, so the
         * two writers can never overlap on a live contract. */
        if (g_pm.status.tps.attached ||
            !PowerManager_TickReached(now_ms, g_pm.bq_next_action_ms)) {
            return PM_JOB_NONE;
        }
        if (g_pm.bq_stage == PM_BQ_WAIT) {
            g_pm.bq_stage = PM_BQ_ID;
            PowerManager_SetState(POWER_MANAGER_BQ_PROBE);
        }
        switch (g_pm.bq_stage) {
            case PM_BQ_ID: return PM_JOB_BQ_READ_ID;
            case PM_BQ_OPTION0:
                return g_pm.bq_write_pending ? PM_JOB_BQ_WRITE_OPTION0 :
                                               PM_JOB_BQ_READ_OPTION0;
            case PM_BQ_OPTION4:
                return g_pm.bq_write_pending ? PM_JOB_BQ_WRITE_OPTION4 :
                                               PM_JOB_BQ_READ_OPTION4;
            case PM_BQ_OPTION1:
                return g_pm.bq_write_pending ? PM_JOB_BQ_WRITE_OPTION1 :
                                               PM_JOB_BQ_READ_OPTION1;
            case PM_BQ_ADC: return PM_JOB_BQ_WRITE_ADC;
            default: return PM_JOB_NONE;
        }
    }

    if (PowerManager_TickReached(now_ms, g_pm.bq_next_telemetry_ms)) {
        return (g_pm.bq_telemetry_phase == 0U) ? PM_JOB_BQ_READ_STATUS_BLOCK :
                                                 PM_JOB_BQ_READ_ADC_BLOCK;
    }
    if (PowerManager_TickReached(now_ms, g_pm.bq_next_config_ms)) {
        return PM_JOB_BQ_READ_CONFIG_BLOCK;
    }
    return PM_JOB_NONE;
}

static PowerManager_Job_t PowerManager_SelectJob(uint32_t now_ms)
{
    if (!PowerManager_TickReached(now_ms, g_pm.retry_hold_ms)) {
        return PM_JOB_NONE;
    }

    if (g_pm.status.tps.mode != TPS25751_MODE_APP) {
        if ((g_pm.status.tps.mode == TPS25751_MODE_PTCH) &&
            PowerManager_TickReached(now_ms, g_pm.next_boot_flags_ms)) {
            return PM_JOB_READ_BOOT_FLAGS;
        }
        return PowerManager_TickReached(now_ms, g_pm.next_mode_ms) ?
               PM_JOB_READ_MODE : PM_JOB_NONE;
    }

    PowerManager_RearmSetup(now_ms);

    /* Port configuration first: nothing else is meaningful until the port
     * runs in the role the user asked for. */
    if (g_pm.port_config_write_pending) {
        return PM_JOB_WRITE_PORT_CONFIG;
    }
    if (g_pm.port_config_read_pending) {
        return PM_JOB_READ_PORT_CONFIG;
    }
    if (g_pm.int_mask_write_pending) {
        return PM_JOB_WRITE_INT_MASK;
    }
    if (g_pm.int_mask_read_pending) {
        return PM_JOB_READ_INT_MASK;
    }
    if (g_pm.event_clear_pending) {
        return PM_JOB_CLEAR_EVENT;
    }
    if (g_pm.port_control_write_pending) {
        return PM_JOB_WRITE_PORT_CONTROL;
    }
    if (g_pm.port_control_read_pending) {
        return PM_JOB_READ_PORT_CONTROL;
    }
    if (g_pm.swap_job_pending) {
        return (g_pm.role_target == PM_ROLE_WANT_SOURCE) ?
               PM_JOB_SWAP_TO_SOURCE : PM_JOB_SWAP_TO_SINK;
    }
    if (g_pm.partner_caps_pending) {
        return PM_JOB_READ_PARTNER_CAPS;
    }
    if (g_pm.local_caps_pending) {
        return PM_JOB_READ_LOCAL_CAPS;
    }
    if (g_pm.sink_caps_write_pending) {
        return PM_JOB_WRITE_SINK_CAPS;
    }
    if (g_pm.sink_caps_read_pending) {
        return PM_JOB_READ_SINK_CAPS;
    }
    if (PowerManager_TickReached(now_ms, g_pm.next_mode_ms)) {
        return PM_JOB_READ_MODE;
    }
    if (PowerManager_TickReached(now_ms, g_pm.next_telemetry_ms)) {
        g_pm.next_telemetry_ms = now_ms + PM_TELEMETRY_STEP_MS;
        return PowerManager_SelectTelemetryJob();
    }
    return PowerManager_SelectBqJob(now_ms);
}

/* ------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

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
    g_pm.port_control_length = TPS25751_PORT_CONTROL_LEN;
    g_pm.accept_swap_to_source = true;
    g_pm.accept_swap_to_sink = true;
    g_pm.next_mode_ms = now_ms;
    g_pm.next_boot_flags_ms = now_ms;
    g_pm.next_telemetry_ms = now_ms;
    g_pm.next_log_ms = now_ms + PM_LOG_PERIOD_MS;
    g_pm.bq_next_action_ms = now_ms + PM_BQ_START_DELAY_MS;
    g_pm.attach_ms = now_ms;

    /* The BQ25731 OTG/VAP/FRS input is strapped high on this board, matching
     * the TI reference wiring, and the TPS25751 gates the actual boost with
     * the EN_OTG register bit. Drive the pin high once and never toggle it:
     * a low level here would fight the hardware strap. */
    HAL_GPIO_WritePin(OTG_EN_GPIO_Port, OTG_EN_Pin, GPIO_PIN_SET);
    g_pm.status.otg_pin_high = true;

    if ((TPS25751_Init(&g_pm.tps, hi2c,
                       TPS25751_I2C_ADDR_DEFAULT) != TPS25751_OK) ||
        (BQ25731_Init(&g_pm.bq, &g_pm.tps,
                      BQ25731_I2C_ADDR_7BIT) != BQ25731_OK)) {
        PowerManager_SetState(POWER_MANAGER_FAULT);
        return;
    }

    g_pm.initialized = true;
    Debug_Printf("[PM] init TPS=0x%02X BQ=0x%02X mode=AUTO rule: partner>5V -> sink, else source",
                 TPS25751_I2C_ADDR_DEFAULT, BQ25731_I2C_ADDR_7BIT);
    PowerManager_SetState(POWER_MANAGER_TPS_WAIT_APP);
}

void PowerManager_Task(void)
{
    uint32_t now_ms;
    TPS25751_Status_t status;
    PowerManager_Job_t next_job;

    if (!g_pm.initialized) {
        return;
    }
    now_ms = HAL_GetTick();

    if (g_pm.job != PM_JOB_NONE) {
        status = TPS25751_Task(&g_pm.tps, now_ms);
        if (status != TPS25751_BUSY) {
            PowerManager_Job_t done = g_pm.job;

            g_pm.job = PM_JOB_NONE;
            g_pm.last_job = done;
            PowerManager_ProcessJob(done, status, now_ms);
        }
        return;
    }

    PowerManager_DecideRole(now_ms);
    PowerManager_MaintainRole(now_ms);
    PowerManager_LogStatus(now_ms);

    next_job = PowerManager_SelectJob(now_ms);
    if (next_job == PM_JOB_NONE) {
        return;
    }
    if (PowerManager_StartJob(next_job) != TPS25751_OK) {
        return;
    }
    g_pm.job = next_job;

    if ((next_job == PM_JOB_SWAP_TO_SOURCE) ||
        (next_job == PM_JOB_SWAP_TO_SINK)) {
        g_pm.swap_job_pending = false;
        ++g_pm.swap_attempts;
        g_pm.swap_next_ms = now_ms + PM_SWAP_BACKOFF_MS;
        Debug_Printf("[PD] requesting PR_Swap to %s (attempt %u/%u)",
                     PowerManager_TargetToString(g_pm.role_target),
                     g_pm.swap_attempts, PM_SWAP_MAX_ATTEMPTS);
    }

    status = TPS25751_Task(&g_pm.tps, now_ms);
    if (status != TPS25751_BUSY) {
        g_pm.job = PM_JOB_NONE;
        g_pm.last_job = next_job;
        PowerManager_ProcessJob(next_job, status, now_ms);
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
        g_pm.status.applied_mode_valid = false;
        g_pm.setup_retry_ms = HAL_GetTick();
        PowerManager_ResetSession(HAL_GetTick());
        Debug_Printf("[PM] user mode -> %s",
                     PowerManager_UserModeToString(mode));
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
