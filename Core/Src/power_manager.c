#include "power_manager.h"

#include "app.h"
#include "debug_uart.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define POWER_MANAGER_TASK_MIN_PERIOD_MS   20U
#define POWER_MANAGER_TPS_POLL_MS          200U
#define POWER_MANAGER_TPS_APP_TIMEOUT_MS   5000U
#define POWER_MANAGER_BQ_PROBE_RETRY_MS    1000U
#define POWER_MANAGER_MONITOR_PERIOD_MS    120U
#define POWER_MANAGER_DEBUG_PERIOD_MS      1000U
#define POWER_MANAGER_PD_CAPS_DEBUG_MS     10000U
#define POWER_MANAGER_TPS_ADDR_MIN         0x20U
#define POWER_MANAGER_TPS_ADDR_MAX         0x27U
#define POWER_MANAGER_TPS_PROBE_REG_MODE   0x03U
#define POWER_MANAGER_PD_LINE_BUF_SIZE     384U
#define POWER_MANAGER_PD_TEST_STEP_MS      5000U
#define POWER_MANAGER_PD_TEST_TIMEOUT_MS   3000U
#define POWER_MANAGER_BQ_ADC_START_DELAY_MS 6000U
#define POWER_MANAGER_BQ_ADC_RETRY_MS       6000U
#define POWER_MANAGER_BQ_ADC_MAX_ATTEMPTS   5U

#define POWER_MANAGER_FAULT_TPS_NOT_AT_CONFIGURED_ADDRESS  0x1001U
#define POWER_MANAGER_FAULT_TPS_APP_TIMEOUT                0x1002U
#define POWER_MANAGER_FAULT_TPS_LOST_APP                   0x1003U

#ifndef POWER_MANAGER_DIAGNOSTIC_TPS_SCAN
#define POWER_MANAGER_DIAGNOSTIC_TPS_SCAN   0U
#endif

#ifndef POWER_MANAGER_PD_RAW_DEBUG
#define POWER_MANAGER_PD_RAW_DEBUG         0U
#endif

#ifndef POWER_MANAGER_GENERAL_DEBUG
#define POWER_MANAGER_GENERAL_DEBUG        1U
#endif

typedef struct {
    I2C_HandleTypeDef *hi2c;
    TPS25751_Device_t tps;
    BQ25731_Device_t bq;
    TPS25751_Telemetry_t tps_telemetry;
    BQ25731_Telemetry_t bq_telemetry;
    BQ25731_MonitorSnapshot_t bq_monitor;
    BQ25731_SafeStartupResult_t bq_safe_start;
    PowerManager_PdSnapshot_t pd_snapshot;
    TPS25751_Status_t tps_status;
    BQ25731_Status_t bq_status;
    PowerManager_State_t state;
    uint32_t next_task_tick_ms;
    uint32_t last_debug_tick_ms;
    uint32_t last_pd_caps_debug_tick_ms;
    uint32_t last_pd_caps_signature;
    uint32_t last_bq_monitor_tick_ms;
    uint32_t bq_adc_next_attempt_ms;
    uint32_t tps_wait_start_tick_ms;
    uint32_t last_tps_mode_log_tick_ms;
    uint32_t last_error_reg;
    uint32_t last_error_code;
    uint8_t tps_addr_selected;
    bool pd_was_connected;
    bool pd_caps_valid;
    bool pd_max_power_attempted;
    bool pd_source_swap_attempted;
    bool bq_adc_running_confirmed;
    bool bq_charging_enabled;
    bool bq_otg_enabled;
    uint32_t bq_applied_input_current_ma;
    uint8_t bq_adc_attempts;
    TPS25751_Mode_t last_logged_tps_mode;
#if (POWER_MANAGER_PD_CYCLE_TEST != 0U)
    uint32_t pd_test_next_tick_ms;
    uint32_t pd_test_last_blocked_log_ms;
    uint8_t pd_test_step;
#endif
    bool initialized;
} PowerManager_Context_t;

static PowerManager_Context_t g_pm;

static void PowerManager_DisableBqOtg(void)
{
    /* Hardware gate first: converter stops even if the I2C bridge is lost. */
    HAL_GPIO_WritePin(OTG_EN_GPIO_Port, OTG_EN_Pin, GPIO_PIN_RESET);
    if (g_pm.bq_otg_enabled) {
        BQ25731_Status_t status = BQ25731_DisableOtg(&g_pm.bq);
        Debug_Printf("[BQ-OTG] disabled: pin=LOW register=%s",
                     BQ25731_StatusToString(status));
    }
    g_pm.bq_otg_enabled = false;
}

static void PowerManager_AutoSourcePolicy(void)
{
    TPS25751_Status_t status;
    uint8_t i;
    bool only_5v = true;

    if (!g_pm.pd_snapshot.attached) {
        PowerManager_DisableBqOtg();
        g_pm.pd_source_swap_attempted = false;
        return;
    }

    /* A DRP connection can resolve directly as Source, without first making
     * a 5 V Sink contract and without an MCU-issued SWSr. PPHV must then be
     * brought up immediately from BQ. */
    if (g_pm.pd_snapshot.power_role == POWER_MANAGER_PD_ROLE_SOURCE) {
        if (!g_pm.bq_otg_enabled) {
            g_pm.bq_status = BQ25731_InhibitCharging(&g_pm.bq, NULL, NULL);
            (void)BQ25731_SetChargeCurrent(&g_pm.bq, 0U, NULL, NULL);
            HAL_GPIO_WritePin(OTG_EN_GPIO_Port, OTG_EN_Pin, GPIO_PIN_RESET);
            if (g_pm.bq_status == BQ25731_OK)
                g_pm.bq_status = BQ25731_DisableOtg(&g_pm.bq);
            if (g_pm.bq_status == BQ25731_OK)
                g_pm.bq_status = BQ25731_EnableOtg(
                    &g_pm.bq, BQ_USER_OTG_INITIAL_VOLTAGE_MV,
                    BQ_USER_OTG_INITIAL_CURRENT_MA, false);
            if (g_pm.bq_status == BQ25731_OK) {
                HAL_GPIO_WritePin(OTG_EN_GPIO_Port, OTG_EN_Pin, GPIO_PIN_SET);
                HAL_Delay(20U);
                g_pm.bq_otg_enabled = true;
                g_pm.bq_charging_enabled = false;
                g_pm.bq_applied_input_current_ma = 0U;
                Debug_Printf("[BQ-OTG] direct Source: pin=HIGH voltage=%lumV current_limit=%lumA",
                             (unsigned long)BQ_USER_OTG_INITIAL_VOLTAGE_MV,
                             (unsigned long)BQ_USER_OTG_INITIAL_CURRENT_MA);
            } else {
                HAL_GPIO_WritePin(OTG_EN_GPIO_Port, OTG_EN_Pin, GPIO_PIN_RESET);
                Debug_Printf("[BQ-OTG] direct Source start FAILED: %s",
                             BQ25731_StatusToString(g_pm.bq_status));
            }
        }
        return;
    }
    /* Do not shut OTG down on a transient role indication without a valid
     * Sink contract. During PR_SWAP TPS briefly reports Sink/no-contract;
     * removing PPHV there collapses VBUS and creates an endless attach loop.
     * A real Sink contract is handled below by the charging policy, which
     * disables OTG before enabling charge. Detach is handled above. */
    if (g_pm.pd_source_swap_attempted ||
        (g_pm.pd_snapshot.power_role != POWER_MANAGER_PD_ROLE_SINK) ||
        (g_pm.pd_snapshot.active_rdo_raw == 0U) ||
        (g_pm.pd_snapshot.contract_voltage_mv > 5500U)) return;

    status = TPS25751_ReadCapabilityLists(&g_pm.tps, &g_pm.tps_telemetry);
    if ((status != TPS25751_OK) ||
        (g_pm.tps_telemetry.rx_source_caps.count == 0U)) return;

    for (i = 0U; i < g_pm.tps_telemetry.rx_source_caps.count; ++i) {
        const TPS25751_PdoInfo_t *pdo = &g_pm.tps_telemetry.rx_source_caps.pdo[i];
        uint32_t highest_mv = (pdo->type == TPS25751_SUPPLY_FIXED) ?
                              pdo->voltage_mv : pdo->max_mv;
        if (highest_mv > 5500U) {
            only_5v = false;
            break;
        }
    }
    if (!only_5v) return;

    g_pm.pd_source_swap_attempted = true;
    g_pm.bq_status = BQ25731_InhibitCharging(&g_pm.bq, NULL, NULL);
    (void)BQ25731_SetChargeCurrent(&g_pm.bq, 0U, NULL, NULL);
    g_pm.bq_charging_enabled = false;
    g_pm.bq_applied_input_current_ma = 0U;
    if (g_pm.bq_status != BQ25731_OK) {
        Debug_Printf("[PD-AUTO-SOURCE] blocked: BQ inhibit failed (%s)",
                     BQ25731_StatusToString(g_pm.bq_status));
        return;
    }

    HAL_GPIO_WritePin(OTG_EN_GPIO_Port, OTG_EN_Pin, GPIO_PIN_RESET);
    g_pm.bq_status = BQ25731_DisableOtg(&g_pm.bq);
    if (g_pm.bq_status == BQ25731_OK) {
        g_pm.bq_status = BQ25731_EnableOtg(&g_pm.bq,
                                            BQ_USER_OTG_INITIAL_VOLTAGE_MV,
                                            BQ_USER_OTG_INITIAL_CURRENT_MA,
                                            false);
    }
    if (g_pm.bq_status != BQ25731_OK) {
        Debug_Printf("[PD-AUTO-SOURCE] blocked: BQ OTG configuration failed (%s)",
                     BQ25731_StatusToString(g_pm.bq_status));
        return;
    }
    HAL_GPIO_WritePin(OTG_EN_GPIO_Port, OTG_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(20U);
    g_pm.bq_otg_enabled = true;
    Debug_Printf("[BQ-OTG] enabled: pin=HIGH voltage=%lumV current_limit=%lumA",
                 (unsigned long)BQ_USER_OTG_INITIAL_VOLTAGE_MV,
                 (unsigned long)BQ_USER_OTG_INITIAL_CURRENT_MA);

    Debug_Printf("[PD-AUTO-SOURCE] partner offers only 5V; requesting PR_SWAP to Source via BQ/PPHV");
    status = TPS25751_RequestPowerRoleSource(&g_pm.tps);
    /* The cached snapshot still describes the old 5 V Sink contract until
     * the next telemetry read. Invalidate it now so the charging policy can
     * never re-enable BQ during the PR_SWAP transition. */
    g_pm.pd_snapshot.power_role = POWER_MANAGER_PD_ROLE_SOURCE;
    g_pm.pd_snapshot.active_rdo_raw = 0U;
    g_pm.pd_snapshot.contract_voltage_mv = 0U;
    g_pm.pd_snapshot.contract_current_ma = 0U;
    g_pm.pd_snapshot.contract_power_mw = 0U;
    Debug_Printf("[PD-AUTO-SOURCE] SWSr command=%s; partner may still accept or reject the swap",
                 TPS25751_StatusToString(status));
}

static void PowerManager_UpdateBqChargingPolicy(void)
{
    uint32_t input_ma = 0U;
    bool valid_contract = (BQ_USER_CHARGING_ENABLE != 0U) &&
        g_pm.pd_snapshot.attached &&
        (g_pm.pd_snapshot.power_role == POWER_MANAGER_PD_ROLE_SINK) &&
        (g_pm.pd_snapshot.active_rdo_raw != 0U) &&
        (g_pm.pd_snapshot.contract_voltage_mv != 0U) &&
        (g_pm.pd_snapshot.contract_current_ma > BQ_USER_PD_CURRENT_MARGIN_MA);

    if (valid_contract && g_pm.bq_otg_enabled) {
        PowerManager_DisableBqOtg();
    }

    if (valid_contract) {
        input_ma = g_pm.pd_snapshot.contract_current_ma - BQ_USER_PD_CURRENT_MARGIN_MA;
        if (input_ma > BQ_USER_MAX_INPUT_CURRENT_MA)
            input_ma = BQ_USER_MAX_INPUT_CURRENT_MA;
        if (input_ma < BQ_USER_MIN_INPUT_CURRENT_MA)
            valid_contract = false;
    }

    if (!valid_contract) {
        if (g_pm.bq_charging_enabled) {
            g_pm.bq_status = BQ25731_InhibitCharging(&g_pm.bq, NULL, NULL);
            (void)BQ25731_SetChargeCurrent(&g_pm.bq, 0U, NULL, NULL);
            Debug_Printf("[BQ-CHARGE] disabled: USB-C sink contract lost/invalid (%s)",
                         BQ25731_StatusToString(g_pm.bq_status));
        }
        g_pm.bq_charging_enabled = false;
        g_pm.bq_applied_input_current_ma = 0U;
        return;
    }

    if (g_pm.bq_charging_enabled &&
        (g_pm.bq_applied_input_current_ma == input_ma)) return;

    Debug_Printf("[BQ-CHARGE] contract=%lumV/%lumA; applying VREG=%lumV IIN=%lumA ICHG_MAX=%lumA",
                 (unsigned long)g_pm.pd_snapshot.contract_voltage_mv,
                 (unsigned long)g_pm.pd_snapshot.contract_current_ma,
                 (unsigned long)BQ_USER_CHARGE_VOLTAGE_MV,
                 (unsigned long)input_ma,
                 (unsigned long)BQ_USER_MAX_CHARGE_CURRENT_MA);
    g_pm.bq_status = BQ25731_ConfigureForCharging(&g_pm.bq, input_ma,
                                                  BQ_USER_MAX_CHARGE_CURRENT_MA,
                                                  BQ_USER_CHARGE_VOLTAGE_MV);
    g_pm.bq_charging_enabled = (g_pm.bq_status == BQ25731_OK);
    g_pm.bq_applied_input_current_ma = g_pm.bq_charging_enabled ? input_ma : 0U;
    Debug_Printf("[BQ-CHARGE] %s (%s)",
                 g_pm.bq_charging_enabled ? "enabled" : "FAILED; charger remains inhibited",
                 BQ25731_StatusToString(g_pm.bq_status));
}

static const char *PowerManager_StateText(PowerManager_State_t state)
{
    switch (state) {
        case POWER_MANAGER_INIT:
            return "INIT";
        case POWER_MANAGER_TPS_WAIT_APP:
            return "TPS_WAIT_APP";
        case POWER_MANAGER_TPS_READY:
            return "TPS_READY";
        case POWER_MANAGER_BQ_PROBE:
            return "BQ_PROBE";
        case POWER_MANAGER_BQ_SAFE_START:
            return "BQ_SAFE_START";
        case POWER_MANAGER_SAFE_MONITORING:
            return "SAFE_MONITORING";
        case POWER_MANAGER_DEGRADED_BQ:
            return "DEGRADED_BQ";
        case POWER_MANAGER_FAULT:
        default:
            return "FAULT";
    }
}

static const char *PowerManager_I2cText(const I2C_HandleTypeDef *hi2c)
{
    if ((hi2c == NULL) || (hi2c->Instance == NULL)) {
        return "I2C?";
    }

    if (hi2c->Instance == I2C3) {
        return "I2C3";
    }
    if (hi2c->Instance == I2C4) {
        return "I2C4";
    }

    return "I2C*";
}

static void PowerManager_DebugSeparator(const char *title)
{
    if (title == NULL) {
        Debug_Printf("-----");
        return;
    }

    Debug_Printf("----- %s -----", title);
}

static uint32_t PowerManager_IntPart(uint32_t value, uint32_t scale)
{
    if (scale == 0U) {
        return value;
    }

    return value / scale;
}

static uint32_t PowerManager_FracPart(uint32_t value, uint32_t scale)
{
    if (scale == 0U) {
        return 0U;
    }

    return value % scale;
}

static uint32_t PowerManager_RoundToNearest(uint32_t value, uint32_t quantum)
{
    if (quantum == 0U) {
        return value;
    }

    return ((value + (quantum / 2U)) / quantum) * quantum;
}

static void PowerManager_Append(char *buffer, size_t buffer_len, const char *fmt, ...)
{
    size_t used;
    va_list args;

    if ((buffer == NULL) || (buffer_len == 0U) || (fmt == NULL)) {
        return;
    }

    used = strlen(buffer);
    if (used >= (buffer_len - 1U)) {
        return;
    }

    va_start(args, fmt);
    (void)vsnprintf(&buffer[used], buffer_len - used, fmt, args);
    va_end(args);
}

static const char *PowerManager_PortRoleText(const TPS25751_Telemetry_t *tps)
{
    if ((tps != NULL) && tps->pd_role_source) {
        return "SRC";
    }

    return "SNK";
}

static const char *PowerManager_DataRoleText(const TPS25751_Telemetry_t *tps)
{
    if ((tps != NULL) && tps->data_role_dfp) {
        return "DFP";
    }

    return "UFP";
}

static const char *PowerManager_PdContractText(uint32_t pd_contract)
{
    return (pd_contract != 0U) ? "YES" : "NO";
}

static const char *PowerManager_RdoDirectionText(const TPS25751_Telemetry_t *tps)
{
    if ((tps != NULL) && tps->pd_role_source) {
        return "sink requested from us";
    }

    return "we requested from source";
}

static bool PowerManager_IsPdConnected(const TPS25751_Telemetry_t *tps)
{
    if (tps == NULL) {
        return false;
    }

    if (tps->plug_present || tps->power_connection || tps->active_rdo.valid) {
        return true;
    }

    if ((tps->connection_state == 6U) || (tps->connection_state == 7U)) {
        return true;
    }

    if ((tps->typec_state == 0x60U) || (tps->typec_state == 0x61U) ||
        (tps->typec_state == 0x62U) || (tps->typec_state == 0x63U)) {
        return true;
    }

    return (tps->vbus_mv > 4000U);
}

static const char *PowerManager_PowerClassText(PowerManager_PowerClass_t power_class)
{
    switch (power_class) {
        case POWER_MANAGER_POWER_LOW:
            return "LOW";
        case POWER_MANAGER_POWER_MEDIUM:
            return "MEDIUM";
        case POWER_MANAGER_POWER_HIGH:
            return "HIGH";
        case POWER_MANAGER_POWER_FULL:
            return "FULL";
        case POWER_MANAGER_POWER_NO_INPUT:
        default:
            return "NO_INPUT";
    }
}

static PowerManager_PowerClass_t PowerManager_ClassifyPower(bool attached,
                                                             uint32_t power_mw)
{
    if ((!attached) || (power_mw == 0U)) {
        return POWER_MANAGER_POWER_NO_INPUT;
    }
    if (power_mw < 15000U) {
        return POWER_MANAGER_POWER_LOW;
    }
    if (power_mw < 45000U) {
        return POWER_MANAGER_POWER_MEDIUM;
    }
    if (power_mw <= 60000U) {
        return POWER_MANAGER_POWER_HIGH;
    }
    return POWER_MANAGER_POWER_FULL;
}

static void PowerManager_UpdatePdSnapshot(void)
{
    const TPS25751_Telemetry_t *tps = &g_pm.tps_telemetry;
    const TPS25751_PdoInfo_t *pdo = &tps->active_pdo;
    const TPS25751_RdoInfo_t *rdo = &tps->active_rdo;
    PowerManager_PdSnapshot_t *snapshot = &g_pm.pd_snapshot;
#if (POWER_MANAGER_PD_POLICY_DEBUG != 0U)
    PowerManager_PdSnapshot_t previous = *snapshot;
#endif

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->attached = PowerManager_IsPdConnected(tps);
    snapshot->power_role = snapshot->attached ?
                           (tps->pd_role_source ? POWER_MANAGER_PD_ROLE_SOURCE :
                                                  POWER_MANAGER_PD_ROLE_SINK) :
                           POWER_MANAGER_PD_ROLE_UNKNOWN;
    snapshot->data_role_dfp = tps->data_role_dfp;
    snapshot->active_pdo_raw = tps->active_pdo_raw;
    snapshot->active_rdo_raw = tps->active_rdo_raw;

    if (snapshot->attached && rdo->valid) {
        if (pdo->type == TPS25751_SUPPLY_FIXED) {
            snapshot->contract_voltage_mv = pdo->voltage_mv;
            snapshot->contract_current_ma = rdo->operating_current_ma;
        } else if (pdo->type == TPS25751_SUPPLY_APDO_PPS) {
            snapshot->contract_voltage_mv = ((rdo->raw >> 9) & 0x0FFFU) * 20U;
            snapshot->contract_current_ma = (rdo->raw & 0x7FU) * 50U;
        } else if (pdo->type == TPS25751_SUPPLY_BATTERY) {
            snapshot->contract_voltage_mv = tps->vbus_mv;
            snapshot->contract_power_mw = ((rdo->raw >> 10) & 0x3FFU) * 250U;
            if (snapshot->contract_voltage_mv != 0U) {
                snapshot->contract_current_ma =
                    (snapshot->contract_power_mw * 1000U) /
                    snapshot->contract_voltage_mv;
            }
        } else if (pdo->type == TPS25751_SUPPLY_VARIABLE) {
            snapshot->contract_voltage_mv = tps->vbus_mv;
            snapshot->contract_current_ma = rdo->operating_current_ma;
        }
    }

    if (snapshot->contract_power_mw == 0U) {
        snapshot->contract_power_mw =
            (snapshot->contract_voltage_mv * snapshot->contract_current_ma) / 1000U;
    }

    snapshot->power_class = PowerManager_ClassifyPower(snapshot->attached,
                                                        snapshot->contract_power_mw);

#if (POWER_MANAGER_PD_POLICY_DEBUG != 0U)
    if ((previous.attached != snapshot->attached) ||
        (previous.active_rdo_raw != snapshot->active_rdo_raw) ||
        (previous.contract_power_mw != snapshot->contract_power_mw)) {
        if (snapshot->attached && (snapshot->active_rdo_raw != 0U) &&
            (snapshot->contract_voltage_mv != 0U) &&
            (snapshot->contract_power_mw != 0U)) {
            Debug_Printf("[PM-PD] snapshot valid: %lumV %lumA %lumW RDO=0x%08lX state=%s BQ=%s",
                         (unsigned long)snapshot->contract_voltage_mv,
                         (unsigned long)snapshot->contract_current_ma,
                         (unsigned long)snapshot->contract_power_mw,
                         (unsigned long)snapshot->active_rdo_raw,
                         PowerManager_StateText(g_pm.state),
                         BQ25731_StatusToString(g_pm.bq_status));
        } else {
            Debug_Printf("[PM-PD] snapshot cleared/invalid: attached=%u RDO=0x%08lX V=%lumV P=%lumW state=%s BQ=%s",
                         snapshot->attached ? 1U : 0U,
                         (unsigned long)snapshot->active_rdo_raw,
                         (unsigned long)snapshot->contract_voltage_mv,
                         (unsigned long)snapshot->contract_power_mw,
                         PowerManager_StateText(g_pm.state),
                         BQ25731_StatusToString(g_pm.bq_status));
        }
    }
#endif
}

static void PowerManager_ClearPdCaps(void)
{
    memset(&g_pm.tps_telemetry.rx_source_caps, 0, sizeof(g_pm.tps_telemetry.rx_source_caps));
    memset(&g_pm.tps_telemetry.rx_sink_caps, 0, sizeof(g_pm.tps_telemetry.rx_sink_caps));
    memset(&g_pm.tps_telemetry.tx_source_caps, 0, sizeof(g_pm.tps_telemetry.tx_source_caps));
    memset(&g_pm.tps_telemetry.tx_sink_caps, 0, sizeof(g_pm.tps_telemetry.tx_sink_caps));

    g_pm.last_pd_caps_signature = 0U;
    g_pm.pd_caps_valid = false;
}

static void PowerManager_FormatCurrent(char *buffer, size_t buffer_len, uint32_t current_ma)
{
    uint32_t rounded_ma;

    if ((buffer == NULL) || (buffer_len == 0U)) {
        return;
    }

    rounded_ma = PowerManager_RoundToNearest(current_ma, 100U);
    (void)snprintf(buffer,
                   buffer_len,
                   "%lu.%luA",
                   (unsigned long)PowerManager_IntPart(rounded_ma, 1000U),
                   (unsigned long)(PowerManager_FracPart(rounded_ma, 1000U) / 100U));
}

#if (POWER_MANAGER_PD_POLICY_DEBUG != 0U)
static void PowerManager_FormatCurrentExact(char *buffer,
                                            size_t buffer_len,
                                            uint32_t current_ma)
{
    if ((buffer == NULL) || (buffer_len == 0U)) {
        return;
    }

    (void)snprintf(buffer, buffer_len, "%lu.%03luA",
                   (unsigned long)(current_ma / 1000U),
                   (unsigned long)(current_ma % 1000U));
}
#endif

static void PowerManager_FormatVoltage(char *buffer, size_t buffer_len, uint32_t voltage_mv)
{
    uint32_t rounded_mv;

    if ((buffer == NULL) || (buffer_len == 0U)) {
        return;
    }

    rounded_mv = PowerManager_RoundToNearest(voltage_mv, 100U);
    (void)snprintf(buffer,
                   buffer_len,
                   "%lu.%luV",
                   (unsigned long)PowerManager_IntPart(rounded_mv, 1000U),
                   (unsigned long)(PowerManager_FracPart(rounded_mv, 1000U) / 100U));
}

static void PowerManager_FormatPower(char *buffer, size_t buffer_len, uint32_t power_mw)
{
    uint32_t rounded_mw;

    if ((buffer == NULL) || (buffer_len == 0U)) {
        return;
    }

    rounded_mw = PowerManager_RoundToNearest(power_mw, 100U);
    (void)snprintf(buffer,
                   buffer_len,
                   "%lu.%luW",
                   (unsigned long)PowerManager_IntPart(rounded_mw, 1000U),
                   (unsigned long)(PowerManager_FracPart(rounded_mw, 1000U) / 100U));
}

static void PowerManager_FormatPdoShort(char *buffer,
                                        size_t buffer_len,
                                        uint8_t index,
                                        const TPS25751_PdoInfo_t *pdo)
{
    char v[16];
    char v_min[16];
    char v_max[16];
    char i[16];
    char p[16];

    if ((buffer == NULL) || (buffer_len == 0U)) {
        return;
    }

    buffer[0] = '\0';

    if (pdo == NULL) {
        return;
    }

    if (pdo->type == TPS25751_SUPPLY_FIXED) {
        PowerManager_FormatVoltage(v, sizeof(v), pdo->voltage_mv);
        PowerManager_FormatCurrent(i, sizeof(i), pdo->current_ma);
        PowerManager_FormatPower(p, sizeof(p), pdo->power_mw);
        (void)snprintf(buffer, buffer_len, "#%u FIXED %s %s %s", (unsigned int)index, v, i, p);
        return;
    }

    if (pdo->type == TPS25751_SUPPLY_BATTERY) {
        PowerManager_FormatVoltage(v_min, sizeof(v_min), pdo->min_mv);
        PowerManager_FormatVoltage(v_max, sizeof(v_max), pdo->max_mv);
        PowerManager_FormatPower(p, sizeof(p), pdo->power_mw);
        (void)snprintf(buffer,
                       buffer_len,
                       "#%u BAT %s-%s %s",
                       (unsigned int)index,
                       v_min,
                       v_max,
                       p);
        return;
    }

    if (pdo->type == TPS25751_SUPPLY_VARIABLE) {
        PowerManager_FormatVoltage(v_min, sizeof(v_min), pdo->min_mv);
        PowerManager_FormatVoltage(v_max, sizeof(v_max), pdo->max_mv);
        PowerManager_FormatCurrent(i, sizeof(i), pdo->current_ma);
        PowerManager_FormatPower(p, sizeof(p), pdo->power_mw);
        (void)snprintf(buffer,
                       buffer_len,
                       "#%u VAR %s-%s %s %s",
                       (unsigned int)index,
                       v_min,
                       v_max,
                       i,
                       p);
        return;
    }

    if (pdo->type == TPS25751_SUPPLY_APDO_PPS) {
        PowerManager_FormatVoltage(v_min, sizeof(v_min), pdo->min_mv);
        PowerManager_FormatVoltage(v_max, sizeof(v_max), pdo->max_mv);
        PowerManager_FormatCurrent(i, sizeof(i), pdo->current_ma);
        PowerManager_FormatPower(p, sizeof(p), pdo->power_mw);
        (void)snprintf(buffer,
                       buffer_len,
                       "#%u PPS %s-%s %s %s",
                       (unsigned int)index,
                       v_min,
                       v_max,
                       i,
                       p);
        return;
    }

    if (pdo->type == TPS25751_SUPPLY_APDO_OTHER) {
#if (POWER_MANAGER_PD_RAW_DEBUG != 0U)
        (void)snprintf(buffer,
                       buffer_len,
                       "#%u APDO? raw=0x%08lX",
                       (unsigned int)index,
                       (unsigned long)pdo->raw);
#else
        (void)snprintf(buffer, buffer_len, "#%u APDO?", (unsigned int)index);
#endif
        return;
    }

    (void)snprintf(buffer, buffer_len, "#%u empty", (unsigned int)index);
}

static bool PowerManager_DebugPdoList(const char *description,
                                      const TPS25751_PdoList_t *list)
{
    char line[POWER_MANAGER_PD_LINE_BUF_SIZE];
    char pdo_text[96];
    uint8_t i;

    if ((description == NULL) || (list == NULL)) {
        return false;
    }

    if (list->count == 0U) {
        return false;
    }

    line[0] = '\0';
    PowerManager_Append(line,
                        sizeof(line),
                        "[PDO] %s: ",
                        description);

    for (i = 0U; i < list->count; ++i) {
        PowerManager_FormatPdoShort(pdo_text,
                                    sizeof(pdo_text),
                                    (uint8_t)(i + 1U),
                                    &list->pdo[i]);
        if (i > 0U) {
            PowerManager_Append(line, sizeof(line), "  ");
        }
        PowerManager_Append(line, sizeof(line), "%s", pdo_text);
    }

    Debug_Printf("%s", line);
    return true;
}

static void PowerManager_FormatRdoFlags(char *buffer,
                                        size_t buffer_len,
                                        const TPS25751_RdoInfo_t *rdo)
{
    bool any = false;

    if ((buffer == NULL) || (buffer_len == 0U)) {
        return;
    }

    buffer[0] = '\0';

    if (rdo == NULL) {
        return;
    }

    if (rdo->capability_mismatch) {
        PowerManager_Append(buffer, buffer_len, "mismatch");
        any = true;
    }
    if (rdo->usb_comm_capable) {
        PowerManager_Append(buffer, buffer_len, "%susb", any ? "," : "");
        any = true;
    }
    if (rdo->no_usb_suspend) {
        PowerManager_Append(buffer, buffer_len, "%sno_suspend", any ? "," : "");
        any = true;
    }
    if (rdo->unchunked_supported) {
        PowerManager_Append(buffer, buffer_len, "%sunchunked", any ? "," : "");
        any = true;
    }

    if (!any) {
        PowerManager_Append(buffer, buffer_len, "none");
    }
}

static void PowerManager_DebugRdo(const TPS25751_Telemetry_t *tps)
{
    const TPS25751_RdoInfo_t *rdo;
    const TPS25751_PdoInfo_t *pdo;
    char flags[48];
    char op[16];
    char max[16];
    char pps_v[16];
    uint32_t operating_power_mw;
    uint32_t max_power_mw;
    uint32_t pps_voltage_mv;
    uint32_t pps_current_ma;

    if (tps == NULL) {
        return;
    }

    rdo = &tps->active_rdo;
    pdo = &tps->active_pdo;

    if (!rdo->valid) {
        return;
    }

    PowerManager_FormatRdoFlags(flags, sizeof(flags), rdo);

    if (pdo->type == TPS25751_SUPPLY_BATTERY) {
        operating_power_mw = ((rdo->raw >> 10) & 0x3FFU) * 250U;
        max_power_mw = (rdo->raw & 0x3FFU) * 250U;
        PowerManager_FormatPower(op, sizeof(op), operating_power_mw);
        PowerManager_FormatPower(max, sizeof(max), max_power_mw);

        Debug_Printf("[RDO] %s: PDO#%u op=%s max=%s flags=%s",
                     PowerManager_RdoDirectionText(tps),
                     (unsigned int)rdo->object_position,
                     op,
                     max,
                     flags);
        return;
    }

    if (pdo->type == TPS25751_SUPPLY_APDO_PPS) {
        pps_voltage_mv = ((rdo->raw >> 9) & 0x0FFFU) * 20U;
        pps_current_ma = (rdo->raw & 0x7FU) * 50U;
        PowerManager_FormatVoltage(pps_v, sizeof(pps_v), pps_voltage_mv);
        PowerManager_FormatCurrent(op, sizeof(op), pps_current_ma);

        Debug_Printf("[RDO] %s: PDO#%u PPS=%s op=%s flags=%s",
                     PowerManager_RdoDirectionText(tps),
                     (unsigned int)rdo->object_position,
                     pps_v,
                     op,
                     flags);
        return;
    }

    PowerManager_FormatCurrent(op, sizeof(op), rdo->operating_current_ma);
    PowerManager_FormatCurrent(max, sizeof(max), rdo->max_current_ma);

    Debug_Printf("[RDO] %s: PDO#%u op=%s max=%s flags=%s",
                 PowerManager_RdoDirectionText(tps),
                 (unsigned int)rdo->object_position,
                 op,
                 max,
                 flags);
}

static uint32_t PowerManager_PdoListSignature(const TPS25751_PdoList_t *list)
{
    uint32_t sig = 2166136261UL;
    uint8_t i;

    if (list == NULL) {
        return 0U;
    }

    sig ^= list->count;
    sig *= 16777619UL;

    for (i = 0U; i < list->count; ++i) {
        sig ^= list->pdo[i].raw;
        sig *= 16777619UL;
    }

    return sig;
}

static uint32_t PowerManager_PdCapsSignature(const TPS25751_Telemetry_t *tps)
{
    uint32_t sig;

    if (tps == NULL) {
        return 0U;
    }

    sig = PowerManager_PdoListSignature(&tps->rx_source_caps);
    sig ^= PowerManager_PdoListSignature(&tps->rx_sink_caps) * 3UL;
    sig ^= PowerManager_PdoListSignature(&tps->tx_source_caps) * 5UL;
    sig ^= PowerManager_PdoListSignature(&tps->tx_sink_caps) * 7UL;

    return sig;
}

void PowerManager_LogPdSinkPolicy(void)
{
#if (POWER_MANAGER_PD_POLICY_DEBUG != 0U)
    TPS25751_AutoNegotiateSink_t policy;
    TPS25751_Status_t status;
    TPS25751_Telemetry_t *tps = &g_pm.tps_telemetry;
    const TPS25751_PdoInfo_t *active_pdo;
    const TPS25751_RdoInfo_t *active_rdo;
    char pdo_text[96];
    char flags[48];
    char raw_line[3U * TPS25751_AUTO_NEGOTIATE_SINK_LEN + 1U];
    char offered_voltage[16];
    char offered_current[16];
    char offered_power[16];
    char requested_current[16];
    char requested_power[16];
    char delta_power[16];
    uint32_t requested_power_mw;
    uint32_t offered_power_mw;
    uint32_t local_matching_power_mw = 0U;
    uint32_t local_matching_current_ma = 0U;
    uint8_t i;

    if ((!g_pm.initialized) ||
        (g_pm.state != POWER_MANAGER_BQ_MONITOR) ||
        (tps->mode != TPS25751_MODE_APP)) {
        return;
    }

    status = TPS25751_ReadTelemetryBasic(&g_pm.tps, tps);
    if (status != TPS25751_OK) {
        Debug_Printf("[PD-POLICY] telemetry read failed: %s",
                     TPS25751_StatusToString(status));
        return;
    }

    status = TPS25751_ReadCapabilityLists(&g_pm.tps, tps);
    if (status != TPS25751_OK) {
        Debug_Printf("[PD-POLICY] capability read failed: %s",
                     TPS25751_StatusToString(status));
        return;
    }

    status = TPS25751_ReadAutoNegotiateSink(&g_pm.tps, &policy);
    if (status != TPS25751_OK) {
        Debug_Printf("[PD-POLICY] AUTO_NEGOTIATE_SINK read failed: %s",
                     TPS25751_StatusToString(status));
        return;
    }

    PowerManager_UpdatePdSnapshot();
    active_pdo = &tps->active_pdo;
    active_rdo = &tps->active_rdo;
    offered_power_mw = active_pdo->power_mw;
    requested_power_mw = g_pm.pd_snapshot.contract_power_mw;

    PowerManager_DebugSeparator("PD SINK POLICY");

    for (i = 0U; i < tps->rx_source_caps.count; ++i) {
        PowerManager_FormatPdoShort(pdo_text, sizeof(pdo_text),
                                    (uint8_t)(i + 1U),
                                    &tps->rx_source_caps.pdo[i]);
        Debug_Printf("[PD-POLICY] source offers %s", pdo_text);
    }

    for (i = 0U; i < tps->tx_sink_caps.count; ++i) {
        const TPS25751_PdoInfo_t *local_pdo = &tps->tx_sink_caps.pdo[i];
        PowerManager_FormatPdoShort(pdo_text, sizeof(pdo_text),
                                    (uint8_t)(i + 1U),
                                    local_pdo);
        Debug_Printf("[PD-POLICY] local sink %s", pdo_text);
        if ((local_pdo->type == TPS25751_SUPPLY_FIXED) &&
            (local_pdo->voltage_mv == active_pdo->voltage_mv) &&
            (local_pdo->power_mw > local_matching_power_mw)) {
            local_matching_power_mw = local_pdo->power_mw;
            local_matching_current_ma = local_pdo->current_ma;
        }
    }

    PowerManager_FormatPdoShort(pdo_text, sizeof(pdo_text),
                                active_rdo->object_position, active_pdo);
    Debug_Printf("[PD-POLICY] active PDO %s raw=0x%08lX",
                 pdo_text, (unsigned long)active_pdo->raw);

    PowerManager_FormatRdoFlags(flags, sizeof(flags), active_rdo);
    PowerManager_FormatCurrentExact(requested_current, sizeof(requested_current),
                                    active_rdo->operating_current_ma);
    PowerManager_FormatPower(requested_power, sizeof(requested_power),
                             requested_power_mw);
    Debug_Printf("[PD-POLICY] active RDO: PDO#%u raw=0x%08lX op=%s max=%lu.%03luA requested=%s flags=%s",
                 (unsigned int)active_rdo->object_position,
                 (unsigned long)active_rdo->raw,
                 requested_current,
                 (unsigned long)(active_rdo->max_current_ma / 1000U),
                 (unsigned long)(active_rdo->max_current_ma % 1000U),
                 requested_power,
                 flags);
    PowerManager_FormatVoltage(offered_voltage, sizeof(offered_voltage),
                               active_pdo->voltage_mv);
    PowerManager_FormatCurrentExact(offered_current, sizeof(offered_current),
                                    active_pdo->current_ma);
    PowerManager_FormatPower(offered_power, sizeof(offered_power), offered_power_mw);
    Debug_Printf("[PD-POLICY] Source offers: %s %s %s",
                 offered_voltage, offered_current, offered_power);
    Debug_Printf("[PD-POLICY] Sink requested: %s %s %s",
                 offered_voltage, requested_current, requested_power);

    raw_line[0] = '\0';
    for (i = 0U; i < TPS25751_AUTO_NEGOTIATE_SINK_LEN; ++i) {
        PowerManager_Append(raw_line, sizeof(raw_line), "%02X%s",
                            policy.raw[i],
                            (i + 1U < TPS25751_AUTO_NEGOTIATE_SINK_LEN) ? " " : "");
    }
    Debug_Printf("[PD-POLICY] AUTO_NEGOTIATE_SINK raw[24]=%s", raw_line);
    Debug_Printf("[PD-POLICY] AUTO_NEGOTIATE_SINK voltage fields: min=%lumV max=%lumV auto_min=%u auto_max=%u (field ignored when corresponding auto=1)",
                 (unsigned long)policy.min_voltage_mv,
                 (unsigned long)policy.max_voltage_mv,
                 policy.auto_compute_min_voltage ? 1U : 0U,
                 policy.auto_compute_max_voltage ? 1U : 0U);
    Debug_Printf("[PD-POLICY] AUTO_NEGOTIATE_SINK power: min_required=%lumW mismatch_threshold=%lumW auto_min_power=%u no_mismatch=%u disable_sink_on_mismatch=%u",
                 (unsigned long)policy.sink_min_required_power_mw,
                 (unsigned long)policy.capability_mismatch_power_mw,
                 policy.auto_compute_min_power ? 1U : 0U,
                 policy.no_capability_mismatch ? 1U : 0U,
                 policy.auto_disable_sink_on_mismatch ? 1U : 0U);
    if (policy.sink_min_required_power_mw == 65000U) {
        Debug_Printf("[PD-POLICY] min_required=65000mW => TPS will request about 65W");
        Debug_Printf("[PD-POLICY] To request 100W set AutoComputeSinkMinPower=1 or ANSinkMinRequiredPower=400 (0x190)");
    }
    Debug_Printf("[PD-POLICY] AUTO_NEGOTIATE_SINK policy: tie_priority=%s no_usb_suspend=%u PPS=%s pps_voltage=%lumV pps_current=%lumA",
                 policy.prefer_lower_voltage_on_tie ? "LOWER_VOLTAGE" : "HIGHER_VOLTAGE",
                 policy.no_usb_suspend ? 1U : 0U,
                 policy.pps_sink_enabled ? "ENABLED" : "DISABLED",
                 (unsigned long)policy.pps_output_voltage_mv,
                 (unsigned long)policy.pps_operating_current_ma);

    if (offered_power_mw > requested_power_mw) {
        PowerManager_FormatPower(delta_power, sizeof(delta_power),
                                 offered_power_mw - requested_power_mw);
        Debug_Printf("[PD-POLICY] WARNING: source offers %s but sink requested only %s (delta=%s)",
                     offered_power, requested_power, delta_power);
        if ((local_matching_current_ma != 0U) &&
            (local_matching_current_ma < active_pdo->current_ma)) {
            Debug_Printf("[PD-POLICY] likely limit source: TX_SINK_CAPS declares only %lumA/%lumW at active voltage",
                         (unsigned long)local_matching_current_ma,
                         (unsigned long)local_matching_power_mw);
        } else if ((!policy.auto_compute_min_power) &&
                   (policy.sink_min_required_power_mw == requested_power_mw)) {
            Debug_Printf("[PD-POLICY] likely limit source: explicit AUTO_NEGOTIATE_SINK min-required-power matches requested power");
        } else if (policy.auto_compute_min_power &&
                   (local_matching_power_mw == requested_power_mw)) {
            Debug_Printf("[PD-POLICY] likely limit source: AUTO_NEGOTIATE_SINK computes min power from TX_SINK_CAPS");
        } else {
            Debug_Printf("[PD-POLICY] likely limit source: AUTO_NEGOTIATE_SINK / TX_SINK_CAPS / EEPROM sink policy; inspect values above");
        }
    }
#else
    return;
#endif
}

static void PowerManager_DebugPdConnectionEvent(uint32_t now_ms, bool pd_connected)
{
    (void)now_ms;

    if (pd_connected && (!g_pm.pd_was_connected)) {
        PowerManager_DebugSeparator("PD CONNECTED");
        Debug_Printf("[PD] device connected");
        g_pm.last_pd_caps_debug_tick_ms = 0U;
        PowerManager_ClearPdCaps();
    } else if ((!pd_connected) && g_pm.pd_was_connected) {
        PowerManager_DebugSeparator("PD DISCONNECTED");
        Debug_Printf("[PD] PDO cleared");
        PowerManager_ClearPdCaps();
    }

    g_pm.pd_was_connected = pd_connected;
}

static void PowerManager_DebugPdCaps(uint32_t now_ms, bool pd_connected)
{
    TPS25751_Telemetry_t *tps = &g_pm.tps_telemetry;
    TPS25751_Status_t caps_status;
    uint32_t signature;
    bool interval_elapsed;
    bool changed;
    bool any_list = false;

    if (!pd_connected) {
        return;
    }

    interval_elapsed = ((uint32_t)(now_ms - g_pm.last_pd_caps_debug_tick_ms) >=
                        POWER_MANAGER_PD_CAPS_DEBUG_MS);
    if (g_pm.pd_caps_valid && (!interval_elapsed)) {
        return;
    }

    caps_status = TPS25751_ReadCapabilityLists(&g_pm.tps, tps);
    if (caps_status != TPS25751_OK) {
        g_pm.last_pd_caps_debug_tick_ms = now_ms;
        g_pm.pd_caps_valid = true;
        PowerManager_DebugSeparator("PD CAPS");
        Debug_Printf("[PD-CAPS] read failed: %s", TPS25751_StatusToString(caps_status));
        return;
    }

    signature = PowerManager_PdCapsSignature(tps);
    changed = (!g_pm.pd_caps_valid) || (signature != g_pm.last_pd_caps_signature);

    if ((!changed) && (!interval_elapsed)) {
        return;
    }

    g_pm.last_pd_caps_debug_tick_ms = now_ms;
    g_pm.last_pd_caps_signature = signature;
    g_pm.pd_caps_valid = true;

    PowerManager_DebugSeparator("PD CAPS");
    Debug_Printf("[PD-CAPS] %s", changed ? "updated" : "periodic");

    any_list |= PowerManager_DebugPdoList("partner source", &tps->rx_source_caps);
    any_list |= PowerManager_DebugPdoList("partner sink", &tps->rx_sink_caps);
    any_list |= PowerManager_DebugPdoList("my source", &tps->tx_source_caps);
    any_list |= PowerManager_DebugPdoList("my sink", &tps->tx_sink_caps);

    if (!any_list) {
        Debug_Printf("[PD-CAPS] no PDO received");
    }

#if (POWER_MANAGER_PD_POLICY_DEBUG != 0U)
    PowerManager_LogPdSinkPolicy();
#endif
}

static TPS25751_Status_t PowerManager_ProbeTpsAddress(uint8_t addr_7bit)
{
    uint8_t mode_ascii[4];
    uint8_t payload_len = 0U;
    TPS25751_Status_t status;

    status = TPS25751_Init(&g_pm.tps, g_pm.hi2c, addr_7bit);
    if (status != TPS25751_OK) {
        return status;
    }

    status = TPS25751_ReadPayload(&g_pm.tps,
                                  POWER_MANAGER_TPS_PROBE_REG_MODE,
                                  mode_ascii,
                                  sizeof(mode_ascii),
                                  &payload_len);
    if ((status == TPS25751_OK) && (payload_len == sizeof(mode_ascii))) {
        return TPS25751_OK;
    }

    return (status == TPS25751_OK) ? TPS25751_BAD_LENGTH : status;
}

static void PowerManager_SetErrorFromTps(const TPS25751_Device_t *tps)
{
    if (tps == NULL) {
        return;
    }

    g_pm.last_error_reg = tps->last_register;
    g_pm.last_error_code = tps->last_error;
}

static void PowerManager_SetErrorFromBq(const BQ25731_Device_t *bq)
{
    if (bq == NULL) {
        return;
    }

    g_pm.last_error_reg = bq->last_bq_register;
    g_pm.last_error_code = bq->last_bq_error_code;
}

static const char *PowerManager_BqSafeErrorText(BQ25731_SafeError_t error)
{
    switch (error) {
        case BQ_ERR_NONE: return "BQ_ERR_NONE";
        case BQ_ERR_PROBE_FAILED: return "BQ_ERR_PROBE_FAILED";
        case BQ_ERR_I2C_READ_FAILED: return "BQ_ERR_I2C_READ_FAILED";
        case BQ_ERR_I2C_WRITE_FAILED: return "BQ_ERR_I2C_WRITE_FAILED";
        case BQ_ERR_CHRG_INHIBIT_NOT_SET: return "BQ_ERR_CHRG_INHIBIT_NOT_SET";
        case BQ_ERR_CHARGE_CURRENT_NOT_ZERO: return "BQ_ERR_CHARGE_CURRENT_NOT_ZERO";
        case BQ_ERR_EN_OTG_STUCK_ON: return "BQ_ERR_EN_OTG_STUCK_ON";
        case BQ_ERR_IN_OTG_ACTIVE: return "BQ_ERR_IN_OTG_ACTIVE";
        case BQ_ERR_IN_FAST_CHARGE: return "BQ_ERR_IN_FAST_CHARGE";
        case BQ_ERR_BRIDGE_WRITE_FAILED: return "BQ_ERR_BRIDGE_WRITE_FAILED";
        default: return "BQ_ERR_UNKNOWN";
    }
}

#if (POWER_MANAGER_PD_CYCLE_TEST != 0U)
static void PowerManager_PdCycleTestTask(uint32_t now_ms)
{
    static const uint32_t requested_voltage_mv[] = {
        20000U, 15000U, 9000U, 5000U
    };
    uint8_t attempts;

    if ((int32_t)(now_ms - g_pm.pd_test_next_tick_ms) < 0) {
        return;
    }

    if ((App_GetRequestedMode() != MODE_IDLE) ||
        (!g_pm.bq_telemetry.charge_inhibited) ||
        g_pm.bq_telemetry.otg_enabled ||
        g_pm.bq_telemetry.in_otg ||
        (!g_pm.pd_snapshot.attached) ||
        (g_pm.pd_snapshot.power_role != POWER_MANAGER_PD_ROLE_SINK)) {
        if ((uint32_t)(now_ms - g_pm.pd_test_last_blocked_log_ms) >= 1000U) {
            Debug_Printf("[PD-TEST] blocked: PSU must be OFF, charging inhibited, OTG off and port in Sink role");
            g_pm.pd_test_last_blocked_log_ms = now_ms;
        }
        g_pm.pd_test_next_tick_ms = now_ms + 1000U;
        return;
    }

    for (attempts = 0U; attempts < 4U; ++attempts) {
        uint32_t target_mv = requested_voltage_mv[g_pm.pd_test_step];
        TPS25751_Status_t status;

        g_pm.pd_test_step = (uint8_t)((g_pm.pd_test_step + 1U) % 4U);
        Debug_Printf("[PD-TEST] request %lumV", (unsigned long)target_mv);

        status = TPS25751_RequestSinkVoltageMv(&g_pm.tps,
                                               target_mv,
                                               POWER_MANAGER_PD_TEST_TIMEOUT_MS);
        if (status == TPS25751_NOT_AVAILABLE) {
            Debug_Printf("[PD-TEST] voltage not available, skipping %lumV",
                         (unsigned long)target_mv);
            continue;
        }

        if (status != TPS25751_OK) {
            Debug_Printf("[PD-TEST] renegotiation failed/timeout: target=%lumV status=%s",
                         (unsigned long)target_mv,
                         TPS25751_StatusToString(status));
            g_pm.pd_test_next_tick_ms = HAL_GetTick() + POWER_MANAGER_PD_TEST_STEP_MS;
            return;
        }

        g_pm.tps_status = TPS25751_ReadTelemetryBasic(&g_pm.tps,
                                                       &g_pm.tps_telemetry);
        if (g_pm.tps_status == TPS25751_OK) {
            PowerManager_UpdatePdSnapshot();
            Debug_Printf("[PD-TEST] active contract %lumV %lumA %lumW",
                         (unsigned long)g_pm.pd_snapshot.contract_voltage_mv,
                         (unsigned long)g_pm.pd_snapshot.contract_current_ma,
                         (unsigned long)g_pm.pd_snapshot.contract_power_mw);
        }

        g_pm.pd_test_next_tick_ms = HAL_GetTick() + POWER_MANAGER_PD_TEST_STEP_MS;
        return;
    }

    Debug_Printf("[PD-TEST] no supported fixed PDO in 20/15/9/5V test sequence");
    g_pm.pd_test_next_tick_ms = now_ms + POWER_MANAGER_PD_TEST_STEP_MS;
}
#endif

static void PowerManager_DebugLine(uint32_t now_ms)
{
    const TPS25751_Telemetry_t *tps = &g_pm.tps_telemetry;
    const BQ25731_Telemetry_t *bq = &g_pm.bq_telemetry;
    uint32_t pd_contract;
    char active_pdo_text[96];
    char vbus_text[16];
    char ibus_text[16];
    int32_t ibat_ma;
    bool pd_connected;

    if ((uint32_t)(now_ms - g_pm.last_debug_tick_ms) < POWER_MANAGER_DEBUG_PERIOD_MS) {
        return;
    }

    g_pm.last_debug_tick_ms = now_ms;

    PowerManager_UpdatePdSnapshot();
    pd_contract = (tps->active_rdo.valid) ? 1U : 0U;
    pd_connected = PowerManager_IsPdConnected(tps);
    PowerManager_DebugPdConnectionEvent(now_ms, pd_connected);

    if (bq->in_otg) {
        ibat_ma = -(int32_t)bq->idchg_ma;
    } else {
        ibat_ma = (int32_t)bq->ichg_ma;
    }
#if (BQ25731_CONTROL_OWNER_TPS_EEPROM != 0U)
    (void)ibat_ma;
#endif

#if (POWER_MANAGER_PD_RAW_DEBUG != 0U)
    Debug_Printf("[PM] state=%s bus=%s tps_addr=0x%02lX tps=%s bq=%s err_reg=0x%02lX err=0x%08lX",
                 PowerManager_StateText(g_pm.state),
                 PowerManager_I2cText(g_pm.hi2c),
                 (unsigned long)g_pm.tps_addr_selected,
                 TPS25751_StatusToString(g_pm.tps_status),
                 BQ25731_StatusToString(g_pm.bq_status),
                 (unsigned long)g_pm.last_error_reg,
                 (unsigned long)g_pm.last_error_code);
#else
    Debug_Printf("[PM] %s %s TPS=%s BQ=%s err=0x%08lX",
                 PowerManager_StateText(g_pm.state),
                 PowerManager_I2cText(g_pm.hi2c),
                 TPS25751_StatusToString(g_pm.tps_status),
                 BQ25731_StatusToString(g_pm.bq_status),
                 (unsigned long)g_pm.last_error_code);
#endif

    if (pd_contract != 0U) {
        PowerManager_FormatPdoShort(active_pdo_text,
                                    sizeof(active_pdo_text),
                                    tps->active_rdo.object_position,
                                    &tps->active_pdo);
    } else {
        (void)snprintf(active_pdo_text, sizeof(active_pdo_text), "none");
    }
    PowerManager_FormatVoltage(vbus_text, sizeof(vbus_text), tps->vbus_mv);
    PowerManager_FormatCurrent(ibus_text, sizeof(ibus_text), tps->ibus_ma);

    Debug_Printf("[PD] cable=%s %s %s/%s contract=%s active=%s VBUS=%s IBUS=%s CC=%s/%s PP1=%s PP3=%s OCP1=%u",
                 pd_connected ? "YES" : "NO",
                 TPS25751_ModeToString(tps->mode),
                 PowerManager_PortRoleText(tps),
                 PowerManager_DataRoleText(tps),
                 PowerManager_PdContractText(pd_contract),
                 active_pdo_text,
                 vbus_text,
                 ibus_text,
                 TPS25751_CcStateToString(tps->cc1_state),
                 TPS25751_CcStateToString(tps->cc2_state),
                 TPS25751_PowerPathSwitchToString(tps->pp1_switch),
                 TPS25751_PowerPathSwitchToString(tps->pp3_switch),
                 tps->pp1_overcurrent ? 1U : 0U);

    Debug_Printf("[PD-SNAPSHOT] attached=%u PDO=0x%08lX RDO=0x%08lX contract=%lumV/%lumA/%lumW class=%s",
                 g_pm.pd_snapshot.attached ? 1U : 0U,
                 (unsigned long)(g_pm.pd_snapshot.active_pdo_raw & 0xFFFFFFFFULL),
                 (unsigned long)g_pm.pd_snapshot.active_rdo_raw,
                 (unsigned long)g_pm.pd_snapshot.contract_voltage_mv,
                 (unsigned long)g_pm.pd_snapshot.contract_current_ma,
                 (unsigned long)g_pm.pd_snapshot.contract_power_mw,
                 PowerManager_PowerClassText(g_pm.pd_snapshot.power_class));

    PowerManager_DebugRdo(tps);

#if (BQ25731_CONTROL_OWNER_TPS_EEPROM != 0U)
    if (g_pm.bq_status == BQ25731_OK) {
        const BQ25731_MonitorSnapshot_t *m = &g_pm.bq_monitor;
        Debug_Printf("[BQ] adc=%s cfg: VREG=%lumV ICHG_SET=%lumA IIN_HOST=%lumA",
                     (m->adc_running && m->adc_required_channels_enabled) ?
                     "available" : "partial/unavailable",
                     (unsigned long)m->charge_voltage_setting_mv,
                     (unsigned long)m->charge_current_setting_ma,
                     (unsigned long)m->iin_host_ma);
    }
#else
    if (g_pm.bq_status == BQ25731_OK) {
        Debug_Printf("[BQ] adc=%u VBAT=%lu.%03luV VSYS=%lu.%03luV VBUS=%lu.%03luV IBAT=%ldmA IIN=%lumA CHG=%lumA/%lu.%03luV EN_OTG=%u IN_OTG=%u OPT3=0x%04X PIN=%u",
                     (unsigned int)bq->adc_enabled,
                     (unsigned long)PowerManager_IntPart(bq->vbat_mv, 1000U),
                     (unsigned long)PowerManager_FracPart(bq->vbat_mv, 1000U),
                     (unsigned long)PowerManager_IntPart(bq->vsys_mv, 1000U),
                     (unsigned long)PowerManager_FracPart(bq->vsys_mv, 1000U),
                     (unsigned long)PowerManager_IntPart(bq->vbus_mv, 1000U),
                     (unsigned long)PowerManager_FracPart(bq->vbus_mv, 1000U),
                     (long)ibat_ma,
                     (unsigned long)bq->iin_ma,
                     (unsigned long)bq->charge_current_ma,
                     (unsigned long)PowerManager_IntPart(bq->charge_voltage_mv, 1000U),
                     (unsigned long)PowerManager_FracPart(bq->charge_voltage_mv, 1000U),
                     bq->otg_enabled ? 1U : 0U,
                     bq->in_otg ? 1U : 0U,
                     bq->charge_option3_raw,
                     HAL_GPIO_ReadPin(OTG_EN_GPIO_Port, OTG_EN_Pin) == GPIO_PIN_SET ? 1U : 0U);
    }
#endif

    PowerManager_DebugPdCaps(now_ms, pd_connected);
    Debug_BlankLine();
}

void PowerManager_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_GPIO_WritePin(OTG_EN_GPIO_Port, OTG_EN_Pin, GPIO_PIN_RESET);
    memset(&g_pm, 0, sizeof(g_pm));

    g_pm.hi2c = hi2c;
    g_pm.state = POWER_MANAGER_INIT;
    g_pm.tps_status = TPS25751_ERROR;
    g_pm.bq_status = BQ25731_ERROR;
    g_pm.next_task_tick_ms = HAL_GetTick();
    g_pm.last_debug_tick_ms = HAL_GetTick();
    g_pm.last_pd_caps_debug_tick_ms = HAL_GetTick();
    g_pm.initialized = (hi2c != NULL);

    Debug_Printf("[PM] TPS25751 normal-operation I2Ct address: 0x%02X",
                 TPS25751_I2C_ADDR_DEFAULT);
    Debug_Printf("[BQ] BQ25731 OTG/VAP/FRS pin has pulldown; OTG disabled in firmware for bring-up.");
    Debug_Printf("[BQ-HW] BQ may be pre-configured by TPS/EEPROM");
    Debug_Printf("[BQ-HW] OTG/VAP/FRS pin: pulldown 100k to GND");
    Debug_Printf("[BQ-HW] OTG allowed: NO");
    Debug_Printf("[BQ-HW] RAC=%umOhm RSR=%umOhm", BQ25731_RAC_MOHM, BQ25731_RSR_MOHM);
    Debug_Printf("[BQ-HW] ILIM_HIZ/EN_EXTILIM not trusted during bring-up");
    Debug_Printf("[BQ-HW] ChargeVoltage may be configured by TPS EEPROM, e.g. 16800mV");

    if (!g_pm.initialized) {
        g_pm.state = POWER_MANAGER_FAULT;
    }
}

void PowerManager_Task(void)
{
    uint32_t now_ms;

    if (!g_pm.initialized) {
        return;
    }

    now_ms = HAL_GetTick();

    if ((int32_t)(now_ms - g_pm.next_task_tick_ms) < 0) {
#if (POWER_MANAGER_GENERAL_DEBUG != 0U)
        PowerManager_DebugLine(now_ms);
#endif
        return;
    }

    g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TASK_MIN_PERIOD_MS;

    switch (g_pm.state) {
        case POWER_MANAGER_INIT:
        {
            g_pm.tps_status = TPS25751_ERROR;
            g_pm.tps_addr_selected = 0U;

            g_pm.tps_status = PowerManager_ProbeTpsAddress(TPS25751_I2C_ADDR_DEFAULT);

            if (g_pm.tps_status == TPS25751_OK) {
                g_pm.tps_addr_selected = TPS25751_I2C_ADDR_DEFAULT;
                g_pm.tps_wait_start_tick_ms = now_ms;
                g_pm.last_tps_mode_log_tick_ms = 0U;
                g_pm.last_logged_tps_mode = TPS25751_MODE_UNKNOWN;
                Debug_Printf("[TPS] device detected at configured address 0x%02X",
                             TPS25751_I2C_ADDR_DEFAULT);
                g_pm.state = POWER_MANAGER_TPS_WAIT_APP;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TPS_POLL_MS;
            } else {
#if (POWER_MANAGER_DIAGNOSTIC_TPS_SCAN != 0U)
                uint8_t addr;
                for (addr = POWER_MANAGER_TPS_ADDR_MIN;
                     addr <= POWER_MANAGER_TPS_ADDR_MAX;
                     ++addr) {
                    if (addr == TPS25751_I2C_ADDR_DEFAULT) {
                        continue;
                    }
                    if (PowerManager_ProbeTpsAddress(addr) == TPS25751_OK) {
                        Debug_Printf("[TPS] diagnostic only: device found at 0x%02X; normal operation rejected",
                                     addr);
                        break;
                    }
                }
#endif
                PowerManager_SetErrorFromTps(&g_pm.tps);
                g_pm.last_error_code = POWER_MANAGER_FAULT_TPS_NOT_AT_CONFIGURED_ADDRESS;
                Debug_Printf("[FAULT] TPS25751 does not respond at configured address 0x%02X",
                             TPS25751_I2C_ADDR_DEFAULT);
                g_pm.state = POWER_MANAGER_FAULT;
            }
            break;
        }

        case POWER_MANAGER_TPS_WAIT_APP:
        {
            uint32_t wait_ms = (uint32_t)(now_ms - g_pm.tps_wait_start_tick_ms);
            TPS25751_Mode_t mode = TPS25751_MODE_UNKNOWN;
            char mode_ascii[5] = "????";

            g_pm.tps_status = TPS25751_ReadMode(&g_pm.tps, &mode, mode_ascii);
            if (g_pm.tps_status != TPS25751_OK) {
                PowerManager_SetErrorFromTps(&g_pm.tps);
                mode = TPS25751_MODE_UNKNOWN;
            }

            g_pm.tps_telemetry.mode = mode;
            memcpy(g_pm.tps_telemetry.mode_ascii, mode_ascii, sizeof(mode_ascii));
            g_pm.tps_telemetry.app_ready = (mode == TPS25751_MODE_APP);

            if ((mode != g_pm.last_logged_tps_mode) ||
                ((uint32_t)(now_ms - g_pm.last_tps_mode_log_tick_ms) >= 1000U)) {
                Debug_Printf("[TPS] MODE='%s' wait=%lums APP=%s",
                             mode_ascii,
                             (unsigned long)wait_ms,
                             (mode == TPS25751_MODE_APP) ? "YES" : "NO");
                g_pm.last_logged_tps_mode = mode;
                g_pm.last_tps_mode_log_tick_ms = now_ms;
            }

            if (mode == TPS25751_MODE_APP) {
                g_pm.tps_status = TPS25751_ReadTelemetryBasic(&g_pm.tps,
                                                               &g_pm.tps_telemetry);
                if (g_pm.tps_status != TPS25751_OK) {
                    PowerManager_SetErrorFromTps(&g_pm.tps);
                    g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TPS_POLL_MS;
                    break;
                }
                Debug_Printf("[TPS] APP reached after %lums; I2Cc bridge access enabled",
                             (unsigned long)wait_ms);
                g_pm.state = POWER_MANAGER_TPS_READY;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TASK_MIN_PERIOD_MS;
            } else if (wait_ms >= POWER_MANAGER_TPS_APP_TIMEOUT_MS) {
                g_pm.last_error_reg = POWER_MANAGER_TPS_PROBE_REG_MODE;
                g_pm.last_error_code = POWER_MANAGER_FAULT_TPS_APP_TIMEOUT;
                Debug_Printf("[FAULT] TPS APP timeout after %lums; last MODE='%s' (%s)",
                             (unsigned long)wait_ms,
                             mode_ascii,
                             TPS25751_ModeToString(mode));
                g_pm.state = POWER_MANAGER_FAULT;
            } else {
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TPS_POLL_MS;
            }
            break;
        }

        case POWER_MANAGER_TPS_READY:
            g_pm.bq_status = BQ25731_Init(&g_pm.bq, &g_pm.tps, BQ25731_I2C_ADDR_7BIT);
            if (g_pm.bq_status == BQ25731_OK) {
                g_pm.state = POWER_MANAGER_BQ_PROBE;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TASK_MIN_PERIOD_MS;
            } else {
                PowerManager_SetErrorFromBq(&g_pm.bq);
                g_pm.state = POWER_MANAGER_FAULT;
            }
            break;

        case POWER_MANAGER_BQ_PROBE:
            g_pm.tps_status = TPS25751_ReadTelemetryBasic(&g_pm.tps, &g_pm.tps_telemetry);
            if (g_pm.tps_status != TPS25751_OK) {
                PowerManager_SetErrorFromTps(&g_pm.tps);
                Debug_Printf("[FAULT] TPS telemetry failed before BQ probe");
                g_pm.state = POWER_MANAGER_FAULT;
                break;
            }

            if (g_pm.tps_telemetry.mode != TPS25751_MODE_APP) {
                g_pm.last_error_reg = POWER_MANAGER_TPS_PROBE_REG_MODE;
                g_pm.last_error_code = POWER_MANAGER_FAULT_TPS_LOST_APP;
                Debug_Printf("[FAULT] TPS is not in APP; BQ bridge probe blocked");
                g_pm.state = POWER_MANAGER_FAULT;
                break;
            }

            g_pm.bq_status = BQ25731_CheckDevice(&g_pm.bq);
            if (g_pm.bq_status != BQ25731_OK) {
                PowerManager_SetErrorFromBq(&g_pm.bq);
                memset(&g_pm.bq_safe_start, 0, sizeof(g_pm.bq_safe_start));
                g_pm.bq_safe_start.fatal_error = true;
                g_pm.bq_safe_start.error = BQ_ERR_PROBE_FAILED;
                g_pm.last_error_code = BQ_ERR_PROBE_FAILED;
                Debug_Printf("[PM-ERR] BQ fatal=1 err=BQ_ERR_PROBE_FAILED");
                g_pm.state = POWER_MANAGER_DEGRADED_BQ;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_BQ_PROBE_RETRY_MS;
                break;
            }

            Debug_Printf("[BQ] probe OK: manufacturer=0x40 device=BQ25730/BQ25731");
#if (BQ25731_CONTROL_OWNER_TPS_EEPROM != 0U)
            Debug_Printf("[BQ-MON] power-policy owner=TPS/EEPROM; STM32 may repair CHGOPT0 and enable monitoring ADC");
#if (BQ25731_ALLOW_STM32_ADC_ENABLE != 0U)
            g_pm.bq_adc_next_attempt_ms = now_ms + POWER_MANAGER_BQ_ADC_START_DELAY_MS;
            Debug_Printf("[BQ-ADC-ENABLE] waiting %ums for TPS/EEPROM charger configuration to finish before ADC takeover",
                         POWER_MANAGER_BQ_ADC_START_DELAY_MS);
#endif
            g_pm.bq_status = BQ25731_OK;
            g_pm.state = POWER_MANAGER_SAFE_MONITORING;
            g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TASK_MIN_PERIOD_MS;
#else
            g_pm.state = POWER_MANAGER_BQ_SAFE_START;
            g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TASK_MIN_PERIOD_MS;
#endif
            break;

        case POWER_MANAGER_BQ_SAFE_START:
        {
#if (BQ25731_BRIDGE_WRITE_TEST != 0U)
            uint16_t bridge_old = 0U;
            uint16_t bridge_new = 0U;
#endif
            Debug_Printf("[BQ] charging inhibited during initialization");
#if (BQ25731_BRIDGE_WRITE_TEST != 0U)
            g_pm.bq_status = BQ25731_BridgeWriteSelfTest(&g_pm.bq,
                                                          &bridge_old,
                                                          &bridge_new);
            if (g_pm.bq_status != BQ25731_OK) {
                memset(&g_pm.bq_safe_start, 0, sizeof(g_pm.bq_safe_start));
                g_pm.bq_safe_start.fatal_error = true;
                g_pm.bq_safe_start.error = BQ_ERR_BRIDGE_WRITE_FAILED;
                g_pm.bq_safe_start.charge_current_raw = bridge_new;
                g_pm.bq_safe_start.charge_current_ma =
                    BQ25731_DecodeChargeCurrent(bridge_new, BQ25731_RSR_MOHM == 5U);
                g_pm.last_error_code = BQ_ERR_BRIDGE_WRITE_FAILED;
                Debug_Printf("[PM-ERR] BQ fatal=1 err=BQ_ERR_BRIDGE_WRITE_FAILED detail=ChargeCurrent readback mismatch old=0x%04X new=0x%04X",
                             bridge_old, bridge_new);
                g_pm.state = POWER_MANAGER_DEGRADED_BQ;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_MONITOR_PERIOD_MS;
                break;
            }
#endif
            g_pm.bq_status = BQ25731_TakeoverSafeState(&g_pm.bq, &g_pm.bq_safe_start);
            if (g_pm.bq_safe_start.fatal_error) {
                PowerManager_SetErrorFromBq(&g_pm.bq);
                g_pm.last_error_code = (uint32_t)g_pm.bq_safe_start.error;
                Debug_Printf("[PM-ERR] BQ fatal=1 err=%s raw=0x%04X decoded=%lumA",
                             PowerManager_BqSafeErrorText(g_pm.bq_safe_start.error),
                             g_pm.bq_safe_start.charge_current_raw,
                             (unsigned long)g_pm.bq_safe_start.charge_current_ma);
                g_pm.state = POWER_MANAGER_DEGRADED_BQ;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_MONITOR_PERIOD_MS;
                break;
            }

            Debug_Printf("[PM-ERR] BQ fatal=0 warnings=0x%08lX err=BQ_ERR_NONE",
                         (unsigned long)g_pm.bq_safe_start.warnings);

            Debug_Printf("[BQ] safe state applied: CHRG_INHIBIT=1 EN_OTG=0 IIN=%umA ICHG=0mA warnings=0x%08lX",
                         BQ25731_SAFE_INPUT_CURRENT_MA,
                         (unsigned long)g_pm.bq_safe_start.warnings);
            Debug_Printf("[BQ] OTG blocked by firmware for Digital PD PSU V2.0 hardware");

            g_pm.bq_status = BQ25731_SetSenseResistors(&g_pm.bq,
                                                        BQ25731_RAC_MOHM == 5U,
                                                        BQ25731_RSR_MOHM == 5U);
            /* Program the battery-safe CV target while charging is still
             * inhibited and ICHG=0. This removes the 21 V 5S POR/EEPROM
             * value even when USB-C is present before the battery. */
            if (g_pm.bq_status == BQ25731_OK) {
                g_pm.bq_status = BQ25731_SetChargeVoltage(
                    &g_pm.bq, BQ_USER_CHARGE_VOLTAGE_MV, NULL, NULL);
            }
            if (g_pm.bq_status == BQ25731_OK) {
                g_pm.bq_status = BQ25731_ConfigureForMonitoring(&g_pm.bq);
            }
            if (g_pm.bq_status == BQ25731_OK) {
                g_pm.bq_status = BQ25731_ApplyUserOptions(&g_pm.bq);
            }
            if (g_pm.bq_status != BQ25731_OK) {
                PowerManager_SetErrorFromBq(&g_pm.bq);
                Debug_Printf("[BQ] monitoring configuration failed: %s",
                             BQ25731_StatusToString(g_pm.bq_status));
                g_pm.state = POWER_MANAGER_DEGRADED_BQ;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_MONITOR_PERIOD_MS;
                break;
            }
            Debug_Printf("[BQ] STM32 owns charger; safe idle VREG=%lumV ICHG=0mA inhibit=1 ADC=continuous",
                         (unsigned long)BQ_USER_CHARGE_VOLTAGE_MV);
            g_pm.state = POWER_MANAGER_SAFE_MONITORING;
            g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_MONITOR_PERIOD_MS;
            break;
        }

        case POWER_MANAGER_SAFE_MONITORING:
        case POWER_MANAGER_DEGRADED_BQ:
            g_pm.tps_status = TPS25751_ReadTelemetryBasic(&g_pm.tps, &g_pm.tps_telemetry);
            if (g_pm.tps_status != TPS25751_OK) {
                PowerManager_SetErrorFromTps(&g_pm.tps);
                g_pm.state = POWER_MANAGER_TPS_WAIT_APP;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TPS_POLL_MS;
                break;
            }

            if (g_pm.tps_telemetry.mode != TPS25751_MODE_APP) {
                g_pm.last_error_reg = POWER_MANAGER_TPS_PROBE_REG_MODE;
                g_pm.last_error_code = POWER_MANAGER_FAULT_TPS_LOST_APP;
                Debug_Printf("[FAULT] TPS left APP during BQ monitoring; bridge disabled");
                g_pm.state = POWER_MANAGER_FAULT;
                break;
            }

            PowerManager_UpdatePdSnapshot();
            PowerManager_AutoSourcePolicy();

#if (POWER_MANAGER_PD_REQUEST_MAX_POWER != 0U)
            if (!g_pm.pd_snapshot.attached) {
                g_pm.pd_max_power_attempted = false;
            } else if ((!g_pm.pd_max_power_attempted) &&
                       (g_pm.tps_telemetry.active_pdo.type == TPS25751_SUPPLY_FIXED) &&
                       (g_pm.tps_telemetry.active_pdo.voltage_mv == 20000U) &&
                       (g_pm.tps_telemetry.active_pdo.current_ma >= 5000U) &&
                       (g_pm.pd_snapshot.contract_power_mw < 90000U)) {
                TPS25751_Status_t max_power_status;
                g_pm.pd_max_power_attempted = true;
                Debug_Printf("[PD-MAX] source/local policy supports 20V/5A; enabling AutoComputeSinkMinPower and requesting new source capabilities");
                max_power_status = TPS25751_RequestMaxSinkPower(&g_pm.tps, 3000U);
                Debug_Printf("[PD-MAX] renegotiation %s",
                             TPS25751_StatusToString(max_power_status));
                g_pm.tps_status = TPS25751_ReadTelemetryBasic(&g_pm.tps,
                                                               &g_pm.tps_telemetry);
                if (g_pm.tps_status == TPS25751_OK)
                    PowerManager_UpdatePdSnapshot();
            }
#endif

            if (g_pm.state == POWER_MANAGER_SAFE_MONITORING) {
                PowerManager_UpdateBqChargingPolicy();
            }

            if (g_pm.state == POWER_MANAGER_DEGRADED_BQ) {
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_MONITOR_PERIOD_MS;
                break;
            }

#if ((BQ25731_CONTROL_OWNER_TPS_EEPROM != 0U) && \
     (BQ25731_ALLOW_STM32_ADC_ENABLE != 0U))
            if ((!g_pm.bq_adc_running_confirmed) &&
                (g_pm.bq_adc_attempts < POWER_MANAGER_BQ_ADC_MAX_ATTEMPTS) &&
                ((int32_t)(now_ms - g_pm.bq_adc_next_attempt_ms) >= 0)) {
                ++g_pm.bq_adc_attempts;
                Debug_Printf("[BQ-ADC-ENABLE] delayed attempt %u/%u",
                             g_pm.bq_adc_attempts,
                             POWER_MANAGER_BQ_ADC_MAX_ATTEMPTS);
                g_pm.bq_status = BQ25731_EnableMonitoringAdcOnly(&g_pm.bq);
                if (g_pm.bq_status == BQ25731_OK) {
                    g_pm.bq_adc_running_confirmed = true;
                    Debug_Printf("[BQ-ADC-ENABLE] continuous ADC confirmed by readback");
                } else {
                    g_pm.bq_adc_next_attempt_ms = HAL_GetTick() +
                                                  POWER_MANAGER_BQ_ADC_RETRY_MS;
                    Debug_Printf("[BQ-ADC-ENABLE] attempt failed: %s; retry in %ums",
                                 BQ25731_StatusToString(g_pm.bq_status),
                                 POWER_MANAGER_BQ_ADC_RETRY_MS);
                }
            }
#endif

            if ((uint32_t)(now_ms - g_pm.last_bq_monitor_tick_ms) < 1000U) {
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_MONITOR_PERIOD_MS;
                break;
            }
            g_pm.last_bq_monitor_tick_ms = now_ms;

#if (BQ25731_CONTROL_OWNER_TPS_EEPROM != 0U)
            g_pm.bq_status = BQ25731_ReadMonitorSnapshot(&g_pm.bq, &g_pm.bq_monitor);
            if (g_pm.bq_status == BQ25731_OK) {
                const BQ25731_MonitorSnapshot_t *m = &g_pm.bq_monitor;
#if (BQ25731_ALLOW_STM32_ADC_ENABLE != 0U)
                if (!m->adc_running || !m->adc_required_channels_enabled) {
                    if (g_pm.bq_adc_running_confirmed) {
                        Debug_Printf("[BQ-ADC-ENABLE] TPS changed ADCOption after confirmation; scheduling re-apply");
                        g_pm.bq_adc_running_confirmed = false;
                        g_pm.bq_adc_attempts = 0U;
                        g_pm.bq_adc_next_attempt_ms = now_ms +
                                                     POWER_MANAGER_BQ_ADC_RETRY_MS;
                    }
                }
#endif
                Debug_Printf("[BQ-MON] cfg: VREG=%lumV ICHG_SET=%lumA IIN_HOST=%lumA CHGOPT0=0x%04X CHGOPT3=0x%04X",
                             (unsigned long)m->charge_voltage_setting_mv,
                             (unsigned long)m->charge_current_setting_ma,
                             (unsigned long)m->iin_host_ma,
                             m->charge_option0_raw, m->charge_option3_raw);
                Debug_Printf("[BQ-ADC-OPT] raw=0x%04X low=0x%02X high=0x%02X VBAT=%u VSYS=%u ICHG=%u IDCHG=%u IIN=%u PSYS=%u VBUS=%u CMPIN=%u CONV=%u START=%u FULLSCALE=%u",
                             m->adc_option_raw, m->adc_option_raw & 0xFFU,
                             m->adc_option_raw >> 8, m->en_adc_vbat,
                             m->en_adc_vsys, m->en_adc_ichg, m->en_adc_idchg,
                             m->en_adc_iin, m->en_adc_psys, m->en_adc_vbus,
                             m->en_adc_cmpin, m->adc_continuous, m->adc_start,
                             m->adc_fullscale);
                Debug_Printf("[BQ-ADC] raw: VBUS/PSYS=0x%04X IBAT=0x%04X IIN/CMPIN=0x%04X VSYS/VBAT=0x%04X",
                             m->adc_vbus_psys_raw, m->adc_ibat_raw,
                             m->adc_iin_cmpin_raw, m->adc_vsys_vbat_raw);
                if (m->low_power_mode != 0U) {
                    Debug_Printf("[BQ-ADC] BQ is in low-power mode; ADC disabled according to datasheet; live telemetry unavailable unless TPS/EEPROM enables performance/ADC");
                } else if (m->adc_any_channel_enabled == 0U) {
                    Debug_Printf("[BQ-ADC] ADC channels are disabled by TPS/EEPROM; live telemetry unavailable in read-only owner mode");
                } else if (!m->adc_running) {
                    if (m->adc_option_raw == 0x2001U)
                        Debug_Printf("[BQ-ADC] live telemetry unavailable because TPS/EEPROM enabled only VBAT and conversion is not running");
                    else
                        Debug_Printf("[BQ-ADC] live telemetry unavailable because TPS/EEPROM enabled only selected channels and conversion is not running");
                } else if (m->adc_required_channels_enabled) {
                    Debug_Printf("[BQ-ADC] live: VBUS=%lumV VSYS=%lumV VBAT=%lumV ICHG=%lumA IDCHG=%lumA IIN=%lumA",
                                 (unsigned long)m->adc_vbus_mv,
                                 (unsigned long)m->adc_vsys_mv,
                                 (unsigned long)m->adc_vbat_mv,
                                 (unsigned long)m->adc_ichg_ma,
                                 (unsigned long)m->adc_idchg_ma,
                                 (unsigned long)m->adc_iin_ma);
                } else {
                    Debug_Printf("[BQ-ADC] live telemetry partial; TPS/EEPROM did not enable all monitoring channels");
                }
                if (!m->vbat_valid) Debug_Printf("[BQ-ADC] VBAT unavailable");
                if (!m->vsys_valid) Debug_Printf("[BQ-ADC] VSYS unavailable");
            }
#else
            g_pm.bq_status = BQ25731_ReadTelemetry(&g_pm.bq, &g_pm.bq_telemetry);
#endif
            if (g_pm.bq_status != BQ25731_OK) {
                PowerManager_SetErrorFromBq(&g_pm.bq);
                Debug_Printf("[BQ] WARNING telemetry failed in SAFE_MONITORING: %s",
                             BQ25731_StatusToString(g_pm.bq_status));
            }
#if (BQ25731_CONTROL_OWNER_TPS_EEPROM == 0U)
            else if ((g_pm.bq_telemetry.in_otg && !g_pm.bq_otg_enabled) ||
                       ((!g_pm.bq_charging_enabled) && !g_pm.bq_otg_enabled &&
                        (g_pm.bq_telemetry.in_fast_charge ||
                         !g_pm.bq_telemetry.charge_inhibited))) {
                Debug_Printf("[FAULT] unsafe BQ state in monitoring: OTG=%u FCHRG=%u inhibit=%u",
                             g_pm.bq_telemetry.in_otg,
                             g_pm.bq_telemetry.in_fast_charge,
                             g_pm.bq_telemetry.charge_inhibited);
                g_pm.state = POWER_MANAGER_FAULT;
            }
#endif

#if (POWER_MANAGER_PD_CYCLE_TEST != 0U)
            PowerManager_PdCycleTestTask(now_ms);
#endif

            g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_MONITOR_PERIOD_MS;
            break;

        case POWER_MANAGER_FAULT:
            PowerManager_DisableBqOtg();
            /* A BQ fault must not freeze USB-C PD observability.  Continue
             * read-only TPS monitoring so hot-plug and detach reach the GUI. */
            if ((g_pm.tps_addr_selected == TPS25751_I2C_ADDR_DEFAULT) &&
                (g_pm.tps_telemetry.mode == TPS25751_MODE_APP)) {
                g_pm.tps_status = TPS25751_ReadTelemetryBasic(&g_pm.tps,
                                                               &g_pm.tps_telemetry);
                if ((g_pm.tps_status == TPS25751_OK) &&
                    (g_pm.tps_telemetry.mode == TPS25751_MODE_APP)) {
                    PowerManager_UpdatePdSnapshot();
                } else {
                    memset(&g_pm.pd_snapshot, 0, sizeof(g_pm.pd_snapshot));
                }
            } else {
                memset(&g_pm.pd_snapshot, 0, sizeof(g_pm.pd_snapshot));
            }
            g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_MONITOR_PERIOD_MS;
            break;

        default:
            /* Latched safe fault: no BQ commands and no automatic retry. */
            break;
    }

#if (POWER_MANAGER_GENERAL_DEBUG != 0U)
    PowerManager_DebugLine(now_ms);
#endif
}

PowerManager_State_t PowerManager_GetState(void)
{
    return g_pm.state;
}

BQ25731_Status_t PowerManager_GetBqStatus(void)
{
    return g_pm.bq_status;
}

void PowerManager_GetStatus(PowerManager_Status_t *status)
{
    if (status == NULL) {
        return;
    }

    status->state = g_pm.state;
    status->tps_status = g_pm.tps_status;
    status->bq_status = g_pm.bq_status;
    status->tps_telemetry = g_pm.tps_telemetry;
    status->bq_telemetry = g_pm.bq_telemetry;
    status->bq_monitor = g_pm.bq_monitor;
    status->bq_safe_start = g_pm.bq_safe_start;
    status->pd_snapshot = g_pm.pd_snapshot;
    status->last_error_reg = g_pm.last_error_reg;
    status->last_error_code = g_pm.last_error_code;
}

bool PowerManager_GetPdSnapshot(PowerManager_PdSnapshot_t *out)
{
    uint32_t primask;

    if (out == NULL) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *out = g_pm.pd_snapshot;
    if (primask == 0U) {
        __enable_irq();
    }

    return out->attached &&
           (out->active_rdo_raw != 0U) &&
           (out->contract_voltage_mv != 0U) &&
           (out->contract_power_mw != 0U);
}

bool PowerManager_IsPdCycleTestEnabled(void)
{
    return (POWER_MANAGER_PD_CYCLE_TEST != 0U);
}
