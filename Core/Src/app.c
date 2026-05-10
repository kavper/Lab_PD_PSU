#include "app.h"
#include "debug_uart.h"
#include "tps25751.h"
#include "bq25731.h"

#include <string.h>

#define APP_TPS_I2C_ADDRESS_7BIT              TPS25751_I2C_ADDR_DEFAULT

#define APP_HEARTBEAT_MS                      500U
#define APP_TPS_POLL_MS                       250U
#define APP_BQ_POLL_MS                        5000U
#define APP_DEBUG_PRINT_MS                    1000U

#define APP_BQ_DEBUG_ENABLE                   1U

#define APP_BQ_ADDRESS_7BIT                   BQ25731_I2C_ADDR_7BIT
#define APP_BQ_RSNS_RAC_IS_5MOHM              1U
#define APP_BQ_RSNS_RSR_IS_5MOHM              1U
#define APP_BQ_VSYS_VBAT_5S_RANGE             0U
#define APP_BQ_IIN_MIN_MA                     100U
#define APP_BQ_IIN_MAX_FROM_PD_MA             5000U
#define APP_BQ_RECONFIG_AFTER_ERRORS          4U

#define APP_PD_ROLE_MODE_AUTO                 0U
#define APP_PD_ROLE_MODE_SINK_ONLY            1U
#define APP_PD_ROLE_MODE_SOURCE_ONLY          2U
#define APP_PD_ROLE_MODE                      APP_PD_ROLE_MODE_AUTO

typedef struct {
    TPS25751_Device_t tps;
    TPS25751_Telemetry_t tps_telemetry;

#if (APP_BQ_DEBUG_ENABLE != 0U)
    BQ25731_Device_t bq;
    BQ25731_Telemetry_t bq_telemetry;
#endif

    I2C_HandleTypeDef *hi2c_tps;
    UART_HandleTypeDef *huart_debug;

    uint32_t last_heartbeat_ms;
    uint32_t last_tps_poll_ms;
#if (APP_BQ_DEBUG_ENABLE != 0U)
    uint32_t last_bq_poll_ms;
#endif
    uint32_t last_debug_print_ms;

    bool initialized;
    bool tps_online;
    bool tps_role_mode_applied;
    TPS25751_TypecStateMachine_t tps_role_mode_current;

#if (APP_BQ_DEBUG_ENABLE != 0U)
    bool bq_online;
    bool bq_read_attempted;
    bool bq_initialized;
    bool bq_configured;
    bool bq_iin_target_valid;
    uint32_t bq_iin_target_ma;
    uint8_t bq_error_streak;
#endif
} App_Context_t;

static App_Context_t app;

static bool App_IsDue(uint32_t now, uint32_t last_tick, uint32_t period_ms)
{
    return ((uint32_t)(now - last_tick) >= period_ms);
}

static const char *App_BoolText(bool value)
{
    return value ? "YES" : "NO";
}

static void App_PrintVoltage(uint32_t mv)
{
    Debug_Printf("%lu.%03lu V",
                 (unsigned long)(mv / 1000U),
                 (unsigned long)(mv % 1000U));
}

static void App_PrintCurrent(uint32_t ma)
{
    Debug_Printf("%lu.%03lu A",
                 (unsigned long)(ma / 1000U),
                 (unsigned long)(ma % 1000U));
}

static void App_PrintPower(uint32_t mw)
{
    Debug_Printf("%lu.%03lu W",
                 (unsigned long)(mw / 1000U),
                 (unsigned long)(mw % 1000U));
}

static void App_PrintPdo(const TPS25751_PdoInfo_t *pdo)
{
    if ((pdo == NULL) || (pdo->raw == 0U)) {
        Debug_Printf("EMPTY");
        return;
    }

    Debug_Printf("%s ", TPS25751_PdoTypeToString(pdo->type));

    if (pdo->type == TPS25751_SUPPLY_FIXED) {
        App_PrintVoltage(pdo->voltage_mv);
        Debug_Printf(" / ");
        App_PrintCurrent(pdo->current_ma);
        Debug_Printf(" / ");
        App_PrintPower(pdo->power_mw);
        return;
    }

    if ((pdo->type == TPS25751_SUPPLY_VARIABLE) ||
        (pdo->type == TPS25751_SUPPLY_APDO_PPS)) {
        App_PrintVoltage(pdo->min_mv);
        Debug_Printf("-");
        App_PrintVoltage(pdo->max_mv);
        Debug_Printf(" / ");
        App_PrintCurrent(pdo->current_ma);
        Debug_Printf(" / ");
        App_PrintPower(pdo->power_mw);
        return;
    }

    if (pdo->type == TPS25751_SUPPLY_BATTERY) {
        App_PrintVoltage(pdo->min_mv);
        Debug_Printf("-");
        App_PrintVoltage(pdo->max_mv);
        Debug_Printf(" / ");
        App_PrintPower(pdo->power_mw);
        return;
    }

    Debug_Printf("raw=0x%08lX", (unsigned long)pdo->raw);
}

static void App_PrintPdoList(const char *title, const TPS25751_PdoList_t *list)
{
    uint8_t i;

    if ((title == NULL) || (list == NULL)) {
        return;
    }

    Debug_Printf("  %-10s count=%u\r\n", title, list->count);

    if (list->count == 0U) {
        Debug_Printf("             brak PDO\r\n");
        return;
    }

    for (i = 0U; i < list->count; ++i) {
        Debug_Printf("             PDO%u: ", (unsigned int)(i + 1U));
        App_PrintPdo(&list->pdo[i]);
        Debug_Printf("   raw=0x%08lX\r\n", (unsigned long)list->pdo[i].raw);
    }
}

static void App_HeartbeatTask(uint32_t now)
{
    if (!App_IsDue(now, app.last_heartbeat_ms, APP_HEARTBEAT_MS)) {
        return;
    }

    app.last_heartbeat_ms = now;
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
}

static void App_TpsStatusLedUpdate(void)
{
    HAL_GPIO_WritePin(LED2_GPIO_Port,
                      LED2_Pin,
                      app.tps_online ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static TPS25751_TypecStateMachine_t App_TpsDesiredTypecStateMachine(void)
{
#if (APP_PD_ROLE_MODE == APP_PD_ROLE_MODE_SINK_ONLY)
    return TPS25751_TYPEC_SM_SINK_ONLY;
#elif (APP_PD_ROLE_MODE == APP_PD_ROLE_MODE_SOURCE_ONLY)
    return TPS25751_TYPEC_SM_SOURCE_ONLY;
#else
    return TPS25751_TYPEC_SM_DRP;
#endif
}

static const char *App_TpsRoleModeText(TPS25751_TypecStateMachine_t mode)
{
    switch (mode) {
        case TPS25751_TYPEC_SM_SINK_ONLY:
            return "SINK_ONLY";
        case TPS25751_TYPEC_SM_SOURCE_ONLY:
            return "SOURCE_ONLY";
        case TPS25751_TYPEC_SM_DRP:
            return "AUTO_DRP";
        case TPS25751_TYPEC_SM_DISABLED:
            return "DISABLED";
        default:
            return "UNKNOWN";
    }
}

static bool App_TpsRolePolicyTask(void)
{
    TPS25751_Status_t status;
    TPS25751_TypecStateMachine_t desired_mode;

    if (!app.tps_online || !app.tps_telemetry.app_ready) {
        return false;
    }

    desired_mode = App_TpsDesiredTypecStateMachine();

    if (app.tps_role_mode_applied &&
        (app.tps_role_mode_current == desired_mode)) {
        return false;
    }

    status = TPS25751_SetTypecStateMachine(&app.tps, desired_mode);
    if (status != TPS25751_OK) {
        Debug_Printf("[TPS] Role mode set failed: %s\r\n",
                     TPS25751_StatusToString(status));
        return false;
    }

    app.tps_role_mode_current = desired_mode;
    app.tps_role_mode_applied = true;

    Debug_Printf("[TPS] Role mode applied: %s\r\n",
                 App_TpsRoleModeText(desired_mode));

    return true;
}

static void App_TpsPollTask(uint32_t now)
{
    TPS25751_Status_t status;
    TPS25751_Telemetry_t temp;

    if (!App_IsDue(now, app.last_tps_poll_ms, APP_TPS_POLL_MS)) {
        return;
    }

    app.last_tps_poll_ms = now;

    memset(&temp, 0, sizeof(temp));

    status = TPS25751_ReadTelemetry(&app.tps, &temp);

    if (status == TPS25751_OK) {
        app.tps_telemetry = temp;
        app.tps_online = true;
    } else {
        app.tps_online = false;
        app.tps_telemetry.status = status;
    }

    App_TpsStatusLedUpdate();
}

#if (APP_BQ_DEBUG_ENABLE != 0U)
static bool App_BqHasFault(const BQ25731_Telemetry_t *bq)
{
    if (bq == NULL) {
        return false;
    }

    return bq->fault_acov ||
           bq->fault_batoc ||
           bq->fault_acoc ||
           bq->fault_sysovp ||
           bq->fault_vsys_uvp ||
           bq->fault_force_converter_off ||
           bq->fault_otg_ovp ||
           bq->fault_otg_uvp;
}

static uint32_t App_BqDesiredInputCurrentMa(const TPS25751_Telemetry_t *tps)
{
    uint32_t contract_ma = 0U;
    uint32_t advertised_ma = 0U;
    uint8_t pdo_index = 0xFFU;

    if (tps == NULL) {
        return 0U;
    }

    if (!tps->power_connection) {
        return 0U;
    }

    if (tps->active_rdo.valid) {
        contract_ma = tps->active_rdo.max_current_ma;
        if (contract_ma == 0U) {
            contract_ma = tps->active_rdo.operating_current_ma;
        }
    }

    advertised_ma = tps->active_pdo.current_ma;
    if ((contract_ma == 0U) && (advertised_ma > 0U)) {
        contract_ma = advertised_ma;
    }

    if (tps->active_rdo.object_position > 0U) {
        pdo_index = (uint8_t)(tps->active_rdo.object_position - 1U);
    }

    if ((contract_ma == 0U) &&
        (pdo_index < tps->rx_source_caps.count)) {
        contract_ma = tps->rx_source_caps.pdo[pdo_index].current_ma;
    }

    if (contract_ma == 0U) {
        return 0U;
    }

    if (contract_ma < APP_BQ_IIN_MIN_MA) {
        contract_ma = APP_BQ_IIN_MIN_MA;
    }

    if (contract_ma > APP_BQ_IIN_MAX_FROM_PD_MA) {
        contract_ma = APP_BQ_IIN_MAX_FROM_PD_MA;
    }

    return contract_ma;
}

static BQ25731_Status_t App_BqApplyInputCurrentLimit(uint32_t desired_ma)
{
    BQ25731_Status_t status;

    if (desired_ma == 0U) {
        return BQ25731_INVALID_ARG;
    }

    if (app.bq_iin_target_valid &&
        (app.bq_iin_target_ma == desired_ma)) {
        return BQ25731_OK;
    }

    status = BQ25731_DisableExternalInputCurrentLimit(&app.bq, NULL, NULL);
    if (status != BQ25731_OK) {
        return status;
    }

    status = BQ25731_SetInputCurrentLimit(&app.bq, desired_ma, NULL, NULL);
    if (status != BQ25731_OK) {
        return status;
    }

    app.bq_iin_target_ma = desired_ma;
    app.bq_iin_target_valid = true;

    Debug_Printf("[BQ] IIN_HOST target updated to %lu mA (PD contract)\r\n",
                 (unsigned long)desired_ma);

    return BQ25731_OK;
}

static void App_BqPollTask(uint32_t now)
{
    BQ25731_Status_t status;
    BQ25731_Telemetry_t temp;
    uint32_t desired_iin_ma;

    if (!App_IsDue(now, app.last_bq_poll_ms, APP_BQ_POLL_MS)) {
        return;
    }

    app.last_bq_poll_ms = now;
    app.bq_read_attempted = true;

    if (!app.tps_telemetry.app_ready) {
        app.bq_online = false;
        app.bq_telemetry.status = BQ25731_TPS_ERROR;
        app.bq_telemetry.address_7bit = APP_BQ_ADDRESS_7BIT;
        return;
    }

    if (!app.bq_initialized) {
        status = BQ25731_Init(&app.bq,
                              &app.tps,
                              APP_BQ_ADDRESS_7BIT);
        if (status != BQ25731_OK) {
            app.bq_online = false;
            app.bq_telemetry.status = status;
            app.bq_telemetry.address_7bit = APP_BQ_ADDRESS_7BIT;
            return;
        }

        BQ25731_SetVsysVbatRange5S(&app.bq,
                                   (APP_BQ_VSYS_VBAT_5S_RANGE != 0U));

        app.bq_initialized = true;
        app.bq_configured = false;
        app.bq_iin_target_valid = false;
        app.bq_error_streak = 0U;
    }

    if (!app.bq_configured) {
        status = BQ25731_CheckDevice(&app.bq);
        if (status != BQ25731_OK) {
            app.bq_online = false;
            app.bq_telemetry.status = status;
            app.bq_telemetry.address_7bit = APP_BQ_ADDRESS_7BIT;
            return;
        }

        status = BQ25731_SetSenseResistors(&app.bq,
                                           (APP_BQ_RSNS_RAC_IS_5MOHM != 0U),
                                           (APP_BQ_RSNS_RSR_IS_5MOHM != 0U));
        if (status != BQ25731_OK) {
            app.bq_online = false;
            app.bq_telemetry.status = status;
            app.bq_telemetry.address_7bit = APP_BQ_ADDRESS_7BIT;
            return;
        }

        status = BQ25731_ConfigureForMonitoring(&app.bq);
        if (status != BQ25731_OK) {
            app.bq_online = false;
            app.bq_telemetry.status = status;
            app.bq_telemetry.address_7bit = APP_BQ_ADDRESS_7BIT;
            return;
        }

        desired_iin_ma = App_BqDesiredInputCurrentMa(&app.tps_telemetry);
        if (desired_iin_ma > 0U) {
            status = App_BqApplyInputCurrentLimit(desired_iin_ma);
            if (status != BQ25731_OK) {
                app.bq_online = false;
                app.bq_telemetry.status = status;
                app.bq_telemetry.address_7bit = APP_BQ_ADDRESS_7BIT;
                return;
            }
        }

        app.bq_configured = true;
        app.bq_error_streak = 0U;
        Debug_Printf("[BQ] Monitor config OK: RAC=%s RSR=%s\r\n",
                     (APP_BQ_RSNS_RAC_IS_5MOHM != 0U) ? "5m" : "10m",
                     (APP_BQ_RSNS_RSR_IS_5MOHM != 0U) ? "5m" : "10m");
    }

    memset(&temp, 0, sizeof(temp));

    status = BQ25731_ReadTelemetry(&app.bq, &temp);

    if (status == BQ25731_OK) {
        desired_iin_ma = App_BqDesiredInputCurrentMa(&app.tps_telemetry);
        if (desired_iin_ma > 0U) {
            status = App_BqApplyInputCurrentLimit(desired_iin_ma);
            if (status != BQ25731_OK) {
                app.bq_online = false;
                app.bq_configured = false;
                app.bq_iin_target_valid = false;
                app.bq_telemetry.status = status;
                app.bq_telemetry.address_7bit = APP_BQ_ADDRESS_7BIT;
                return;
            }
        }

        app.bq_telemetry = temp;
        app.bq_online = true;
        app.bq_error_streak = 0U;
    } else {
        app.bq_online = false;
        if (app.bq_error_streak < 0xFFU) {
            app.bq_error_streak++;
        }

        if (app.bq_error_streak >= APP_BQ_RECONFIG_AFTER_ERRORS) {
            app.bq_configured = false;
            app.bq_iin_target_valid = false;
            app.bq_error_streak = 0U;
            Debug_Printf("[BQ] Reconfig scheduled after repeated read errors.\r\n");
        }

        app.bq_telemetry.status = status;
        app.bq_telemetry.address_7bit = APP_BQ_ADDRESS_7BIT;
    }
}
#endif

static void App_PrintHeader(uint32_t now)
{
    Debug_Printf("\r\n");
    Debug_Printf("============================================================\r\n");
    Debug_Printf(" USB-C PD DEBUG | t=%lu ms | TPS=%s | I2C=0x%02X | %s\r\n",
                 (unsigned long)now,
                 app.tps_online ? "ONLINE" : "OFFLINE",
                 app.tps.address_7bit,
                 TPS25751_StatusToString(app.tps_telemetry.status));
    Debug_Printf("============================================================\r\n");
}

static void App_PrintMainStatus(const TPS25751_Telemetry_t *t)
{
    Debug_Printf("  MODE       %s  ascii=\"%s\"  app_ready=%s\r\n",
                 TPS25751_ModeToString(t->mode),
                 t->mode_ascii,
                 App_BoolText(t->app_ready));

    Debug_Printf("  ATTACH     plug=%s  power=%s  conn=%s\r\n",
                 App_BoolText(t->plug_present),
                 App_BoolText(t->power_connection),
                 TPS25751_ConnectionStateToString(t->connection_state));

    Debug_Printf("  ROLE       TypeC=%s  PD=%s  Data=%s\r\n",
                 t->port_role_source ? "SOURCE" : "SINK",
                 t->pd_role_source ? "SOURCE" : "SINK",
                 t->data_role_dfp ? "DFP" : "UFP");

    Debug_Printf("  VBUS       status=%s  adc=",
                 TPS25751_VbusStatusToString(t->vbus_status));
    App_PrintVoltage(t->vbus_mv);
    Debug_Printf("  ibus_pp5v=");
    App_PrintCurrent(t->ibus_ma);
    Debug_Printf("  mean_pp5v=");
    App_PrintCurrent(t->ibus_mean_ma);
    Debug_Printf("\r\n");

    Debug_Printf("  CC         pd_pin=%u  CC1=%s  CC2=%s  orient=%s\r\n",
                 t->cc_pin_for_pd,
                 TPS25751_CcStateToString(t->cc1_state),
                 TPS25751_CcStateToString(t->cc2_state),
                 t->orientation_cc2 ? "CC2" : "CC1");

    Debug_Printf("  TYPEC      state=0x%02X %s  current=%s\r\n",
                 t->typec_state,
                 TPS25751_TypecStateToString(t->typec_state),
                 TPS25751_TypecCurrentToString(t->typec_current_status));

    Debug_Printf("  PATH       PP1=%s  PP3=%s  VCONN=%s  power_source=%u\r\n",
                 TPS25751_PowerPathSwitchToString(t->pp1_switch),
                 TPS25751_PowerPathSwitchToString(t->pp3_switch),
                 TPS25751_PowerPathSwitchToString(t->ppcable_switch),
                 t->power_source);

    Debug_Printf("  FAULT      pp1_oc=%s  ppcable_oc=%s  hard_reset=%u  soft_reset=%u\r\n",
                 App_BoolText(t->pp1_overcurrent),
                 App_BoolText(t->ppcable_overcurrent),
                 t->hard_reset_reason,
                 t->soft_reset_reason);
}

static void App_PrintContract(const TPS25751_Telemetry_t *t)
{
    Debug_Printf("  CONTRACT   ");

    if ((t->active_pdo.raw == 0U) || !t->active_rdo.valid) {
        Debug_Printf("brak aktywnego kontraktu PD\r\n");
        return;
    }

    Debug_Printf("PDO%u  ",
                 (unsigned int)t->active_rdo.object_position);
    App_PrintPdo(&t->active_pdo);

    Debug_Printf("  requested=");
    App_PrintCurrent(t->active_rdo.operating_current_ma);
    Debug_Printf("  max=");
    App_PrintCurrent(t->active_rdo.max_current_ma);

    Debug_Printf("  mismatch=%s",
                 App_BoolText(t->active_rdo.capability_mismatch));

    Debug_Printf("\r\n");
}

#if (APP_BQ_DEBUG_ENABLE != 0U)
static void App_PrintBqDebug(void)
{
    const BQ25731_Telemetry_t *bq = &app.bq_telemetry;
    uint8_t bq_addr = app.bq_initialized ? app.bq.device_address : APP_BQ_ADDRESS_7BIT;

    Debug_Printf("------------------------------------------------------------\r\n");
    Debug_Printf(" BQ25731 DEBUG | %s | addr=0x%02X | %s\r\n",
                 app.bq_online ? "ONLINE" : "OFFLINE",
                 bq_addr,
                 BQ25731_StatusToString(bq->status));
    Debug_Printf("------------------------------------------------------------\r\n");

    if (!app.bq_read_attempted) {
        Debug_Printf("  NOTE      BQ read not attempted yet. Waiting for first polling window.\r\n");
        return;
    }

    if (!app.bq_online) {
        Debug_Printf("  NOTE      BQ read failed or TPS I2Cr is not ready. No valid BQ data printed.\r\n");
        if (app.bq_initialized) {
            char diagnostic[96];
            if (BQ25731_GetDiagnosticText(&app.bq, diagnostic, sizeof(diagnostic)) > 0) {
                Debug_Printf("  DIAG      %s\r\n", diagnostic);
            }
        }
        return;
    }

    Debug_Printf("  ID        mfg=0x%02X dev=0x%02X ok=%s\r\n",
                 bq->manufacturer_id,
                 bq->device_id,
                 App_BoolText(bq->id_ok));

    Debug_Printf("  CFG       opt0=0x%04X opt1=0x%04X opt2=0x%04X opt3=0x%04X adc=0x%04X\r\n",
                 bq->charge_option0_raw,
                 bq->charge_option1_raw,
                 bq->charge_option2_raw,
                 bq->charge_option3_raw,
                 bq->adc_option_raw);

    Debug_Printf("  SENSE     RAC=%s RSR=%s ext_ilim=%s hiz=%s inhibit=%s watchdog=%s low_power=%s\r\n",
                 bq->rsns_rac_5mohm ? "5m" : "10m",
                 bq->rsns_rsr_5mohm ? "5m" : "10m",
                 App_BoolText(bq->external_input_current_limit_enabled),
                 App_BoolText(bq->hiz_enabled),
                 App_BoolText(bq->charge_inhibited),
                 App_BoolText(bq->watchdog_enabled),
                 App_BoolText(bq->low_power_mode));

    Debug_Printf("  LIMITS    IIN_HOST=");
    App_PrintCurrent(bq->iin_host_ma);
    Debug_Printf(" IIN_DPM=");
    App_PrintCurrent(bq->iin_dpm_ma);
    Debug_Printf(" ICHG_REG=");
    App_PrintCurrent(bq->charge_current_ma);
    Debug_Printf(" VCHG_REG=");
    App_PrintVoltage(bq->charge_voltage_mv);
    Debug_Printf("\r\n");

    Debug_Printf("  OTG_REG   VOTG=");
    App_PrintVoltage(bq->otg_voltage_mv);
    Debug_Printf(" IOTG=");
    App_PrintCurrent(bq->otg_current_ma);
    Debug_Printf(" otg_en=%s in_otg=%s\r\n",
                 App_BoolText(bq->otg_enabled),
                 App_BoolText(bq->in_otg));

    Debug_Printf("  ADC       VBUS=");
    App_PrintVoltage(bq->vbus_mv);
    Debug_Printf(" VBAT=");
    App_PrintVoltage(bq->vbat_mv);
    Debug_Printf(" VSYS=");
    App_PrintVoltage(bq->vsys_mv);
    Debug_Printf("\r\n");

    Debug_Printf("            IIN=");
    App_PrintCurrent(bq->iin_ma);
    Debug_Printf(" ICHG=");
    App_PrintCurrent(bq->ichg_ma);
    Debug_Printf(" IDCHG=");
    App_PrintCurrent(bq->idchg_ma);
    Debug_Printf(" PSYS_ADC=%lumV\r\n",
                 (unsigned long)bq->psys_mv);

    Debug_Printf("  POWER     input=");
    App_PrintPower(bq->input_power_mw);
    Debug_Printf(" charge=");
    App_PrintPower(bq->charge_power_mw);
    Debug_Printf("\r\n");

    Debug_Printf("  STATE     input=%s fast=%s iin_dpm=%s vindpm=%s vap=%s ico=%s\r\n",
                 App_BoolText(bq->input_present),
                 App_BoolText(bq->in_fast_charge),
                 App_BoolText(bq->in_iin_dpm),
                 App_BoolText(bq->in_vindpm),
                 App_BoolText(bq->in_vap),
                 App_BoolText(bq->ico_done));

    Debug_Printf("  FAULTS    any=%s acov=%s batoc=%s acoc=%s sysovp=%s vsys_uvp=%s force_off=%s otg_ovp=%s otg_uvp=%s\r\n",
                 App_BoolText(App_BqHasFault(bq)),
                 App_BoolText(bq->fault_acov),
                 App_BoolText(bq->fault_batoc),
                 App_BoolText(bq->fault_acoc),
                 App_BoolText(bq->fault_sysovp),
                 App_BoolText(bq->fault_vsys_uvp),
                 App_BoolText(bq->fault_force_converter_off),
                 App_BoolText(bq->fault_otg_ovp),
                 App_BoolText(bq->fault_otg_uvp));
}
#endif

static void App_PrintNotes(const TPS25751_Telemetry_t *t)
{
    Debug_Printf("  NOTE       ");

    if (!app.tps_online) {
        Debug_Printf("TPS nie odpowiada. Sprawdz I2C, adres, zasilanie i reset.\r\n");
        return;
    }

    if (!t->app_ready) {
        Debug_Printf("TPS odpowiada, ale nie jest w APP. Sprawdz EEPROM/config TPS.\r\n");
        return;
    }

    if (!t->plug_present) {
        Debug_Printf("Brak kabla USB-C. TPS zyje, ale nie ma attach.\r\n");
        return;
    }

    if (!t->power_connection) {
        Debug_Printf("Kabel jest, ale brak polaczenia mocy. Patrz CC, TypeC state i power path.\r\n");
        return;
    }

    if (t->pd_role_source) {
        Debug_Printf("TPS aktualnie dziala jako SOURCE. Patrz TX_SOURCE i RX_SINK.\r\n");
        return;
    }

#if (APP_BQ_DEBUG_ENABLE != 0U)
    Debug_Printf("TPS aktualnie dziala jako SINK. Patrz RX_SOURCE, aktywny kontrakt i BQ debug.\r\n");
#else
    Debug_Printf("TPS aktualnie dziala jako SINK. BQ debug jest chwilowo wylaczony.\r\n");
#endif
}

static void App_DebugPrintTask(uint32_t now)
{
    const TPS25751_Telemetry_t *t = &app.tps_telemetry;

    if (!App_IsDue(now, app.last_debug_print_ms, APP_DEBUG_PRINT_MS)) {
        return;
    }

    app.last_debug_print_ms = now;

    App_PrintHeader(now);
    App_PrintMainStatus(t);
    App_PrintContract(t);

    Debug_Printf("------------------------------------------------------------\r\n");
    Debug_Printf(" PDO TABLES\r\n");
    Debug_Printf("------------------------------------------------------------\r\n");

    App_PrintPdoList("RX_SOURCE", &t->rx_source_caps);
    App_PrintPdoList("RX_SINK", &t->rx_sink_caps);
    App_PrintPdoList("TX_SOURCE", &t->tx_source_caps);
    App_PrintPdoList("TX_SINK", &t->tx_sink_caps);

#if (APP_BQ_DEBUG_ENABLE != 0U)
    App_PrintBqDebug();
#endif

    Debug_Printf("------------------------------------------------------------\r\n");
    Debug_Printf(" RAW SUMMARY\r\n");
    Debug_Printf("------------------------------------------------------------\r\n");

    Debug_Printf("  raw_status=0x%02lX%08lX  raw_power_path=0x%02lX%08lX\r\n",
                 (unsigned long)((t->status_raw >> 32) & 0xFFUL),
                 (unsigned long)(t->status_raw & 0xFFFFFFFFUL),
                 (unsigned long)((t->power_path_raw >> 32) & 0xFFUL),
                 (unsigned long)(t->power_path_raw & 0xFFFFFFFFUL));

    Debug_Printf("  raw_power=0x%04lX  raw_pd=0x%08lX  raw_typec=0x%08lX\r\n",
                 (unsigned long)t->power_status_raw,
                 (unsigned long)t->pd_status_raw,
                 (unsigned long)t->typec_raw);

    Debug_Printf("  last_i2c reg=0x%02X req_len=%u reported_len=%u err=0x%08lX\r\n",
                 app.tps.last_register,
                 app.tps.last_requested_payload_len,
                 app.tps.last_reported_payload_len,
                 (unsigned long)app.tps.last_error);

    Debug_Printf("------------------------------------------------------------\r\n");
    App_PrintNotes(t);
}

void App_Init(I2C_HandleTypeDef *hi2c_tps,
              UART_HandleTypeDef *huart_debug)
{
    uint32_t now = HAL_GetTick();
    TPS25751_Status_t status;

    memset(&app, 0, sizeof(app));

    app.hi2c_tps = hi2c_tps;
    app.huart_debug = huart_debug;

    app.last_heartbeat_ms = now;
    app.last_tps_poll_ms = now - APP_TPS_POLL_MS;
    app.last_debug_print_ms = now - APP_DEBUG_PRINT_MS;

#if (APP_BQ_DEBUG_ENABLE != 0U)
    app.last_bq_poll_ms = now - APP_BQ_POLL_MS;
    app.bq_telemetry.address_7bit = APP_BQ_ADDRESS_7BIT;
    app.bq_telemetry.status = BQ25731_TPS_ERROR;
#endif

    Debug_Init(huart_debug);

    Debug_Printf("\r\n\r\n");
    Debug_Printf("============================================================\r\n");
    Debug_Printf(" Digital PSU firmware bring-up\r\n");
#if (APP_BQ_DEBUG_ENABLE != 0U)
    Debug_Printf(" Stage 3: readable TPS25751 + BQ25731 debug\r\n");
#else
    Debug_Printf(" Stage 3: readable TPS25751 debug, BQ disabled\r\n");
#endif
    Debug_Printf(" PD role policy: %s\r\n",
                 App_TpsRoleModeText(App_TpsDesiredTypecStateMachine()));
    Debug_Printf("============================================================\r\n");

    status = TPS25751_Init(&app.tps,
                           hi2c_tps,
                           APP_TPS_I2C_ADDRESS_7BIT);

    app.initialized = (status == TPS25751_OK);

    if (app.initialized) {
        Debug_Printf("[APP] Init OK. TPS address=0x%02X\r\n",
                     APP_TPS_I2C_ADDRESS_7BIT);
    } else {
        Debug_Printf("[APP] Init FAILED. status=%s\r\n",
                     TPS25751_StatusToString(status));
    }

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
}

void App_Run(void)
{
    uint32_t now = HAL_GetTick();

    App_HeartbeatTask(now);

    if (!app.initialized) {
        App_DebugPrintTask(now);
        return;
    }

    App_TpsPollTask(now);

#if (APP_BQ_DEBUG_ENABLE != 0U)
    if (!App_TpsRolePolicyTask()) {
        App_BqPollTask(now);
    }
#else
    (void)App_TpsRolePolicyTask();
#endif

    App_DebugPrintTask(now);
}
