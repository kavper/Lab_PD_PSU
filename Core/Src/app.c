#include "app.h"
#include "debug_uart.h"
#include "tps25751.h"

#include <string.h>

#define APP_TPS_I2C_ADDRESS_7BIT              TPS25751_I2C_ADDR_DEFAULT

#define APP_HEARTBEAT_MS                      500U
#define APP_TPS_POLL_MS                       250U
#define APP_DEBUG_PRINT_MS                    1000U

typedef struct {
    TPS25751_Device_t tps;
    TPS25751_Telemetry_t tps_telemetry;

    I2C_HandleTypeDef *hi2c_tps;
    UART_HandleTypeDef *huart_debug;

    uint32_t last_heartbeat_ms;
    uint32_t last_tps_poll_ms;
    uint32_t last_debug_print_ms;

    bool initialized;
    bool tps_online;
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

static void App_TpsPollTask(uint32_t now)
{
    TPS25751_Status_t status;

    if (!App_IsDue(now, app.last_tps_poll_ms, APP_TPS_POLL_MS)) {
        return;
    }

    app.last_tps_poll_ms = now;

    status = TPS25751_ReadTelemetry(&app.tps, &app.tps_telemetry);
    app.tps_online = (status == TPS25751_OK);

    if (status != TPS25751_OK) {
        app.tps_telemetry.status = status;
    }

    App_TpsStatusLedUpdate();
}

static void App_PrintHeader(uint32_t now)
{
    Debug_Printf("\r\n");
    Debug_Printf("============================================================\r\n");
    Debug_Printf(" USB-C PD DEBUG | t=%lu ms | TPS=%s | I2C=0x%02X | %s\r\n",
                 (unsigned long)now,
                 app.tps_online ? "ONLINE" : "OFFLINE",
                 app.tps.address_7bit,
                 TPS25751_StatusToString(app.tps_telemetry.status));
    Debug_Printf("================================================------------\r\n");
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
    Debug_Printf("  ibus=");
    App_PrintCurrent(t->ibus_ma);
    Debug_Printf("  mean=");
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

static void App_PrintNotes(const TPS25751_Telemetry_t *t)
{
    Debug_Printf("  NOTE       ");

    if (!app.tps_online) {
        Debug_Printf("TPS nie odpowiada. Sprawdz I2C, adres, zasilanie i reset.\r\n");
        return;
    }

    if (!t->app_ready) {
        Debug_Printf("TPS odpowiada, ale nie jest w APP. Jesli widzisz ascii=\"APP \", debug juz naprawiony.\r\n");
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

    Debug_Printf("TPS aktualnie dziala jako SINK. Patrz RX_SOURCE i aktywny kontrakt.\r\n");
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

    Debug_Printf("------------------------------------------------------------\r\n");
    Debug_Printf(" RAW SUMMARY\r\n");
    Debug_Printf("------------------------------------------------------------\r\n");

    Debug_Printf("  raw_status=0x%010llX  raw_power_path=0x%010llX\r\n",
                 (unsigned long long)t->status_raw,
                 (unsigned long long)t->power_path_raw);

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

    Debug_Init(huart_debug);

    Debug_Printf("\r\n\r\n");
    Debug_Printf("============================================================\r\n");
    Debug_Printf(" Digital PSU firmware bring-up\r\n");
    Debug_Printf(" Stage 2: readable TPS25751 USB-C PD debug\r\n");
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
    App_DebugPrintTask(now);
}