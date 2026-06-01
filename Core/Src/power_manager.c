#include "power_manager.h"

#include "debug_uart.h"

#include <string.h>

#define POWER_MANAGER_TASK_MIN_PERIOD_MS   20U
#define POWER_MANAGER_TPS_POLL_MS          200U
#define POWER_MANAGER_BQ_PROBE_RETRY_MS    1000U
#define POWER_MANAGER_MONITOR_PERIOD_MS    120U
#define POWER_MANAGER_DEBUG_PERIOD_MS      500U
#define POWER_MANAGER_ERROR_RETRY_MS       1500U
#define POWER_MANAGER_TPS_ADDR_MIN         0x20U
#define POWER_MANAGER_TPS_ADDR_MAX         0x27U
#define POWER_MANAGER_TPS_PROBE_REG_MODE   0x03U

typedef struct {
    I2C_HandleTypeDef *hi2c;
    TPS25751_Device_t tps;
    BQ25731_Device_t bq;
    TPS25751_Telemetry_t tps_telemetry;
    BQ25731_Telemetry_t bq_telemetry;
    TPS25751_Status_t tps_status;
    BQ25731_Status_t bq_status;
    PowerManager_State_t state;
    uint32_t next_task_tick_ms;
    uint32_t last_debug_tick_ms;
    uint32_t last_error_reg;
    uint32_t last_error_code;
    uint8_t tps_addr_selected;
    bool initialized;
} PowerManager_Context_t;

static PowerManager_Context_t g_pm;

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
        case POWER_MANAGER_BQ_MONITOR:
            return "BQ_MONITOR";
        case POWER_MANAGER_ERROR:
        default:
            return "ERROR";
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

static void PowerManager_DebugLine(uint32_t now_ms)
{
    const TPS25751_Telemetry_t *tps = &g_pm.tps_telemetry;
    const BQ25731_Telemetry_t *bq = &g_pm.bq_telemetry;
    uint32_t pd_contract;
    uint32_t pdo_raw;
    uint32_t rdo_raw;
    int32_t ibat_ma;

    if ((uint32_t)(now_ms - g_pm.last_debug_tick_ms) < POWER_MANAGER_DEBUG_PERIOD_MS) {
        return;
    }

    g_pm.last_debug_tick_ms = now_ms;

    pd_contract = (tps->active_rdo.valid) ? 1U : 0U;
    pdo_raw = (uint32_t)(tps->active_pdo_raw & 0xFFFFFFFFULL);
    rdo_raw = tps->active_rdo_raw;

    if (bq->in_otg) {
        ibat_ma = -(int32_t)bq->idchg_ma;
    } else {
        ibat_ma = (int32_t)bq->ichg_ma;
    }

    Debug_Printf("PM=%s BUS=%s TPS_ADDR=0x%02lX PD_MODE=%s PD_CONTRACT=%lu PDO=0x%08lX RDO=0x%08lX VBUS=%lumV IBUS=%lumA "
                 "BQ_STATUS=%s BQ_ADC=%u VBAT=%lumV VSYS=%lumV BQ_VBUS=%lumV IBAT=%ldmA IIN=%lumA "
                 "ICHG_SET=%lumA VCHG_SET=%lumV ERR_REG=0x%02lX ERR=0x%08lX",
                 PowerManager_StateText(g_pm.state),
                 PowerManager_I2cText(g_pm.hi2c),
                 (unsigned long)g_pm.tps_addr_selected,
                 TPS25751_ModeToString(tps->mode),
                 (unsigned long)pd_contract,
                 (unsigned long)pdo_raw,
                 (unsigned long)rdo_raw,
                 (unsigned long)tps->vbus_mv,
                 (unsigned long)tps->ibus_ma,
                 BQ25731_StatusToString(g_pm.bq_status),
                 (unsigned int)bq->adc_enabled,
                 (unsigned long)bq->vbat_mv,
                 (unsigned long)bq->vsys_mv,
                 (unsigned long)bq->vbus_mv,
                 (long)ibat_ma,
                 (unsigned long)bq->iin_ma,
                 (unsigned long)bq->charge_current_ma,
                 (unsigned long)bq->charge_voltage_mv,
                 (unsigned long)g_pm.last_error_reg,
                 (unsigned long)g_pm.last_error_code);
}

void PowerManager_Init(I2C_HandleTypeDef *hi2c)
{
    memset(&g_pm, 0, sizeof(g_pm));

    g_pm.hi2c = hi2c;
    g_pm.state = POWER_MANAGER_INIT;
    g_pm.tps_status = TPS25751_ERROR;
    g_pm.bq_status = BQ25731_ERROR;
    g_pm.next_task_tick_ms = HAL_GetTick();
    g_pm.last_debug_tick_ms = HAL_GetTick();
    g_pm.initialized = (hi2c != NULL);

    if (!g_pm.initialized) {
        g_pm.state = POWER_MANAGER_ERROR;
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
        PowerManager_DebugLine(now_ms);
        return;
    }

    g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TASK_MIN_PERIOD_MS;

    switch (g_pm.state) {
        case POWER_MANAGER_INIT:
        {
            uint8_t addr;

            g_pm.tps_status = TPS25751_ERROR;
            g_pm.tps_addr_selected = 0U;

            for (addr = POWER_MANAGER_TPS_ADDR_MIN;
                 addr <= POWER_MANAGER_TPS_ADDR_MAX;
                 addr++) {
                g_pm.tps_status = PowerManager_ProbeTpsAddress(addr);
                if (g_pm.tps_status == TPS25751_OK) {
                    g_pm.tps_addr_selected = addr;
                    break;
                }
            }

            if (g_pm.tps_status == TPS25751_OK) {
                g_pm.state = POWER_MANAGER_TPS_WAIT_APP;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TPS_POLL_MS;
            } else {
                PowerManager_SetErrorFromTps(&g_pm.tps);
                g_pm.state = POWER_MANAGER_ERROR;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_ERROR_RETRY_MS;
            }
            break;
        }

        case POWER_MANAGER_TPS_WAIT_APP:
            g_pm.tps_status = TPS25751_ReadTelemetry(&g_pm.tps, &g_pm.tps_telemetry);
            if (g_pm.tps_status != TPS25751_OK) {
                PowerManager_SetErrorFromTps(&g_pm.tps);
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TPS_POLL_MS;
                break;
            }

            if (g_pm.tps_telemetry.mode == TPS25751_MODE_APP) {
                g_pm.state = POWER_MANAGER_TPS_READY;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TASK_MIN_PERIOD_MS;
            } else {
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TPS_POLL_MS;
            }
            break;

        case POWER_MANAGER_TPS_READY:
            g_pm.bq_status = BQ25731_Init(&g_pm.bq, &g_pm.tps, BQ25731_I2C_ADDR_7BIT);
            if (g_pm.bq_status == BQ25731_OK) {
                g_pm.state = POWER_MANAGER_BQ_PROBE;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TASK_MIN_PERIOD_MS;
            } else {
                PowerManager_SetErrorFromBq(&g_pm.bq);
                g_pm.state = POWER_MANAGER_ERROR;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_ERROR_RETRY_MS;
            }
            break;

        case POWER_MANAGER_BQ_PROBE:
            g_pm.tps_status = TPS25751_ReadTelemetry(&g_pm.tps, &g_pm.tps_telemetry);
            if (g_pm.tps_status != TPS25751_OK) {
                PowerManager_SetErrorFromTps(&g_pm.tps);
                g_pm.state = POWER_MANAGER_TPS_WAIT_APP;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TPS_POLL_MS;
                break;
            }

            g_pm.bq_status = BQ25731_CheckDevice(&g_pm.bq);
            if (g_pm.bq_status != BQ25731_OK) {
                PowerManager_SetErrorFromBq(&g_pm.bq);
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_BQ_PROBE_RETRY_MS;
                break;
            }

            g_pm.bq_status = BQ25731_ConfigureForMonitoring(&g_pm.bq);
            if (g_pm.bq_status != BQ25731_OK) {
                PowerManager_SetErrorFromBq(&g_pm.bq);
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_BQ_PROBE_RETRY_MS;
                break;
            }

            g_pm.state = POWER_MANAGER_BQ_MONITOR;
            g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_MONITOR_PERIOD_MS;
            break;

        case POWER_MANAGER_BQ_MONITOR:
            g_pm.tps_status = TPS25751_ReadTelemetry(&g_pm.tps, &g_pm.tps_telemetry);
            if (g_pm.tps_status != TPS25751_OK) {
                PowerManager_SetErrorFromTps(&g_pm.tps);
                g_pm.state = POWER_MANAGER_TPS_WAIT_APP;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TPS_POLL_MS;
                break;
            }

            if (g_pm.tps_telemetry.mode != TPS25751_MODE_APP) {
                g_pm.state = POWER_MANAGER_TPS_WAIT_APP;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_TPS_POLL_MS;
                break;
            }

            g_pm.bq_status = BQ25731_ReadTelemetry(&g_pm.bq, &g_pm.bq_telemetry);
            if (g_pm.bq_status != BQ25731_OK) {
                PowerManager_SetErrorFromBq(&g_pm.bq);
                g_pm.state = POWER_MANAGER_BQ_PROBE;
                g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_BQ_PROBE_RETRY_MS;
                break;
            }

            g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_MONITOR_PERIOD_MS;
            break;

        case POWER_MANAGER_ERROR:
        default:
            g_pm.state = POWER_MANAGER_INIT;
            g_pm.next_task_tick_ms = now_ms + POWER_MANAGER_ERROR_RETRY_MS;
            break;
    }

    PowerManager_DebugLine(now_ms);
}

PowerManager_State_t PowerManager_GetState(void)
{
    return g_pm.state;
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
    status->last_error_reg = g_pm.last_error_reg;
    status->last_error_code = g_pm.last_error_code;
}
