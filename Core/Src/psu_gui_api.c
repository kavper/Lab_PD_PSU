#include "psu_gui_api.h"

#include "app.h"
#include "debug_uart.h"
#include "power_manager.h"
#include "power_stage.h"

#include <stdint.h>

#define PSU_GUI_VOLTAGE_MAX_V 30.0f
#define PSU_GUI_CURRENT_MAX_A 5.8f

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

void PSU_GuiInit(void)
{
}

void PSU_GuiReset(void)
{
    PSU_Stop();
}

void PSU_GuiSetTargetVoltage(float voltage_v)
{
    App_SetCvSetpoint(clampf(voltage_v, 0.0f, PSU_GUI_VOLTAGE_MAX_V));
}

void PSU_GuiSetTargetCurrent(float current_a)
{
    App_SetCurrentLimit(clampf(current_a, 0.0f, PSU_GUI_CURRENT_MAX_A));
}

float PSU_GuiGetTargetVoltage(void)
{
    return App_GetCvSetpoint();
}

float PSU_GuiGetTargetCurrent(void)
{
    return App_GetCurrentLimit();
}

float PSU_GuiGetInputVoltage(void)
{
    return App_GetInputVoltage();
}

float PSU_GuiGetOutputVoltage(void)
{
    return App_GetOutputVoltage();
}

float PSU_GuiGetOutputCurrent(void)
{
    return App_GetOutputCurrent();
}

float PSU_GuiGetBoostVoltage(void)
{
    return App_GetInputVoltage();
}

float PSU_GuiGetInputCurrent(void)
{
    return 0.0f;
}

float PSU_GuiGetSlewedSetpointVoltage(void)
{
    return App_GetCvRampedSetpoint();
}

float PSU_GuiGetDuty(void)
{
    return PowerStage_GetDutyA();
}

static bool PSU_GuiReadPdSnapshot(PowerManager_PdSnapshot_t *snapshot)
{
    bool valid = PowerManager_GetPdSnapshot(snapshot);

#if (PSU_GUI_PD_DEBUG != 0U)
    static uint32_t last_log_ms = 0U;
    uint32_t now_ms = HAL_GetTick();
    const char *pm_state;

    switch (PowerManager_GetState()) {
        case POWER_MANAGER_INIT:         pm_state = "INIT"; break;
        case POWER_MANAGER_TPS_WAIT_APP: pm_state = "TPS_WAIT_APP"; break;
        case POWER_MANAGER_TPS_READY:    pm_state = "TPS_READY"; break;
        case POWER_MANAGER_BQ_PROBE:     pm_state = "BQ_PROBE"; break;
        case POWER_MANAGER_BQ_MONITOR:   pm_state = "BQ_MONITOR"; break;
        case POWER_MANAGER_FAULT:
        default:                         pm_state = "FAULT"; break;
    }

    if ((uint32_t)(now_ms - last_log_ms) >= 1000U) {
        if (valid) {
            Debug_Printf("[GUI-PD] getter sees: valid=1 V=%lumV I=%lumA P=%lumW attached=%u RDO=0x%08lX bq=%s pm_state=%s",
                         (unsigned long)snapshot->contract_voltage_mv,
                         (unsigned long)snapshot->contract_current_ma,
                         (unsigned long)snapshot->contract_power_mw,
                         snapshot->attached ? 1U : 0U,
                         (unsigned long)snapshot->active_rdo_raw,
                         BQ25731_StatusToString(PowerManager_GetBqStatus()),
                         pm_state);
        } else {
            const char *reason = "unknown";
            if (!snapshot->attached) {
                reason = "not_attached";
            } else if (snapshot->active_rdo_raw == 0U) {
                reason = "no_active_rdo";
            } else if (snapshot->contract_voltage_mv == 0U) {
                reason = "zero_contract_voltage";
            } else if (snapshot->contract_power_mw == 0U) {
                reason = "zero_contract_power";
            }
            Debug_Printf("[GUI-PD] valid=0 reason=%s attached=%u RDO=0x%08lX V=%lumV I=%lumA P=%lumW bq=%s pm_state=%s",
                         reason,
                         snapshot->attached ? 1U : 0U,
                         (unsigned long)snapshot->active_rdo_raw,
                         (unsigned long)snapshot->contract_voltage_mv,
                         (unsigned long)snapshot->contract_current_ma,
                         (unsigned long)snapshot->contract_power_mw,
                         BQ25731_StatusToString(PowerManager_GetBqStatus()),
                         pm_state);
        }
        last_log_ms = now_ms;
    }
#endif

    return valid;
}

uint8_t PSU_GuiGetPdContract(float *voltage_v,
                             float *current_a,
                             float *power_w,
                             uint32_t *active_rdo_raw)
{
    PowerManager_PdSnapshot_t snapshot;
    bool valid = PSU_GuiReadPdSnapshot(&snapshot);

    if (voltage_v != NULL) {
        *voltage_v = valid ? ((float)snapshot.contract_voltage_mv / 1000.0f) : 0.0f;
    }
    if (current_a != NULL) {
        *current_a = valid ? ((float)snapshot.contract_current_ma / 1000.0f) : 0.0f;
    }
    if (power_w != NULL) {
        *power_w = valid ? ((float)snapshot.contract_power_mw / 1000.0f) : 0.0f;
    }
    if (active_rdo_raw != NULL) {
        *active_rdo_raw = valid ? snapshot.active_rdo_raw : 0U;
    }

    return valid ? 1U : 0U;
}

float PSU_GuiGetPdContractVoltage(void)
{
    float voltage_v = 0.0f;
    (void)PSU_GuiGetPdContract(&voltage_v, NULL, NULL, NULL);
    return voltage_v;
}

float PSU_GuiGetPdContractCurrent(void)
{
    float current_a = 0.0f;
    (void)PSU_GuiGetPdContract(NULL, &current_a, NULL, NULL);
    return current_a;
}

float PSU_GuiGetPdContractPower(void)
{
    float power_w = 0.0f;
    (void)PSU_GuiGetPdContract(NULL, NULL, &power_w, NULL);
    return power_w;
}

uint8_t PSU_GuiIsPdContractValid(void)
{
    return PSU_GuiGetPdContract(NULL, NULL, NULL, NULL);
}

uint8_t PSU_GuiGetTransferPower(float *power_w)
{
    PowerManager_Status_t status;
    int32_t selected_power_mw = 0;
    bool valid = false;

    PowerManager_GetStatus(&status);
    if (status.bq.online && status.bq.adc_sample_valid &&
        status.tps.attached &&
        (status.tps.connection_state >= 6U) &&
        status.tps.active_pdo.valid &&
        status.tps.active_rdo.valid) {
        if ((status.tps.role == TPS25751_ROLE_SINK) &&
            (!status.bq.in_otg) &&
            (status.bq.battery_current_ma > 0)) {
            /* Charging: use the battery-side ADC result. */
            selected_power_mw = status.bq.battery_power_mw;
            valid = true;
        } else if ((status.tps.role == TPS25751_ROLE_SOURCE) &&
                   status.bq.in_otg &&
                   (status.bq.battery_current_ma < 0)) {
            /* OTG: RAC/ADCIIN is 100 mA/LSB versus IDCHG at 512 mA/LSB. */
            selected_power_mw = -(int32_t)status.bq.input_power_mw;
            valid = true;
        }
    }

    if (power_w != NULL) {
        *power_w = valid ? ((float)selected_power_mw / 1000.0f) : 0.0f;
    }

    return valid ? 1U : 0U;
}

PSU_GuiControlMode_t PSU_GuiGetControlMode(void)
{
    if (App_GetRequestedMode() == MODE_IDLE) {
        return PSU_GUI_CONTROL_MODE_OFF;
    }

    if (App_GetRequestedMode() == MODE_CC) {
        return PSU_GUI_CONTROL_MODE_CC;
    }

    return PSU_GUI_CONTROL_MODE_CV;
}

void PSU_Start(void)
{
    if (PowerManager_IsPdCycleTestEnabled()) {
        return;
    }

    App_ClearFaults();
    App_SetRequestedMode(MODE_CV);
}

void PSU_Stop(void)
{
    App_SetRequestedMode(MODE_IDLE);
}

uint8_t PSU_IsRunning(void)
{
    return (App_GetRequestedMode() != MODE_IDLE) ? 1U : 0U;
}

uint8_t PSU_IsCurrentLimitActive(void)
{
    return (App_GetMode() == MODE_CC) ? 1U : 0U;
}
