#include "psu_gui_api.h"

#include "app.h"
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
