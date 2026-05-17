#ifndef PSU_GUI_API_H
#define PSU_GUI_API_H

#include <stdint.h>

typedef enum {
    PSU_GUI_CONTROL_MODE_OFF = 0,
    PSU_GUI_CONTROL_MODE_CV,
    PSU_GUI_CONTROL_MODE_CC
} PSU_GuiControlMode_t;

void PSU_GuiInit(void);
void PSU_GuiReset(void);

void PSU_GuiSetTargetVoltage(float voltage_v);
void PSU_GuiSetTargetCurrent(float current_a);

float PSU_GuiGetTargetVoltage(void);
float PSU_GuiGetTargetCurrent(void);
float PSU_GuiGetInputVoltage(void);
float PSU_GuiGetOutputVoltage(void);
float PSU_GuiGetOutputCurrent(void);
float PSU_GuiGetBoostVoltage(void);
float PSU_GuiGetInputCurrent(void);
float PSU_GuiGetSlewedSetpointVoltage(void);
float PSU_GuiGetDuty(void);
PSU_GuiControlMode_t PSU_GuiGetControlMode(void);

void PSU_Start(void);
void PSU_Stop(void);
uint8_t PSU_IsRunning(void);
uint8_t PSU_IsCurrentLimitActive(void);

#endif /* PSU_GUI_API_H */
