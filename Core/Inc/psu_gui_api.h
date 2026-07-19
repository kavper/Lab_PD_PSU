#ifndef PSU_GUI_API_H
#define PSU_GUI_API_H

#include <stdint.h>

#ifndef PSU_GUI_PD_DEBUG
#define PSU_GUI_PD_DEBUG 0U
#endif

typedef enum {
    PSU_GUI_CONTROL_MODE_OFF = 0,
    PSU_GUI_CONTROL_MODE_CV,
    PSU_GUI_CONTROL_MODE_CC
} PSU_GuiControlMode_t;

typedef enum {
    PSU_GUI_USB_MODE_AUTO = 0,
    PSU_GUI_USB_MODE_SINK_ONLY,
    PSU_GUI_USB_MODE_SOURCE_ONLY
} PSU_GuiUsbMode_t;

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
float PSU_GuiGetPdContractVoltage(void);
float PSU_GuiGetPdContractCurrent(void);
float PSU_GuiGetPdContractPower(void);
uint8_t PSU_GuiIsPdContractValid(void);
uint8_t PSU_GuiGetPdContract(float *voltage_v,
                             float *current_a,
                             float *power_w,
                             uint32_t *active_rdo_raw);
uint8_t PSU_GuiGetTransferPower(float *power_w);
PSU_GuiControlMode_t PSU_GuiGetControlMode(void);
PSU_GuiUsbMode_t PSU_GuiGetUsbMode(void);
uint8_t PSU_GuiSetUsbMode(PSU_GuiUsbMode_t mode);

void PSU_Start(void);
void PSU_Stop(void);
uint8_t PSU_IsRunning(void);
uint8_t PSU_IsCurrentLimitActive(void);

#endif /* PSU_GUI_API_H */
