#ifndef APP_H
#define APP_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MODE_IDLE = 0,
    MODE_OFF = MODE_IDLE,
    MODE_SAFE = MODE_IDLE,
    MODE_CV,
    MODE_CC
} App_Mode_t;

typedef enum {
    FAULT_NONE   = 0U,
    FAULT_DRIVER = (1U << 0),
    FAULT_OVP    = (1U << 1),
    FAULT_OCP    = (1U << 2),
    FAULT_UVIN   = (1U << 3),
    FAULT_ADC    = (1U << 4)
} App_FaultFlags_t;

void App_Init(HRTIM_HandleTypeDef *hhrtim,
              ADC_HandleTypeDef *hadc1,
              ADC_HandleTypeDef *hadc2,
              UART_HandleTypeDef *huart_debug);

void App_Run(void);
void App_SetRequestedMode(App_Mode_t mode);
App_Mode_t App_GetMode(void);
App_Mode_t App_GetRequestedMode(void);
uint32_t App_GetFaultFlags(void);
void App_ClearFaults(void);
void App_SetCvSetpoint(float setpoint_v);
float App_GetCvSetpoint(void);
float App_GetCvRampedSetpoint(void);
void App_SetCurrentLimit(float current_limit_a);
float App_GetCurrentLimit(void);
float App_GetInputVoltage(void);
float App_GetOutputVoltage(void);
float App_GetOutputCurrent(void);
uint32_t APP_GetPower(void);

#endif
