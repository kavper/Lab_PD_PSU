#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float vin;
    float vout;
    float iout;
    float vbat;
    float i_hs_buck;   /* I_IN_BUCK INA296, DMA-window average + IIR */
    float i_hs_boost;  /* I_OUT_BOOST INA296, DMA-window average + IIR */
    uint16_t raw_vin;
    uint16_t raw_vout;
    uint16_t raw_iout;
    uint16_t raw_i_hs_buck;
    uint16_t raw_i_hs_boost;
    bool valid;
} Measurements_t;

/* CubeMX ADC ranks (Lab_PD_PSU.ioc) must match DMA unpacking in measurements.c:
 * ADC1 rank1 PA3 I_OUT_BOOST (6.5 cyc), rank2 PA0 ADC_VBAT VIN (24.5 cyc).
 * ADC2 rank1 PA1 I_IN_BUCK (6.5 cyc), rank2 PB2 ADC_VOUT (24.5 cyc).
 * VIN/VOUT and both HS currents are DMA-window averaged. ACS37100 is unused. */

void Measurements_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);
bool Measurements_Update(Measurements_t *meas);
uint8_t Measurements_GetLastError(void);
HAL_StatusTypeDef Measurements_GetLastHalStatus(void);
uint32_t Measurements_GetDmaUpdateCount(void);
bool Measurements_IsDmaRunning(void);

#endif
