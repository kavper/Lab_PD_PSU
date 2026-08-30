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
    uint16_t raw_vin;
    uint16_t raw_vout;
    uint16_t raw_iout;
    bool valid;
} Measurements_t;

/* CubeMX ADC ranks (Lab_PD_PSU.ioc) must match DMA unpacking in measurements.c:
 * ADC1 rank1 PA0 ADC_VBAT VIN, rank2 PA3 I_OUT_BOOST, ADC2 rank1 PB2 ADC_VOUT. */

void Measurements_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);
bool Measurements_Update(Measurements_t *meas);
uint8_t Measurements_GetLastError(void);
HAL_StatusTypeDef Measurements_GetLastHalStatus(void);
uint32_t Measurements_GetDmaUpdateCount(void);
bool Measurements_IsDmaRunning(void);

#endif
