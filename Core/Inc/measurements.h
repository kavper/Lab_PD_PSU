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

void Measurements_Init(ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2);
bool Measurements_Update(Measurements_t *meas);
uint8_t Measurements_GetLastError(void);
HAL_StatusTypeDef Measurements_GetLastHalStatus(void);
uint32_t Measurements_GetDmaUpdateCount(void);
bool Measurements_IsDmaRunning(void);

#endif
