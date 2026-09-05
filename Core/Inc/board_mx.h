#ifndef BOARD_MX_H
#define BOARD_MX_H

#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * CubeMX-safe extras. Call only from USER CODE blocks in generated files.
 * CubeMX will not delete this file.
 *
 * HRTIM FLT3 (ACS37100 series inductor, active low) stays in the IOC pinout
 * but is disabled: current protection is high-side INA296 OCP, not I_L_MEAS.
 * BoardMx_ApplyHrtimFault() turns FLT3 and timer fault enables off.
 */
HAL_StatusTypeDef BoardMx_StartHsiPll(void);
HAL_StatusTypeDef BoardMx_EnsureSysclkPll(void);
void BoardMx_ApplyHrtimFault(HRTIM_HandleTypeDef *hhrtim);
void BoardMx_EarlyLedPulse(uint8_t count);
void BoardMx_FaultLedTick(void);

void BoardMx_GpioExtiCallback(uint16_t gpio_pin);
bool BoardMx_TakeUsbPdIrq(void);
bool BoardMx_TakeBmsAlert(void);

#endif /* BOARD_MX_H */
