#ifndef BOARD_MX_H
#define BOARD_MX_H

#include "stm32g4xx_hal.h"

/*
 * CubeMX-safe extras. Call only from USER CODE blocks in generated files.
 * CubeMX will not delete this file.
 */
HAL_StatusTypeDef BoardMx_StartHsiPll(void);
void BoardMx_ApplyHrtimFault(HRTIM_HandleTypeDef *hhrtim);

#endif /* BOARD_MX_H */
