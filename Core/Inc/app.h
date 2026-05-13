#ifndef APP_H
#define APP_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

void App_Init(I2C_HandleTypeDef *hi2c_tps,
              UART_HandleTypeDef *huart_debug);

void App_Run(void);

#endif