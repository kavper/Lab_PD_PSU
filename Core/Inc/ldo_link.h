#ifndef LDO_LINK_H
#define LDO_LINK_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * G0 LDO link on USART3 PB14/PB15 (115200 8N1). See docs/G4_LDO_UART.md.
 */

typedef struct {
    bool telemetry_valid;
    bool output_on;
    uint8_t mode;
    uint8_t bleed_request;
    uint8_t fan_percent;
    uint8_t pgood;
    uint8_t kill_reported;
    uint8_t cc_cv;
    uint32_t last_tlm_ms;
    uint32_t tlm_count;
    char fault[16];
} LdoLink_Status_t;

void LdoLink_Init(UART_HandleTypeDef *huart_g0);
void LdoLink_Task(void);
void LdoLink_RxCplt(UART_HandleTypeDef *huart);
void LdoLink_OnUartError(UART_HandleTypeDef *huart);

void LdoLink_SetDcdcPermitRequest(bool permit);
bool LdoLink_IsDcdcPermitRequested(void);
void LdoLink_GetStatus(LdoLink_Status_t *out);
bool LdoLink_IsPowerPermitted(void);

#endif /* LDO_LINK_H */
