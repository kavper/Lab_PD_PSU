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
    uint32_t vset_mv;
    uint32_t vout_mv;
    uint32_t vin_mv;
    uint32_t iset_ma;
    uint32_t iout_ma;
    uint32_t vpre_mv;
    bool vpre_present;
    uint32_t last_tlm_ms;
    uint32_t last_rx_ms;
    uint32_t tlm_count;
    uint32_t rx_bytes;
    uint32_t rx_lines;
    uint32_t rx_errors;
    uint32_t last_error_code; /* HAL_UART_ERROR_* latch */
    uint8_t first_rx[16];
    uint8_t first_rx_len;
    bool first_rx_dumped;
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
