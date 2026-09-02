#ifndef LDO_LINK_H
#define LDO_LINK_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/*
 * G0 LDO link on USART2 PB3 TX / PB4 RX (115200 8N1).
 * Protocol: docs/G4_LDO_UART.md + LDO_controller docs/G4_G0_UART_PROTOCOL.md
 */

typedef enum {
    LDO_G0_CTRL_IDLE = 0,
    LDO_G0_CTRL_WAIT_LINK,
    LDO_G0_CTRL_WAIT_PERMIT,
    LDO_G0_CTRL_WAIT_VIN,
    LDO_G0_CTRL_SEND_SET,
    LDO_G0_CTRL_WAIT_SET_ACK,
    LDO_G0_CTRL_WAIT_VOUT_ZERO,
    LDO_G0_CTRL_SEND_OUT_ON,
    LDO_G0_CTRL_WAIT_OUT_ON_ACK,
    LDO_G0_CTRL_RUNNING,
    LDO_G0_CTRL_SEND_OUT_OFF,
    LDO_G0_CTRL_WAIT_OUT_OFF_ACK,
    LDO_G0_CTRL_FAULT
} LdoLink_CtrlState_t;

typedef struct {
    bool telemetry_valid;
    bool output_on;
    uint8_t mode;
    uint8_t bleed_request;
    uint8_t fan_percent;
    uint8_t pgood;
    uint8_t kill_reported;
    uint8_t outoff_reported;
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
    char fault[24];
    char last_nack[48];
    LdoLink_CtrlState_t ctrl_state;
    bool output_wanted;
    uint32_t ack_ok_count;
    uint32_t nack_count;
    uint32_t cmd_timeout_count;
} LdoLink_Status_t;

void LdoLink_Init(UART_HandleTypeDef *huart_g0);
void LdoLink_Task(void);
void LdoLink_RxCplt(UART_HandleTypeDef *huart);
void LdoLink_OnUartError(UART_HandleTypeDef *huart);

void LdoLink_SetDcdcPermitRequest(bool permit);
bool LdoLink_IsDcdcPermitRequested(void);
void LdoLink_GetStatus(LdoLink_Status_t *out);
bool LdoLink_IsPowerPermitted(void);

/* G0 final-output control (ASCII SET / OUT ON / OUT OFF). */
void LdoLink_RequestOutput(bool on);
bool LdoLink_IsOutputWanted(void);
void LdoLink_SetG0Setpoint(float volts, float amps);
void LdoLink_SetG0Voltage(float volts);
void LdoLink_SetG0Current(float amps);
float LdoLink_GetG0Voltage(void);
float LdoLink_GetG0Current(void);
LdoLink_CtrlState_t LdoLink_GetCtrlState(void);

/*
 * Remote sense path (PB6 REMOTE_ON). Default OFF = local Kelvin on ADC_VOUT (PB2).
 * Host "REMOTE ON" asserts REMOTE_ON so ADC_REMOTE_P/N (PB0/PB1) sense path is
 * selected in hardware. CV regulation still uses ADC_VOUT (PB2) until remote
 * channels are added to ADC DMA ranks.
 */
void LdoLink_SetRemoteSense(bool enable);
bool LdoLink_IsRemoteSenseEnabled(void);

/* Bench: dump USART2 ISR/GPIO/SWAP; optional runtime TX/RX SWAP. */
void LdoLink_DumpDiag(void);
void LdoLink_SetUartPinSwap(bool enable);
bool LdoLink_IsUartPinSwapEnabled(void);

#endif /* LDO_LINK_H */
