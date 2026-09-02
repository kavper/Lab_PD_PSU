#include "ldo_link.h"

#include "board_rev.h"
#include "debug_uart.h"
#include "fan_pwm.h"
#include "host_link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LDO_RX_LINE_MAX              192U
#define LDO_TLM_STALE_MS             500U
#define LDO_FAN_FAILSAFE_PERCENT     40U

static UART_HandleTypeDef *s_huart_g0 = NULL;
static uint8_t s_rx_byte;
static char s_rx_line[LDO_RX_LINE_MAX];
static volatile uint8_t s_rx_len;
static volatile bool s_line_ready;
static volatile bool s_rx_overflow;

static LdoLink_Status_t s_status;
static bool s_dcdc_permit_request;
static uint8_t s_applied_bleed;
static uint8_t s_applied_fan;
static bool s_applied_permit;

static void LdoLink_ArmRx(void)
{
    if (s_huart_g0 != NULL) {
        (void)HAL_UART_Receive_IT(s_huart_g0, &s_rx_byte, 1U);
    }
}

static void LdoLink_SetBleed(bool on)
{
    HAL_GPIO_WritePin(BLEED_ON_GPIO_Port, BLEED_ON_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    s_applied_bleed = on ? 1U : 0U;
}

static void LdoLink_SetPermitPin(bool permit)
{
#if (BOARD_POWER_PERMIT_ACTIVE_HIGH != 0U)
    HAL_GPIO_WritePin(POWER_PERMIT_G4_GPIO_Port, POWER_PERMIT_G4_Pin,
                      permit ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(POWER_PERMIT_G4_GPIO_Port, POWER_PERMIT_G4_Pin,
                      permit ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
    s_applied_permit = permit;
}

static bool LdoLink_ParseU8Token(const char *line, const char *key, uint8_t *out)
{
    const char *p;
    char *end;
    unsigned long value;

    if ((line == NULL) || (key == NULL) || (out == NULL)) {
        return false;
    }

    p = strstr(line, key);
    if (p == NULL) {
        return false;
    }

    value = strtoul(p + strlen(key), &end, 10);
    if (end == (p + strlen(key))) {
        return false;
    }

    *out = (uint8_t)value;
    return true;
}

static bool LdoLink_ParseU32Token(const char *line, const char *key, uint32_t *out)
{
    const char *p;
    char *end;
    unsigned long value;

    if ((line == NULL) || (key == NULL) || (out == NULL)) {
        return false;
    }

    p = strstr(line, key);
    if (p == NULL) {
        return false;
    }

    value = strtoul(p + strlen(key), &end, 10);
    if (end == (p + strlen(key))) {
        return false;
    }

    *out = (uint32_t)value;
    return true;
}

static void LdoLink_ParseFaultToken(const char *line)
{
    const char *p;
    const char *end;
    size_t len;

    memset(s_status.fault, 0, sizeof(s_status.fault));
    p = strstr(line, "fault=");
    if (p == NULL) {
        return;
    }

    p += 6U;
    end = p;
    while ((*end != '\0') && (*end != ' ') && (*end != '\r') && (*end != '\n')) {
        end++;
    }

    len = (size_t)(end - p);
    if (len >= sizeof(s_status.fault)) {
        len = sizeof(s_status.fault) - 1U;
    }
    memcpy(s_status.fault, p, len);
}

static void LdoLink_HandleTlmLine(const char *line)
{
    uint8_t value;

    s_status.rx_lines++;
    if ((line == NULL) || (strncmp(line, "TLM ", 4U) != 0)) {
        return;
    }

    s_status.telemetry_valid = true;
    s_status.last_tlm_ms = HAL_GetTick();
    s_status.tlm_count++;

    if (LdoLink_ParseU8Token(line, "out=", &value)) {
        s_status.output_on = (value != 0U);
    }
    if (LdoLink_ParseU8Token(line, "mode=", &value)) {
        s_status.mode = value;
    }
    if (LdoLink_ParseU8Token(line, "bleed=", &value)) {
        s_status.bleed_request = (value != 0U) ? 1U : 0U;
    }
    if (LdoLink_ParseU8Token(line, "fan=", &value)) {
        s_status.fan_percent = value;
    }
    if (LdoLink_ParseU8Token(line, "pgood=", &value)) {
        s_status.pgood = (value != 0U) ? 1U : 0U;
    }
    if (LdoLink_ParseU8Token(line, "kill=", &value)) {
        s_status.kill_reported = (value != 0U) ? 1U : 0U;
    }
    if (LdoLink_ParseU8Token(line, "cccv=", &value)) {
        s_status.cc_cv = (value != 0U) ? 1U : 0U;
    }
    if (LdoLink_ParseU32Token(line, "vset=", &s_status.vset_mv)) {
        /* keep */
    }
    if (LdoLink_ParseU32Token(line, "vout=", &s_status.vout_mv)) {
        /* keep */
    }
    if (LdoLink_ParseU32Token(line, "vin=", &s_status.vin_mv)) {
        /* keep */
    }
    if (LdoLink_ParseU32Token(line, "iset=", &s_status.iset_ma)) {
        /* keep */
    }
    if (LdoLink_ParseU32Token(line, "iout=", &s_status.iout_ma)) {
        /* keep */
    }
    if (LdoLink_ParseU32Token(line, "vpre=", &s_status.vpre_mv)) {
        s_status.vpre_present = true;
    }
    LdoLink_ParseFaultToken(line);

    HostLink_ForwardLine(line);
    Debug_Write(line);
    Debug_Write("\r\n");
}

static void LdoLink_ApplyActuators(uint32_t now_ms)
{
    uint8_t fan = s_status.fan_percent;
    bool permit;

    if (!s_status.telemetry_valid ||
        ((uint32_t)(now_ms - s_status.last_tlm_ms) > LDO_TLM_STALE_MS)) {
        fan = LDO_FAN_FAILSAFE_PERCENT;
    }

    if (fan > 100U) {
        fan = 100U;
    }

    LdoLink_SetBleed(s_status.bleed_request != 0U);
    FanPwm_SetPercent(fan);
    s_applied_fan = fan;

    permit = s_dcdc_permit_request;
#if (BOARD_POWER_PERMIT_ACTIVE_HIGH != 0U)
    if (s_applied_permit != permit) {
        LdoLink_SetPermitPin(permit);
    }
#else
    if (s_applied_permit != permit) {
        LdoLink_SetPermitPin(permit);
    }
#endif
}

void LdoLink_Init(UART_HandleTypeDef *huart_g0)
{
    memset(&s_status, 0, sizeof(s_status));
    strncpy(s_status.fault, "NONE", sizeof(s_status.fault) - 1U);

    s_huart_g0 = huart_g0;
    s_rx_len = 0U;
    s_line_ready = false;
    s_rx_overflow = false;
    s_dcdc_permit_request = false;

    FanPwm_Init();
    LdoLink_SetBleed(false);
#if (BOARD_BRINGUP_PERMIT_EARLY != 0U)
    LdoLink_SetPermitPin(true);
    s_dcdc_permit_request = true;
#else
    LdoLink_SetPermitPin(false);
#endif
    HAL_GPIO_WritePin(REMOTE_ON_GPIO_Port, REMOTE_ON_Pin, GPIO_PIN_RESET);

    LdoLink_ArmRx();
    Debug_Printf("[LDO] USART3 G0 link ready (TLM forward on USART1)\r\n");
#if (BOARD_BRINGUP_PERMIT_EARLY != 0U)
    Debug_Printf("[LDO] Bring-up: POWER_PERMIT early HIGH (clear G0 POWER_KILL)\r\n");
#endif
}

void LdoLink_SetDcdcPermitRequest(bool permit)
{
    s_dcdc_permit_request = permit;
}

bool LdoLink_IsDcdcPermitRequested(void)
{
    return s_dcdc_permit_request;
}

void LdoLink_GetStatus(LdoLink_Status_t *out)
{
    if (out != NULL) {
        *out = s_status;
    }
}

void LdoLink_Task(void)
{
    char line[LDO_RX_LINE_MAX];
    uint32_t now_ms = HAL_GetTick();

    if (s_line_ready) {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        memcpy(line, s_rx_line, sizeof(line));
        s_line_ready = false;
        s_rx_len = 0U;
        if (primask == 0U) {
            __enable_irq();
        }
        LdoLink_HandleTlmLine(line);
    } else if (s_rx_overflow) {
        s_rx_overflow = false;
        Debug_Printf("[LDO] WARN: G0 RX line overflow\r\n");
    }

    LdoLink_ApplyActuators(now_ms);
}

void LdoLink_RxCplt(UART_HandleTypeDef *huart)
{
    uint8_t ch;

    if ((huart == NULL) || (huart != s_huart_g0)) {
        return;
    }

    ch = s_rx_byte;
    s_status.rx_bytes++;
    s_status.last_rx_ms = HAL_GetTick();
    if ((ch == (uint8_t)'\n') || (ch == (uint8_t)'\r')) {
        if (s_rx_len > 0U) {
            s_rx_line[s_rx_len] = '\0';
            s_line_ready = true;
            s_rx_len = 0U;
        }
    } else if (!s_line_ready) {
        if (s_rx_len < (LDO_RX_LINE_MAX - 1U)) {
            s_rx_line[s_rx_len++] = (char)ch;
        } else {
            s_rx_overflow = true;
            s_rx_len = 0U;
        }
    }

    LdoLink_ArmRx();
}

void LdoLink_OnUartError(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart == s_huart_g0)) {
        s_status.rx_errors++;
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        LdoLink_ArmRx();
    }
}

bool LdoLink_IsPowerPermitted(void)
{
#if (BOARD_POWER_PERMIT_ACTIVE_HIGH != 0U)
    return HAL_GPIO_ReadPin(POWER_PERMIT_G4_GPIO_Port, POWER_PERMIT_G4_Pin) ==
           GPIO_PIN_SET;
#else
    return HAL_GPIO_ReadPin(POWER_PERMIT_G4_GPIO_Port, POWER_PERMIT_G4_Pin) ==
           GPIO_PIN_RESET;
#endif
}
