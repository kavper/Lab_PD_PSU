#include "fan_tach.h"

#include "main.h"

#include <string.h>

/*
 * J7 pin 3 is the open-collector tach, pulled to +3V3 by R73 and filtered
 * by R74 1 kΩ + C79 470 pF into PA5 (TIM2_CH1, AF1). TIM2 counts falling
 * edges. A one-second gate turns that count into RPM.
 */
static TIM_HandleTypeDef s_htim2;
static uint32_t s_window_ms;
static uint16_t s_rpm;
static uint8_t s_ready;
static uint8_t s_have;

void FanTach_Init(void)
{
    TIM_SlaveConfigTypeDef slave = {0};

    memset(&s_htim2, 0, sizeof(s_htim2));
    s_ready = 0U;
    s_have = 0U;
    s_rpm = 0U;

    s_htim2.Instance = TIM2;
    s_htim2.Init.Prescaler = 0U;
    s_htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_htim2.Init.Period = 0xFFFFFFFFU;
    s_htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&s_htim2) != HAL_OK) {
        return;
    }

    /*
     * CC1S has to select the pin. External-clock setup writes the filter
     * into the input-stage bits and leaves that selection alone.
     */
    {
        TIM_IC_InitTypeDef ic = {0};

        ic.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
        ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
        ic.ICPrescaler = TIM_ICPSC_DIV1;
        ic.ICFilter = 4U;
        if (HAL_TIM_IC_ConfigChannel(&s_htim2, &ic, TIM_CHANNEL_1) != HAL_OK) {
            return;
        }
    }

    slave.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
    slave.InputTrigger = TIM_TS_TI1FP1;
    slave.TriggerPolarity = TIM_TRIGGERPOLARITY_FALLING;
    slave.TriggerPrescaler = TIM_TRIGGERPRESCALER_DIV1;
    slave.TriggerFilter = 4U;
    if (HAL_TIM_SlaveConfigSynchro(&s_htim2, &slave) != HAL_OK) {
        return;
    }

    __HAL_TIM_SET_COUNTER(&s_htim2, 0U);
    if (HAL_TIM_Base_Start(&s_htim2) != HAL_OK) {
        return;
    }

    s_window_ms = HAL_GetTick();
    s_ready = 1U;
}

void FanTach_Task(void)
{
    uint32_t now;
    uint32_t elapsed;
    uint32_t edges;

    if (s_ready == 0U) {
        return;
    }

    now = HAL_GetTick();
    elapsed = now - s_window_ms;
    if (elapsed < FAN_TACH_WINDOW_MS) {
        return;
    }

    edges = __HAL_TIM_GET_COUNTER(&s_htim2);
    __HAL_TIM_SET_COUNTER(&s_htim2, 0U);
    s_rpm = FanTach_RpmFromEdges(edges, elapsed);
    s_have = 1U;
    s_window_ms = now;
}

uint16_t FanTach_Rpm(void)
{
    if (s_have == 0U) {
        return FAN_TACH_RPM_NONE;
    }
    return s_rpm;
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio = {0};

    if ((htim == NULL) || (htim->Instance != TIM2)) {
        return;
    }

    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = FAN_TACH_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(FAN_TACH_GPIO_Port, &gpio);
}
