#include "fan_pwm.h"

#include "main.h"

#include <string.h>

#define FAN_PWM_TIMER_HZ           25000U
#define FAN_PWM_PERCENT_MAX        100U

static TIM_HandleTypeDef s_htim16;
static uint8_t s_ready;

static uint32_t FanPwm_TimerClockHz(void)
{
    return HAL_RCC_GetPCLK2Freq();
}

void FanPwm_Init(void)
{
    TIM_OC_InitTypeDef oc = {0};
    uint32_t tim_clk;
    uint32_t period;

    memset(&s_htim16, 0, sizeof(s_htim16));
    s_ready = 0U;

    __HAL_RCC_TIM16_CLK_ENABLE();

    tim_clk = FanPwm_TimerClockHz();
    if (tim_clk == 0U) {
        tim_clk = SystemCoreClock;
    }

    period = (tim_clk / FAN_PWM_TIMER_HZ);
    if (period < 100U) {
        period = 100U;
    }
    if (period > 0xFFFFU) {
        period = 0xFFFFU;
    }

    s_htim16.Instance = TIM16;
    s_htim16.Init.Prescaler = 0U;
    s_htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_htim16.Init.Period = (uint16_t)(period - 1U);
    s_htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_htim16.Init.RepetitionCounter = 0U;
    s_htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&s_htim16) != HAL_OK) {
        return;
    }

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0U;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&s_htim16, &oc, TIM_CHANNEL_1) != HAL_OK) {
        return;
    }

    if (HAL_TIM_PWM_Start(&s_htim16, TIM_CHANNEL_1) != HAL_OK) {
        return;
    }

    s_ready = 1U;
    FanPwm_SetPercent(0U);
}

void FanPwm_SetPercent(uint8_t percent)
{
    uint32_t pulse;
    uint32_t period;

    if (percent > FAN_PWM_PERCENT_MAX) {
        percent = FAN_PWM_PERCENT_MAX;
    }

    if ((s_ready == 0U) || (s_htim16.Instance == NULL)) {
        HAL_GPIO_WritePin(FAN_PWM_GPIO_Port, FAN_PWM_Pin,
                          percent ? GPIO_PIN_SET : GPIO_PIN_RESET);
        return;
    }

    period = (uint32_t)__HAL_TIM_GET_AUTORELOAD(&s_htim16) + 1U;
    pulse = ((period * (uint32_t)percent) + (FAN_PWM_PERCENT_MAX / 2U)) /
            FAN_PWM_PERCENT_MAX;
    __HAL_TIM_SET_COMPARE(&s_htim16, TIM_CHANNEL_1, pulse);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef gpio = {0};

    if ((htim == NULL) || (htim->Instance != TIM16)) {
        return;
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = FAN_PWM_Pin;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM16;
    HAL_GPIO_Init(FAN_PWM_GPIO_Port, &gpio);
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim)
{
    if ((htim == NULL) || (htim->Instance != TIM16)) {
        return;
    }

    HAL_GPIO_DeInit(FAN_PWM_GPIO_Port, FAN_PWM_Pin);
    __HAL_RCC_TIM16_CLK_DISABLE();
}
