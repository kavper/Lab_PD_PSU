#include "board_mx.h"
#include "main.h"

static volatile uint8_t usbpd_irq_pending;
static volatile uint8_t bms_alert_pending;

void BoardMx_EarlyLedPulse(uint8_t count)
{
    uint8_t i;

    for (i = 0U; i < count; i++) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
        HAL_Delay(80U);
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        HAL_Delay(80U);
    }
}

void BoardMx_FaultLedTick(void)
{
    static uint32_t last_ms;
    uint32_t now_ms = HAL_GetTick();

    if ((uint32_t)(now_ms - last_ms) >= 120U) {
        last_ms = now_ms;
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
}

HAL_StatusTypeDef BoardMx_StartHsiPll(void)
{
    RCC_OscInitTypeDef osc = {0};

    /* Same 4 MHz PLL input as HSE8/DIV2, so 170 MHz SYSCLK is unchanged. */
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    osc.HSIState = RCC_HSI_ON;
    osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    osc.PLL.PLLM = RCC_PLLM_DIV4;
    osc.PLL.PLLN = 85;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = RCC_PLLQ_DIV2;
    osc.PLL.PLLR = RCC_PLLR_DIV2;
    return HAL_RCC_OscConfig(&osc);
}

HAL_StatusTypeDef BoardMx_EnsureSysclkPll(void)
{
    RCC_ClkInitTypeDef clk = {0};

    if (__HAL_RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) {
        return HAL_OK;
    }

    if (BoardMx_StartHsiPll() != HAL_OK) {
        return HAL_ERROR;
    }

    clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV1;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    return HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4);
}

void BoardMx_ApplyHrtimFault(HRTIM_HandleTypeDef *hhrtim)
{
    if (hhrtim == NULL) {
        return;
    }

    /*
     * ACS37100 inductor FAULT (HRTIM FLT3, active low) is ignored for now.
     * High-side INA296 software OCP is the current protection. CubeMX still
     * wires FLT3 onto Timer A/C; disable the channel and the timer enables
     * so PWM cannot be hardware-killed by I_L_MEAS.
     */
    HAL_HRTIM_FaultModeCtl(hhrtim, HRTIM_FAULT_3, HRTIM_FAULTMODECTL_DISABLED);

    if (hhrtim->Instance != NULL) {
        CLEAR_BIT(hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].FLTxR,
                  HRTIM_FLTR_FLT3EN);
        CLEAR_BIT(hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].FLTxR,
                  HRTIM_FLTR_FLT3EN);
    }
}

void BoardMx_GpioExtiCallback(uint16_t gpio_pin)
{
    if (gpio_pin == I2C_USBPD_IRQ_Pin) {
        usbpd_irq_pending = 1U;
    }
    if (gpio_pin == BMS_ALERT_Pin) {
        bms_alert_pending = 1U;
    }
}

bool BoardMx_TakeUsbPdIrq(void)
{
    uint8_t pending;

    pending = usbpd_irq_pending;
    usbpd_irq_pending = 0U;
    return pending != 0U;
}

bool BoardMx_TakeBmsAlert(void)
{
    uint8_t pending;

    pending = bms_alert_pending;
    bms_alert_pending = 0U;
    return pending != 0U;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BoardMx_GpioExtiCallback(GPIO_Pin);
}
