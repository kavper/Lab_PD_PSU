#include "board_mx.h"
#include "main.h"

static volatile uint8_t usbpd_irq_pending;
static volatile uint8_t bms_alert_pending;

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
    HRTIM_FaultCfgTypeDef fault_cfg = {0};

    if (hhrtim == NULL) {
        return;
    }

    fault_cfg.Source = HRTIM_FAULTSOURCE_DIGITALINPUT;
    fault_cfg.Polarity = HRTIM_FAULTPOLARITY_LOW;
    fault_cfg.Filter = HRTIM_FAULTFILTER_NONE;
    fault_cfg.Lock = HRTIM_FAULTLOCK_READWRITE;
    (void)HAL_HRTIM_FaultConfig(hhrtim, HRTIM_FAULT_3, &fault_cfg);
    HAL_HRTIM_FaultModeCtl(hhrtim, HRTIM_FAULT_3, HRTIM_FAULTMODECTL_ENABLED);

    /* CubeMX G4 may also enable fault blanking; ACS37100 FAULT must be live. */
#ifdef __HAL_HRTIM_FAULT_BLANKING_DISABLE
    __HAL_HRTIM_FAULT_BLANKING_DISABLE(hhrtim, HRTIM_FAULT_3);
#endif
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
