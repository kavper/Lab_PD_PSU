#include "board_mx.h"

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
