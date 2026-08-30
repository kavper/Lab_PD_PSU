#ifndef BOARD_REV_H
#define BOARD_REV_H

/*
 * Hardware revision 2 (schematic 2026-08-30). Previous PCB stays on git main.
 *
 * CubeMX workflow:
 * - Pins, ADC ranks, HRTIM, I2C4, USART2, DMA live in Lab_PD_PSU.ioc.
 *   After you change something in CubeMX, Generate Code (Keep User Code = ON).
 * - Application logic is in Core/Src files CubeMX does not own
 *   (app, measurements, power_stage, power_manager, board_mx, ...).
 * - Extra HAL that CubeMX would wipe goes in USER CODE blocks, which call
 *   board_mx.c. Do not put app logic in the generated MX_* bodies.
 * - main.h pin #defines are regenerated from CubeMX labels. Aliases
 *   (LED1, OTG_EN, FLT) stay in USER CODE BEGIN Private defines.
 *
 * If CubeMX restores HSE-only clock (Error_Handler on oscillator fail),
 * keep the USER CODE call to BoardMx_StartHsiPll() in SystemClock_Config.
 */

#define BOARD_HW_REV                         2U
#define BOARD_HW_REV_STRING                  "rev2"

#define BOARD_VREF_V                         3.0f
#define BOARD_DIVIDER_TOP_OHM                51000.0f
#define BOARD_DIVIDER_BOT_OHM                4700.0f
#define BOARD_DIVIDER_RATIO \
    ((BOARD_DIVIDER_TOP_OHM + BOARD_DIVIDER_BOT_OHM) / BOARD_DIVIDER_BOT_OHM)

#define BOARD_INA296_GAIN                    100.0f
#define BOARD_INA296_SHUNT_OHM               0.001f
#define BOARD_INA296_A_PER_V \
    (1.0f / (BOARD_INA296_GAIN * BOARD_INA296_SHUNT_OHM))
#define BOARD_INA296_OFFSET_V                (BOARD_VREF_V * 0.5f)

#define BOARD_ACS37100_V_PER_A               0.0528f

#define BOARD_HAS_OLED                       0U
#define BOARD_HAS_ENCODER                    0U
#define BOARD_HAS_FRONT_BUTTONS              0U
#define BOARD_HAS_ISOLATED_GAN_SUPPLY        1U

#endif /* BOARD_REV_H */
