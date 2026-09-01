#ifndef BOARD_REV_H
#define BOARD_REV_H

/*
 * Hardware revision 2 (schematic 2026-08-30). Previous PCB stays on git main.
 *
 * =============================================================================
 * CubeMX <-> handwritten firmware  /  CubeMX <-> kod ręczny
 * =============================================================================
 *
 * PL:
 * 1. Lab_PD_PSU.ioc jest jedynym źródłem prawdy dla pinów, zegara, ADC, HRTIM,
 *    I2C4, USART1/3, DMA i NVIC. To, co ma być widoczne w CubeMX, musi być w IOC.
 *    Nie chowaj konfiguracji MX tylko w plikach .c.
 * 2. Project Manager: Keep User Code = ON (KeepUserCode=true w .ioc).
 *    Generate Code NIE może kasować firmware w USER CODE ani plików, których
 *    CubeMX nie regeneruje.
 * 3. Po Generate sprawdź cmake/stm32cubemx/CMakeLists.txt (CubeMX go nadpisuje).
 *    Źródła aplikacji dopisuj w głównym CMakeLists.txt ORAZ w .extSettings
 *    (Keil/IAR/Makefile). Nie edytuj cmake/stm32cubemx ręcznie.
 * 4. Nie edytuj ciał MX_* (poza blokami USER CODE). Dodatkowy HAL:
 *    board_mx.c, wołany z USER CODE.
 * 5. main.h: etykiety pinów regeneruje CubeMX. Aliasy (LED1, OTG_EN, FLT)
 *    zostają w USER CODE BEGIN Private defines.
 * 6. gui.c / OLED nie są na tej rewizji MCU — nie dodawaj ich do CMake/.extSettings.
 *
 * EN:
 * 1. Lab_PD_PSU.ioc is the GUI source of truth for pins/clocks/peripherals.
 * 2. Keep User Code ON. App logic lives in Core/Src files CubeMX does not own.
 * 3. New app .c files: add to root CMakeLists.txt and .extSettings (same list).
 * 4. Do not put MX-visible pin config only in C.
 *
 * Clock: HSE 8 MHz -> PLL 170 MHz is in the IOC. If the crystal fails,
 * USER CODE Clock_OscConfig_Error and SysInit call board_mx HSI PLL fallback.
 * If CubeMX rewrites SystemClock_Config to Error_Handler-only, Keep User Code
 * still preserves those USER CODE blocks; SysInit is the stable second hook.
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
/*
 * Isolated DC-DC gate-driver supplies (BUCK_TR_EN / BOOST_TR_EN + TR_FLT).
 * Set to 0 for bring-up with direct +5V MP1918 drivers (no translator boards).
 */
#ifndef BOARD_HAS_ISOLATED_GAN_SUPPLY
#define BOARD_HAS_ISOLATED_GAN_SUPPLY        0U
#endif

/* Silk TPS2571D is TPS25751D — keep tps25751.c. */
#define BOARD_USBPD_IS_TPS25751              1U
/* BQ25731 CELL_BATPRESZ 4-cell strap is intentional (sheet title still says 5S). */
#define BOARD_CHARGER_CELL_COUNT             4U
/* POWER_PERMIT_G4 PB6: HIGH enables power-stage LDO, LOW kills it. */
#define BOARD_POWER_PERMIT_ACTIVE_HIGH       1U

/* Pre-regulator headroom — must match G0 app_config.h (VPRE_*). */
#define BOARD_VPRE_MIN_V                     3.0f
#define BOARD_VPRE_MAX_V                     36.0f
#define BOARD_VPRE_MARGIN_V                  3.0f

/*
 * Bring-up without G0 TLM: host "ON" grants POWER_PERMIT and runs local CV.
 * Set to 0 when G0 LDO must control PERMIT exclusively.
 */
#ifndef BOARD_BRINGUP_LOCAL_CV
#define BOARD_BRINGUP_LOCAL_CV               1U
#endif

/*
 * No G0 on USART3: auto-enter CV after startup hold (2 s) without host ON.
 * Disable before shipping with G0 LDO.
 */
#ifndef BOARD_BRINGUP_AUTO_ON
#define BOARD_BRINGUP_AUTO_ON                1U
#endif

/* Wait after POWER_PERMIT before enabling isolated GaN supplies (no G0). */
#ifndef BOARD_PERMIT_HW_SETTLE_MS
#define BOARD_PERMIT_HW_SETTLE_MS            250U
#endif

#endif /* BOARD_REV_H */
