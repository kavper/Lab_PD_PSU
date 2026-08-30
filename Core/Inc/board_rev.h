#ifndef BOARD_REV_H
#define BOARD_REV_H

/*
 * This firmware branch targets PCB revision 2 (August 2026 schematic).
 * Previous hardware stays on git branch main.
 *
 * Rev2 highlights:
 * - USB-PD: TPS25751D family on I2C4 (PC6/PC7), IRQ on PB9
 * - BMS: BQ76922 on the same I2C_USBPD bus, address 0x08, ALERT on PA10
 * - Charger BQ25731 still on TPS master I2C (I2C_CONFIG), STM_OTG_EN on PA4
 * - GaN MP1918 + UCC33420 isolated HS supplies (100% duty, no bootstrap refresh)
 * - Output current: INA296A3 on 1 mOhm, I_OUT_BOOST = PA3
 * - Inductor current ACS37100 is debug-only (PB11/PB15) + HRTIM_FLT on PB10
 * - No OLED/encoder/buttons on this MCU (GUI lives off-board via USART1 to H7)
 */

#define BOARD_HW_REV                         2U
#define BOARD_HW_REV_STRING                  "rev2"

#define BOARD_VREF_V                         3.0f
#define BOARD_DIVIDER_TOP_OHM                51000.0f
#define BOARD_DIVIDER_BOT_OHM                4700.0f
#define BOARD_DIVIDER_RATIO \
    ((BOARD_DIVIDER_TOP_OHM + BOARD_DIVIDER_BOT_OHM) / BOARD_DIVIDER_BOT_OHM)

/* INA296A3: 100 V/V, 1 mOhm shunt, REF1=GND REF2=+VREF => mid-scale 1.5 V. */
#define BOARD_INA296_GAIN                    100.0f
#define BOARD_INA296_SHUNT_OHM               0.001f
#define BOARD_INA296_A_PER_V \
    (1.0f / (BOARD_INA296_GAIN * BOARD_INA296_SHUNT_OHM))
#define BOARD_INA296_OFFSET_V                (BOARD_VREF_V * 0.5f)

/* ACS37100-025B3: ~52.8 mV/A, differential VOUT-VREF. Debug only. */
#define BOARD_ACS37100_V_PER_A               0.0528f

#define BOARD_HAS_OLED                       0U
#define BOARD_HAS_ENCODER                    0U
#define BOARD_HAS_FRONT_BUTTONS              0U
#define BOARD_HAS_ISOLATED_GAN_SUPPLY        1U

#endif /* BOARD_REV_H */
