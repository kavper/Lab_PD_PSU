#ifndef BQ25731_H
#define BQ25731_H

#include "tps25751.h"

#include <stdbool.h>
#include <stdint.h>

#define BQ25731_I2C_ADDR_7BIT           0x6BU

#define BQ25731_REG_CHARGE_OPTION0      0x00U
#define BQ25731_REG_CHARGE_CURRENT      0x02U
#define BQ25731_REG_CHARGE_VOLTAGE      0x04U
#define BQ25731_REG_OTG_VOLTAGE         0x06U
#define BQ25731_REG_IIN_HOST            0x0EU
#define BQ25731_REG_CHARGER_STATUS      0x20U
#define BQ25731_REG_IIN_DPM             0x24U
#define BQ25731_REG_ADC_VBUS_PSYS       0x26U
#define BQ25731_REG_ADC_IBAT            0x28U
#define BQ25731_REG_ADC_IIN_CMPIN       0x2AU
#define BQ25731_REG_ADC_VSYS_VBAT       0x2CU
#define BQ25731_REG_MANUFACTURER_ID     0x2EU
#define BQ25731_REG_CHARGE_OPTION1      0x30U
#define BQ25731_REG_ADC_OPTION          0x3AU
#define BQ25731_REG_CHARGE_OPTION4      0x3CU

#define BQ25731_CONFIG_BLOCK_LEN         8U
#define BQ25731_STATUS_BLOCK_LEN         6U
#define BQ25731_ADC_BLOCK_LEN            8U

#define BQ25731_CHARGE_OPTION0_LWPWR    0x8000U
#define BQ25731_CHARGE_OPTION0_WDT_MASK 0x6000U
#define BQ25731_CHARGE_OPTION0_INHIBIT  0x0001U
#define BQ25731_CHARGE_OPTION0_EN_OOA   0x0400U
#define BQ25731_CHARGE_OPTION0_PWM_FREQ 0x0200U
#define BQ25731_CHARGE_OPTION4_DITHER_MASK 0x1800U
#define BQ25731_CHARGE_OPTION1_5MOHM_MASK  0x0D00U
#define BQ25731_CHARGE_OPTION1_FAST_5MOHM  0x0100U

/* Startup-only acoustic/EMI tuning. 400 kHz minimizes switching loss and is
 * the TI-recommended setting for a 4.7 uH power inductor; use 800 kHz only
 * with the matching 2.2 uH power stage. */
#define BQ25731_INIT_PWM_FREQUENCY_KHZ  400U
#define BQ25731_INIT_DITHER_PERCENT       6U
#define BQ25731_INIT_OUT_OF_AUDIO_ENABLE  1U

#if ((BQ25731_INIT_PWM_FREQUENCY_KHZ != 400U) && \
     (BQ25731_INIT_PWM_FREQUENCY_KHZ != 800U))
#error "BQ25731 PWM frequency must be 400 or 800 kHz"
#endif
#if ((BQ25731_INIT_DITHER_PERCENT != 0U) && \
     (BQ25731_INIT_DITHER_PERCENT != 2U) && \
     (BQ25731_INIT_DITHER_PERCENT != 4U) && \
     (BQ25731_INIT_DITHER_PERCENT != 6U))
#error "BQ25731 dithering must be 0, 2, 4 or 6 percent"
#endif
#define BQ25731_ADC_OPTION_MONITORING   0xE05FU
/* ADC_START (bit 14) is a trigger and may read back as zero. Verify the
 * persistent continuous-conversion, full-scale and channel-enable fields. */
#define BQ25731_ADC_OPTION_VERIFY_MASK  0xA05FU

typedef enum {
    BQ25731_OK = 0,
    BQ25731_BUSY,
    BQ25731_ERROR,
    BQ25731_I2C_ERROR,
    BQ25731_INVALID_ARG,
    BQ25731_NOT_READY,
    BQ25731_DEVICE_ID_MISMATCH,
    BQ25731_TPS_ERROR
} BQ25731_Status_t;

typedef struct {
    TPS25751_Device_t *tps;
    uint8_t address_7bit;
} BQ25731_Device_t;

typedef struct {
    bool online;
    bool id_valid;
    bool adc_configured;
    uint8_t manufacturer_id;
    uint8_t device_id;
    uint16_t charge_option0;
    uint16_t charge_option4;
    uint16_t adc_option;
    uint16_t charge_voltage;
    uint16_t otg_voltage;
    uint16_t charge_current;
    uint16_t iin_host;
    uint16_t charger_status;
    uint16_t iin_dpm;
    uint16_t adc_vbus_psys;
    uint16_t adc_ibat;
    uint16_t adc_iin_cmpin;
    uint16_t adc_vsys_vbat;
    uint32_t charge_voltage_mv;
    uint32_t otg_voltage_mv;
    uint32_t charge_current_ma;
    uint32_t input_current_ma;
    uint32_t iin_dpm_ma;
    uint32_t adc_vbus_mv;
    uint32_t adc_vbat_mv;
    uint32_t adc_vsys_mv;
    uint32_t adc_iin_ma;
    uint32_t adc_ichg_ma;
    uint32_t adc_idchg_ma;
    int32_t battery_current_ma;
    uint32_t input_power_mw;
    int32_t battery_power_mw;
    uint8_t fault_flags;
    bool input_present;
    bool in_precharge;
    bool in_fast_charge;
    bool in_iin_dpm;
    bool in_vindpm;
    bool in_otg;
    bool adc_sample_valid;
    uint32_t updated_ms;
} BQ25731_Telemetry_t;

BQ25731_Status_t BQ25731_Init(BQ25731_Device_t *dev,
                              TPS25751_Device_t *tps,
                              uint8_t address_7bit);
BQ25731_Status_t BQ25731_StartRead16(BQ25731_Device_t *dev, uint8_t reg);
BQ25731_Status_t BQ25731_StartReadId(BQ25731_Device_t *dev);
BQ25731_Status_t BQ25731_StartReadConfigBlock(BQ25731_Device_t *dev);
BQ25731_Status_t BQ25731_StartReadStatusBlock(BQ25731_Device_t *dev);
BQ25731_Status_t BQ25731_StartReadAdcBlock(BQ25731_Device_t *dev);
BQ25731_Status_t BQ25731_StartWriteStartupOption0(
    BQ25731_Device_t *dev, uint16_t value);
BQ25731_Status_t BQ25731_StartWriteStartupOption4(
    BQ25731_Device_t *dev, uint16_t value);
BQ25731_Status_t BQ25731_StartWriteStartupOption1(
    BQ25731_Device_t *dev, uint16_t value);
BQ25731_Status_t BQ25731_StartConfigureMonitoringAdc(
    BQ25731_Device_t *dev);
BQ25731_Status_t BQ25731_MapTpsStatus(TPS25751_Status_t status);

uint16_t BQ25731_BuildStartupOption0(uint16_t current);
uint16_t BQ25731_BuildStartupOption4(uint16_t current);
uint16_t BQ25731_BuildStartupOption1(uint16_t current);
uint32_t BQ25731_DecodePwmFrequencyKhz(uint16_t option0);
uint32_t BQ25731_DecodeDitherPercent(uint16_t option4);
uint32_t BQ25731_DecodeChargeVoltageMv(uint16_t raw);
uint32_t BQ25731_DecodeOtgVoltageMv(uint16_t raw);
uint32_t BQ25731_DecodeChargeCurrentMa(uint16_t raw);
uint32_t BQ25731_DecodeInputCurrentMa(uint16_t raw);
bool BQ25731_DecodeConfigBlock(BQ25731_Telemetry_t *telemetry,
                               const uint8_t *data,
                               uint8_t length);
bool BQ25731_DecodeStatusBlock(BQ25731_Telemetry_t *telemetry,
                              const uint8_t *data,
                              uint8_t length);
bool BQ25731_DecodeAdcBlock(BQ25731_Telemetry_t *telemetry,
                           const uint8_t *data,
                           uint8_t length);

const char *BQ25731_StatusToString(BQ25731_Status_t status);

#endif
