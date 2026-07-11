#ifndef BQ25731_H
#define BQ25731_H

#include "main.h"
#include "tps25751.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* USER CONFIGURATION
 * Keep all normal BQ25731 tuning here. 16.5 V is the complete 5S LiFePO4
 * pack voltage, not a per-cell value. Verify every current limit against the
 * BMS, inductor, MOSFETs, PCB and cooling before increasing it. */
#define BQ_USER_CHARGING_ENABLE                    1U
#define BQ_USER_CHARGE_VOLTAGE_MV              16500U
#define BQ_USER_MAX_CHARGE_CURRENT_MA            8000U
#define BQ_USER_MAX_INPUT_CURRENT_MA             5000U
#define BQ_USER_PD_CURRENT_MARGIN_MA              250U
#define BQ_USER_MIN_INPUT_CURRENT_MA               500U
#define BQ_USER_RAC_MOHM                             5U
#define BQ_USER_RSR_MOHM                             5U
#define BQ_USER_BATTERY_IS_5S                        1U
/* Board calibration: the fitted BQ reports VBAT/VSYS 90 ADC counts low.
 * 90 * 64 mV = 5760 mV. Set to 0 after CELL_BATPRESZ/hardware is corrected. */
#define BQ_USER_VBAT_VSYS_CORRECTION_MV            5760U
/* 400 kHz requires the matching 4.7 uH/compensation design. */
#define BQ_USER_PWM_FREQUENCY_KHZ                   400U
/* EMI frequency spreading: allowed values 0, 2, 4 or 6 percent. */
#define BQ_USER_DITHER_PERCENT                        2U
#define BQ_USER_OUT_OF_AUDIO_ENABLE                   1U
#define BQ_USER_LOW_PTM_RIPPLE_ENABLE                 1U
/* Watchdog stays off; enabling it requires a periodic register refresh. */
#define BQ_USER_WATCHDOG_ENABLE                       0U

#if ((BQ_USER_RAC_MOHM != 5U) && (BQ_USER_RAC_MOHM != 10U)) || \
    ((BQ_USER_RSR_MOHM != 5U) && (BQ_USER_RSR_MOHM != 10U))
#error "BQ sense resistors must be 5 or 10 mOhm"
#endif
#if ((BQ_USER_PWM_FREQUENCY_KHZ != 400U) && (BQ_USER_PWM_FREQUENCY_KHZ != 800U))
#error "BQ PWM frequency must be 400 or 800 kHz"
#endif
#if ((BQ_USER_DITHER_PERCENT != 0U) && (BQ_USER_DITHER_PERCENT != 2U) && \
     (BQ_USER_DITHER_PERCENT != 4U) && (BQ_USER_DITHER_PERCENT != 6U))
#error "BQ dither must be 0, 2, 4 or 6 percent"
#endif
#if (BQ_USER_WATCHDOG_ENABLE != 0U)
#error "BQ watchdog refresh task is not implemented"
#endif
#if ((BQ_USER_CHARGE_VOLTAGE_MV < 1024U) || (BQ_USER_CHARGE_VOLTAGE_MV > 23000U))
#error "BQ charge voltage must be 1024..23000 mV"
#endif
#if ((BQ_USER_MAX_INPUT_CURRENT_MA == 0U) || (BQ_USER_MAX_INPUT_CURRENT_MA > 5000U))
#error "Configured USB-C input current must be 1..5000 mA"
#endif

#define BQ25731_I2C_ADDR_7BIT                         0x6BU
#define BQ25731_RAW_REGISTER_SNAPSHOT_LENGTH          0x40U
#define BQ25731_SAFE_INPUT_CURRENT_MA                 500U

#ifndef BQ25731_CONTROL_OWNER_TPS_EEPROM
#define BQ25731_CONTROL_OWNER_TPS_EEPROM              0U
#endif
#ifndef BQ25731_ALLOW_STM32_ADC_ENABLE
#define BQ25731_ALLOW_STM32_ADC_ENABLE                1U
#endif
#define BQ25731_CELL_COUNT                            (BQ_USER_BATTERY_IS_5S ? 5U : 4U)
#if ((BQ25731_CELL_COUNT != 4U) && (BQ25731_CELL_COUNT != 5U))
#error "BQ25731_CELL_COUNT must be 4 or 5 for this firmware"
#endif

/* Digital PD PSU V2.0 bring-up: OTG/VAP/FRS has a 100k pulldown. */
#define BQ25731_HW_OTG_ALLOWED                        0U

#define BQ25731_RAC_MOHM                              BQ_USER_RAC_MOHM
#define BQ25731_RSR_MOHM                              BQ_USER_RSR_MOHM
#ifndef BQ25731_BRIDGE_WRITE_TEST
#define BQ25731_BRIDGE_WRITE_TEST                     0U
#endif

typedef enum {
    BQ_SAFE_WARN_NONE                    = 0U,
    BQ_SAFE_WARN_WATCHDOG_STUCK          = (1UL << 0),
    BQ_SAFE_WARN_LOW_POWER_STUCK         = (1UL << 1),
    BQ_SAFE_WARN_IIN_MISMATCH            = (1UL << 2),
    BQ_SAFE_WARN_OTG_VAP_MODE_STUCK      = (1UL << 3),
    BQ_SAFE_WARN_EN_VBUS_VAP_STUCK       = (1UL << 4),
    BQ_SAFE_WARN_EN_ICO_STUCK            = (1UL << 5),
    BQ_SAFE_WARN_ADC_CONFIG              = (1UL << 6),
    BQ_SAFE_WARN_IN_VAP                  = (1UL << 7),
    BQ_SAFE_WARN_SENSE_CONFIG_DEFAULTED  = (1UL << 8),
    BQ_SAFE_WARN_EN_EXTILIM_STUCK        = (1UL << 9),
    BQ_SAFE_WARN_EEPROM_CONFIG_PRESENT   = (1UL << 10)
} BQ25731_SafeWarning_t;

typedef enum {
    BQ25731_OK = 0,
    BQ25731_OK_WITH_WARNINGS,
    BQ25731_ERROR,
    BQ25731_I2C_ERROR,
    BQ25731_INVALID_ARG,
    BQ25731_NOT_READY,
    BQ25731_DEVICE_ID_MISMATCH,
    BQ25731_TPS_ERROR,
    BQ25731_OTG_BLOCKED
} BQ25731_Status_t;

typedef enum {
    BQ_ERR_NONE = 0,
    BQ_ERR_PROBE_FAILED,
    BQ_ERR_I2C_READ_FAILED,
    BQ_ERR_I2C_WRITE_FAILED,
    BQ_ERR_CHRG_INHIBIT_NOT_SET,
    BQ_ERR_CHARGE_CURRENT_NOT_ZERO,
    BQ_ERR_EN_OTG_STUCK_ON,
    BQ_ERR_IN_OTG_ACTIVE,
    BQ_ERR_IN_FAST_CHARGE,
    BQ_ERR_BRIDGE_WRITE_FAILED
} BQ25731_SafeError_t;

typedef struct {
    TPS25751_Device_t *tps;
    uint8_t device_address;

    bool online;
    uint8_t last_register;
    uint32_t last_error_code;
    uint16_t adc_option_before;
    uint16_t adc_option_after;
    uint16_t adc_option_expected;
    TPS25751_Status_t last_bridge_status;
    uint8_t last_task_return_code;
    uint8_t last_bq_register;
    uint32_t last_bq_error_code;

    uint32_t last_read_tick_ms;
    uint32_t last_write_tick_ms;
    uint32_t adc_enabled_tick_ms;

    bool sense_resistor_known;
    bool rsns_rac_5mohm;
    bool rsns_rsr_5mohm;

    bool vsys_vbat_range_is_5s;
} BQ25731_Device_t;

typedef struct {
    BQ25731_Status_t status;
    uint8_t address_7bit;

    bool device_id_valid;
    bool id_ok;
    uint8_t manufacturer_id;
    uint8_t device_id;

    uint8_t raw_registers[BQ25731_RAW_REGISTER_SNAPSHOT_LENGTH];

    uint16_t charge_option0_raw;
    uint16_t charge_current_raw;
    uint16_t charge_voltage_raw;
    uint16_t otg_voltage_raw;
    uint16_t otg_current_raw;
    uint16_t iin_host_raw;

    uint16_t charger_status_raw;
    uint16_t iin_dpm_raw;
    uint16_t adc_vbus_psys_raw;
    uint16_t adc_ibat_raw;
    uint16_t adc_iin_cmpin_raw;
    uint16_t adc_vsys_vbat_raw;

    uint16_t charge_option1_raw;
    uint16_t charge_option2_raw;
    uint16_t charge_option3_raw;
    uint16_t adc_option_raw;

    uint8_t raw_adc_psys;
    uint8_t raw_adc_vbus;
    uint8_t raw_adc_idchg;
    uint8_t raw_adc_ichg;
    uint8_t raw_adc_cmpin;
    uint8_t raw_adc_iin;
    uint8_t raw_adc_vbat;
    uint8_t raw_adc_vsys;

    bool low_power_mode;
    bool watchdog_enabled;
    bool charge_inhibited;

    bool rsns_rac_5mohm;
    bool rsns_rsr_5mohm;

    bool external_input_current_limit_enabled;
    bool hiz_enabled;

    bool otg_enabled;
    bool otg_vap_mode;
    bool otg_bigcap;

    bool adc_fullscale_3v06;
    bool adc_enabled;

    bool in_otg;
    bool in_fast_charge;
    bool in_iin_dpm;
    bool in_vindpm;
    bool in_vap;
    bool ico_done;
    bool input_present;

    bool fault_otg_uvp;
    bool fault_otg_ovp;
    bool fault_force_converter_off;
    bool fault_vsys_uvp;
    bool fault_sysovp;
    bool fault_acoc;
    bool fault_batoc;
    bool fault_acov;

    uint32_t charge_current_ma;
    uint32_t charge_voltage_mv;
    uint32_t otg_voltage_mv;
    uint32_t otg_current_ma;

    uint32_t iin_host_ma;
    uint32_t iin_dpm_ma;

    uint32_t psys_mv;
    uint32_t cmpin_mv;

    uint32_t vbus_mv;
    uint32_t vbat_mv;
    uint32_t vsys_mv;

    uint32_t iin_ma;
    uint32_t ichg_ma;
    uint32_t idchg_ma;

    uint32_t input_power_mw;
    uint32_t charge_power_mw;
    uint32_t discharge_power_mw;
    uint32_t otg_output_power_mw;
} BQ25731_Telemetry_t;

typedef struct {
    uint16_t charge_current_setting_raw;
    uint16_t charge_voltage_setting_raw;
    uint16_t iin_host_raw;
    uint16_t charger_status_raw;
    uint16_t charge_option0_raw;
    uint16_t charge_option1_raw;
    uint16_t charge_option2_raw;
    uint16_t charge_option3_raw;
    uint16_t adc_option_raw;
    uint16_t adc_vbus_psys_raw;
    uint16_t adc_ibat_raw;
    uint16_t adc_iin_cmpin_raw;
    uint16_t adc_vsys_vbat_raw;
    uint32_t charge_current_setting_ma;
    uint32_t charge_voltage_setting_mv;
    uint32_t iin_host_ma;
    uint32_t adc_vbus_mv;
    uint32_t adc_vsys_mv;
    uint32_t adc_vbat_mv;
    uint32_t adc_ichg_ma;
    uint32_t adc_idchg_ma;
    uint32_t adc_iin_ma;
    uint8_t en_adc_vbat, en_adc_vsys, en_adc_ichg, en_adc_idchg;
    uint8_t en_adc_iin, en_adc_psys, en_adc_vbus, en_adc_cmpin;
    uint8_t adc_any_channel_enabled;
    uint8_t adc_required_channels_enabled;
    uint8_t adc_running;
    uint8_t adc_continuous;
    uint8_t adc_start;
    uint8_t adc_fullscale;
    uint8_t vbus_valid, vsys_valid, vbat_valid;
    uint8_t iin_valid, ichg_valid, idchg_valid;
    uint8_t low_power_mode;
} BQ25731_MonitorSnapshot_t;

typedef struct {
    bool fatal_error;
    BQ25731_SafeError_t error;
    uint32_t warnings;
    uint8_t manufacturer_id;
    uint8_t device_id;
    uint16_t charge_option0_before, charge_option0_after;
    uint16_t charge_option1_before, charge_option1_after;
    uint16_t charge_option2_before, charge_option2_after;
    uint16_t charge_option3_before, charge_option3_after;
    uint16_t charge_current_raw;
    uint16_t iin_host_raw;
    uint16_t adc_option_before, adc_option_after;
    uint16_t charger_status_raw;
    uint16_t charge_voltage_raw;
    uint32_t charge_current_ma;
    uint32_t charge_voltage_mv;
    uint32_t iin_host_ma;
} BQ25731_SafeStartupResult_t;

BQ25731_Status_t BQ25731_Init(BQ25731_Device_t *ctx,
                              TPS25751_Device_t *tps,
                              uint8_t device_address);

void BQ25731_SetVsysVbatRange5S(BQ25731_Device_t *ctx,
                                bool enabled);

BQ25731_Status_t BQ25731_CheckDevice(BQ25731_Device_t *ctx);
BQ25731_Status_t BQ25731_TestCommunication(BQ25731_Device_t *ctx);

BQ25731_Status_t BQ25731_ReadReg16(BQ25731_Device_t *ctx, uint8_t reg,
                                    uint16_t *raw);
BQ25731_Status_t BQ25731_WriteReg16(BQ25731_Device_t *ctx, uint8_t reg,
                                     uint16_t raw);
BQ25731_Status_t BQ25731_UpdateReg16(BQ25731_Device_t *ctx, uint8_t reg,
                                      uint16_t clear_mask, uint16_t set_mask,
                                      uint16_t *before, uint16_t *written,
                                      uint16_t *after);

BQ25731_Status_t BQ25731_SetSenseResistors(BQ25731_Device_t *ctx,
                                           bool rac_5mohm,
                                           bool rsr_5mohm);

BQ25731_Status_t BQ25731_EnableAdc(BQ25731_Device_t *ctx);
BQ25731_Status_t BQ25731_EnableAdcOnce(BQ25731_Device_t *ctx);

bool BQ25731_IsAdcReady(const BQ25731_Device_t *ctx,
                        const BQ25731_Telemetry_t *telemetry);

BQ25731_Status_t BQ25731_DisableExternalInputCurrentLimit(BQ25731_Device_t *ctx,
                                                          uint16_t *before_raw,
                                                          uint16_t *after_raw);

BQ25731_Status_t BQ25731_ConfigureForMonitoring(BQ25731_Device_t *ctx);
BQ25731_Status_t BQ25731_ApplyUserOptions(BQ25731_Device_t *ctx);

BQ25731_Status_t BQ25731_ConfigureSafeStartup(BQ25731_Device_t *ctx);
BQ25731_Status_t BQ25731_SafeStartup(BQ25731_Device_t *ctx,
                                      BQ25731_SafeStartupResult_t *result);
BQ25731_Status_t BQ25731_TakeoverSafeState(BQ25731_Device_t *ctx,
                                            BQ25731_SafeStartupResult_t *result);
BQ25731_Status_t BQ25731_BridgeWriteSelfTest(BQ25731_Device_t *ctx,
                                              uint16_t *old_charge_current,
                                              uint16_t *new_charge_current);
uint32_t BQ25731_DecodeChargeCurrent(uint16_t raw, bool rsns_rsr_5mohm);

BQ25731_Status_t BQ25731_PrepareForCharging(BQ25731_Device_t *ctx,
                                            uint16_t *before_raw,
                                            uint16_t *after_raw);

BQ25731_Status_t BQ25731_InhibitCharging(BQ25731_Device_t *ctx,
                                         uint16_t *before_raw,
                                         uint16_t *after_raw);

BQ25731_Status_t BQ25731_SetInputCurrentLimit(BQ25731_Device_t *ctx,
                                              uint32_t limit_ma,
                                              uint16_t *before_raw,
                                              uint16_t *after_raw);

BQ25731_Status_t BQ25731_SetChargeCurrent(BQ25731_Device_t *ctx,
                                          uint32_t charge_current_ma,
                                          uint16_t *before_raw,
                                          uint16_t *after_raw);

BQ25731_Status_t BQ25731_SetChargeVoltage(BQ25731_Device_t *ctx,
                                          uint32_t charge_voltage_mv,
                                          uint16_t *before_raw,
                                          uint16_t *after_raw);

BQ25731_Status_t BQ25731_ConfigureForCharging(BQ25731_Device_t *ctx,
                                              uint32_t input_current_ma,
                                              uint32_t charge_current_ma,
                                              uint32_t charge_voltage_mv);

BQ25731_Status_t BQ25731_EnableOtg(BQ25731_Device_t *ctx,
                                   uint32_t voltage_mv,
                                   uint32_t current_limit_ma,
                                   bool large_output_cap);

BQ25731_Status_t BQ25731_DisableOtg(BQ25731_Device_t *ctx);

BQ25731_Status_t BQ25731_ReadTelemetry(BQ25731_Device_t *ctx,
                                       BQ25731_Telemetry_t *telemetry);
BQ25731_Status_t BQ25731_ReadMonitorSnapshot(BQ25731_Device_t *dev,
                                              BQ25731_MonitorSnapshot_t *out);
BQ25731_Status_t BQ25731_EnableMonitoringAdcOnly(BQ25731_Device_t *dev);

void BQ25731_PrintRawDebug(const BQ25731_Telemetry_t *telemetry);

const char *BQ25731_StatusToString(BQ25731_Status_t status);

int BQ25731_GetDiagnosticText(const BQ25731_Device_t *ctx,
                              char *buffer,
                              size_t length);

#endif
