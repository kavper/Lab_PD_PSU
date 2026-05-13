#ifndef BQ25731_H
#define BQ25731_H

#include "main.h"
#include "tps25751.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BQ25731_I2C_ADDR_7BIT                         0x6BU
#define BQ25731_RAW_REGISTER_SNAPSHOT_LENGTH          0x40U

typedef enum {
    BQ25731_OK = 0,
    BQ25731_ERROR,
    BQ25731_I2C_ERROR,
    BQ25731_INVALID_ARG,
    BQ25731_NOT_READY,
    BQ25731_DEVICE_ID_MISMATCH,
    BQ25731_TPS_ERROR
} BQ25731_Status_t;

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

BQ25731_Status_t BQ25731_Init(BQ25731_Device_t *ctx,
                              TPS25751_Device_t *tps,
                              uint8_t device_address);

void BQ25731_SetVsysVbatRange5S(BQ25731_Device_t *ctx,
                                bool enabled);

BQ25731_Status_t BQ25731_CheckDevice(BQ25731_Device_t *ctx);
BQ25731_Status_t BQ25731_TestCommunication(BQ25731_Device_t *ctx);

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

void BQ25731_PrintRawDebug(const BQ25731_Telemetry_t *telemetry);

const char *BQ25731_StatusToString(BQ25731_Status_t status);

int BQ25731_GetDiagnosticText(const BQ25731_Device_t *ctx,
                              char *buffer,
                              size_t length);

#endif
