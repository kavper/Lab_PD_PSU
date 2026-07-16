#include "bq25731.h"

#include <string.h>

#define BQ25731_CHGSTATUS_IN_OTG          0x0100U
#define BQ25731_CHGSTATUS_IN_PRECHARGE    0x0200U
#define BQ25731_CHGSTATUS_IN_FAST_CHARGE  0x0400U
#define BQ25731_CHGSTATUS_IN_IIN_DPM      0x0800U
#define BQ25731_CHGSTATUS_IN_VINDPM       0x1000U
#define BQ25731_CHGSTATUS_INPUT_PRESENT   0x8000U

#define BQ25731_ADC_VBUS_MV_PER_LSB       96U
#define BQ25731_ADC_IIN_MA_PER_LSB        100U
#define BQ25731_ADC_ICHG_MA_PER_LSB       128U
#define BQ25731_ADC_IDCHG_MA_PER_LSB      512U
/* CELL_BATPRESZ is strapped for the BQ25731 4-cell voltage profile.
 * Per the datasheet, ADCVSYS/ADCVBAT use a 2.88 V zero-code offset for
 * this profile (the former 8.16 V offset was only valid for 5 cells). */
#define BQ25731_ADC_VBAT_VSYS_BASE_MV     2880U
#define BQ25731_ADC_VBAT_VSYS_MV_PER_LSB  64U
#define BQ25731_OTG_VOLTAGE_MV_PER_LSB        8U

BQ25731_Status_t BQ25731_Init(BQ25731_Device_t *dev,
                              TPS25751_Device_t *tps,
                              uint8_t address_7bit)
{
    if ((dev == NULL) || (tps == NULL) || (address_7bit > 0x7FU)) {
        return BQ25731_INVALID_ARG;
    }
    memset(dev, 0, sizeof(*dev));
    dev->tps = tps;
    dev->address_7bit = address_7bit;
    return BQ25731_OK;
}

BQ25731_Status_t BQ25731_MapTpsStatus(TPS25751_Status_t status)
{
    switch (status) {
        case TPS25751_OK: return BQ25731_OK;
        case TPS25751_BUSY: return BQ25731_BUSY;
        case TPS25751_INVALID_ARG: return BQ25731_INVALID_ARG;
        case TPS25751_NOT_AVAILABLE: return BQ25731_NOT_READY;
        case TPS25751_I2C_ERROR: return BQ25731_I2C_ERROR;
        default: return BQ25731_TPS_ERROR;
    }
}

BQ25731_Status_t BQ25731_StartRead16(BQ25731_Device_t *dev, uint8_t reg)
{
    if ((dev == NULL) || (dev->tps == NULL)) {
        return BQ25731_INVALID_ARG;
    }
    return BQ25731_MapTpsStatus(TPS25751_StartI2cControllerRead(
        dev->tps, dev->address_7bit, reg, 2U));
}

BQ25731_Status_t BQ25731_StartReadId(BQ25731_Device_t *dev)
{
    if ((dev == NULL) || (dev->tps == NULL)) {
        return BQ25731_INVALID_ARG;
    }
    return BQ25731_MapTpsStatus(TPS25751_StartI2cControllerRead(
        dev->tps, dev->address_7bit, BQ25731_REG_MANUFACTURER_ID, 2U));
}

BQ25731_Status_t BQ25731_StartReadConfigBlock(BQ25731_Device_t *dev)
{
    if ((dev == NULL) || (dev->tps == NULL)) {
        return BQ25731_INVALID_ARG;
    }
    return BQ25731_MapTpsStatus(TPS25751_StartI2cControllerRead(
        dev->tps, dev->address_7bit, BQ25731_REG_CHARGE_OPTION0,
        BQ25731_CONFIG_BLOCK_LEN));
}

BQ25731_Status_t BQ25731_StartReadStatusBlock(BQ25731_Device_t *dev)
{
    if ((dev == NULL) || (dev->tps == NULL)) {
        return BQ25731_INVALID_ARG;
    }
    return BQ25731_MapTpsStatus(TPS25751_StartI2cControllerRead(
        dev->tps, dev->address_7bit, BQ25731_REG_CHARGER_STATUS,
        BQ25731_STATUS_BLOCK_LEN));
}

BQ25731_Status_t BQ25731_StartReadAdcBlock(BQ25731_Device_t *dev)
{
    if ((dev == NULL) || (dev->tps == NULL)) {
        return BQ25731_INVALID_ARG;
    }
    return BQ25731_MapTpsStatus(TPS25751_StartI2cControllerRead(
        dev->tps, dev->address_7bit, BQ25731_REG_ADC_VBUS_PSYS,
        BQ25731_ADC_BLOCK_LEN));
}

BQ25731_Status_t BQ25731_StartConfigureMonitoringAdc(
    BQ25731_Device_t *dev)
{
    const uint16_t value = BQ25731_ADC_OPTION_MONITORING;
    uint8_t data[2];

    if ((dev == NULL) || (dev->tps == NULL)) {
        return BQ25731_INVALID_ARG;
    }
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    return BQ25731_MapTpsStatus(TPS25751_StartI2cControllerWrite(
        dev->tps, dev->address_7bit, BQ25731_REG_ADC_OPTION,
        data, sizeof(data)));
}

uint32_t BQ25731_DecodeChargeVoltageMv(uint16_t raw)
{
    return (uint32_t)((raw >> 3) & 0x1FFFU) * 8U;
}

uint32_t BQ25731_DecodeOtgVoltageMv(uint16_t raw)
{
    return (uint32_t)((raw >> 2) & 0x0FFFU) *
           BQ25731_OTG_VOLTAGE_MV_PER_LSB;
}

uint32_t BQ25731_DecodeChargeCurrentMa(uint16_t raw)
{
    /* Debug value for the board's configured 5 mOhm RSR. */
    return (uint32_t)((raw >> 6) & 0x7FU) * 128U;
}

uint32_t BQ25731_DecodeInputCurrentMa(uint16_t raw)
{
    /* Debug value for the board's configured 5 mOhm RAC. */
    return (uint32_t)((raw >> 8) & 0x7FU) * 100U;
}

bool BQ25731_DecodeConfigBlock(BQ25731_Telemetry_t *telemetry,
                               const uint8_t *data,
                               uint8_t length)
{
    if ((telemetry == NULL) || (data == NULL) ||
        (length < BQ25731_CONFIG_BLOCK_LEN)) {
        return false;
    }

    telemetry->charge_option0 = TPS25751_ReadLe16(&data[0x00U]);
    telemetry->charge_current = TPS25751_ReadLe16(&data[0x02U]);
    telemetry->charge_voltage = TPS25751_ReadLe16(&data[0x04U]);
    telemetry->otg_voltage = TPS25751_ReadLe16(&data[0x06U]);
    telemetry->iin_host = TPS25751_ReadLe16(&data[0x0EU]);
    telemetry->charge_current_ma =
        BQ25731_DecodeChargeCurrentMa(telemetry->charge_current);
    telemetry->charge_voltage_mv =
        BQ25731_DecodeChargeVoltageMv(telemetry->charge_voltage);
    telemetry->otg_voltage_mv =
        BQ25731_DecodeOtgVoltageMv(telemetry->otg_voltage);
    telemetry->input_current_ma =
        BQ25731_DecodeInputCurrentMa(telemetry->iin_host);
    return true;
}

bool BQ25731_DecodeStatusBlock(BQ25731_Telemetry_t *telemetry,
                              const uint8_t *data,
                              uint8_t length)
{
    if ((telemetry == NULL) || (data == NULL) ||
        (length < BQ25731_STATUS_BLOCK_LEN)) {
        return false;
    }

    telemetry->charger_status = TPS25751_ReadLe16(&data[0x00U]);
    telemetry->iin_dpm = TPS25751_ReadLe16(&data[0x04U]);
    telemetry->iin_dpm_ma =
        (uint32_t)((telemetry->iin_dpm >> 8) & 0x7FU) *
        BQ25731_ADC_IIN_MA_PER_LSB;
    if (telemetry->iin_dpm_ma == 0U) {
        telemetry->iin_dpm_ma = BQ25731_ADC_IIN_MA_PER_LSB;
    }
    telemetry->fault_flags = (uint8_t)telemetry->charger_status;
    telemetry->input_present =
        (telemetry->charger_status & BQ25731_CHGSTATUS_INPUT_PRESENT) != 0U;
    telemetry->in_precharge =
        (telemetry->charger_status & BQ25731_CHGSTATUS_IN_PRECHARGE) != 0U;
    telemetry->in_fast_charge =
        (telemetry->charger_status & BQ25731_CHGSTATUS_IN_FAST_CHARGE) != 0U;
    telemetry->in_iin_dpm =
        (telemetry->charger_status & BQ25731_CHGSTATUS_IN_IIN_DPM) != 0U;
    telemetry->in_vindpm =
        (telemetry->charger_status & BQ25731_CHGSTATUS_IN_VINDPM) != 0U;
    telemetry->in_otg =
        (telemetry->charger_status & BQ25731_CHGSTATUS_IN_OTG) != 0U;
    telemetry->online = true;
    return true;
}

bool BQ25731_DecodeAdcBlock(BQ25731_Telemetry_t *telemetry,
                           const uint8_t *data,
                           uint8_t length)
{
    uint8_t vbus;
    uint8_t idchg;
    uint8_t ichg;
    uint8_t iin;
    uint8_t vbat;
    uint8_t vsys;

    if ((telemetry == NULL) || (data == NULL) ||
        (length < BQ25731_ADC_BLOCK_LEN)) {
        return false;
    }

    telemetry->adc_vbus_psys = TPS25751_ReadLe16(&data[0x00U]);
    telemetry->adc_ibat = TPS25751_ReadLe16(&data[0x02U]);
    telemetry->adc_iin_cmpin = TPS25751_ReadLe16(&data[0x04U]);
    telemetry->adc_vsys_vbat = TPS25751_ReadLe16(&data[0x06U]);

    vbus = (uint8_t)(telemetry->adc_vbus_psys >> 8);
    idchg = (uint8_t)(telemetry->adc_ibat & 0x7FU);
    ichg = (uint8_t)((telemetry->adc_ibat >> 8) & 0x7FU);
    iin = (uint8_t)(telemetry->adc_iin_cmpin >> 8);
    vbat = (uint8_t)telemetry->adc_vsys_vbat;
    vsys = (uint8_t)(telemetry->adc_vsys_vbat >> 8);

    telemetry->adc_vbus_mv = (uint32_t)vbus *
                             BQ25731_ADC_VBUS_MV_PER_LSB;
    telemetry->adc_iin_ma = (uint32_t)iin *
                            BQ25731_ADC_IIN_MA_PER_LSB;
    telemetry->adc_ichg_ma = (uint32_t)ichg *
                             BQ25731_ADC_ICHG_MA_PER_LSB;
    telemetry->adc_idchg_ma = (uint32_t)idchg *
                              BQ25731_ADC_IDCHG_MA_PER_LSB;
    telemetry->adc_vbat_mv = BQ25731_ADC_VBAT_VSYS_BASE_MV +
        ((uint32_t)vbat * BQ25731_ADC_VBAT_VSYS_MV_PER_LSB);
    telemetry->adc_vsys_mv = BQ25731_ADC_VBAT_VSYS_BASE_MV +
        ((uint32_t)vsys * BQ25731_ADC_VBAT_VSYS_MV_PER_LSB);
    telemetry->adc_sample_valid = telemetry->adc_configured;
    telemetry->input_power_mw =
        telemetry->adc_vbus_mv * telemetry->adc_iin_ma / 1000U;
    telemetry->battery_current_ma = (int32_t)telemetry->adc_ichg_ma -
                                    (int32_t)telemetry->adc_idchg_ma;
    telemetry->battery_power_mw = (int32_t)(
        ((int64_t)telemetry->adc_vbat_mv *
         (int64_t)telemetry->battery_current_ma) / 1000LL);
    telemetry->online = true;
    return true;
}

const char *BQ25731_StatusToString(BQ25731_Status_t status)
{
    switch (status) {
        case BQ25731_OK: return "OK";
        case BQ25731_BUSY: return "BUSY";
        case BQ25731_I2C_ERROR: return "I2C_ERROR";
        case BQ25731_INVALID_ARG: return "INVALID_ARG";
        case BQ25731_NOT_READY: return "NOT_READY";
        case BQ25731_DEVICE_ID_MISMATCH: return "ID_MISMATCH";
        case BQ25731_TPS_ERROR: return "TPS_ERROR";
        default: return "ERROR";
    }
}
