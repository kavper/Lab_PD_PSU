#include "bq25731.h"

#include <stdio.h>
#include <string.h>

#define BQ25731_TPS_READ_GAP_MS               6U
#define BQ25731_TPS_WRITE_GAP_MS              6U
#define BQ25731_ADC_SETTLE_MS                 250U
#define BQ25731_TPS_READ_CHUNK_MAX            8U
#define BQ25731_WRITE_VERIFY_RETRIES          2U
#define BQ25731_WRITE_VERIFY_DELAY_MS         8U

#define BQ25731_REG_CHARGE_OPTION0            0x00U
#define BQ25731_REG_CHARGE_CURRENT            0x02U
#define BQ25731_REG_CHARGE_VOLTAGE            0x04U
#define BQ25731_REG_OTG_VOLTAGE               0x06U
#define BQ25731_REG_OTG_CURRENT               0x08U
#define BQ25731_REG_IIN_HOST                  0x0EU

#define BQ25731_REG_CHARGER_STATUS            0x20U
#define BQ25731_REG_IIN_DPM                   0x24U
#define BQ25731_REG_ADC_VBUS_PSYS             0x26U
#define BQ25731_REG_ADC_IBAT                  0x28U
#define BQ25731_REG_ADC_IIN_CMPIN             0x2AU
#define BQ25731_REG_ADC_VSYS_VBAT             0x2CU
#define BQ25731_REG_MANUFACTURE_ID            0x2EU
#define BQ25731_REG_DEVICE_ID                 0x2FU

#define BQ25731_REG_CHARGE_OPTION1            0x30U
#define BQ25731_REG_CHARGE_OPTION2            0x32U
#define BQ25731_REG_CHARGE_OPTION3            0x34U
#define BQ25731_REG_ADC_OPTION                0x3AU

#define BQ25731_MANUFACTURE_ID_EXPECTED       0x40U
#define BQ25731_DEVICE_ID_BQ25730             0xD5U
#define BQ25731_DEVICE_ID_EXPECTED            0xD6U

#define BQ25731_CHGOPT0_EN_LWPWR_MASK         0x8000U
#define BQ25731_CHGOPT0_WDTMR_ADJ_MASK        0x6000U
#define BQ25731_CHGOPT0_EN_IIN_DPM_MASK       0x0002U
#define BQ25731_CHGOPT0_CHRG_INHIBIT_MASK     0x0001U

#define BQ25731_CHGOPT1_RSNS_RAC_MASK         0x0800U
#define BQ25731_CHGOPT1_RSNS_RSR_MASK         0x0400U
#define BQ25731_CHGOPT1_EN_FAST_5MOHM_MASK    0x0100U

#define BQ25731_CHGOPT2_EN_EXTILIM_MASK       0x0080U

#define BQ25731_CHGOPT3_EN_HIZ_MASK           0x8000U
#define BQ25731_CHGOPT3_EN_OTG_MASK           0x1000U
#define BQ25731_CHGOPT3_EN_OTG_BIGCAP_MASK    0x0100U
#define BQ25731_CHGOPT3_OTG_VAP_MODE_MASK     0x0020U
#define BQ25731_CHGOPT3_IL_AVG_15A            0x0010U
#define BQ25731_CHGOPT3_CMP_EN_MASK           0x0004U

#define BQ25731_CHGOPT3_POR_BASE              (BQ25731_CHGOPT3_OTG_VAP_MODE_MASK | \
                                                BQ25731_CHGOPT3_IL_AVG_15A |        \
                                                BQ25731_CHGOPT3_CMP_EN_MASK)

#define BQ25731_CHGSTATUS_IN_OTG_MASK         0x0100U
#define BQ25731_CHGSTATUS_IN_FCHRG_MASK       0x0400U
#define BQ25731_CHGSTATUS_IN_IIN_DPM_MASK     0x0800U
#define BQ25731_CHGSTATUS_IN_VINDPM_MASK      0x1000U
#define BQ25731_CHGSTATUS_IN_VAP_MASK         0x2000U
#define BQ25731_CHGSTATUS_ICO_DONE_MASK       0x4000U
#define BQ25731_CHGSTATUS_INPUT_PRESENT_MASK  0x8000U
#define BQ25731_CHGSTATUS_FAULT_OTG_UVP_MASK  0x0001U
#define BQ25731_CHGSTATUS_FAULT_OTG_OVP_MASK  0x0002U
#define BQ25731_CHGSTATUS_FORCE_OFF_MASK      0x0004U
#define BQ25731_CHGSTATUS_FAULT_VSYS_UVP_MASK 0x0008U
#define BQ25731_CHGSTATUS_FAULT_SYSOVP_MASK   0x0010U
#define BQ25731_CHGSTATUS_FAULT_ACOC_MASK     0x0020U
#define BQ25731_CHGSTATUS_FAULT_BATOC_MASK    0x0040U
#define BQ25731_CHGSTATUS_FAULT_ACOV_MASK     0x0080U

#define BQ25731_ADCOPT_ADC_CONV_MASK          0x8000U
#define BQ25731_ADCOPT_ADC_START_MASK         0x4000U
#define BQ25731_ADCOPT_ADC_FULLSCALE_MASK     0x2000U
#define BQ25731_ADCOPT_ENABLE_MASK            0x007FU
#define BQ25731_ADCOPT_EXPECTED               0xE05FU

#define BQ25731_VERIFY_ALL_MASK               0xFFFFU
#define BQ25731_ADCOPT_VERIFY_MASK            (BQ25731_ADCOPT_ADC_CONV_MASK | \
                                                BQ25731_ADCOPT_ADC_FULLSCALE_MASK | \
                                                0x005FU)

#define BQ25731_ADC_PSYS_MV_PER_LSB_LOW       8U
#define BQ25731_ADC_PSYS_MV_PER_LSB_HIGH      12U
#define BQ25731_ADC_CMPIN_MV_PER_LSB_LOW      8U
#define BQ25731_ADC_CMPIN_MV_PER_LSB_HIGH     12U
#define BQ25731_ADC_VBUS_MV_PER_LSB           96U

#define BQ25731_ADC_IIN_MA_PER_LSB_10MOHM     50U
#define BQ25731_ADC_IIN_MA_PER_LSB_5MOHM      100U
#define BQ25731_ADC_ICHG_MA_PER_LSB_10MOHM    64U
#define BQ25731_ADC_ICHG_MA_PER_LSB_5MOHM     128U
#define BQ25731_ADC_IDCHG_MA_PER_LSB_10MOHM   256U
#define BQ25731_ADC_IDCHG_MA_PER_LSB_5MOHM    512U

#define BQ25731_ADC_VBAT_VSYS_BASE_MV_1S_4S   2880U
#define BQ25731_ADC_VBAT_VSYS_BASE_MV_5S      8160U
#define BQ25731_ADC_VBAT_VSYS_MV_PER_LSB      64U

#define BQ25731_OTG_VOLTAGE_MIN_MV            3000U
#define BQ25731_OTG_VOLTAGE_MAX_MV            24000U
#define BQ25731_OTG_VOLTAGE_MV_PER_LSB        8U

#define BQ25731_OTG_CURRENT_MAX_MA_5MOHM      12700U
#define BQ25731_OTG_CURRENT_MAX_MA_10MOHM     6350U
#define BQ25731_OTG_CURRENT_MA_PER_LSB_5MOHM  100U
#define BQ25731_OTG_CURRENT_MA_PER_LSB_10MOHM 50U

static BQ25731_Status_t BQ25731_MapTransportStatus(BQ25731_Device_t *ctx,
                                                   uint8_t reg_address,
                                                   TPS25751_Status_t status)
{
    if (ctx == NULL) {
        return BQ25731_INVALID_ARG;
    }

    ctx->last_bridge_status = status;
    ctx->last_task_return_code = 0U;
    ctx->last_bq_register = reg_address;
    ctx->last_register = reg_address;

    if (ctx->tps != NULL) {
        ctx->last_task_return_code = ctx->tps->last_i2c_controller_task_return_code;
        ctx->last_bq_error_code = ctx->tps->last_error;
    } else {
        ctx->last_bq_error_code = (uint32_t)status;
    }

    if (status == TPS25751_OK) {
        ctx->online = true;
        ctx->last_error_code = 0U;
        return BQ25731_OK;
    }

    ctx->online = false;
    ctx->last_error_code = ctx->last_bq_error_code;

    if (status == TPS25751_INVALID_ARG) {
        return BQ25731_INVALID_ARG;
    }

    if ((status == TPS25751_BAD_LENGTH) ||
        (status == TPS25751_COMMAND_ERROR)) {
        return BQ25731_TPS_ERROR;
    }

    return BQ25731_I2C_ERROR;
}

static void BQ25731_WaitForGap(uint32_t last_tick_ms, uint32_t gap_ms)
{
    uint32_t elapsed_ms;

    if (last_tick_ms == 0U) {
        return;
    }

    elapsed_ms = (uint32_t)(HAL_GetTick() - last_tick_ms);

    if (elapsed_ms >= gap_ms) {
        return;
    }

    HAL_Delay(gap_ms - elapsed_ms);
}

static BQ25731_Status_t BQ25731_ReadBlockChunk(BQ25731_Device_t *ctx,
                                               uint8_t reg_address,
                                               uint8_t *buffer,
                                               uint8_t length)
{
    TPS25751_Status_t status;

    if ((ctx == NULL) || (ctx->tps == NULL) || (buffer == NULL) || (length == 0U)) {
        return BQ25731_INVALID_ARG;
    }

    BQ25731_WaitForGap(ctx->last_read_tick_ms, BQ25731_TPS_READ_GAP_MS);

    status = TPS25751_I2cControllerRead(ctx->tps,
                                        ctx->device_address,
                                        reg_address,
                                        buffer,
                                        length);

    if (status != TPS25751_OK) {
        return BQ25731_MapTransportStatus(ctx, reg_address, status);
    }

    ctx->last_read_tick_ms = HAL_GetTick();
    ctx->online = true;
    ctx->last_bridge_status = TPS25751_OK;
    ctx->last_task_return_code = (ctx->tps != NULL) ?
                                 ctx->tps->last_i2c_controller_task_return_code :
                                 0U;
    ctx->last_bq_register = reg_address;
    ctx->last_bq_error_code = 0U;
    ctx->last_register = reg_address;
    ctx->last_error_code = 0U;

    return BQ25731_OK;
}

static BQ25731_Status_t BQ25731_ReadBlock(BQ25731_Device_t *ctx,
                                          uint8_t reg_address,
                                          uint8_t *buffer,
                                          uint8_t length)
{
    uint8_t offset = 0U;

    if ((ctx == NULL) || (buffer == NULL) || (length == 0U)) {
        return BQ25731_INVALID_ARG;
    }

    while (offset < length) {
        uint8_t chunk_length = (uint8_t)(length - offset);
        BQ25731_Status_t status;

        if (chunk_length > BQ25731_TPS_READ_CHUNK_MAX) {
            chunk_length = BQ25731_TPS_READ_CHUNK_MAX;
        }

        status = BQ25731_ReadBlockChunk(ctx,
                                        (uint8_t)(reg_address + offset),
                                        &buffer[offset],
                                        chunk_length);

        if (status != BQ25731_OK) {
            return status;
        }

        offset = (uint8_t)(offset + chunk_length);
    }

    return BQ25731_OK;
}

static BQ25731_Status_t BQ25731_Write16(BQ25731_Device_t *ctx,
                                        uint8_t reg_address,
                                        uint16_t value)
{
    uint8_t raw[2];
    TPS25751_Status_t status;

    if ((ctx == NULL) || (ctx->tps == NULL)) {
        return BQ25731_INVALID_ARG;
    }

    raw[0] = (uint8_t)(value & 0xFFU);
    raw[1] = (uint8_t)((value >> 8) & 0xFFU);

    BQ25731_WaitForGap(ctx->last_write_tick_ms, BQ25731_TPS_WRITE_GAP_MS);

    status = TPS25751_I2cControllerWrite(ctx->tps,
                                         ctx->device_address,
                                         reg_address,
                                         raw,
                                         sizeof(raw));

    if (status != TPS25751_OK) {
        return BQ25731_MapTransportStatus(ctx, reg_address, status);
    }

    ctx->last_write_tick_ms = HAL_GetTick();
    ctx->online = true;
    ctx->last_bridge_status = TPS25751_OK;
    ctx->last_task_return_code = (ctx->tps != NULL) ?
                                 ctx->tps->last_i2c_controller_task_return_code :
                                 0U;
    ctx->last_bq_register = reg_address;
    ctx->last_bq_error_code = 0U;
    ctx->last_register = reg_address;
    ctx->last_error_code = 0U;

    return BQ25731_OK;
}

static BQ25731_Status_t BQ25731_Read16(BQ25731_Device_t *ctx,
                                       uint8_t reg_address,
                                       uint16_t *value)
{
    uint8_t raw[2];
    BQ25731_Status_t status;

    if (value == NULL) {
        return BQ25731_INVALID_ARG;
    }

    status = BQ25731_ReadBlock(ctx, reg_address, raw, sizeof(raw));
    if (status != BQ25731_OK) {
        return status;
    }

    *value = (uint16_t)raw[0] | ((uint16_t)raw[1] << 8);

    return BQ25731_OK;
}

static BQ25731_Status_t BQ25731_Write16Verified(BQ25731_Device_t *ctx,
                                                uint8_t reg_address,
                                                uint16_t value,
                                                uint16_t verify_mask)
{
    BQ25731_Status_t status;
    uint16_t readback = 0U;
    uint8_t attempt;

    if ((ctx == NULL) || (verify_mask == 0U)) {
        return BQ25731_INVALID_ARG;
    }

    for (attempt = 0U; attempt <= BQ25731_WRITE_VERIFY_RETRIES; ++attempt) {
        status = BQ25731_Write16(ctx, reg_address, value);
        if (status != BQ25731_OK) {
            return status;
        }

        HAL_Delay(BQ25731_WRITE_VERIFY_DELAY_MS);

        status = BQ25731_Read16(ctx, reg_address, &readback);
        if (status != BQ25731_OK) {
            return status;
        }

        if ((readback & verify_mask) == (value & verify_mask)) {
            return BQ25731_OK;
        }
    }

    ctx->online = false;
    ctx->last_bridge_status = TPS25751_COMMAND_ERROR;
    ctx->last_bq_register = reg_address;
    ctx->last_bq_error_code = ((uint32_t)(value & verify_mask) << 16) |
                              (uint32_t)(readback & verify_mask);
    ctx->last_register = reg_address;
    ctx->last_error_code = ctx->last_bq_error_code;

    return BQ25731_ERROR;
}

static void BQ25731_SaveRawRange(BQ25731_Telemetry_t *telemetry,
                                 uint8_t reg_address,
                                 const uint8_t *buffer,
                                 uint8_t length)
{
    uint8_t i;

    if ((telemetry == NULL) || (buffer == NULL)) {
        return;
    }

    for (i = 0U; i < length; ++i) {
        uint16_t target = (uint16_t)reg_address + (uint16_t)i;

        if (target >= BQ25731_RAW_REGISTER_SNAPSHOT_LENGTH) {
            return;
        }

        telemetry->raw_registers[target] = buffer[i];
    }
}

static void BQ25731_SaveRaw16(BQ25731_Telemetry_t *telemetry,
                              uint8_t reg_address,
                              uint16_t value)
{
    uint8_t raw[2];

    raw[0] = (uint8_t)(value & 0xFFU);
    raw[1] = (uint8_t)((value >> 8) & 0xFFU);

    BQ25731_SaveRawRange(telemetry, reg_address, raw, sizeof(raw));
}

static BQ25731_Status_t BQ25731_Read16WithSnapshot(BQ25731_Device_t *ctx,
                                                   BQ25731_Telemetry_t *telemetry,
                                                   uint8_t reg_address,
                                                   uint16_t *value)
{
    BQ25731_Status_t status;

    status = BQ25731_Read16(ctx, reg_address, value);
    if (status != BQ25731_OK) {
        return status;
    }

    BQ25731_SaveRaw16(telemetry, reg_address, *value);

    return BQ25731_OK;
}

static bool BQ25731_IsSupportedDeviceId(uint8_t manufacturer_id, uint8_t device_id)
{
    return (manufacturer_id == BQ25731_MANUFACTURE_ID_EXPECTED) &&
           ((device_id == BQ25731_DEVICE_ID_EXPECTED) ||
            (device_id == BQ25731_DEVICE_ID_BQ25730));
}

static uint32_t BQ25731_GetVsysVbatBaseMv(const BQ25731_Device_t *ctx)
{
    if ((ctx != NULL) && ctx->vsys_vbat_range_is_5s) {
        return BQ25731_ADC_VBAT_VSYS_BASE_MV_5S;
    }

    return BQ25731_ADC_VBAT_VSYS_BASE_MV_1S_4S;
}

static void BQ25731_SaveSenseResistorConfig(BQ25731_Device_t *ctx,
                                            uint16_t charge_option1)
{
    if (ctx == NULL) {
        return;
    }

    ctx->sense_resistor_known = true;
    ctx->rsns_rac_5mohm = (charge_option1 & BQ25731_CHGOPT1_RSNS_RAC_MASK) != 0U;
    ctx->rsns_rsr_5mohm = (charge_option1 & BQ25731_CHGOPT1_RSNS_RSR_MASK) != 0U;
}

static uint32_t BQ25731_DecodeInputLimitMa(uint16_t raw, bool rsns_rac_5mohm)
{
    uint32_t code = (uint32_t)((raw >> 8) & 0x7FU);
    uint32_t step_ma = rsns_rac_5mohm ? 100U : 50U;

    if (code == 0U) {
        return step_ma;
    }

    return code * step_ma;
}

static uint32_t BQ25731_DecodeChargeCurrentMa(uint16_t raw, bool rsns_rsr_5mohm)
{
    uint32_t code = (uint32_t)((raw >> 6) & 0x7FU);
    uint32_t step_ma = rsns_rsr_5mohm ? 128U : 64U;

    return code * step_ma;
}

static uint32_t BQ25731_DecodeChargeVoltageMv(uint16_t raw)
{
    uint32_t code = (uint32_t)((raw >> 3) & 0x0FFFU);

    return code * 8U;
}

static uint32_t BQ25731_DecodeOtgVoltageMv(uint16_t raw)
{
    uint32_t code = (uint32_t)((raw >> 2) & 0x0FFFU);

    return code * BQ25731_OTG_VOLTAGE_MV_PER_LSB;
}

static uint32_t BQ25731_DecodeOtgCurrentMa(uint16_t raw, bool rsns_rac_5mohm)
{
    uint32_t code = (uint32_t)((raw >> 8) & 0x7FU);
    uint32_t step_ma = rsns_rac_5mohm ?
                       BQ25731_OTG_CURRENT_MA_PER_LSB_5MOHM :
                       BQ25731_OTG_CURRENT_MA_PER_LSB_10MOHM;

    return code * step_ma;
}

static uint16_t BQ25731_EncodeInputLimitRaw(uint32_t limit_ma,
                                            bool rsns_rac_5mohm)
{
    uint32_t step_ma = rsns_rac_5mohm ? 100U : 50U;
    uint32_t code;

    if (limit_ma <= step_ma) {
        code = 0U;
    } else {
        code = (limit_ma + step_ma - 1U) / step_ma;
    }

    if (code > 127U) {
        code = 127U;
    }

    return (uint16_t)((code & 0x7FU) << 8);
}

static uint16_t BQ25731_EncodeChargeCurrentRaw(uint32_t charge_current_ma,
                                               bool rsns_rsr_5mohm)
{
    uint32_t step_ma = rsns_rsr_5mohm ? 128U : 64U;
    uint32_t code = (charge_current_ma + step_ma - 1U) / step_ma;

    if (code > 127U) {
        code = 127U;
    }

    return (uint16_t)((code & 0x7FU) << 6);
}

static uint16_t BQ25731_EncodeChargeVoltageRaw(uint32_t charge_voltage_mv)
{
    uint32_t code;

    if (charge_voltage_mv < 1024U) {
        charge_voltage_mv = 1024U;
    }

    if (charge_voltage_mv > 23000U) {
        charge_voltage_mv = 23000U;
    }

    code = (charge_voltage_mv + 7U) / 8U;

    if (code > 0x0FFFU) {
        code = 0x0FFFU;
    }

    return (uint16_t)((code & 0x0FFFU) << 3);
}

static uint16_t BQ25731_EncodeOtgVoltageRaw(uint32_t voltage_mv)
{
    uint32_t code;

    if (voltage_mv < BQ25731_OTG_VOLTAGE_MIN_MV) {
        voltage_mv = BQ25731_OTG_VOLTAGE_MIN_MV;
    }

    if (voltage_mv > BQ25731_OTG_VOLTAGE_MAX_MV) {
        voltage_mv = BQ25731_OTG_VOLTAGE_MAX_MV;
    }

    code = (voltage_mv + (BQ25731_OTG_VOLTAGE_MV_PER_LSB - 1U)) /
           BQ25731_OTG_VOLTAGE_MV_PER_LSB;

    if (code > 0x0FFFU) {
        code = 0x0FFFU;
    }

    return (uint16_t)((code & 0x0FFFU) << 2);
}

static uint16_t BQ25731_EncodeOtgCurrentRaw(uint32_t current_limit_ma,
                                            bool rsns_rac_5mohm)
{
    uint32_t step_ma;
    uint32_t max_ma;
    uint32_t code;

    if (rsns_rac_5mohm) {
        step_ma = BQ25731_OTG_CURRENT_MA_PER_LSB_5MOHM;
        max_ma = BQ25731_OTG_CURRENT_MAX_MA_5MOHM;
    } else {
        step_ma = BQ25731_OTG_CURRENT_MA_PER_LSB_10MOHM;
        max_ma = BQ25731_OTG_CURRENT_MAX_MA_10MOHM;
    }

    if (current_limit_ma > max_ma) {
        current_limit_ma = max_ma;
    }

    if (current_limit_ma < step_ma) {
        current_limit_ma = step_ma;
    }

    code = (current_limit_ma + step_ma - 1U) / step_ma;

    if (code > 0x7FU) {
        code = 0x7FU;
    }

    return (uint16_t)((code & 0x7FU) << 8);
}

static void BQ25731_DecodeTelemetry(const BQ25731_Device_t *ctx,
                                    BQ25731_Telemetry_t *telemetry)
{
    uint32_t psys_lsb_mv;
    uint32_t cmpin_lsb_mv;
    uint32_t iin_lsb_ma;
    uint32_t ichg_lsb_ma;
    uint32_t idchg_lsb_ma;
    uint32_t vsys_vbat_base_mv;

    if ((ctx == NULL) || (telemetry == NULL)) {
        return;
    }

    telemetry->low_power_mode = (telemetry->charge_option0_raw & BQ25731_CHGOPT0_EN_LWPWR_MASK) != 0U;
    telemetry->watchdog_enabled = (telemetry->charge_option0_raw & BQ25731_CHGOPT0_WDTMR_ADJ_MASK) != 0U;
    telemetry->charge_inhibited = (telemetry->charge_option0_raw & BQ25731_CHGOPT0_CHRG_INHIBIT_MASK) != 0U;

    telemetry->rsns_rac_5mohm = (telemetry->charge_option1_raw & BQ25731_CHGOPT1_RSNS_RAC_MASK) != 0U;
    telemetry->rsns_rsr_5mohm = (telemetry->charge_option1_raw & BQ25731_CHGOPT1_RSNS_RSR_MASK) != 0U;

    telemetry->external_input_current_limit_enabled =
        (telemetry->charge_option2_raw & BQ25731_CHGOPT2_EN_EXTILIM_MASK) != 0U;

    telemetry->otg_enabled = (telemetry->charge_option3_raw & BQ25731_CHGOPT3_EN_OTG_MASK) != 0U;
    telemetry->otg_vap_mode = (telemetry->charge_option3_raw & BQ25731_CHGOPT3_OTG_VAP_MODE_MASK) != 0U;
    telemetry->otg_bigcap = (telemetry->charge_option3_raw & BQ25731_CHGOPT3_EN_OTG_BIGCAP_MASK) != 0U;
    telemetry->hiz_enabled = (telemetry->charge_option3_raw & BQ25731_CHGOPT3_EN_HIZ_MASK) != 0U;

    telemetry->adc_fullscale_3v06 = (telemetry->adc_option_raw & BQ25731_ADCOPT_ADC_FULLSCALE_MASK) != 0U;

    telemetry->in_otg = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_IN_OTG_MASK) != 0U;
    telemetry->in_fast_charge = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_IN_FCHRG_MASK) != 0U;
    telemetry->in_iin_dpm = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_IN_IIN_DPM_MASK) != 0U;
    telemetry->in_vindpm = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_IN_VINDPM_MASK) != 0U;
    telemetry->in_vap = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_IN_VAP_MASK) != 0U;
    telemetry->ico_done = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_ICO_DONE_MASK) != 0U;
    telemetry->input_present = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_INPUT_PRESENT_MASK) != 0U;

    telemetry->fault_otg_uvp = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_FAULT_OTG_UVP_MASK) != 0U;
    telemetry->fault_otg_ovp = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_FAULT_OTG_OVP_MASK) != 0U;
    telemetry->fault_force_converter_off = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_FORCE_OFF_MASK) != 0U;
    telemetry->fault_vsys_uvp = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_FAULT_VSYS_UVP_MASK) != 0U;
    telemetry->fault_sysovp = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_FAULT_SYSOVP_MASK) != 0U;
    telemetry->fault_acoc = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_FAULT_ACOC_MASK) != 0U;
    telemetry->fault_batoc = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_FAULT_BATOC_MASK) != 0U;
    telemetry->fault_acov = (telemetry->charger_status_raw & BQ25731_CHGSTATUS_FAULT_ACOV_MASK) != 0U;

    psys_lsb_mv = telemetry->adc_fullscale_3v06 ? BQ25731_ADC_PSYS_MV_PER_LSB_HIGH :
                                                  BQ25731_ADC_PSYS_MV_PER_LSB_LOW;

    cmpin_lsb_mv = telemetry->adc_fullscale_3v06 ? BQ25731_ADC_CMPIN_MV_PER_LSB_HIGH :
                                                   BQ25731_ADC_CMPIN_MV_PER_LSB_LOW;

    iin_lsb_ma = telemetry->rsns_rac_5mohm ? BQ25731_ADC_IIN_MA_PER_LSB_5MOHM :
                                             BQ25731_ADC_IIN_MA_PER_LSB_10MOHM;

    ichg_lsb_ma = telemetry->rsns_rsr_5mohm ? BQ25731_ADC_ICHG_MA_PER_LSB_5MOHM :
                                              BQ25731_ADC_ICHG_MA_PER_LSB_10MOHM;

    idchg_lsb_ma = telemetry->rsns_rsr_5mohm ? BQ25731_ADC_IDCHG_MA_PER_LSB_5MOHM :
                                               BQ25731_ADC_IDCHG_MA_PER_LSB_10MOHM;

    vsys_vbat_base_mv = BQ25731_GetVsysVbatBaseMv(ctx);

    telemetry->charge_current_ma =
        BQ25731_DecodeChargeCurrentMa(telemetry->charge_current_raw,
                                      telemetry->rsns_rsr_5mohm);

    telemetry->charge_voltage_mv =
        BQ25731_DecodeChargeVoltageMv(telemetry->charge_voltage_raw);

    telemetry->otg_voltage_mv =
        BQ25731_DecodeOtgVoltageMv(telemetry->otg_voltage_raw);

    telemetry->otg_current_ma =
        BQ25731_DecodeOtgCurrentMa(telemetry->otg_current_raw,
                                   telemetry->rsns_rac_5mohm);

    telemetry->iin_host_ma =
        BQ25731_DecodeInputLimitMa(telemetry->iin_host_raw,
                                   telemetry->rsns_rac_5mohm);

    telemetry->iin_dpm_ma =
        BQ25731_DecodeInputLimitMa(telemetry->iin_dpm_raw,
                                   telemetry->rsns_rac_5mohm);

    telemetry->psys_mv = (uint32_t)telemetry->raw_adc_psys * psys_lsb_mv;
    telemetry->vbus_mv = (uint32_t)telemetry->raw_adc_vbus * BQ25731_ADC_VBUS_MV_PER_LSB;

    telemetry->idchg_ma = (uint32_t)(telemetry->raw_adc_idchg & 0x7FU) * idchg_lsb_ma;
    telemetry->ichg_ma = (uint32_t)(telemetry->raw_adc_ichg & 0x7FU) * ichg_lsb_ma;

    telemetry->cmpin_mv = (uint32_t)telemetry->raw_adc_cmpin * cmpin_lsb_mv;
    telemetry->iin_ma = (uint32_t)telemetry->raw_adc_iin * iin_lsb_ma;

    telemetry->vbat_mv = vsys_vbat_base_mv +
                         ((uint32_t)telemetry->raw_adc_vbat * BQ25731_ADC_VBAT_VSYS_MV_PER_LSB);

    telemetry->vsys_mv = vsys_vbat_base_mv +
                         ((uint32_t)telemetry->raw_adc_vsys * BQ25731_ADC_VBAT_VSYS_MV_PER_LSB);

    telemetry->input_power_mw = (telemetry->vbus_mv * telemetry->iin_ma) / 1000U;
    telemetry->charge_power_mw = (telemetry->vbat_mv * telemetry->ichg_ma) / 1000U;
    telemetry->discharge_power_mw = (telemetry->vbat_mv * telemetry->idchg_ma) / 1000U;
    telemetry->otg_output_power_mw = (telemetry->vbus_mv * telemetry->idchg_ma) / 1000U;
}

BQ25731_Status_t BQ25731_Init(BQ25731_Device_t *ctx,
                              TPS25751_Device_t *tps,
                              uint8_t device_address)
{
    if ((ctx == NULL) || (tps == NULL)) {
        return BQ25731_INVALID_ARG;
    }

    memset(ctx, 0, sizeof(*ctx));

    ctx->tps = tps;
    ctx->device_address = device_address;
    ctx->vsys_vbat_range_is_5s = false;
    ctx->adc_option_before = 0U;
    ctx->adc_option_after = 0U;
    ctx->adc_option_expected = BQ25731_ADCOPT_EXPECTED;
    ctx->last_bridge_status = TPS25751_OK;

    return BQ25731_OK;
}

void BQ25731_SetVsysVbatRange5S(BQ25731_Device_t *ctx, bool enabled)
{
    if (ctx == NULL) {
        return;
    }

    ctx->vsys_vbat_range_is_5s = enabled;
}

BQ25731_Status_t BQ25731_CheckDevice(BQ25731_Device_t *ctx)
{
    return BQ25731_TestCommunication(ctx);
}

BQ25731_Status_t BQ25731_TestCommunication(BQ25731_Device_t *ctx)
{
    BQ25731_Status_t status;
    uint8_t id[2];

    if (ctx == NULL) {
        return BQ25731_INVALID_ARG;
    }

    status = BQ25731_ReadBlock(ctx, BQ25731_REG_MANUFACTURE_ID, id, sizeof(id));
    if (status != BQ25731_OK) {
        return status;
    }

    if (!BQ25731_IsSupportedDeviceId(id[0], id[1])) {
        ctx->online = false;
        ctx->last_bq_register = BQ25731_REG_MANUFACTURE_ID;
        ctx->last_bq_error_code = ((uint32_t)id[0] << 8) | id[1];
        ctx->last_error_code = ctx->last_bq_error_code;
        return BQ25731_DEVICE_ID_MISMATCH;
    }

    ctx->online = true;
    ctx->last_error_code = 0U;
    ctx->last_bq_error_code = 0U;

    return BQ25731_OK;
}

BQ25731_Status_t BQ25731_SetSenseResistors(BQ25731_Device_t *ctx,
                                           bool rac_5mohm,
                                           bool rsr_5mohm)
{
    BQ25731_Status_t status;
    uint16_t option1;

    if (ctx == NULL) {
        return BQ25731_INVALID_ARG;
    }

    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION1, &option1);
    if (status != BQ25731_OK) {
        return status;
    }

    if (rac_5mohm) {
        option1 |= BQ25731_CHGOPT1_RSNS_RAC_MASK;
        option1 |= BQ25731_CHGOPT1_EN_FAST_5MOHM_MASK;
    } else {
        option1 &= (uint16_t)~BQ25731_CHGOPT1_RSNS_RAC_MASK;
    }

    if (rsr_5mohm) {
        option1 |= BQ25731_CHGOPT1_RSNS_RSR_MASK;
    } else {
        option1 &= (uint16_t)~BQ25731_CHGOPT1_RSNS_RSR_MASK;
    }

    status = BQ25731_Write16Verified(ctx,
                                     BQ25731_REG_CHARGE_OPTION1,
                                     option1,
                                     BQ25731_VERIFY_ALL_MASK);
    if (status != BQ25731_OK) {
        return status;
    }

    BQ25731_SaveSenseResistorConfig(ctx, option1);

    return BQ25731_OK;
}

BQ25731_Status_t BQ25731_EnableAdc(BQ25731_Device_t *ctx)
{
    BQ25731_Status_t status;
    uint16_t option0 = 0U;
    uint16_t adc_before = 0U;
    uint16_t adc_after = 0U;
    uint16_t adc_config = BQ25731_ADCOPT_EXPECTED;

    if (ctx == NULL) {
        return BQ25731_INVALID_ARG;
    }

    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION0, &option0);
    if (status != BQ25731_OK) {
        return status;
    }

    option0 &= (uint16_t)~BQ25731_CHGOPT0_EN_LWPWR_MASK;

    status = BQ25731_Write16(ctx, BQ25731_REG_CHARGE_OPTION0, option0);
    if (status != BQ25731_OK) {
        return status;
    }

    HAL_Delay(20U);

    status = BQ25731_Read16(ctx, BQ25731_REG_ADC_OPTION, &adc_before);
    if (status != BQ25731_OK) {
        return status;
    }

    status = BQ25731_Write16(ctx, BQ25731_REG_ADC_OPTION, adc_config);
    if (status != BQ25731_OK) {
        return status;
    }

    HAL_Delay(80U);

    status = BQ25731_Read16(ctx, BQ25731_REG_ADC_OPTION, &adc_after);
    if (status != BQ25731_OK) {
        return status;
    }

    ctx->adc_option_before = adc_before;
    ctx->adc_option_after = adc_after;
    ctx->adc_option_expected = adc_config;
    ctx->last_bq_register = BQ25731_REG_ADC_OPTION;
    ctx->last_error_code = ((uint32_t)adc_before << 16) | adc_after;
    ctx->last_bq_error_code = ctx->last_error_code;
    ctx->adc_enabled_tick_ms = HAL_GetTick();

    return BQ25731_OK;
}

BQ25731_Status_t BQ25731_EnableAdcOnce(BQ25731_Device_t *ctx)
{
    BQ25731_Status_t status;
    uint16_t adc_before = 0U;
    uint16_t adc_after = 0U;
    uint16_t adc_expected = BQ25731_ADCOPT_EXPECTED;

    if (ctx == NULL) {
        return BQ25731_INVALID_ARG;
    }

    status = BQ25731_Read16(ctx, BQ25731_REG_ADC_OPTION, &adc_before);
    if (status != BQ25731_OK) {
        return status;
    }

    status = BQ25731_Write16(ctx, BQ25731_REG_ADC_OPTION, adc_expected);
    if (status != BQ25731_OK) {
        return status;
    }

    HAL_Delay(20U);

    status = BQ25731_Read16(ctx, BQ25731_REG_ADC_OPTION, &adc_after);
    if (status != BQ25731_OK) {
        return status;
    }

    ctx->adc_option_before = adc_before;
    ctx->adc_option_after = adc_after;
    ctx->adc_option_expected = adc_expected;
    ctx->last_bq_register = BQ25731_REG_ADC_OPTION;
    ctx->last_error_code = ((uint32_t)adc_before << 16) | adc_after;
    ctx->last_bq_error_code = ctx->last_error_code;

    if ((adc_after & BQ25731_ADCOPT_VERIFY_MASK) !=
        (adc_expected & BQ25731_ADCOPT_VERIFY_MASK)) {
        ctx->adc_enabled_tick_ms = 0U;
        ctx->online = false;
        return BQ25731_ERROR;
    }

    ctx->adc_enabled_tick_ms = HAL_GetTick();
    ctx->online = true;
    return BQ25731_OK;
}


bool BQ25731_IsAdcReady(const BQ25731_Device_t *ctx,
                        const BQ25731_Telemetry_t *telemetry)
{
    uint32_t elapsed_ms;

    if ((ctx == NULL) || (telemetry == NULL)) {
        return false;
    }

    if ((telemetry->adc_option_raw & BQ25731_ADCOPT_VERIFY_MASK) !=
        (ctx->adc_option_expected & BQ25731_ADCOPT_VERIFY_MASK)) {
        return false;
    }

    if (ctx->adc_enabled_tick_ms == 0U) {
        return false;
    }

    elapsed_ms = (uint32_t)(HAL_GetTick() - ctx->adc_enabled_tick_ms);

    return (elapsed_ms >= BQ25731_ADC_SETTLE_MS);
}

BQ25731_Status_t BQ25731_DisableExternalInputCurrentLimit(BQ25731_Device_t *ctx,
                                                          uint16_t *before_raw,
                                                          uint16_t *after_raw)
{
    BQ25731_Status_t status;
    uint16_t option2;

    if (ctx == NULL) {
        return BQ25731_INVALID_ARG;
    }

    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION2, &option2);
    if (status != BQ25731_OK) {
        return status;
    }

    if (before_raw != NULL) {
        *before_raw = option2;
    }

    option2 &= (uint16_t)~BQ25731_CHGOPT2_EN_EXTILIM_MASK;

    status = BQ25731_Write16Verified(ctx,
                                     BQ25731_REG_CHARGE_OPTION2,
                                     option2,
                                     BQ25731_VERIFY_ALL_MASK);
    if (status != BQ25731_OK) {
        return status;
    }

    if (after_raw != NULL) {
        *after_raw = option2;
    }

    return BQ25731_OK;
}

BQ25731_Status_t BQ25731_ConfigureForMonitoring(BQ25731_Device_t *ctx)
{
    BQ25731_Status_t status;
    uint16_t option0;
    uint16_t option3;

    if (ctx == NULL) {
        return BQ25731_INVALID_ARG;
    }

    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION0, &option0);
    if (status != BQ25731_OK) {
        return status;
    }

    option0 &= (uint16_t)~BQ25731_CHGOPT0_EN_LWPWR_MASK;
    option0 &= (uint16_t)~BQ25731_CHGOPT0_WDTMR_ADJ_MASK;
    option0 |= BQ25731_CHGOPT0_EN_IIN_DPM_MASK;

    status = BQ25731_Write16Verified(ctx,
                                     BQ25731_REG_CHARGE_OPTION0,
                                     option0,
                                     BQ25731_VERIFY_ALL_MASK);
    if (status != BQ25731_OK) {
        return status;
    }

    status = BQ25731_DisableExternalInputCurrentLimit(ctx, NULL, NULL);
    if (status != BQ25731_OK) {
        return status;
    }

    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION3, &option3);
    if (status != BQ25731_OK) {
        return status;
    }

    option3 &= (uint16_t)~BQ25731_CHGOPT3_EN_HIZ_MASK;
    option3 &= (uint16_t)~BQ25731_CHGOPT3_EN_OTG_MASK;
    option3 |= BQ25731_CHGOPT3_POR_BASE;

    status = BQ25731_Write16Verified(ctx,
                                     BQ25731_REG_CHARGE_OPTION3,
                                     option3,
                                     BQ25731_VERIFY_ALL_MASK);
    if (status != BQ25731_OK) {
        return status;
    }

    return BQ25731_EnableAdc(ctx);
}

BQ25731_Status_t BQ25731_PrepareForCharging(BQ25731_Device_t *ctx,
                                            uint16_t *before_raw,
                                            uint16_t *after_raw)
{
    BQ25731_Status_t status;
    uint16_t option0;
    uint16_t option3;

    if (ctx == NULL) {
        return BQ25731_INVALID_ARG;
    }

    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION0, &option0);
    if (status != BQ25731_OK) {
        return status;
    }

    if (before_raw != NULL) {
        *before_raw = option0;
    }

    option0 &= (uint16_t)~BQ25731_CHGOPT0_EN_LWPWR_MASK;
    option0 &= (uint16_t)~BQ25731_CHGOPT0_WDTMR_ADJ_MASK;
    option0 &= (uint16_t)~BQ25731_CHGOPT0_CHRG_INHIBIT_MASK;
    option0 |= BQ25731_CHGOPT0_EN_IIN_DPM_MASK;

    status = BQ25731_Write16Verified(ctx,
                                     BQ25731_REG_CHARGE_OPTION0,
                                     option0,
                                     BQ25731_VERIFY_ALL_MASK);
    if (status != BQ25731_OK) {
        return status;
    }

    if (after_raw != NULL) {
        *after_raw = option0;
    }

    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION3, &option3);
    if (status != BQ25731_OK) {
        return status;
    }

    option3 &= (uint16_t)~BQ25731_CHGOPT3_EN_HIZ_MASK;
    option3 &= (uint16_t)~BQ25731_CHGOPT3_EN_OTG_MASK;
    option3 |= BQ25731_CHGOPT3_POR_BASE;

    return BQ25731_Write16Verified(ctx,
                                   BQ25731_REG_CHARGE_OPTION3,
                                   option3,
                                   BQ25731_VERIFY_ALL_MASK);
}

BQ25731_Status_t BQ25731_InhibitCharging(BQ25731_Device_t *ctx,
                                         uint16_t *before_raw,
                                         uint16_t *after_raw)
{
    BQ25731_Status_t status;
    uint16_t option0;

    if (ctx == NULL) {
        return BQ25731_INVALID_ARG;
    }

    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION0, &option0);
    if (status != BQ25731_OK) {
        return status;
    }

    if (before_raw != NULL) {
        *before_raw = option0;
    }

    option0 |= BQ25731_CHGOPT0_CHRG_INHIBIT_MASK;

    status = BQ25731_Write16Verified(ctx,
                                     BQ25731_REG_CHARGE_OPTION0,
                                     option0,
                                     BQ25731_VERIFY_ALL_MASK);
    if (status != BQ25731_OK) {
        return status;
    }

    if (after_raw != NULL) {
        *after_raw = option0;
    }

    return BQ25731_OK;
}

BQ25731_Status_t BQ25731_SetInputCurrentLimit(BQ25731_Device_t *ctx,
                                              uint32_t limit_ma,
                                              uint16_t *before_raw,
                                              uint16_t *after_raw)
{
    BQ25731_Status_t status;
    uint16_t option1;
    uint16_t raw;

    if ((ctx == NULL) || (limit_ma == 0U)) {
        return BQ25731_INVALID_ARG;
    }

    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION1, &option1);
    if (status != BQ25731_OK) {
        return status;
    }

    BQ25731_SaveSenseResistorConfig(ctx, option1);

    if (before_raw != NULL) {
        status = BQ25731_Read16(ctx, BQ25731_REG_IIN_HOST, before_raw);
        if (status != BQ25731_OK) {
            return status;
        }
    }

    raw = BQ25731_EncodeInputLimitRaw(limit_ma, ctx->rsns_rac_5mohm);

    status = BQ25731_Write16Verified(ctx,
                                     BQ25731_REG_IIN_HOST,
                                     raw,
                                     BQ25731_VERIFY_ALL_MASK);
    if (status != BQ25731_OK) {
        return status;
    }

    if (after_raw != NULL) {
        *after_raw = raw;
    }

    return BQ25731_OK;
}

BQ25731_Status_t BQ25731_SetChargeCurrent(BQ25731_Device_t *ctx,
                                          uint32_t charge_current_ma,
                                          uint16_t *before_raw,
                                          uint16_t *after_raw)
{
    BQ25731_Status_t status;
    uint16_t option1;
    uint16_t raw;

    if (ctx == NULL) {
        return BQ25731_INVALID_ARG;
    }

    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION1, &option1);
    if (status != BQ25731_OK) {
        return status;
    }

    BQ25731_SaveSenseResistorConfig(ctx, option1);

    if (before_raw != NULL) {
        status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_CURRENT, before_raw);
        if (status != BQ25731_OK) {
            return status;
        }
    }

    raw = BQ25731_EncodeChargeCurrentRaw(charge_current_ma, ctx->rsns_rsr_5mohm);

    status = BQ25731_Write16Verified(ctx,
                                     BQ25731_REG_CHARGE_CURRENT,
                                     raw,
                                     BQ25731_VERIFY_ALL_MASK);
    if (status != BQ25731_OK) {
        return status;
    }

    if (after_raw != NULL) {
        *after_raw = raw;
    }

    return BQ25731_OK;
}

BQ25731_Status_t BQ25731_SetChargeVoltage(BQ25731_Device_t *ctx,
                                          uint32_t charge_voltage_mv,
                                          uint16_t *before_raw,
                                          uint16_t *after_raw)
{
    BQ25731_Status_t status;
    uint16_t raw;

    if ((ctx == NULL) || (charge_voltage_mv == 0U)) {
        return BQ25731_INVALID_ARG;
    }

    if (before_raw != NULL) {
        status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_VOLTAGE, before_raw);
        if (status != BQ25731_OK) {
            return status;
        }
    }

    raw = BQ25731_EncodeChargeVoltageRaw(charge_voltage_mv);

    status = BQ25731_Write16Verified(ctx,
                                     BQ25731_REG_CHARGE_VOLTAGE,
                                     raw,
                                     BQ25731_VERIFY_ALL_MASK);
    if (status != BQ25731_OK) {
        return status;
    }

    if (after_raw != NULL) {
        *after_raw = raw;
    }

    return BQ25731_OK;
}

BQ25731_Status_t BQ25731_ConfigureForCharging(BQ25731_Device_t *ctx,
                                              uint32_t input_current_ma,
                                              uint32_t charge_current_ma,
                                              uint32_t charge_voltage_mv)
{
    BQ25731_Status_t status;

    if ((ctx == NULL) ||
        (input_current_ma == 0U) ||
        (charge_current_ma == 0U) ||
        (charge_voltage_mv == 0U)) {
        return BQ25731_INVALID_ARG;
    }

    status = BQ25731_ConfigureForMonitoring(ctx);
    if (status != BQ25731_OK) {
        return status;
    }

    status = BQ25731_PrepareForCharging(ctx, NULL, NULL);
    if (status != BQ25731_OK) {
        return status;
    }

    status = BQ25731_DisableExternalInputCurrentLimit(ctx, NULL, NULL);
    if (status != BQ25731_OK) {
        return status;
    }

    status = BQ25731_SetInputCurrentLimit(ctx, input_current_ma, NULL, NULL);
    if (status != BQ25731_OK) {
        return status;
    }

    status = BQ25731_SetChargeVoltage(ctx, charge_voltage_mv, NULL, NULL);
    if (status != BQ25731_OK) {
        return status;
    }

    status = BQ25731_SetChargeCurrent(ctx, charge_current_ma, NULL, NULL);
    if (status != BQ25731_OK) {
        return status;
    }

    return BQ25731_EnableAdc(ctx);
}

BQ25731_Status_t BQ25731_EnableOtg(BQ25731_Device_t *ctx,
                                   uint32_t voltage_mv,
                                   uint32_t current_limit_ma,
                                   bool large_output_cap)
{
    BQ25731_Status_t status;
    uint16_t option1;
    uint16_t option3;
    uint16_t raw_current;

    if (ctx == NULL) {
        return BQ25731_INVALID_ARG;
    }

    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION1, &option1);
    if (status != BQ25731_OK) {
        return status;
    }

    BQ25731_SaveSenseResistorConfig(ctx, option1);

    status = BQ25731_Write16Verified(ctx,
                                     BQ25731_REG_OTG_VOLTAGE,
                                     BQ25731_EncodeOtgVoltageRaw(voltage_mv),
                                     BQ25731_VERIFY_ALL_MASK);
    if (status != BQ25731_OK) {
        return status;
    }

    raw_current = BQ25731_EncodeOtgCurrentRaw(current_limit_ma,
                                              ctx->rsns_rac_5mohm);

    status = BQ25731_Write16Verified(ctx,
                                     BQ25731_REG_OTG_CURRENT,
                                     raw_current,
                                     BQ25731_VERIFY_ALL_MASK);
    if (status != BQ25731_OK) {
        return status;
    }

    option3 = BQ25731_CHGOPT3_POR_BASE | BQ25731_CHGOPT3_EN_OTG_MASK;

    if (large_output_cap) {
        option3 |= BQ25731_CHGOPT3_EN_OTG_BIGCAP_MASK;
    }

    return BQ25731_Write16Verified(ctx,
                                   BQ25731_REG_CHARGE_OPTION3,
                                   option3,
                                   BQ25731_VERIFY_ALL_MASK);
}

BQ25731_Status_t BQ25731_DisableOtg(BQ25731_Device_t *ctx)
{
    BQ25731_Status_t status;
    uint16_t option3;

    if (ctx == NULL) {
        return BQ25731_INVALID_ARG;
    }

    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION3, &option3);
    if (status != BQ25731_OK) {
        return status;
    }

    option3 &= (uint16_t)~BQ25731_CHGOPT3_EN_OTG_MASK;

    return BQ25731_Write16Verified(ctx,
                                   BQ25731_REG_CHARGE_OPTION3,
                                   option3,
                                   BQ25731_VERIFY_ALL_MASK);
}

BQ25731_Status_t BQ25731_ReadTelemetry(BQ25731_Device_t *ctx,
                                       BQ25731_Telemetry_t *telemetry)
{
    BQ25731_Status_t status;
    uint8_t id_raw[2];

    if ((ctx == NULL) || (telemetry == NULL)) {
        return BQ25731_INVALID_ARG;
    }

    memset(telemetry, 0, sizeof(*telemetry));

    telemetry->address_7bit = ctx->device_address;
    telemetry->status = BQ25731_ERROR;

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_CHARGE_OPTION0,
                                        &telemetry->charge_option0_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_CHARGE_CURRENT,
                                        &telemetry->charge_current_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_CHARGE_VOLTAGE,
                                        &telemetry->charge_voltage_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_OTG_VOLTAGE,
                                        &telemetry->otg_voltage_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_OTG_CURRENT,
                                        &telemetry->otg_current_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_IIN_HOST,
                                        &telemetry->iin_host_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_CHARGER_STATUS,
                                        &telemetry->charger_status_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_IIN_DPM,
                                        &telemetry->iin_dpm_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_ADC_VBUS_PSYS,
                                        &telemetry->adc_vbus_psys_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_ADC_IBAT,
                                        &telemetry->adc_ibat_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_ADC_IIN_CMPIN,
                                        &telemetry->adc_iin_cmpin_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_ADC_VSYS_VBAT,
                                        &telemetry->adc_vsys_vbat_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_ReadBlock(ctx, BQ25731_REG_MANUFACTURE_ID, id_raw, sizeof(id_raw));
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    BQ25731_SaveRawRange(telemetry,
                         BQ25731_REG_MANUFACTURE_ID,
                         id_raw,
                         sizeof(id_raw));

    telemetry->manufacturer_id = id_raw[0];
    telemetry->device_id = id_raw[1];

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_CHARGE_OPTION1,
                                        &telemetry->charge_option1_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_CHARGE_OPTION2,
                                        &telemetry->charge_option2_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_CHARGE_OPTION3,
                                        &telemetry->charge_option3_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    status = BQ25731_Read16WithSnapshot(ctx,
                                        telemetry,
                                        BQ25731_REG_ADC_OPTION,
                                        &telemetry->adc_option_raw);
    if (status != BQ25731_OK) {
        telemetry->status = status;
        return status;
    }

    telemetry->raw_adc_psys = (uint8_t)(telemetry->adc_vbus_psys_raw & 0xFFU);
    telemetry->raw_adc_vbus = (uint8_t)((telemetry->adc_vbus_psys_raw >> 8) & 0xFFU);

    telemetry->raw_adc_idchg = (uint8_t)(telemetry->adc_ibat_raw & 0x7FU);
    telemetry->raw_adc_ichg = (uint8_t)((telemetry->adc_ibat_raw >> 8) & 0x7FU);

    telemetry->raw_adc_cmpin = (uint8_t)(telemetry->adc_iin_cmpin_raw & 0xFFU);
    telemetry->raw_adc_iin = (uint8_t)((telemetry->adc_iin_cmpin_raw >> 8) & 0xFFU);

    telemetry->raw_adc_vbat = (uint8_t)(telemetry->adc_vsys_vbat_raw & 0xFFU);
    telemetry->raw_adc_vsys = (uint8_t)((telemetry->adc_vsys_vbat_raw >> 8) & 0xFFU);

    BQ25731_SaveSenseResistorConfig(ctx, telemetry->charge_option1_raw);

    telemetry->device_id_valid =
        BQ25731_IsSupportedDeviceId(telemetry->manufacturer_id, telemetry->device_id);

    telemetry->id_ok = telemetry->device_id_valid;

    if (!telemetry->device_id_valid) {
        ctx->online = false;
        ctx->last_bq_register = BQ25731_REG_MANUFACTURE_ID;
        ctx->last_bq_error_code =
            ((uint32_t)telemetry->manufacturer_id << 8) | telemetry->device_id;
        ctx->last_error_code = ctx->last_bq_error_code;
        telemetry->status = BQ25731_DEVICE_ID_MISMATCH;
        return BQ25731_DEVICE_ID_MISMATCH;
    }

    BQ25731_DecodeTelemetry(ctx, telemetry);

    telemetry->adc_enabled = BQ25731_IsAdcReady(ctx, telemetry);
    telemetry->status = BQ25731_OK;

    return BQ25731_OK;
}

void BQ25731_PrintRawDebug(const BQ25731_Telemetry_t *telemetry)
{
    if (telemetry == NULL) {
        return;
    }

    printf("BQ RAW opt0=0x%04X opt1=0x%04X opt2=0x%04X opt3=0x%04X adcopt=0x%04X status=0x%04X\r\n",
           telemetry->charge_option0_raw,
           telemetry->charge_option1_raw,
           telemetry->charge_option2_raw,
           telemetry->charge_option3_raw,
           telemetry->adc_option_raw,
           telemetry->charger_status_raw);

    printf("BQ RAW cfg ichg=0x%04X vchg=0x%04X iin_host=0x%04X iin_dpm=0x%04X mfg=0x%02X dev=0x%02X\r\n",
           telemetry->charge_current_raw,
           telemetry->charge_voltage_raw,
           telemetry->iin_host_raw,
           telemetry->iin_dpm_raw,
           telemetry->manufacturer_id,
           telemetry->device_id);

    printf("BQ RAW adc_vbus_psys=0x%04X raw_vbus=%u raw_psys=%u vbus=%lumV\r\n",
           telemetry->adc_vbus_psys_raw,
           telemetry->raw_adc_vbus,
           telemetry->raw_adc_psys,
           (unsigned long)telemetry->vbus_mv);

    printf("BQ RAW adc_ibat=0x%04X raw_ichg=%u raw_idchg=%u ichg=%lumA idchg=%lumA\r\n",
           telemetry->adc_ibat_raw,
           telemetry->raw_adc_ichg,
           telemetry->raw_adc_idchg,
           (unsigned long)telemetry->ichg_ma,
           (unsigned long)telemetry->idchg_ma);

    printf("BQ RAW adc_iin_cmpin=0x%04X raw_iin=%u raw_cmpin=%u iin=%lumA cmpin=%lumV\r\n",
           telemetry->adc_iin_cmpin_raw,
           telemetry->raw_adc_iin,
           telemetry->raw_adc_cmpin,
           (unsigned long)telemetry->iin_ma,
           (unsigned long)telemetry->cmpin_mv);

    printf("BQ RAW adc_vsys_vbat=0x%04X raw_vsys=%u raw_vbat=%u vsys=%lumV vbat=%lumV\r\n",
           telemetry->adc_vsys_vbat_raw,
           telemetry->raw_adc_vsys,
           telemetry->raw_adc_vbat,
           (unsigned long)telemetry->vsys_mv,
           (unsigned long)telemetry->vbat_mv);
}

const char *BQ25731_StatusToString(BQ25731_Status_t status)
{
    switch (status) {
        case BQ25731_OK:
            return "OK";
        case BQ25731_ERROR:
            return "ERROR";
        case BQ25731_I2C_ERROR:
            return "I2C_ERROR";
        case BQ25731_INVALID_ARG:
            return "INVALID_ARG";
        case BQ25731_NOT_READY:
            return "NOT_READY";
        case BQ25731_DEVICE_ID_MISMATCH:
            return "DEVICE_ID_MISMATCH";
        case BQ25731_TPS_ERROR:
            return "TPS_ERROR";
        default:
            return "UNKNOWN_STATUS";
    }
}

int BQ25731_GetDiagnosticText(const BQ25731_Device_t *ctx,
                              char *buffer,
                              size_t length)
{
    if ((ctx == NULL) || (buffer == NULL) || (length == 0U)) {
        return -1;
    }

    return snprintf(buffer,
                    length,
                    "BQ25731 I2C@0x%02X online=%s reg=0x%02X bridge=%s task=0x%02X err=0x%08lX adc=%04X/%04X/%04X",
                    ctx->device_address,
                    ctx->online ? "YES" : "NO",
                    ctx->last_bq_register,
                    TPS25751_StatusToString(ctx->last_bridge_status),
                    ctx->last_task_return_code,
                    (unsigned long)ctx->last_bq_error_code,
                    ctx->adc_option_before,
                    ctx->adc_option_after,
                    ctx->adc_option_expected);
}
