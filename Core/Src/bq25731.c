#include "bq25731.h"
#include "debug_uart.h"

#include <stdio.h>
#include <string.h>

#define BQ25731_TPS_READ_GAP_MS                 5U
#define BQ25731_TPS_WRITE_GAP_MS               10U
#define BQ25731_TPS_WRITE_SETTLE_MS            10U
#define BQ25731_TPS_QUEUE_DRAIN_MS              100U
#define BQ25731_ADC_SETTLE_MS                 250U
#define BQ25731_TPS_READ_CHUNK_MAX            8U
#define BQ25731_WRITE_VERIFY_RETRIES          2U
#define BQ25731_WRITE_VERIFY_DELAY_MS         8U

#define BQ25731_REG_CHARGE_OPTION0            0x00U
#define BQ25731_REG_CHARGE_CURRENT            0x02U
#define BQ25731_REG_CHARGE_VOLTAGE            0x04U
#define BQ25731_REG_OTG_VOLTAGE               0x06U
#define BQ25731_REG_OTG_CURRENT               0x08U
#define BQ25731_REG_INPUT_VOLTAGE             0x0AU
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
#define BQ25731_REG_CHARGE_OPTION4            0x3CU

#define BQ25731_MANUFACTURE_ID_EXPECTED       0x40U
#define BQ25731_DEVICE_ID_BQ25730             0xD5U
#define BQ25731_DEVICE_ID_EXPECTED            0xD6U

#define BQ25731_CHGOPT0_EN_LWPWR_MASK         0x8000U
#define BQ25731_CHGOPT0_WDTMR_ADJ_MASK        0x6000U
#define BQ25731_CHGOPT0_EN_OOA_MASK            0x0400U
#define BQ25731_CHGOPT0_PWM_FREQ_MASK          0x0200U
#define BQ25731_CHGOPT0_LOW_PTM_RIPPLE_MASK    0x0100U
#define BQ25731_CHGOPT0_EN_IIN_DPM_MASK       0x0002U
#define BQ25731_CHGOPT0_CHRG_INHIBIT_MASK     0x0001U
#define BQ25731_CHGOPT0_BAD_I2CW_SIGNATURE    0x0030U

#define BQ25731_CHGOPT1_RSNS_RAC_MASK         0x0800U
#define BQ25731_CHGOPT1_RSNS_RSR_MASK         0x0400U
#define BQ25731_CHGOPT1_EN_FAST_5MOHM_MASK    0x0100U

#define BQ25731_CHGOPT2_EN_EXTILIM_MASK       0x0080U

#define BQ25731_CHGOPT3_EN_HIZ_MASK           0x8000U
#define BQ25731_CHGOPT3_EN_OTG_MASK           0x1000U
#define BQ25731_CHGOPT3_EN_ICO_MODE_MASK      0x0800U
#define BQ25731_CHGOPT3_EN_OTG_BIGCAP_MASK    0x0100U
#define BQ25731_CHGOPT3_EN_VBUS_VAP_MASK      0x0040U
#define BQ25731_CHGOPT3_OTG_VAP_MODE_MASK     0x0020U
#define BQ25731_CHGOPT3_IL_AVG_15A            0x0010U
#define BQ25731_CHGOPT3_CMP_EN_MASK           0x0004U
#define BQ25731_CHGOPT4_EN_DITHER_MASK         0x1800U

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
#define BQ25731_IIN_HOST_VERIFY_MASK          0x7F00U
#define BQ25731_CHARGE_CURRENT_VERIFY_MASK    0x1FC0U
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

    status = TPS25751_I2CcRead(ctx->tps,
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

#if (BQ25731_CONTROL_OWNER_TPS_EEPROM != 0U)
    (void)reg_address;
    (void)value;
    return BQ25731_NOT_READY;
#endif

    raw[0] = (uint8_t)(value & 0xFFU);
    raw[1] = (uint8_t)((value >> 8) & 0xFFU);

    BQ25731_WaitForGap(ctx->last_write_tick_ms, BQ25731_TPS_WRITE_GAP_MS);

    status = TPS25751_I2CcWrite(ctx->tps,
                               ctx->device_address,
                               reg_address,
                               raw,
                               sizeof(raw));

    if (status != TPS25751_OK) {
        return BQ25731_MapTransportStatus(ctx, reg_address, status);
    }

    ctx->last_write_tick_ms = HAL_GetTick();
    /* I2Cw success means queued, not physically completed on I2Cc. */
    HAL_Delay(BQ25731_TPS_WRITE_SETTLE_MS);
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

BQ25731_Status_t BQ25731_EnableMonitoringAdcOnly(BQ25731_Device_t *dev)
{
#if (BQ25731_ALLOW_STM32_ADC_ENABLE != 0U)
    BQ25731_Status_t status;
    TPS25751_Status_t bridge_status;
    uint16_t option0 = 0U;
    uint16_t adc_before = 0U;
    uint16_t adc_after = 0U;
    /* Continuous conversion + 3.06 V full scale; ADC_START is only needed
     * for one-shot mode. Channels: VBUS, IIN, IDCHG, ICHG, VSYS and VBAT. */
    const uint16_t adc_target = 0xA05FU;
    uint8_t raw[2];

    if ((dev == NULL) || (dev->tps == NULL)) return BQ25731_INVALID_ARG;

    /* TPS25751 TRM 4.4.2/4.4.3: I2Cw completion only means that the
     * transaction entered the I2Cc queue, and I2Cr must not be issued less
     * than 5 s after the previous controller transaction.  Monitoring reads
     * otherwise keep the queue busy and the immediate readback sees the old
     * BQ value even though I2Cw returned task_rc=0. */
    BQ25731_WaitForGap(dev->last_read_tick_ms,
                       BQ25731_TPS_QUEUE_DRAIN_MS);
    status = BQ25731_ReadReg16(dev, BQ25731_REG_CHARGE_OPTION0, &option0);
    if (status != BQ25731_OK) return status;

    /* Recover the exact corruption caused by the old malformed TPS I2Cw
     * frame: its spurious reserved byte addressed register 0x00 and wrote
     * 0x3A, changing the normal 0x070A/0x070B to 0x073A/0x073B. */
    if (((option0 & 0x00FEU) == 0x003AU)) {
        uint16_t recovered =
            (uint16_t)(option0 & ~BQ25731_CHGOPT0_BAD_I2CW_SIGNATURE);

        raw[0] = (uint8_t)(recovered & 0xFFU);
        raw[1] = (uint8_t)(recovered >> 8);
        Debug_Printf("[BQ-RECOVERY] malformed-I2Cw damage detected CHGOPT0=0x%04X; restoring 0x%04X",
                     option0, recovered);
        bridge_status = TPS25751_I2CcWrite(dev->tps, dev->device_address,
                                           BQ25731_REG_CHARGE_OPTION0,
                                           raw, sizeof(raw));
        if (bridge_status != TPS25751_OK)
            return BQ25731_MapTransportStatus(dev,
                                              BQ25731_REG_CHARGE_OPTION0,
                                              bridge_status);

        HAL_Delay(BQ25731_TPS_QUEUE_DRAIN_MS);
        status = BQ25731_ReadReg16(dev, BQ25731_REG_CHARGE_OPTION0, &option0);
        Debug_Printf("[BQ-RECOVERY] CHGOPT0 readback=0x%04X %s",
                     option0,
                     ((status == BQ25731_OK) &&
                      ((option0 & BQ25731_CHGOPT0_BAD_I2CW_SIGNATURE) == 0U)) ?
                     "OK" : "FAIL");
        if ((status != BQ25731_OK) ||
            ((option0 & BQ25731_CHGOPT0_BAD_I2CW_SIGNATURE) != 0U))
            return BQ25731_ERROR;
    }

    if ((option0 & BQ25731_CHGOPT0_EN_LWPWR_MASK) != 0U) {
        uint16_t target = (uint16_t)(option0 & ~BQ25731_CHGOPT0_EN_LWPWR_MASK);
        raw[0] = (uint8_t)(target & 0xFFU);
        raw[1] = (uint8_t)(target >> 8);
        bridge_status = TPS25751_I2CcWrite(dev->tps, dev->device_address,
                                           BQ25731_REG_CHARGE_OPTION0,
                                           raw, sizeof(raw));
        if (bridge_status != TPS25751_OK)
            return BQ25731_MapTransportStatus(dev, BQ25731_REG_CHARGE_OPTION0,
                                              bridge_status);
        HAL_Delay(BQ25731_TPS_WRITE_SETTLE_MS);
        status = BQ25731_ReadReg16(dev, BQ25731_REG_CHARGE_OPTION0, &option0);
        if ((status != BQ25731_OK) ||
            ((option0 & BQ25731_CHGOPT0_EN_LWPWR_MASK) != 0U)) {
            Debug_Printf("[BQ-ADC-ENABLE] clear EN_LWPWR failed readback=0x%04X", option0);
            return BQ25731_ERROR;
        }
        HAL_Delay(BQ25731_TPS_WRITE_GAP_MS);
    }

    status = BQ25731_ReadReg16(dev, BQ25731_REG_ADC_OPTION, &adc_before);
    if (status != BQ25731_OK) return status;
    BQ25731_WaitForGap(dev->last_read_tick_ms,
                       BQ25731_TPS_QUEUE_DRAIN_MS);
    /* BQ25731 charger registers are transferred as 16-bit little-endian
     * words. Send 3Bh/3Ah in one I2Cc transaction; byte writes can be
     * acknowledged by TPS while being ignored by the charger. */
    raw[0] = (uint8_t)(adc_target & 0xFFU);
    raw[1] = (uint8_t)(adc_target >> 8);
    Debug_Printf("[BQ-ADC-ENABLE] corrected TPS DATA1 payload=%02X 03 3A %02X %02X",
                 dev->device_address, raw[0], raw[1]);
    Debug_Printf("[BQ-ADC-ENABLE] I2Cw target=0x%02X reg=0x3A data_len=2 transaction_len=3 data=%02X %02X",
                 dev->device_address, raw[0], raw[1]);
    bridge_status = TPS25751_I2CcWrite(dev->tps, dev->device_address,
                                       BQ25731_REG_ADC_OPTION,
                                       raw, sizeof(raw));
    Debug_Printf("[BQ-ADC-ENABLE] I2Cw 0x3A/0x3B 4CC=%s task_rc=0x%02X",
                 TPS25751_StatusToString(bridge_status),
                 dev->tps->last_i2c_controller_task_return_code);
    if (bridge_status != TPS25751_OK)
        return BQ25731_MapTransportStatus(dev, BQ25731_REG_ADC_OPTION,
                                          bridge_status);
    /* I2Cw completion means queued by TPS, not necessarily executed on I2Cc.
     * Poll readback long enough for the charger transaction queue to drain. */
    Debug_Printf("[BQ-ADC-ENABLE] writes queued; waiting %ums for TPS I2Cc queue to drain",
                 BQ25731_TPS_QUEUE_DRAIN_MS);
    HAL_Delay(BQ25731_TPS_QUEUE_DRAIN_MS);
    status = BQ25731_ReadReg16(dev, BQ25731_REG_ADC_OPTION, &adc_after);
    Debug_Printf("[BQ-ADC-ENABLE] ADCOption before=0x%04X write=0x%04X after=0x%04X %s",
                 adc_before, adc_target, adc_after,
                 ((status == BQ25731_OK) &&
                  ((adc_after & BQ25731_ADCOPT_VERIFY_MASK) ==
                   (adc_target & BQ25731_ADCOPT_VERIFY_MASK))) ? "OK" : "FAIL");
    if ((status != BQ25731_OK) ||
        ((adc_after & BQ25731_ADCOPT_VERIFY_MASK) !=
         (adc_target & BQ25731_ADCOPT_VERIFY_MASK))) return BQ25731_ERROR;
    dev->adc_enabled_tick_ms = HAL_GetTick();
    return BQ25731_OK;
#else
    (void)dev;
    return BQ25731_NOT_READY;
#endif
}

BQ25731_Status_t BQ25731_BridgeWriteSelfTest(BQ25731_Device_t *ctx,
                                              uint16_t *old_charge_current,
                                              uint16_t *new_charge_current)
{
    BQ25731_Status_t status;
    uint16_t old_current = 0U, new_current = 0U;
    uint16_t old_option0 = 0U, new_option0 = 0U;

    if (ctx == NULL) return BQ25731_INVALID_ARG;
    status = BQ25731_ReadReg16(ctx, BQ25731_REG_CHARGE_CURRENT, &old_current);
    if (status != BQ25731_OK) return status;
    status = BQ25731_WriteReg16(ctx, BQ25731_REG_CHARGE_CURRENT, 0x0000U);
    if (status == BQ25731_OK)
        status = BQ25731_ReadReg16(ctx, BQ25731_REG_CHARGE_CURRENT, &new_current);
    if (old_charge_current != NULL) *old_charge_current = old_current;
    if (new_charge_current != NULL) *new_charge_current = new_current;
    Debug_Printf("[BQ-BRIDGE-TEST] ChargeCurrent old=0x%04X write=0x0000 new=0x%04X %s",
                 old_current, new_current,
                 (status == BQ25731_OK && new_current == 0U) ? "OK" : "FAIL");
    if ((status != BQ25731_OK) || (new_current != 0U)) {
        Debug_Printf("[BQ-BRIDGE-TEST] I2Cw probably not modifying BQ registers; check TPS I2Cw payload/address/length");
        return BQ25731_ERROR;
    }

    status = BQ25731_ReadReg16(ctx, BQ25731_REG_CHARGE_OPTION0, &old_option0);
    if (status != BQ25731_OK) return status;
    status = BQ25731_WriteReg16(ctx, BQ25731_REG_CHARGE_OPTION0,
                                (uint16_t)(old_option0 | BQ25731_CHGOPT0_CHRG_INHIBIT_MASK));
    if (status == BQ25731_OK)
        status = BQ25731_ReadReg16(ctx, BQ25731_REG_CHARGE_OPTION0, &new_option0);
    Debug_Printf("[BQ-BRIDGE-TEST] CHRG_INHIBIT old=0x%04X write=0x%04X new=0x%04X %s",
                 old_option0,
                 (uint16_t)(old_option0 | BQ25731_CHGOPT0_CHRG_INHIBIT_MASK),
                 new_option0,
                 (status == BQ25731_OK &&
                  (new_option0 & BQ25731_CHGOPT0_CHRG_INHIBIT_MASK) != 0U) ? "OK" : "FAIL");
    return (status == BQ25731_OK &&
            (new_option0 & BQ25731_CHGOPT0_CHRG_INHIBIT_MASK) != 0U) ?
           BQ25731_OK : BQ25731_ERROR;
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

BQ25731_Status_t BQ25731_ReadReg16(BQ25731_Device_t *ctx, uint8_t reg,
                                    uint16_t *raw)
{
    return BQ25731_Read16(ctx, reg, raw);
}

BQ25731_Status_t BQ25731_WriteReg16(BQ25731_Device_t *ctx, uint8_t reg,
                                     uint16_t raw)
{
    BQ25731_Status_t status;
    uint16_t before = 0U;
    uint16_t after = 0U;

    if (reg == BQ25731_REG_CHARGE_OPTION0)
        (void)BQ25731_Read16(ctx, reg, &before);
    status = BQ25731_Write16(ctx, reg, raw);
    if (reg == BQ25731_REG_CHARGE_OPTION0) {
        (void)BQ25731_Read16(ctx, reg, &after);
        Debug_Printf("[BQ-WRITE-TRACE] REG0x00 caller=WriteReg16 before=0x%04X write=0x%04X after=0x%04X status=%s",
                     before, raw, after, BQ25731_StatusToString(status));
    }
    return status;
}

BQ25731_Status_t BQ25731_UpdateReg16(BQ25731_Device_t *ctx, uint8_t reg,
                                      uint16_t clear_mask, uint16_t set_mask,
                                      uint16_t *before, uint16_t *written,
                                      uint16_t *after)
{
    BQ25731_Status_t write_status = BQ25731_OK;
    BQ25731_Status_t read_status;
    uint16_t old_raw = 0U;
    uint16_t target;
    uint16_t readback = 0U;
    uint16_t mask = (uint16_t)(clear_mask | set_mask);

    read_status = BQ25731_ReadReg16(ctx, reg, &old_raw);
    if (read_status != BQ25731_OK) return read_status;
    target = (uint16_t)((old_raw & (uint16_t)~clear_mask) | set_mask);
    if (before != NULL) *before = old_raw;
    if (written != NULL) *written = target;

    if ((old_raw & mask) != (target & mask))
        write_status = BQ25731_WriteReg16(ctx, reg, target);

    HAL_Delay(BQ25731_WRITE_VERIFY_DELAY_MS);
    read_status = BQ25731_ReadReg16(ctx, reg, &readback);
    if (after != NULL) *after = readback;
    Debug_Printf("[BQ-I2C] reg=0x%02X before=0x%04X write=0x%04X after=0x%04X mask=0x%04X %s",
                 reg, old_raw, target, readback, mask,
                 ((read_status == BQ25731_OK) &&
                  ((readback & mask) == (target & mask))) ? "OK" : "MISMATCH");
    if (read_status != BQ25731_OK) return read_status;
    if (write_status != BQ25731_OK) return write_status;
    return ((readback & mask) == (target & mask)) ? BQ25731_OK : BQ25731_ERROR;
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

    /* Reaching the requested state is success even when no write is needed.
     * This also avoids unnecessary I2Cw tasks through the TPS bridge. */
    status = BQ25731_Read16(ctx, reg_address, &readback);
    if (status != BQ25731_OK) {
        return status;
    }
    if ((readback & verify_mask) == (value & verify_mask)) {
        return BQ25731_OK;
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

static BQ25731_Status_t BQ25731_EnsureMaskedState(BQ25731_Device_t *ctx,
                                                  uint8_t reg_address,
                                                  uint16_t before,
                                                  uint16_t mask,
                                                  bool expected_set,
                                                  uint16_t *after,
                                                  bool *changed)
{
    BQ25731_Status_t status;
    uint16_t expected = expected_set ? mask : 0U;
    uint16_t target;
    uint16_t readback = before;

    if ((ctx == NULL) || (after == NULL) || (changed == NULL) || (mask == 0U)) {
        return BQ25731_INVALID_ARG;
    }

    *after = before;
    *changed = false;

    if ((before & mask) == expected) {
        return BQ25731_OK;
    }

    target = expected_set ? (uint16_t)(before | mask) :
                            (uint16_t)(before & (uint16_t)~mask);
    status = BQ25731_Write16Verified(ctx, reg_address, target, mask);

    /* Always try to report the actual register state, including when the
     * transport reported an error after a write may already have completed. */
    if (BQ25731_Read16(ctx, reg_address, &readback) == BQ25731_OK) {
        *after = readback;
        if ((readback & mask) == expected) {
            *changed = true;
            return BQ25731_OK;
        }
    } else {
        *after = target;
    }

    return (status != BQ25731_OK) ? status : BQ25731_ERROR;
}

static const char *BQ25731_SafeStateResult(BQ25731_Status_t status,
                                           bool changed,
                                           bool expected_set)
{
    if (status == BQ25731_OK) {
        if (changed) {
            return "OK changed";
        }
        return expected_set ? "OK already_set" : "OK already_clear";
    }

    return expected_set ? "FAIL expected_set" : "FAIL expected_clear";
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
    uint32_t code = ((uint32_t)(raw >> 6) & 0x0003U) |
                    ((((uint32_t)raw >> 8) & 0x001FU) << 2);
    uint32_t step_ma = rsns_rsr_5mohm ? 128U : 64U;

    return code * step_ma;
}

uint32_t BQ25731_DecodeChargeCurrent(uint16_t raw, bool rsns_rsr_5mohm)
{
    return BQ25731_DecodeChargeCurrentMa(raw, rsns_rsr_5mohm);
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

    return (uint16_t)(((code & 0x03U) << 6) |
                      (((code >> 2) & 0x1FU) << 8));
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

#if (BQ25731_HW_OTG_ALLOWED != 0U)
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
#endif

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

    telemetry->vbat_mv = vsys_vbat_base_mv + BQ_USER_VBAT_VSYS_CORRECTION_MV +
                         ((uint32_t)telemetry->raw_adc_vbat * BQ25731_ADC_VBAT_VSYS_MV_PER_LSB);

    telemetry->vsys_mv = vsys_vbat_base_mv + BQ_USER_VBAT_VSYS_CORRECTION_MV +
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
                                     BQ25731_CHGOPT2_EN_EXTILIM_MASK);
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
    option3 &= (uint16_t)~BQ25731_CHGOPT3_OTG_VAP_MODE_MASK;

    status = BQ25731_Write16Verified(ctx,
                                     BQ25731_REG_CHARGE_OPTION3,
                                     option3,
                                     BQ25731_VERIFY_ALL_MASK);
    if (status != BQ25731_OK) {
        return status;
    }

    return BQ25731_EnableAdc(ctx);
}

BQ25731_Status_t BQ25731_ApplyUserOptions(BQ25731_Device_t *ctx)
{
    BQ25731_Status_t status;
    uint16_t option0;
    uint16_t option4;
    uint16_t dither_bits = (uint16_t)(BQ_USER_DITHER_PERCENT / 2U);

    if (ctx == NULL) return BQ25731_INVALID_ARG;

    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION0, &option0);
    if (status != BQ25731_OK) return status;
    option0 &= (uint16_t)~(BQ25731_CHGOPT0_PWM_FREQ_MASK |
                           BQ25731_CHGOPT0_EN_OOA_MASK |
                           BQ25731_CHGOPT0_LOW_PTM_RIPPLE_MASK);
#if (BQ_USER_PWM_FREQUENCY_KHZ == 400U)
    option0 |= BQ25731_CHGOPT0_PWM_FREQ_MASK;
#endif
#if (BQ_USER_OUT_OF_AUDIO_ENABLE != 0U)
    option0 |= BQ25731_CHGOPT0_EN_OOA_MASK;
#endif
#if (BQ_USER_LOW_PTM_RIPPLE_ENABLE != 0U)
    option0 |= BQ25731_CHGOPT0_LOW_PTM_RIPPLE_MASK;
#endif
    status = BQ25731_Write16Verified(ctx, BQ25731_REG_CHARGE_OPTION0,
                                     option0, BQ25731_VERIFY_ALL_MASK);
    if (status != BQ25731_OK) return status;

    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION4, &option4);
    if (status != BQ25731_OK) return status;
    option4 = (uint16_t)((option4 & ~BQ25731_CHGOPT4_EN_DITHER_MASK) |
                        ((dither_bits << 11U) & BQ25731_CHGOPT4_EN_DITHER_MASK));
    return BQ25731_Write16Verified(ctx, BQ25731_REG_CHARGE_OPTION4,
                                   option4, BQ25731_VERIFY_ALL_MASK);
}

BQ25731_Status_t BQ25731_ConfigureSafeStartup(BQ25731_Device_t *ctx)
{
    BQ25731_Status_t status;
    uint16_t option0_before;
    uint16_t option0_after;
    uint16_t option3_before;
    uint16_t option3_after;
    uint16_t target;
    uint16_t iin_raw = 0U;
    uint16_t charge_current_raw = 0U;
    bool changed = false;

    if (ctx == NULL) {
        return BQ25731_INVALID_ARG;
    }

    /* Block charging before changing any current limits. */
    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION0, &option0_before);
    if (status != BQ25731_OK) {
        Debug_Printf("[BQ-SAFE] read ChargeOption0 FAIL status=%s",
                     BQ25731_StatusToString(status));
        return status;
    }
    Debug_Printf("[BQ-SAFE] read ChargeOption0 raw=0x%04X", option0_before);

    status = BQ25731_EnsureMaskedState(ctx,
                                      BQ25731_REG_CHARGE_OPTION0,
                                      option0_before,
                                      BQ25731_CHGOPT0_CHRG_INHIBIT_MASK,
                                      true,
                                      &option0_after,
                                      &changed);
    Debug_Printf("[BQ-SAFE] set CHRG_INHIBIT before=0x%04X after=0x%04X mask=0x%04X %s",
                 option0_before, option0_after,
                 BQ25731_CHGOPT0_CHRG_INHIBIT_MASK,
                 BQ25731_SafeStateResult(status, changed, true));
    if (status != BQ25731_OK) {
        return status;
    }

    option0_before = option0_after;
    status = BQ25731_EnsureMaskedState(ctx,
                                      BQ25731_REG_CHARGE_OPTION0,
                                      option0_before,
                                      BQ25731_CHGOPT0_WDTMR_ADJ_MASK,
                                      false,
                                      &option0_after,
                                      &changed);
    Debug_Printf("[BQ-SAFE] watchdog OFF before=0x%04X after=0x%04X mask=0x%04X %s",
                 option0_before, option0_after,
                 BQ25731_CHGOPT0_WDTMR_ADJ_MASK,
                 BQ25731_SafeStateResult(status, changed, false));
    if (status != BQ25731_OK) {
        return status;
    }

    target = option0_after;
    target &= (uint16_t)~BQ25731_CHGOPT0_EN_LWPWR_MASK;
    target |= BQ25731_CHGOPT0_EN_IIN_DPM_MASK;
    status = BQ25731_Write16Verified(ctx, BQ25731_REG_CHARGE_OPTION0,
                                     target,
                                     BQ25731_CHGOPT0_EN_LWPWR_MASK |
                                     BQ25731_CHGOPT0_EN_IIN_DPM_MASK);
    if (status != BQ25731_OK) {
        Debug_Printf("[BQ-SAFE] configure LWPWR/IINDPM mask=0x%04X FAIL",
                     BQ25731_CHGOPT0_EN_LWPWR_MASK |
                     BQ25731_CHGOPT0_EN_IIN_DPM_MASK);
        return status;
    }

    /* OTG remains disabled during bring-up even though the hardware pin now
     * has a pulldown. Keep both enabling bits clear as defense in depth. */
    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_OPTION3, &option3_before);
    if (status != BQ25731_OK) {
        Debug_Printf("[BQ-SAFE] read ChargeOption3 FAIL status=%s",
                     BQ25731_StatusToString(status));
        return status;
    }
    Debug_Printf("[BQ-SAFE] read ChargeOption3 raw=0x%04X", option3_before);

    status = BQ25731_EnsureMaskedState(ctx,
                                      BQ25731_REG_CHARGE_OPTION3,
                                      option3_before,
                                      BQ25731_CHGOPT3_EN_OTG_MASK,
                                      false,
                                      &option3_after,
                                      &changed);
    Debug_Printf("[BQ-SAFE] clear EN_OTG before=0x%04X after=0x%04X mask=0x%04X %s",
                 option3_before, option3_after, BQ25731_CHGOPT3_EN_OTG_MASK,
                 BQ25731_SafeStateResult(status, changed, false));
    if (status != BQ25731_OK) {
        return status;
    }

    option3_before = option3_after;
    status = BQ25731_EnsureMaskedState(ctx,
                                      BQ25731_REG_CHARGE_OPTION3,
                                      option3_before,
                                      BQ25731_CHGOPT3_OTG_VAP_MODE_MASK,
                                      false,
                                      &option3_after,
                                      &changed);
    Debug_Printf("[BQ-SAFE] clear OTG_VAP_MODE before=0x%04X after=0x%04X mask=0x%04X %s",
                 option3_before, option3_after,
                 BQ25731_CHGOPT3_OTG_VAP_MODE_MASK,
                 BQ25731_SafeStateResult(status, changed, false));
    if (status != BQ25731_OK) {
        Debug_Printf("[BQ-SAFE] WARNING OTG_VAP_MODE stuck; non-fatal because EN_OTG=0, OTG API blocked and hardware pin has pulldown");
    }

    status = BQ25731_DisableExternalInputCurrentLimit(ctx, NULL, NULL);
    if (status != BQ25731_OK) {
        Debug_Printf("[BQ-SAFE] clear EN_EXTILIM mask=0x%04X FAIL",
                     BQ25731_CHGOPT2_EN_EXTILIM_MASK);
        return status;
    }

    status = BQ25731_SetInputCurrentLimit(ctx,
                                          BQ25731_SAFE_INPUT_CURRENT_MA,
                                          NULL,
                                          NULL);
    if (status != BQ25731_OK) {
        Debug_Printf("[BQ-SAFE] set IIN_HOST=%umA FAIL status=%s",
                     BQ25731_SAFE_INPUT_CURRENT_MA,
                     BQ25731_StatusToString(status));
        return status;
    }
    status = BQ25731_Read16(ctx, BQ25731_REG_IIN_HOST, &iin_raw);
    Debug_Printf("[BQ-SAFE] set IIN_HOST=%umA raw=0x%04X mask=0x%04X %s",
                 BQ25731_SAFE_INPUT_CURRENT_MA, iin_raw,
                 BQ25731_IIN_HOST_VERIFY_MASK,
                 (status == BQ25731_OK) ? "OK" : "FAIL");
    if (status != BQ25731_OK) {
        return status;
    }

    status = BQ25731_SetChargeCurrent(ctx, 0U, NULL, NULL);
    if (status != BQ25731_OK) {
        Debug_Printf("[BQ-SAFE] set ChargeCurrent=0 FAIL status=%s",
                     BQ25731_StatusToString(status));
        return status;
    }
    status = BQ25731_Read16(ctx, BQ25731_REG_CHARGE_CURRENT,
                            &charge_current_raw);
    Debug_Printf("[BQ-SAFE] set ChargeCurrent=0 raw=0x%04X mask=0x%04X %s",
                 charge_current_raw, BQ25731_CHARGE_CURRENT_VERIFY_MASK,
                 (status == BQ25731_OK) ? "OK" : "FAIL");
    if (status != BQ25731_OK) {
        return status;
    }

    status = BQ25731_EnableAdc(ctx);
    Debug_Printf("[BQ-SAFE] ADCOption before=0x%04X after=0x%04X expected_mask=0x%04X actual=0x%04X %s",
                 ctx->adc_option_before,
                 ctx->adc_option_after,
                 (uint16_t)(BQ25731_ADCOPT_EXPECTED & BQ25731_ADCOPT_VERIFY_MASK),
                 (uint16_t)(ctx->adc_option_after & BQ25731_ADCOPT_VERIFY_MASK),
                 ((status == BQ25731_OK) &&
                  ((ctx->adc_option_after & BQ25731_ADCOPT_VERIFY_MASK) ==
                   (BQ25731_ADCOPT_EXPECTED & BQ25731_ADCOPT_VERIFY_MASK))) ?
                 "OK" : "FAIL");
    if ((status != BQ25731_OK) ||
        ((ctx->adc_option_after & BQ25731_ADCOPT_VERIFY_MASK) !=
         (BQ25731_ADCOPT_EXPECTED & BQ25731_ADCOPT_VERIFY_MASK))) {
        return (status != BQ25731_OK) ? status : BQ25731_ERROR;
    }

    return BQ25731_OK;
}

BQ25731_Status_t BQ25731_SafeStartup(BQ25731_Device_t *ctx,
                                      BQ25731_SafeStartupResult_t *result)
{
    BQ25731_Status_t status;
    BQ25731_Telemetry_t telemetry;
    uint8_t ids[2];
    uint16_t written = 0U;
    uint32_t iin_step_ma;

    if ((ctx == NULL) || (result == NULL)) return BQ25731_INVALID_ARG;
    memset(result, 0, sizeof(*result));

#if ((BQ25731_RAC_MOHM != 5U) && (BQ25731_RAC_MOHM != 10U)) || \
    ((BQ25731_RSR_MOHM != 5U) && (BQ25731_RSR_MOHM != 10U))
#error "BQ25731_RAC_MOHM and BQ25731_RSR_MOHM must be 5 or 10"
#endif

    status = BQ25731_CheckDevice(ctx);
    if (status != BQ25731_OK) { result->fatal_error = true; return status; }
    status = BQ25731_ReadBlock(ctx, BQ25731_REG_MANUFACTURE_ID, ids, 2U);
    if (status != BQ25731_OK) { result->fatal_error = true; return status; }
    result->manufacturer_id = ids[0];
    result->device_id = ids[1];
    Debug_Printf("[BQ-SAFE] IDs manufacturer=0x%02X device=0x%02X OK",
                 result->manufacturer_id, result->device_id);

#if defined(BQ25731_RAC_VALUE_DEFAULTED) || defined(BQ25731_RSR_VALUE_DEFAULTED)
    result->warnings |= BQ_SAFE_WARN_SENSE_CONFIG_DEFAULTED;
    Debug_Printf("[BQ-SAFE] WARNING sense resistors defaulted at compile time");
#endif
    Debug_Printf("[BQ-SAFE] sense RAC=%umOhm RSR=%umOhm",
                 BQ25731_RAC_MOHM, BQ25731_RSR_MOHM);
    status = BQ25731_UpdateReg16(ctx, BQ25731_REG_CHARGE_OPTION1,
             BQ25731_CHGOPT1_RSNS_RAC_MASK | BQ25731_CHGOPT1_RSNS_RSR_MASK |
             BQ25731_CHGOPT1_EN_FAST_5MOHM_MASK,
             ((BQ25731_RAC_MOHM == 5U) ? (BQ25731_CHGOPT1_RSNS_RAC_MASK |
              BQ25731_CHGOPT1_EN_FAST_5MOHM_MASK) : 0U) |
             ((BQ25731_RSR_MOHM == 5U) ? BQ25731_CHGOPT1_RSNS_RSR_MASK : 0U),
             &result->charge_option1_before, &written,
             &result->charge_option1_after);
    if (status != BQ25731_OK)
        Debug_Printf("[BQ-SAFE] WARNING ChargeOption1 sense readback mismatch");

    status = BQ25731_ConfigureSafeStartup(ctx);
    if (status != BQ25731_OK) result->fatal_error = true;

    {
        BQ25731_Status_t option3_status = BQ25731_UpdateReg16(
            ctx, BQ25731_REG_CHARGE_OPTION3,
            BQ25731_CHGOPT3_EN_VBUS_VAP_MASK | BQ25731_CHGOPT3_EN_ICO_MODE_MASK,
            0U, &result->charge_option3_before, &written,
            &result->charge_option3_after);
        if ((result->charge_option3_after & BQ25731_CHGOPT3_EN_VBUS_VAP_MASK) != 0U)
            result->warnings |= BQ_SAFE_WARN_EN_VBUS_VAP_STUCK;
        if ((result->charge_option3_after & BQ25731_CHGOPT3_EN_ICO_MODE_MASK) != 0U)
            result->warnings |= BQ_SAFE_WARN_EN_ICO_STUCK;
        if (option3_status != BQ25731_OK)
            Debug_Printf("[BQ-SAFE] WARNING optional ChargeOption3 policy bits did not clear");
    }

    (void)BQ25731_ReadReg16(ctx, BQ25731_REG_CHARGE_OPTION0,
                            &result->charge_option0_after);
    result->charge_option0_before = result->charge_option0_after;
    (void)BQ25731_ReadReg16(ctx, BQ25731_REG_CHARGE_OPTION2,
                            &result->charge_option2_after);
    result->charge_option2_before = result->charge_option2_after;
    (void)BQ25731_ReadReg16(ctx, BQ25731_REG_CHARGE_OPTION3,
                            &result->charge_option3_after);
    (void)BQ25731_ReadReg16(ctx, BQ25731_REG_CHARGE_CURRENT,
                            &result->charge_current_raw);
    (void)BQ25731_ReadReg16(ctx, BQ25731_REG_IIN_HOST,
                            &result->iin_host_raw);
    (void)BQ25731_ReadReg16(ctx, BQ25731_REG_ADC_OPTION,
                            &result->adc_option_after);
    result->adc_option_before = ctx->adc_option_before;
    (void)BQ25731_ReadReg16(ctx, BQ25731_REG_CHARGER_STATUS,
                            &result->charger_status_raw);

    if ((result->charge_option3_after & BQ25731_CHGOPT3_OTG_VAP_MODE_MASK) != 0U)
        result->warnings |= BQ_SAFE_WARN_OTG_VAP_MODE_STUCK;
    if ((result->charge_option0_after & BQ25731_CHGOPT0_WDTMR_ADJ_MASK) != 0U)
        result->warnings |= BQ_SAFE_WARN_WATCHDOG_STUCK;
    if ((result->charge_option0_after & BQ25731_CHGOPT0_EN_LWPWR_MASK) != 0U)
        result->warnings |= BQ_SAFE_WARN_LOW_POWER_STUCK;

    iin_step_ma = (BQ25731_RAC_MOHM == 5U) ? 100U : 50U;
    if (BQ25731_ReadTelemetry(ctx, &telemetry) == BQ25731_OK) {
        if ((telemetry.iin_host_ma + iin_step_ma < BQ25731_SAFE_INPUT_CURRENT_MA) ||
            (telemetry.iin_host_ma > BQ25731_SAFE_INPUT_CURRENT_MA + iin_step_ma))
            result->warnings |= BQ_SAFE_WARN_IIN_MISMATCH;
        if (telemetry.in_vap) result->warnings |= BQ_SAFE_WARN_IN_VAP;
        Debug_Printf("[BQ-SAFE] ChargerStatus=0x%04X AC=%u OTG=%u VAP=%u VINDPM=%u IINDPM=%u FCHRG=%u faults=0x%02X",
                     telemetry.charger_status_raw, telemetry.input_present,
                     telemetry.in_otg, telemetry.in_vap, telemetry.in_vindpm,
                     telemetry.in_iin_dpm, telemetry.in_fast_charge,
                     (unsigned)(telemetry.charger_status_raw & 0x00FFU));
        if (telemetry.in_otg || telemetry.in_fast_charge) result->fatal_error = true;
    }
    if ((result->charge_option0_after & BQ25731_CHGOPT0_CHRG_INHIBIT_MASK) == 0U ||
        (result->charge_current_raw & BQ25731_CHARGE_CURRENT_VERIFY_MASK) != 0U ||
        (result->charge_option3_after & BQ25731_CHGOPT3_EN_OTG_MASK) != 0U)
        result->fatal_error = true;

    Debug_Printf("[BQ-SAFE] summary fatal=%u warnings=0x%08lX inhibit=%u EN_OTG=%u OTG_VAP=%u ICHGraw=0x%04X",
                 result->fatal_error, (unsigned long)result->warnings,
                 (result->charge_option0_after & BQ25731_CHGOPT0_CHRG_INHIBIT_MASK) != 0U,
                 (result->charge_option3_after & BQ25731_CHGOPT3_EN_OTG_MASK) != 0U,
                 (result->charge_option3_after & BQ25731_CHGOPT3_OTG_VAP_MODE_MASK) != 0U,
                 result->charge_current_raw);
    if (result->fatal_error) return (status != BQ25731_OK) ? status : BQ25731_ERROR;
    return BQ25731_OK;
}

static BQ25731_Status_t BQ25731_TakeoverRead(BQ25731_Device_t *ctx,
                                              uint8_t reg, uint16_t *raw,
                                              BQ25731_SafeStartupResult_t *result)
{
    BQ25731_Status_t status = BQ25731_ReadReg16(ctx, reg, raw);
    if (status != BQ25731_OK) {
        result->fatal_error = true;
        result->error = BQ_ERR_I2C_READ_FAILED;
    }
    return status;
}

BQ25731_Status_t BQ25731_TakeoverSafeState(BQ25731_Device_t *ctx,
                                            BQ25731_SafeStartupResult_t *result)
{
    BQ25731_Status_t status;
    uint8_t ids[2];
    uint16_t chgopt0, chgcur, chgvolt, iin, chgstatus;
    uint16_t chgopt1, chgopt2, chgopt3, adcopt;
    uint16_t written, readback;
    uint32_t current_ma;
    uint8_t attempt;

    if ((ctx == NULL) || (result == NULL)) return BQ25731_INVALID_ARG;
    memset(result, 0, sizeof(*result));
    result->error = BQ_ERR_NONE;

    status = BQ25731_CheckDevice(ctx);
    if (status != BQ25731_OK) {
        result->fatal_error = true;
        result->error = BQ_ERR_PROBE_FAILED;
        return status;
    }
    status = BQ25731_ReadBlock(ctx, BQ25731_REG_MANUFACTURE_ID, ids, 2U);
    if (status != BQ25731_OK) {
        result->fatal_error = true;
        result->error = BQ_ERR_I2C_READ_FAILED;
        return status;
    }
    result->manufacturer_id = ids[0]; result->device_id = ids[1];

#define TAKEOVER_READ(reg_, dst_) \
    do { if (BQ25731_TakeoverRead(ctx, (reg_), &(dst_), result) != BQ25731_OK) \
             return BQ25731_I2C_ERROR; } while (0)
    TAKEOVER_READ(BQ25731_REG_CHARGE_OPTION0, chgopt0);
    TAKEOVER_READ(BQ25731_REG_CHARGE_CURRENT, chgcur);
    TAKEOVER_READ(BQ25731_REG_CHARGE_VOLTAGE, chgvolt);
    TAKEOVER_READ(BQ25731_REG_IIN_HOST, iin);
    TAKEOVER_READ(BQ25731_REG_CHARGER_STATUS, chgstatus);
    TAKEOVER_READ(BQ25731_REG_CHARGE_OPTION1, chgopt1);
    TAKEOVER_READ(BQ25731_REG_CHARGE_OPTION2, chgopt2);
    TAKEOVER_READ(BQ25731_REG_CHARGE_OPTION3, chgopt3);
    TAKEOVER_READ(BQ25731_REG_ADC_OPTION, adcopt);

    result->charge_option0_before = chgopt0;
    result->charge_option1_before = chgopt1;
    result->charge_option2_before = chgopt2;
    result->charge_option3_before = chgopt3;
    result->adc_option_before = adcopt;
    current_ma = BQ25731_DecodeChargeCurrentMa(chgcur, BQ25731_RSR_MOHM == 5U);
    Debug_Printf("[BQ-TAKEOVER] as_found: ChargeVoltage=%lumV ChargeCurrent=%lumA IIN_HOST=%lumA ChgOpt0=0x%04X ChgOpt3=0x%04X Status=0x%04X",
                 (unsigned long)BQ25731_DecodeChargeVoltageMv(chgvolt),
                 (unsigned long)current_ma,
                 (unsigned long)BQ25731_DecodeInputLimitMa(iin, BQ25731_RAC_MOHM == 5U),
                 chgopt0, chgopt3, chgstatus);
    Debug_Printf("[BQ-TAKEOVER] ChargeCurrent as_found raw=0x%04X decoded=%lumA",
                 chgcur, (unsigned long)current_ma);
    Debug_Printf("[BQ-TAKEOVER] ChargeCurrent decode check: 0000=%lu 0040=%lu 0080=%lu 0100=%lu 1000=%lu 10C0=%lumA",
                 (unsigned long)BQ25731_DecodeChargeCurrentMa(0x0000U, true),
                 (unsigned long)BQ25731_DecodeChargeCurrentMa(0x0040U, true),
                 (unsigned long)BQ25731_DecodeChargeCurrentMa(0x0080U, true),
                 (unsigned long)BQ25731_DecodeChargeCurrentMa(0x0100U, true),
                 (unsigned long)BQ25731_DecodeChargeCurrentMa(0x1000U, true),
                 (unsigned long)BQ25731_DecodeChargeCurrentMa(0x10C0U, true));
    Debug_Printf("[BQ-TAKEOVER] applying safe state: inhibit=1 charge_current=0 EN_OTG=0");

    status = BQ25731_UpdateReg16(ctx, BQ25731_REG_CHARGE_OPTION0, 0U,
                                 BQ25731_CHGOPT0_CHRG_INHIBIT_MASK,
                                 NULL, &written, &readback);
    if (status != BQ25731_OK) {
        result->fatal_error = true; result->error = BQ_ERR_I2C_WRITE_FAILED;
        return status;
    }

    for (attempt = 0U; attempt < 2U; ++attempt) {
        status = BQ25731_WriteReg16(ctx, BQ25731_REG_CHARGE_CURRENT, 0x0000U);
        if (status != BQ25731_OK) {
            result->fatal_error = true; result->error = BQ_ERR_I2C_WRITE_FAILED;
            return status;
        }
        HAL_Delay(2U);
        TAKEOVER_READ(BQ25731_REG_CHARGE_CURRENT, chgcur);
        current_ma = BQ25731_DecodeChargeCurrentMa(chgcur, BQ25731_RSR_MOHM == 5U);
        if (current_ma == 0U) break;
    }
    Debug_Printf("[BQ-TAKEOVER] ChargeCurrent disable write=0x0000 read=0x%04X decoded=%lumA %s",
                 chgcur, (unsigned long)current_ma, current_ma == 0U ? "OK" : "FAIL");

    status = BQ25731_UpdateReg16(ctx, BQ25731_REG_CHARGE_OPTION3,
                                 BQ25731_CHGOPT3_EN_OTG_MASK, 0U,
                                 NULL, &written, &readback);
    if ((status != BQ25731_OK) &&
        ((readback & BQ25731_CHGOPT3_EN_OTG_MASK) != 0U)) {
        result->fatal_error = true; result->error = BQ_ERR_EN_OTG_STUCK_ON;
        return status;
    }

    /* Optional in monitor-only: a TPS/EEPROM configuration may own ILIM_HIZ. */
    status = BQ25731_UpdateReg16(ctx, BQ25731_REG_CHARGE_OPTION2,
                                 BQ25731_CHGOPT2_EN_EXTILIM_MASK, 0U,
                                 NULL, &written, &readback);
    if (status != BQ25731_OK) {
        result->warnings |= BQ_SAFE_WARN_EN_EXTILIM_STUCK;
        Debug_Printf("[BQ-TAKEOVER] WARNING EN_EXTILIM could not be cleared; ignored in monitor-only because charging is inhibited and ChargeCurrent=0");
    }

    /* Final decision uses only fresh, direct register reads. */
    TAKEOVER_READ(BQ25731_REG_CHARGE_OPTION0, chgopt0);
    TAKEOVER_READ(BQ25731_REG_CHARGE_CURRENT, chgcur);
    TAKEOVER_READ(BQ25731_REG_CHARGE_VOLTAGE, chgvolt);
    TAKEOVER_READ(BQ25731_REG_IIN_HOST, iin);
    TAKEOVER_READ(BQ25731_REG_CHARGER_STATUS, chgstatus);
    TAKEOVER_READ(BQ25731_REG_CHARGE_OPTION1, chgopt1);
    TAKEOVER_READ(BQ25731_REG_CHARGE_OPTION2, chgopt2);
    TAKEOVER_READ(BQ25731_REG_CHARGE_OPTION3, chgopt3);
    TAKEOVER_READ(BQ25731_REG_ADC_OPTION, adcopt);
#undef TAKEOVER_READ

    result->charge_option0_after = chgopt0;
    result->charge_option1_after = chgopt1;
    result->charge_option2_after = chgopt2;
    result->charge_option3_after = chgopt3;
    result->charge_current_raw = chgcur;
    result->charge_voltage_raw = chgvolt;
    result->iin_host_raw = iin;
    result->charger_status_raw = chgstatus;
    result->adc_option_after = adcopt;
    result->charge_current_ma = BQ25731_DecodeChargeCurrentMa(chgcur, BQ25731_RSR_MOHM == 5U);
    result->charge_voltage_mv = BQ25731_DecodeChargeVoltageMv(chgvolt);
    result->iin_host_ma = BQ25731_DecodeInputLimitMa(iin, BQ25731_RAC_MOHM == 5U);

#if defined(BQ25731_RAC_VALUE_DEFAULTED) || defined(BQ25731_RSR_VALUE_DEFAULTED)
    result->warnings |= BQ_SAFE_WARN_SENSE_CONFIG_DEFAULTED;
#endif
    if (chgvolt != 0U) result->warnings |= BQ_SAFE_WARN_EEPROM_CONFIG_PRESENT;
    if ((chgopt3 & BQ25731_CHGOPT3_OTG_VAP_MODE_MASK) != 0U) {
        result->warnings |= BQ_SAFE_WARN_OTG_VAP_MODE_STUCK;
        Debug_Printf("[BQ-TAKEOVER] WARNING OTG_VAP_MODE remains set, safe because EN_OTG=0 and OTG pin has pulldown");
    }
    if ((adcopt & BQ25731_ADCOPT_VERIFY_MASK) !=
        (BQ25731_ADCOPT_EXPECTED & BQ25731_ADCOPT_VERIFY_MASK))
        result->warnings |= BQ_SAFE_WARN_ADC_CONFIG;

    Debug_Printf("[BQ-TAKEOVER] after: ChargeVoltage=%lumV ChargeCurrent=%lumA IIN_HOST=%lumA ChgOpt0=0x%04X ChgOpt3=0x%04X Status=0x%04X",
                 (unsigned long)result->charge_voltage_mv,
                 (unsigned long)result->charge_current_ma,
                 (unsigned long)result->iin_host_ma, chgopt0, chgopt3, chgstatus);
    Debug_Printf("[BQ-SAFE-FINAL] chgopt0=0x%04X chgcur=0x%04X chgopt3=0x%04X status=0x%04X",
                 chgopt0, chgcur, chgopt3, chgstatus);
    Debug_Printf("[BQ-SAFE-FINAL] inhibit=%u charge_current_ma=%lu EN_OTG=%u OTG_VAP_MODE=%u IN_OTG=%u IN_FCHRG=%u",
                 (chgopt0 & BQ25731_CHGOPT0_CHRG_INHIBIT_MASK) != 0U,
                 (unsigned long)result->charge_current_ma,
                 (chgopt3 & BQ25731_CHGOPT3_EN_OTG_MASK) != 0U,
                 (chgopt3 & BQ25731_CHGOPT3_OTG_VAP_MODE_MASK) != 0U,
                 (chgstatus & BQ25731_CHGSTATUS_IN_OTG_MASK) != 0U,
                 (chgstatus & BQ25731_CHGSTATUS_IN_FCHRG_MASK) != 0U);

    if ((chgopt0 & BQ25731_CHGOPT0_CHRG_INHIBIT_MASK) == 0U)
        result->error = BQ_ERR_CHRG_INHIBIT_NOT_SET;
    else if (result->charge_current_ma != 0U)
        result->error = BQ_ERR_CHARGE_CURRENT_NOT_ZERO;
    else if ((chgopt3 & BQ25731_CHGOPT3_EN_OTG_MASK) != 0U)
        result->error = BQ_ERR_EN_OTG_STUCK_ON;
    else if ((chgstatus & BQ25731_CHGSTATUS_IN_OTG_MASK) != 0U)
        result->error = BQ_ERR_IN_OTG_ACTIVE;
    else if ((chgstatus & BQ25731_CHGSTATUS_IN_FCHRG_MASK) != 0U)
        result->error = BQ_ERR_IN_FAST_CHARGE;
    result->fatal_error = result->error != BQ_ERR_NONE;

    if (result->charge_voltage_mv != 0U)
        Debug_Printf("[BQ-TAKEOVER] ChargeVoltage left at %lumV from TPS/EEPROM; safe because CHRG_INHIBIT=1 and ChargeCurrent=0",
                     (unsigned long)result->charge_voltage_mv);
    if (result->fatal_error) return BQ25731_ERROR;
    Debug_Printf("[BQ-TAKEOVER] completed %s",
                 result->warnings != 0U ? "with warnings" : "OK");
    return result->warnings != 0U ? BQ25731_OK_WITH_WARNINGS : BQ25731_OK;
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
    option3 &= (uint16_t)~BQ25731_CHGOPT3_OTG_VAP_MODE_MASK;

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
                                     BQ25731_IIN_HOST_VERIFY_MASK);
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
                                     BQ25731_CHARGE_CURRENT_VERIFY_MASK);
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

    /* Keep the converter inhibited until every new limit has been verified. */
    status = BQ25731_InhibitCharging(ctx, NULL, NULL);
    if (status != BQ25731_OK) {
        return status;
    }

    status = BQ25731_ConfigureForMonitoring(ctx);
    if (status != BQ25731_OK) {
        return status;
    }

    status = BQ25731_ApplyUserOptions(ctx);
    if (status != BQ25731_OK) return status;

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

    status = BQ25731_EnableAdc(ctx);
    if (status != BQ25731_OK) return status;

    /* This is intentionally last: charging cannot start on stale settings. */
    return BQ25731_PrepareForCharging(ctx, NULL, NULL);
}

BQ25731_Status_t BQ25731_EnableOtg(BQ25731_Device_t *ctx,
                                   uint32_t voltage_mv,
                                   uint32_t current_limit_ma,
                                   bool large_output_cap)
{
#if (BQ25731_HW_OTG_ALLOWED == 0U)
    (void)ctx;
    (void)voltage_mv;
    (void)current_limit_ma;
    (void)large_output_cap;
    return BQ25731_OTG_BLOCKED;
#else
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
#endif
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
    option3 &= (uint16_t)~BQ25731_CHGOPT3_OTG_VAP_MODE_MASK;

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

BQ25731_Status_t BQ25731_ReadMonitorSnapshot(BQ25731_Device_t *dev,
                                              BQ25731_MonitorSnapshot_t *out)
{
    BQ25731_Status_t status;
    uint32_t battery_base_mv;
    uint8_t vbus, ichg, idchg, iin, vbat, vsys;
    uint8_t config_raw[0x10U];
    uint8_t monitor_raw[0x1CU];

#define SNAP_LE16(buf_, off_) \
    ((uint16_t)(buf_)[(off_)] | ((uint16_t)(buf_)[(off_) + 1U] << 8))

    if ((dev == NULL) || (out == NULL)) return BQ25731_INVALID_ARG;
    memset(out, 0, sizeof(*out));

    /* Read two contiguous ranges instead of issuing 13 separate I2Cr tasks.
     * This keeps the TPS25751 controller queue below its documented/typical
     * service rate and makes the four ADC result words one coherent sample. */
    status = BQ25731_ReadBlockChunk(dev, BQ25731_REG_CHARGE_OPTION0,
                                    config_raw, sizeof(config_raw));
    if (status != BQ25731_OK) return status;
    status = BQ25731_ReadBlockChunk(dev, BQ25731_REG_CHARGER_STATUS,
                                    monitor_raw, sizeof(monitor_raw));
    if (status != BQ25731_OK) return status;

    out->charge_option0_raw = SNAP_LE16(config_raw, 0x00U);
    out->charge_current_setting_raw = SNAP_LE16(config_raw, 0x02U);
    out->charge_voltage_setting_raw = SNAP_LE16(config_raw, 0x04U);
    out->iin_host_raw = SNAP_LE16(config_raw, 0x0EU);

    out->charger_status_raw = SNAP_LE16(monitor_raw, 0x00U);
    out->adc_vbus_psys_raw = SNAP_LE16(monitor_raw, 0x06U);
    out->adc_ibat_raw = SNAP_LE16(monitor_raw, 0x08U);
    out->adc_iin_cmpin_raw = SNAP_LE16(monitor_raw, 0x0AU);
    out->adc_vsys_vbat_raw = SNAP_LE16(monitor_raw, 0x0CU);
    out->charge_option1_raw = SNAP_LE16(monitor_raw, 0x10U);
    out->charge_option2_raw = SNAP_LE16(monitor_raw, 0x12U);
    out->charge_option3_raw = SNAP_LE16(monitor_raw, 0x14U);
    out->adc_option_raw = SNAP_LE16(monitor_raw, 0x1AU);

    out->charge_current_setting_ma = BQ25731_DecodeChargeCurrentMa(
        out->charge_current_setting_raw, BQ25731_RSR_MOHM == 5U);
    out->charge_voltage_setting_mv =
        BQ25731_DecodeChargeVoltageMv(out->charge_voltage_setting_raw);
    out->iin_host_ma = BQ25731_DecodeInputLimitMa(
        out->iin_host_raw, BQ25731_RAC_MOHM == 5U);

    vbus = (uint8_t)(out->adc_vbus_psys_raw >> 8);
    idchg = (uint8_t)(out->adc_ibat_raw & 0xFFU);
    ichg = (uint8_t)(out->adc_ibat_raw >> 8);
    iin = (uint8_t)(out->adc_iin_cmpin_raw >> 8);
    vbat = (uint8_t)(out->adc_vsys_vbat_raw & 0xFFU);
    vsys = (uint8_t)(out->adc_vsys_vbat_raw >> 8);
    battery_base_mv = (BQ25731_CELL_COUNT == 5U) ? 8160U : 2880U;

    out->low_power_mode =
        (out->charge_option0_raw & BQ25731_CHGOPT0_EN_LWPWR_MASK) != 0U;
    out->en_adc_vbat = (out->adc_option_raw & 0x0001U) != 0U;
    out->en_adc_vsys = (out->adc_option_raw & 0x0002U) != 0U;
    out->en_adc_ichg = (out->adc_option_raw & 0x0004U) != 0U;
    out->en_adc_idchg = (out->adc_option_raw & 0x0008U) != 0U;
    out->en_adc_iin = (out->adc_option_raw & 0x0010U) != 0U;
    out->en_adc_psys = (out->adc_option_raw & 0x0020U) != 0U;
    out->en_adc_vbus = (out->adc_option_raw & 0x0040U) != 0U;
    out->en_adc_cmpin = (out->adc_option_raw & 0x0080U) != 0U;
    out->adc_continuous =
        (out->adc_option_raw & BQ25731_ADCOPT_ADC_CONV_MASK) != 0U;
    out->adc_start = (out->adc_option_raw & BQ25731_ADCOPT_ADC_START_MASK) != 0U;
    out->adc_fullscale =
        (out->adc_option_raw & BQ25731_ADCOPT_ADC_FULLSCALE_MASK) != 0U;
    out->adc_any_channel_enabled = (out->adc_option_raw & 0x00FFU) != 0U;
    out->adc_required_channels_enabled =
        (out->adc_option_raw & 0x005FU) == 0x005FU;
    out->adc_running = out->adc_continuous || out->adc_start;

    out->vbus_valid = out->en_adc_vbus && (vbus != 0U) && out->adc_running;
    out->vsys_valid = out->en_adc_vsys && (vsys != 0U) && out->adc_running;
    out->vbat_valid = out->en_adc_vbat && (vbat != 0U) && out->adc_running;
    out->iin_valid = out->en_adc_iin && out->adc_running;
    out->ichg_valid = out->en_adc_ichg && out->adc_running;
    out->idchg_valid = out->en_adc_idchg && out->adc_running;

    if (out->vbus_valid) out->adc_vbus_mv = (uint32_t)vbus * 96U;
    if (out->ichg_valid) out->adc_ichg_ma = (uint32_t)(ichg & 0x7FU) *
        ((BQ25731_RSR_MOHM == 5U) ? 128U : 64U);
    if (out->idchg_valid) out->adc_idchg_ma = (uint32_t)(idchg & 0x7FU) *
        ((BQ25731_RSR_MOHM == 5U) ? 512U : 256U);
    if (out->iin_valid) out->adc_iin_ma = (uint32_t)iin *
        ((BQ25731_RAC_MOHM == 5U) ? 100U : 50U);
    if (out->vbat_valid)
        out->adc_vbat_mv = battery_base_mv + BQ_USER_VBAT_VSYS_CORRECTION_MV +
                           ((uint32_t)vbat * 64U);
    if (out->vsys_valid)
        out->adc_vsys_mv = battery_base_mv + BQ_USER_VBAT_VSYS_CORRECTION_MV +
                           ((uint32_t)vsys * 64U);

    /* Expected ADCOption for useful monitoring: 0xE05F enables conversion,
     * start/full-scale and VBAT/VSYS/ICHG/IDCHG/IIN/VBUS. In TPS_EEPROM owner
     * mode this must be configured in EEPROM/TI GUI, never by this reader. */

    return BQ25731_OK;
#undef SNAP_LE16
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
        case BQ25731_OK_WITH_WARNINGS:
            return "OK_WITH_WARNINGS";
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
        case BQ25731_OTG_BLOCKED:
            return "OTG_BLOCKED";
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
