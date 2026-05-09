#include "tps25751.h"

#include <string.h>

#define TPS25751_I2C_TIMEOUT_MS               100U

#define TPS25751_REG_MODE                     0x03U
#define TPS25751_REG_STATUS                   0x1AU
#define TPS25751_REG_POWER_PATH_STATUS        0x26U
#define TPS25751_REG_RX_SOURCE_CAPS           0x30U
#define TPS25751_REG_RX_SINK_CAPS             0x31U
#define TPS25751_REG_TX_SOURCE_CAPS           0x32U
#define TPS25751_REG_TX_SINK_CAPS             0x33U
#define TPS25751_REG_ACTIVE_PDO               0x34U
#define TPS25751_REG_ACTIVE_RDO               0x35U
#define TPS25751_REG_POWER_STATUS             0x3FU
#define TPS25751_REG_PD_STATUS                0x40U
#define TPS25751_REG_TYPEC_STATUS             0x69U
#define TPS25751_REG_ADC_RESULTS              0x6AU

static uint16_t TPS25751_ReadLe16(const uint8_t *data)
{
    return ((uint16_t)data[0]) |
           ((uint16_t)data[1] << 8);
}

static uint32_t TPS25751_ReadLe32(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static uint64_t TPS25751_ReadLe64Partial(const uint8_t *data, uint8_t len)
{
    uint64_t value = 0U;
    uint8_t i;

    if (len > 8U) {
        len = 8U;
    }

    for (i = 0U; i < len; ++i) {
        value |= ((uint64_t)data[i]) << (8U * i);
    }

    return value;
}

static TPS25751_Mode_t TPS25751_DecodeMode(const uint8_t mode_ascii[5])
{
    if ((mode_ascii[0] == 'A') &&
        (mode_ascii[1] == 'P') &&
        (mode_ascii[2] == 'P') &&
        (mode_ascii[3] == ' ')) {
        return TPS25751_MODE_APP;
    }

    if ((mode_ascii[0] == 'P') &&
        (mode_ascii[1] == 'T') &&
        (mode_ascii[2] == 'C') &&
        (mode_ascii[3] == 'H')) {
        return TPS25751_MODE_PTCH;
    }

    if ((mode_ascii[0] == 'B') &&
        (mode_ascii[1] == 'O') &&
        (mode_ascii[2] == 'O') &&
        (mode_ascii[3] == 'T')) {
        return TPS25751_MODE_BOOT;
    }

    return TPS25751_MODE_UNKNOWN;
}

static TPS25751_Status_t TPS25751_MapHalStatus(TPS25751_Device_t *dev,
                                               uint8_t reg,
                                               uint8_t requested_payload_len,
                                               HAL_StatusTypeDef hal_status)
{
    if (dev != NULL) {
        dev->last_register = reg;
        dev->last_requested_payload_len = requested_payload_len;

        if (dev->hi2c != NULL) {
            dev->last_error = HAL_I2C_GetError(dev->hi2c);
        }
    }

    if (hal_status == HAL_OK) {
        if (dev != NULL) {
            dev->last_error = 0U;
        }

        return TPS25751_OK;
    }

    if ((hal_status == HAL_ERROR) ||
        (hal_status == HAL_TIMEOUT) ||
        (hal_status == HAL_BUSY)) {
        return TPS25751_I2C_ERROR;
    }

    return TPS25751_ERROR;
}

static TPS25751_PdoInfo_t TPS25751_DecodePdo(uint32_t raw)
{
    TPS25751_PdoInfo_t pdo;
    uint32_t supply_type;
    uint32_t apdo_type;

    memset(&pdo, 0, sizeof(pdo));

    pdo.raw = raw;
    supply_type = (raw >> 30) & 0x03U;

    if (raw == 0U) {
        pdo.type = TPS25751_SUPPLY_UNKNOWN;
        return pdo;
    }

    if (supply_type == 0U) {
        pdo.type = TPS25751_SUPPLY_FIXED;
        pdo.voltage_mv = ((raw >> 10) & 0x3FFU) * 50U;
        pdo.current_ma = (raw & 0x3FFU) * 10U;
        pdo.power_mw = (pdo.voltage_mv * pdo.current_ma) / 1000U;
        return pdo;
    }

    if (supply_type == 1U) {
        pdo.type = TPS25751_SUPPLY_BATTERY;
        pdo.min_mv = ((raw >> 10) & 0x3FFU) * 50U;
        pdo.max_mv = ((raw >> 20) & 0x3FFU) * 50U;
        pdo.power_mw = (raw & 0x3FFU) * 250U;
        return pdo;
    }

    if (supply_type == 2U) {
        pdo.type = TPS25751_SUPPLY_VARIABLE;
        pdo.min_mv = ((raw >> 10) & 0x3FFU) * 50U;
        pdo.max_mv = ((raw >> 20) & 0x3FFU) * 50U;
        pdo.current_ma = (raw & 0x3FFU) * 10U;
        pdo.power_mw = (pdo.max_mv * pdo.current_ma) / 1000U;
        return pdo;
    }

    apdo_type = (raw >> 28) & 0x03U;

    if (apdo_type == 0U) {
        pdo.type = TPS25751_SUPPLY_APDO_PPS;
        pdo.min_mv = ((raw >> 8) & 0xFFU) * 100U;
        pdo.max_mv = ((raw >> 17) & 0xFFU) * 100U;
        pdo.current_ma = (raw & 0x7FU) * 50U;
        pdo.power_mw = (pdo.max_mv * pdo.current_ma) / 1000U;
    } else {
        pdo.type = TPS25751_SUPPLY_APDO_OTHER;
    }

    return pdo;
}

static TPS25751_RdoInfo_t TPS25751_DecodeRdo(uint32_t raw)
{
    TPS25751_RdoInfo_t rdo;

    memset(&rdo, 0, sizeof(rdo));

    rdo.raw = raw;
    rdo.valid = (raw != 0U);

    if (!rdo.valid) {
        return rdo;
    }

    rdo.object_position = (uint8_t)((raw >> 28) & 0x0FU);
    rdo.capability_mismatch = ((raw >> 26) & 0x01U) != 0U;
    rdo.usb_comm_capable = ((raw >> 25) & 0x01U) != 0U;
    rdo.no_usb_suspend = ((raw >> 24) & 0x01U) != 0U;
    rdo.unchunked_supported = ((raw >> 23) & 0x01U) != 0U;

    rdo.operating_current_ma = ((raw >> 10) & 0x3FFU) * 10U;
    rdo.max_current_ma = (raw & 0x3FFU) * 10U;

    return rdo;
}

static void TPS25751_DecodeCapsRegister(const uint8_t *payload,
                                        uint8_t payload_len,
                                        uint8_t pdo_offset,
                                        TPS25751_PdoList_t *list)
{
    uint8_t count;
    uint8_t i;

    if ((payload == NULL) || (list == NULL) || (payload_len == 0U)) {
        return;
    }

    memset(list, 0, sizeof(*list));

    count = payload[0] & 0x07U;

    if (count > 7U) {
        count = 7U;
    }

    list->count = count;

    for (i = 0U; i < count; ++i) {
        uint8_t offset = (uint8_t)(pdo_offset + (i * 4U));

        if ((uint8_t)(offset + 3U) >= payload_len) {
            list->count = i;
            return;
        }

        list->pdo[i] = TPS25751_DecodePdo(TPS25751_ReadLe32(&payload[offset]));
    }
}

static void TPS25751_DecodeTelemetry(TPS25751_Telemetry_t *t)
{
    if (t == NULL) {
        return;
    }

    t->plug_present = ((t->status_raw >> 0) & 0x01U) != 0U;
    t->connection_state = (uint8_t)((t->status_raw >> 1) & 0x07U);
    t->orientation_cc2 = ((t->status_raw >> 4) & 0x01U) != 0U;
    t->port_role_source = ((t->status_raw >> 5) & 0x01U) != 0U;
    t->data_role_dfp = ((t->status_raw >> 6) & 0x01U) != 0U;
    t->vbus_status = (uint8_t)((t->status_raw >> 20) & 0x03U);
    t->usb_host_present = (uint8_t)((t->status_raw >> 22) & 0x03U);
    t->legacy_status = (uint8_t)((t->status_raw >> 24) & 0x03U);

    t->ppcable_switch = (uint8_t)((t->power_path_raw >> 0) & 0x03U);
    t->pp1_switch = (uint8_t)((t->power_path_raw >> 6) & 0x07U);
    t->pp3_switch = (uint8_t)((t->power_path_raw >> 12) & 0x07U);
    t->pp1_overcurrent = ((t->power_path_raw >> 28) & 0x01U) != 0U;
    t->ppcable_overcurrent = ((t->power_path_raw >> 34) & 0x01U) != 0U;
    t->power_source = (uint8_t)((t->power_path_raw >> 38) & 0x03U);

    t->power_connection = ((t->power_status_raw >> 0) & 0x01U) != 0U;
    t->power_status_source = ((t->power_status_raw >> 1) & 0x01U) != 0U;
    t->typec_current_status = (uint8_t)((t->power_status_raw >> 2) & 0x03U);

    t->cc_pullup = (uint8_t)((t->pd_status_raw >> 2) & 0x03U);
    t->pd_port_type = (uint8_t)((t->pd_status_raw >> 4) & 0x03U);
    t->pd_role_source = ((t->pd_status_raw >> 6) & 0x01U) != 0U;
    t->soft_reset_reason = (uint8_t)((t->pd_status_raw >> 8) & 0x1FU);
    t->hard_reset_reason = (uint8_t)((t->pd_status_raw >> 16) & 0x3FU);

    t->cc_pin_for_pd = (uint8_t)((t->typec_raw >> 0) & 0xFFU);
    t->cc1_state = (uint8_t)((t->typec_raw >> 8) & 0xFFU);
    t->cc2_state = (uint8_t)((t->typec_raw >> 16) & 0xFFU);
    t->typec_state = (uint8_t)((t->typec_raw >> 24) & 0xFFU);
}

TPS25751_Status_t TPS25751_Init(TPS25751_Device_t *dev,
                                I2C_HandleTypeDef *hi2c,
                                uint8_t address_7bit)
{
    if ((dev == NULL) || (hi2c == NULL)) {
        return TPS25751_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));

    dev->hi2c = hi2c;
    dev->address_7bit = address_7bit;

    return TPS25751_OK;
}

TPS25751_Status_t TPS25751_ReadPayload(TPS25751_Device_t *dev,
                                       uint8_t reg,
                                       uint8_t *payload,
                                       uint8_t payload_capacity,
                                       uint8_t *payload_len)
{
    uint8_t raw[TPS25751_MAX_REG_PAYLOAD + 1U];
    uint8_t reported_len;
    uint8_t copy_len;
    HAL_StatusTypeDef hal_status;

    if ((dev == NULL) ||
        (dev->hi2c == NULL) ||
        (payload == NULL) ||
        (payload_capacity == 0U)) {
        return TPS25751_INVALID_ARG;
    }

    memset(raw, 0, sizeof(raw));
    memset(payload, 0, payload_capacity);

    hal_status = HAL_I2C_Mem_Read(dev->hi2c,
                                  (uint16_t)(dev->address_7bit << 1),
                                  reg,
                                  I2C_MEMADD_SIZE_8BIT,
                                  raw,
                                  (uint16_t)(payload_capacity + 1U),
                                  TPS25751_I2C_TIMEOUT_MS);

    if (hal_status != HAL_OK) {
        return TPS25751_MapHalStatus(dev, reg, payload_capacity, hal_status);
    }

    reported_len = raw[0];
    copy_len = reported_len;

    if (copy_len > payload_capacity) {
        copy_len = payload_capacity;
    }

    memcpy(payload, &raw[1], copy_len);

    dev->last_register = reg;
    dev->last_requested_payload_len = payload_capacity;
    dev->last_reported_payload_len = reported_len;
    dev->last_error = 0U;

    if (payload_len != NULL) {
        *payload_len = copy_len;
    }

    return TPS25751_OK;
}

TPS25751_Status_t TPS25751_ReadTelemetry(TPS25751_Device_t *dev,
                                         TPS25751_Telemetry_t *t)
{
    TPS25751_Status_t status;
    uint8_t payload[TPS25751_MAX_REG_PAYLOAD];
    uint8_t len;

    if ((dev == NULL) || (t == NULL)) {
        return TPS25751_INVALID_ARG;
    }

    memset(t, 0, sizeof(*t));

    status = TPS25751_ReadPayload(dev, TPS25751_REG_MODE, payload, 4U, &len);
    if (status != TPS25751_OK) {
        t->status = status;
        return status;
    }

    memcpy(t->mode_ascii, payload, 4U);
    t->mode_ascii[4] = '\0';
    t->mode = TPS25751_DecodeMode((const uint8_t *)t->mode_ascii);
    t->app_ready = (t->mode == TPS25751_MODE_APP);

    status = TPS25751_ReadPayload(dev, TPS25751_REG_STATUS, payload, 5U, &len);
    if (status != TPS25751_OK) {
        t->status = status;
        return status;
    }
    t->status_raw = TPS25751_ReadLe64Partial(payload, len);

    status = TPS25751_ReadPayload(dev, TPS25751_REG_POWER_PATH_STATUS, payload, 5U, &len);
    if (status != TPS25751_OK) {
        t->status = status;
        return status;
    }
    t->power_path_raw = TPS25751_ReadLe64Partial(payload, len);

    status = TPS25751_ReadPayload(dev, TPS25751_REG_POWER_STATUS, payload, 2U, &len);
    if (status != TPS25751_OK) {
        t->status = status;
        return status;
    }
    t->power_status_raw = TPS25751_ReadLe16(payload);

    status = TPS25751_ReadPayload(dev, TPS25751_REG_PD_STATUS, payload, 4U, &len);
    if (status != TPS25751_OK) {
        t->status = status;
        return status;
    }
    t->pd_status_raw = TPS25751_ReadLe32(payload);

    status = TPS25751_ReadPayload(dev, TPS25751_REG_TYPEC_STATUS, payload, 4U, &len);
    if (status != TPS25751_OK) {
        t->status = status;
        return status;
    }
    t->typec_raw = TPS25751_ReadLe32(payload);

    status = TPS25751_ReadPayload(dev, TPS25751_REG_ADC_RESULTS, payload, TPS25751_ADC_RESULTS_LEN, &len);
    if (status == TPS25751_OK) {
        t->adcin1_mv = (uint32_t)payload[0] * 14U;
        t->adcin2_mv = (uint32_t)payload[1] * 14U;
        t->ldo3v3_mv = (uint32_t)payload[2] * 14U;
        t->vbus_mv = (uint32_t)payload[3] * 98U;
        t->ibus_ma = (uint32_t)payload[5] * 165U / 10U;
        t->ibus_mean_ma = (uint32_t)payload[11] * 165U / 10U;
    }

    status = TPS25751_ReadPayload(dev, TPS25751_REG_RX_SOURCE_CAPS, payload, TPS25751_RX_SOURCE_CAPS_MAX_LEN, &len);
    if (status == TPS25751_OK) {
        TPS25751_DecodeCapsRegister(payload, len, 1U, &t->rx_source_caps);
    }

    status = TPS25751_ReadPayload(dev, TPS25751_REG_RX_SINK_CAPS, payload, TPS25751_RX_SINK_CAPS_MAX_LEN, &len);
    if (status == TPS25751_OK) {
        TPS25751_DecodeCapsRegister(payload, len, 1U, &t->rx_sink_caps);
    }

    status = TPS25751_ReadPayload(dev, TPS25751_REG_TX_SOURCE_CAPS, payload, TPS25751_TX_SOURCE_CAPS_MAX_LEN, &len);
    if (status == TPS25751_OK) {
        TPS25751_DecodeCapsRegister(payload, len, 3U, &t->tx_source_caps);
    }

    status = TPS25751_ReadPayload(dev, TPS25751_REG_TX_SINK_CAPS, payload, TPS25751_TX_SINK_CAPS_MAX_LEN, &len);
    if (status == TPS25751_OK) {
        TPS25751_DecodeCapsRegister(payload, len, 1U, &t->tx_sink_caps);
    }

    status = TPS25751_ReadPayload(dev, TPS25751_REG_ACTIVE_PDO, payload, 6U, &len);
    if (status == TPS25751_OK) {
        t->active_pdo_raw = TPS25751_ReadLe64Partial(payload, len);
        t->active_pdo = TPS25751_DecodePdo(TPS25751_ReadLe32(payload));
    }

    status = TPS25751_ReadPayload(dev, TPS25751_REG_ACTIVE_RDO, payload, 12U, &len);
    if (status == TPS25751_OK) {
        t->active_rdo_raw = TPS25751_ReadLe32(payload);
        t->active_rdo = TPS25751_DecodeRdo(t->active_rdo_raw);
    }

    TPS25751_DecodeTelemetry(t);

    t->status = TPS25751_OK;
    return TPS25751_OK;
}

const char *TPS25751_StatusToString(TPS25751_Status_t status)
{
    switch (status) {
        case TPS25751_OK:
            return "OK";
        case TPS25751_ERROR:
            return "ERROR";
        case TPS25751_I2C_ERROR:
            return "I2C_ERROR";
        case TPS25751_INVALID_ARG:
            return "INVALID_ARG";
        case TPS25751_BAD_LENGTH:
            return "BAD_LENGTH";
        default:
            return "UNKNOWN";
    }
}

const char *TPS25751_ModeToString(TPS25751_Mode_t mode)
{
    switch (mode) {
        case TPS25751_MODE_BOOT:
            return "BOOT";
        case TPS25751_MODE_PTCH:
            return "PTCH";
        case TPS25751_MODE_APP:
            return "APP";
        default:
            return "UNKNOWN";
    }
}

const char *TPS25751_PdoTypeToString(TPS25751_PdoSupply_t type)
{
    switch (type) {
        case TPS25751_SUPPLY_FIXED:
            return "FIXED";
        case TPS25751_SUPPLY_BATTERY:
            return "BAT";
        case TPS25751_SUPPLY_VARIABLE:
            return "VAR";
        case TPS25751_SUPPLY_APDO_PPS:
            return "PPS";
        case TPS25751_SUPPLY_APDO_OTHER:
            return "APDO";
        default:
            return "EMPTY";
    }
}

const char *TPS25751_ConnectionStateToString(uint8_t state)
{
    switch (state) {
        case 0:
            return "No connection";
        case 1:
            return "Port disabled";
        case 2:
            return "Audio accessory";
        case 3:
            return "Debug accessory";
        case 4:
            return "Ra only";
        case 6:
            return "Attached";
        case 7:
            return "Attached + Ra";
        default:
            return "Reserved";
    }
}

const char *TPS25751_VbusStatusToString(uint8_t state)
{
    switch (state) {
        case 0:
            return "vSafe0V";
        case 1:
            return "vSafe5V";
        case 2:
            return "Expected";
        case 3:
            return "Out of range";
        default:
            return "Unknown";
    }
}

const char *TPS25751_TypecStateToString(uint8_t state)
{
    switch (state) {
        case 0x00:
            return "Disabled";
        case 0x05:
            return "ErrorRecovery";
        case 0x45:
            return "Try.SRC";
        case 0x4E:
            return "TryWait.SNK";
        case 0x4F:
            return "Try.SNK";
        case 0x50:
            return "TryWait.SRC";
        case 0x60:
            return "Attached.SRC";
        case 0x61:
            return "Attached.SNK";
        case 0x62:
            return "AudioAccessory";
        case 0x63:
            return "DebugAccessory";
        case 0x64:
            return "AttachWait.SRC";
        case 0x65:
            return "AttachWait.SNK";
        case 0x66:
            return "Unattached.SNK";
        case 0x67:
            return "Unattached.SRC";
        default:
            return "Unknown";
    }
}

const char *TPS25751_CcStateToString(uint8_t state)
{
    switch (state) {
        case 0:
            return "Open";
        case 1:
            return "Ra";
        case 2:
            return "Rd";
        case 3:
            return "Default";
        case 4:
            return "1.5A";
        case 5:
            return "3.0A";
        default:
            return "Unknown";
    }
}

const char *TPS25751_PowerPathSwitchToString(uint8_t state)
{
    switch (state) {
        case 0:
            return "OFF";
        case 1:
            return "FAULT/OFF";
        case 2:
            return "OUT";
        case 3:
            return "IN";
        default:
            return "Unknown";
    }
}

const char *TPS25751_TypecCurrentToString(uint8_t value)
{
    switch (value) {
        case 0:
            return "Default";
        case 1:
            return "1.5A";
        case 2:
            return "3.0A";
        case 3:
            return "PD contract";
        default:
            return "Unknown";
    }
}