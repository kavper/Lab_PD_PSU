#include "tps25751.h"

#include <string.h>

#define TPS25751_TRANSFER_TIMEOUT_MS  25U
#define TPS25751_COMMAND_TIMEOUT_MS   1000U
#define TPS25751_COMMAND_POLL_MS      10U

enum {
    TPS_OP_STATE_START = 0,
    TPS_OP_STATE_WAIT_REGISTER,
    TPS_OP_STATE_START_DATA1,
    TPS_OP_STATE_WAIT_DATA1,
    TPS_OP_STATE_START_COMMAND,
    TPS_OP_STATE_WAIT_COMMAND,
    TPS_OP_STATE_POLL_DELAY,
    TPS_OP_STATE_WAIT_POLL,
    TPS_OP_STATE_START_RESULT,
    TPS_OP_STATE_WAIT_RESULT
};

static uint64_t TPS25751_ReadLe40(const uint8_t *data)
{
    return ((uint64_t)data[0]) |
           ((uint64_t)data[1] << 8) |
           ((uint64_t)data[2] << 16) |
           ((uint64_t)data[3] << 24) |
           ((uint64_t)data[4] << 32);
}

uint16_t TPS25751_ReadLe16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0]) |
                      ((uint16_t)data[1] << 8));
}

uint32_t TPS25751_ReadLe32(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static uint32_t TPS25751_GetLeBitField(const uint8_t *data,
                                       uint16_t start_bit,
                                       uint8_t width)
{
    uint32_t value = 0U;
    uint8_t i;

    for (i = 0U; i < width; ++i) {
        uint16_t bit = (uint16_t)(start_bit + i);
        if ((data[bit / 8U] & (uint8_t)(1U << (bit % 8U))) != 0U) {
            value |= (1UL << i);
        }
    }
    return value;
}

static void TPS25751_SetLeBitField(uint8_t *data,
                                   uint16_t start_bit,
                                   uint8_t width,
                                   uint32_t value)
{
    uint8_t i;

    for (i = 0U; i < width; ++i) {
        uint16_t bit = (uint16_t)(start_bit + i);
        uint8_t mask = (uint8_t)(1U << (bit % 8U));

        if ((value & (1UL << i)) != 0U) {
            data[bit / 8U] |= mask;
        } else {
            data[bit / 8U] &= (uint8_t)~mask;
        }
    }
}

static void TPS25751_WriteLe32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8);
    data[2] = (uint8_t)(value >> 16);
    data[3] = (uint8_t)(value >> 24);
}

static uint32_t TPS25751_MakeFourCc(const char text[4])
{
    return ((uint32_t)(uint8_t)text[0]) |
           ((uint32_t)(uint8_t)text[1] << 8) |
           ((uint32_t)(uint8_t)text[2] << 16) |
           ((uint32_t)(uint8_t)text[3] << 24);
}

static bool TPS25751_TickReached(uint32_t now_ms, uint32_t deadline_ms)
{
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static void TPS25751_Finish(TPS25751_Device_t *dev,
                            TPS25751_Status_t status)
{
    dev->operation_status = status;
    dev->transfer_active = false;
}

static TPS25751_Status_t TPS25751_StartRawRead(TPS25751_Device_t *dev,
                                               uint8_t reg,
                                               uint8_t payload_length,
                                               uint32_t now_ms)
{
    HAL_StatusTypeDef hal_status;

    memset(dev->rx_buffer, 0, (size_t)payload_length + 1U);
    dev->register_address = reg;
    dev->requested_length = payload_length;
    dev->reported_length = 0U;
    dev->transfer_is_read = true;

    hal_status = HAL_I2C_Mem_Read_IT(dev->hi2c,
                                     (uint16_t)(dev->address_7bit << 1),
                                     reg,
                                     I2C_MEMADD_SIZE_8BIT,
                                     dev->rx_buffer,
                                     (uint16_t)(payload_length + 1U));
    if (hal_status == HAL_BUSY) {
        return TPS25751_BUSY;
    }
    if (hal_status != HAL_OK) {
        dev->hal_error = HAL_I2C_GetError(dev->hi2c);
        TPS25751_Finish(dev, TPS25751_I2C_ERROR);
        return TPS25751_I2C_ERROR;
    }

    dev->transfer_active = true;
    dev->transfer_started_ms = now_ms;
    return TPS25751_BUSY;
}

static TPS25751_Status_t TPS25751_StartRawWrite(TPS25751_Device_t *dev,
                                                uint8_t reg,
                                                const uint8_t *payload,
                                                uint8_t payload_length,
                                                uint32_t now_ms)
{
    HAL_StatusTypeDef hal_status;

    dev->tx_buffer[0] = payload_length;
    memcpy(&dev->tx_buffer[1], payload, payload_length);
    dev->register_address = reg;
    dev->requested_length = payload_length;
    dev->transfer_is_read = false;

    hal_status = HAL_I2C_Mem_Write_IT(dev->hi2c,
                                      (uint16_t)(dev->address_7bit << 1),
                                      reg,
                                      I2C_MEMADD_SIZE_8BIT,
                                      dev->tx_buffer,
                                      (uint16_t)(payload_length + 1U));
    if (hal_status == HAL_BUSY) {
        return TPS25751_BUSY;
    }
    if (hal_status != HAL_OK) {
        dev->hal_error = HAL_I2C_GetError(dev->hi2c);
        TPS25751_Finish(dev, TPS25751_I2C_ERROR);
        return TPS25751_I2C_ERROR;
    }

    dev->transfer_active = true;
    dev->transfer_started_ms = now_ms;
    return TPS25751_BUSY;
}

static TPS25751_Status_t TPS25751_CheckRead(TPS25751_Device_t *dev,
                                            bool exact_length)
{
    uint8_t copy_length;

    dev->reported_length = dev->rx_buffer[0];
    if (exact_length) {
        if (dev->reported_length != dev->requested_length) {
            return TPS25751_BAD_LENGTH;
        }
    } else if (dev->reported_length < dev->requested_length) {
        return TPS25751_BAD_LENGTH;
    }

    copy_length = dev->reported_length;
    if (copy_length > dev->requested_length) {
        copy_length = dev->requested_length;
    }
    memcpy(dev->result, &dev->rx_buffer[1], copy_length);
    dev->result_length = copy_length;
    return TPS25751_OK;
}

static TPS25751_Status_t TPS25751_StartOperation(TPS25751_Device_t *dev,
                                                 TPS25751_Operation_t operation)
{
    if ((dev == NULL) || (dev->hi2c == NULL)) {
        return TPS25751_INVALID_ARG;
    }
    if (TPS25751_IsBusy(dev)) {
        return TPS25751_BUSY;
    }
    if (HAL_I2C_GetState(dev->hi2c) != HAL_I2C_STATE_READY) {
        return TPS25751_BUSY;
    }

    dev->operation = operation;
    dev->operation_status = TPS25751_BUSY;
    dev->operation_state = TPS_OP_STATE_START;
    dev->transfer_active = false;
    dev->result_length = 0U;
    dev->reported_length = 0U;
    dev->task_return_code = 0U;
    dev->hal_error = 0U;
    dev->operation_started_ms = HAL_GetTick();
    return TPS25751_OK;
}

TPS25751_Status_t TPS25751_Init(TPS25751_Device_t *dev,
                                I2C_HandleTypeDef *hi2c,
                                uint8_t address_7bit)
{
    if ((dev == NULL) || (hi2c == NULL) || (address_7bit > 0x7FU)) {
        return TPS25751_INVALID_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->hi2c = hi2c;
    dev->address_7bit = address_7bit;
    dev->operation_status = TPS25751_OK;
    return TPS25751_OK;
}

TPS25751_Status_t TPS25751_StartReadRegister(TPS25751_Device_t *dev,
                                             uint8_t reg,
                                             uint8_t length)
{
    TPS25751_Status_t status;

    if ((length == 0U) || (length > TPS25751_MAX_PAYLOAD)) {
        return TPS25751_INVALID_ARG;
    }
    status = TPS25751_StartOperation(dev, TPS25751_OP_READ_REGISTER);
    if (status != TPS25751_OK) {
        return status;
    }
    dev->register_address = reg;
    dev->requested_length = length;
    return TPS25751_OK;
}

TPS25751_Status_t TPS25751_StartWriteRegister(TPS25751_Device_t *dev,
                                              uint8_t reg,
                                              const uint8_t *data,
                                              uint8_t length)
{
    TPS25751_Status_t status;

    if ((data == NULL) || (length == 0U) ||
        (length > TPS25751_MAX_PAYLOAD)) {
        return TPS25751_INVALID_ARG;
    }
    status = TPS25751_StartOperation(dev, TPS25751_OP_WRITE_REGISTER);
    if (status != TPS25751_OK) {
        return status;
    }
    dev->register_address = reg;
    dev->requested_length = length;
    memcpy(dev->result, data, length);
    return TPS25751_OK;
}

TPS25751_Status_t TPS25751_StartCommand(TPS25751_Device_t *dev,
                                        const char command[4],
                                        const uint8_t *input,
                                        uint8_t input_length,
                                        uint8_t output_length)
{
    TPS25751_Status_t status;

    if ((command == NULL) || (input_length > sizeof(dev->command_input)) ||
        (output_length > TPS25751_MAX_PAYLOAD) ||
        ((input_length > 0U) && (input == NULL))) {
        return TPS25751_INVALID_ARG;
    }
    status = TPS25751_StartOperation(dev, TPS25751_OP_COMMAND);
    if (status != TPS25751_OK) {
        return status;
    }

    dev->command_raw = TPS25751_MakeFourCc(command);
    dev->command_input_length = input_length;
    dev->command_output_length = output_length;
    if (input_length > 0U) {
        memcpy(dev->command_input, input, input_length);
        dev->operation_state = TPS_OP_STATE_START_DATA1;
    } else {
        dev->operation_state = TPS_OP_STATE_START_COMMAND;
    }
    return TPS25751_OK;
}

TPS25751_Status_t TPS25751_StartI2cControllerRead(TPS25751_Device_t *dev,
                                                  uint8_t target_addr_7bit,
                                                  uint8_t target_register,
                                                  uint8_t length)
{
    uint8_t input[3];
    TPS25751_Status_t status;

    if ((target_addr_7bit > 0x7FU) || (length == 0U) || (length > 63U)) {
        return TPS25751_INVALID_ARG;
    }
    input[0] = target_addr_7bit;
    input[1] = target_register;
    input[2] = length;
    status = TPS25751_StartCommand(dev, "I2Cr", input, sizeof(input),
                                   (uint8_t)(length + 1U));
    if (status == TPS25751_OK) {
        dev->operation = TPS25751_OP_I2C_CONTROLLER_READ;
    }
    return status;
}

TPS25751_Status_t TPS25751_StartI2cControllerWrite(TPS25751_Device_t *dev,
                                                   uint8_t target_addr_7bit,
                                                   uint8_t target_register,
                                                   const uint8_t *data,
                                                   uint8_t length)
{
    uint8_t input[14];
    TPS25751_Status_t status;

    if ((target_addr_7bit > 0x7FU) || (data == NULL) ||
        (length == 0U) || (length > 10U)) {
        return TPS25751_INVALID_ARG;
    }

    /* TPS25751 firmware erratum: no reserved length-high byte. The I2Cc
     * transaction length includes the register-offset byte. */
    input[0] = target_addr_7bit;
    input[1] = (uint8_t)(length + 1U);
    input[2] = target_register;
    memcpy(&input[3], data, length);

    status = TPS25751_StartCommand(dev, "I2Cw", input,
                                   (uint8_t)(length + 3U), 1U);
    if (status == TPS25751_OK) {
        dev->operation = TPS25751_OP_I2C_CONTROLLER_WRITE;
    }
    return status;
}

bool TPS25751_IsBusy(const TPS25751_Device_t *dev)
{
    return (dev != NULL) && (dev->operation_status == TPS25751_BUSY);
}

TPS25751_Status_t TPS25751_GetOperationStatus(const TPS25751_Device_t *dev)
{
    return (dev == NULL) ? TPS25751_INVALID_ARG : dev->operation_status;
}

const uint8_t *TPS25751_GetResult(const TPS25751_Device_t *dev,
                                 uint8_t *length)
{
    if (length != NULL) {
        *length = (dev == NULL) ? 0U : dev->result_length;
    }
    return (dev == NULL) ? NULL : dev->result;
}

static TPS25751_Status_t TPS25751_ProcessTransferComplete(
    TPS25751_Device_t *dev,
    uint32_t now_ms)
{
    TPS25751_Status_t status;
    uint32_t command_value;

    if (HAL_I2C_GetError(dev->hi2c) != HAL_I2C_ERROR_NONE) {
        dev->hal_error = HAL_I2C_GetError(dev->hi2c);
        TPS25751_Finish(dev, TPS25751_I2C_ERROR);
        return TPS25751_I2C_ERROR;
    }

    if (dev->operation == TPS25751_OP_READ_REGISTER) {
        bool active_contract_register =
            (dev->register_address == TPS25751_REG_ACTIVE_PDO) ||
            (dev->register_address == TPS25751_REG_ACTIVE_RDO);
        status = TPS25751_CheckRead(dev, !active_contract_register);
        TPS25751_Finish(dev, status);
        return status;
    }
    if (dev->operation == TPS25751_OP_WRITE_REGISTER) {
        dev->result_length = 0U;
        TPS25751_Finish(dev, TPS25751_OK);
        return TPS25751_OK;
    }

    switch (dev->operation_state) {
        case TPS_OP_STATE_WAIT_DATA1:
            dev->operation_state = TPS_OP_STATE_START_COMMAND;
            return TPS25751_BUSY;

        case TPS_OP_STATE_WAIT_COMMAND:
            dev->next_action_ms = now_ms + TPS25751_COMMAND_POLL_MS;
            dev->operation_state = TPS_OP_STATE_POLL_DELAY;
            return TPS25751_BUSY;

        case TPS_OP_STATE_WAIT_POLL:
            status = TPS25751_CheckRead(dev, true);
            if (status != TPS25751_OK) {
                TPS25751_Finish(dev, status);
                return status;
            }
            command_value = TPS25751_ReadLe32(dev->result);
            if ((command_value == 0U) ||
                (command_value == TPS25751_MakeFourCc("!CMD"))) {
                if (dev->command_output_length == 0U) {
                    status = (command_value == 0U) ? TPS25751_OK :
                                                    TPS25751_COMMAND_ERROR;
                    TPS25751_Finish(dev, status);
                    return status;
                }
                dev->operation_state = TPS_OP_STATE_START_RESULT;
                return TPS25751_BUSY;
            }
            if ((uint32_t)(now_ms - dev->operation_started_ms) >=
                TPS25751_COMMAND_TIMEOUT_MS) {
                TPS25751_Finish(dev, TPS25751_TIMEOUT);
                return TPS25751_TIMEOUT;
            }
            dev->next_action_ms = now_ms + TPS25751_COMMAND_POLL_MS;
            dev->operation_state = TPS_OP_STATE_POLL_DELAY;
            return TPS25751_BUSY;

        case TPS_OP_STATE_WAIT_RESULT:
            status = TPS25751_CheckRead(dev, false);
            if (status != TPS25751_OK) {
                TPS25751_Finish(dev, status);
                return status;
            }
            if (dev->result_length < 1U) {
                TPS25751_Finish(dev, TPS25751_BAD_LENGTH);
                return TPS25751_BAD_LENGTH;
            }
            dev->task_return_code = dev->result[0];
            if (dev->task_return_code != 0U) {
                TPS25751_Finish(dev, TPS25751_COMMAND_ERROR);
                return TPS25751_COMMAND_ERROR;
            }
            if (dev->operation == TPS25751_OP_I2C_CONTROLLER_READ) {
                memmove(dev->result, &dev->result[1],
                        (size_t)dev->result_length - 1U);
                --dev->result_length;
            } else if (dev->operation == TPS25751_OP_I2C_CONTROLLER_WRITE) {
                dev->result_length = 0U;
            }
            TPS25751_Finish(dev, TPS25751_OK);
            return TPS25751_OK;

        default:
            TPS25751_Finish(dev, TPS25751_ERROR);
            return TPS25751_ERROR;
    }
}

TPS25751_Status_t TPS25751_Task(TPS25751_Device_t *dev, uint32_t now_ms)
{
    uint8_t command[4];

    if ((dev == NULL) || (dev->hi2c == NULL)) {
        return TPS25751_INVALID_ARG;
    }
    if (!TPS25751_IsBusy(dev)) {
        return dev->operation_status;
    }

    if (dev->transfer_active) {
        if (HAL_I2C_GetState(dev->hi2c) != HAL_I2C_STATE_READY) {
            if ((uint32_t)(now_ms - dev->transfer_started_ms) >=
                TPS25751_TRANSFER_TIMEOUT_MS) {
                dev->hal_error = HAL_I2C_GetError(dev->hi2c);
                (void)HAL_I2C_Master_Abort_IT(
                    dev->hi2c, (uint16_t)(dev->address_7bit << 1));
                TPS25751_Finish(dev, TPS25751_TIMEOUT);
                return TPS25751_TIMEOUT;
            }
            return TPS25751_BUSY;
        }

        dev->transfer_active = false;
        return TPS25751_ProcessTransferComplete(dev, now_ms);
    }

    if (dev->operation == TPS25751_OP_READ_REGISTER) {
        dev->operation_state = TPS_OP_STATE_WAIT_REGISTER;
        return TPS25751_StartRawRead(dev, dev->register_address,
                                     dev->requested_length, now_ms);
    }
    if (dev->operation == TPS25751_OP_WRITE_REGISTER) {
        dev->operation_state = TPS_OP_STATE_WAIT_REGISTER;
        return TPS25751_StartRawWrite(dev, dev->register_address,
                                      dev->result, dev->requested_length,
                                      now_ms);
    }

    switch (dev->operation_state) {
        case TPS_OP_STATE_START_DATA1:
            dev->operation_state = TPS_OP_STATE_WAIT_DATA1;
            return TPS25751_StartRawWrite(dev, TPS25751_REG_DATA1,
                                          dev->command_input,
                                          dev->command_input_length, now_ms);

        case TPS_OP_STATE_START_COMMAND:
            TPS25751_WriteLe32(command, dev->command_raw);
            dev->operation_state = TPS_OP_STATE_WAIT_COMMAND;
            return TPS25751_StartRawWrite(dev, TPS25751_REG_CMD1,
                                          command, sizeof(command), now_ms);

        case TPS_OP_STATE_POLL_DELAY:
            if (!TPS25751_TickReached(now_ms, dev->next_action_ms)) {
                return TPS25751_BUSY;
            }
            dev->operation_state = TPS_OP_STATE_WAIT_POLL;
            return TPS25751_StartRawRead(dev, TPS25751_REG_CMD1, 4U,
                                         now_ms);

        case TPS_OP_STATE_START_RESULT:
            dev->operation_state = TPS_OP_STATE_WAIT_RESULT;
            return TPS25751_StartRawRead(dev, TPS25751_REG_DATA1,
                                         dev->command_output_length, now_ms);

        default:
            TPS25751_Finish(dev, TPS25751_ERROR);
            return TPS25751_ERROR;
    }
}

TPS25751_Mode_t TPS25751_DecodeMode(const uint8_t data[4])
{
    if (data == NULL) {
        return TPS25751_MODE_UNKNOWN;
    }
    if (memcmp(data, "APP ", 4U) == 0) {
        return TPS25751_MODE_APP;
    }
    if (memcmp(data, "PTCH", 4U) == 0) {
        return TPS25751_MODE_PTCH;
    }
    if (memcmp(data, "BOOT", 4U) == 0) {
        return TPS25751_MODE_BOOT;
    }
    return TPS25751_MODE_UNKNOWN;
}

void TPS25751_DecodeStatus(TPS25751_Telemetry_t *telemetry,
                           const uint8_t data[TPS25751_STATUS_LEN])
{
    uint64_t raw;

    if ((telemetry == NULL) || (data == NULL)) {
        return;
    }
    raw = TPS25751_ReadLe40(data);
    telemetry->status_raw = raw;
    telemetry->connection_state = (uint8_t)((raw >> 1) & 0x07U);
    /* STATUS.PlugPresent is the physical attach indication.  ConnectionState
     * may briefly leave states 6/7 while a compliant PR_Swap is in progress;
     * treating that transition as a detach restarts policy in mid-swap. */
    telemetry->attached = (raw & 0x01U) != 0U;
    telemetry->role = ((raw & (1ULL << 5)) != 0U) ?
                      TPS25751_ROLE_SOURCE : TPS25751_ROLE_SINK;
    telemetry->data_role_dfp = (raw & (1ULL << 6)) != 0U;
    telemetry->vbus_state = (uint8_t)((raw >> 20) & 0x03U);

    if (!telemetry->attached) {
        telemetry->active_pdo_raw = 0U;
        telemetry->active_rdo_raw = 0U;
        memset(&telemetry->active_pdo, 0, sizeof(telemetry->active_pdo));
        memset(&telemetry->active_rdo, 0, sizeof(telemetry->active_rdo));
    }
}

void TPS25751_DecodePowerPath(TPS25751_Telemetry_t *telemetry,
                              const uint8_t data[TPS25751_POWER_PATH_LEN])
{
    uint64_t raw;

    if ((telemetry == NULL) || (data == NULL)) {
        return;
    }
    raw = TPS25751_ReadLe40(data);
    telemetry->power_path_raw = raw;
    telemetry->pp5v_state = (uint8_t)((raw >> 6) & 0x07U);
    telemetry->pphv_state = (uint8_t)((raw >> 12) & 0x07U);
    telemetry->pp5v_overcurrent = ((raw >> 28) & 1U) != 0U;
    telemetry->ppcable_overcurrent = ((raw >> 34) & 1U) != 0U;
}

void TPS25751_DecodePowerStatus(TPS25751_Telemetry_t *telemetry,
                                const uint8_t data[TPS25751_POWER_STATUS_LEN])
{
    if ((telemetry != NULL) && (data != NULL)) {
        telemetry->power_status_raw = TPS25751_ReadLe16(data);
    }
}

void TPS25751_DecodePdStatus(TPS25751_Telemetry_t *telemetry,
                             const uint8_t data[TPS25751_PD_STATUS_LEN])
{
    if ((telemetry == NULL) || (data == NULL)) {
        return;
    }
    telemetry->pd_status_raw = TPS25751_ReadLe32(data);
    telemetry->hard_reset_reason =
        (uint8_t)((telemetry->pd_status_raw >> 16) & 0x3FU);
}

void TPS25751_DecodeTypecState(
    TPS25751_Telemetry_t *telemetry,
    const uint8_t data[TPS25751_TYPE_C_STATE_LEN])
{
    uint32_t raw;

    if ((telemetry == NULL) || (data == NULL)) {
        return;
    }
    raw = TPS25751_ReadLe32(data);
    telemetry->typec_state_raw = raw;
    telemetry->pd_cc_pin = (uint8_t)(raw & 0xFFU);
    telemetry->cc1_state = (uint8_t)((raw >> 8) & 0xFFU);
    telemetry->cc2_state = (uint8_t)((raw >> 16) & 0xFFU);
    telemetry->typec_port_state = (uint8_t)((raw >> 24) & 0xFFU);
}

void TPS25751_DecodeAdcResults(TPS25751_Telemetry_t *telemetry,
                               const uint8_t data[TPS25751_ADC_RESULTS_LEN])
{
    if ((telemetry != NULL) && (data != NULL)) {
        telemetry->vbus_mv = (uint32_t)data[3] * 98U;
    }
}

TPS25751_Pdo_t TPS25751_DecodePdo(uint32_t raw)
{
    TPS25751_Pdo_t pdo;
    uint32_t supply_type;

    memset(&pdo, 0, sizeof(pdo));
    pdo.raw = raw;
    if (raw == 0U) {
        return pdo;
    }

    supply_type = (raw >> 30) & 0x03U;
    if (supply_type == 0U) {
        pdo.valid = true;
        pdo.voltage_mv = ((raw >> 10) & 0x3FFU) * 50U;
        pdo.min_voltage_mv = pdo.voltage_mv;
        pdo.max_voltage_mv = pdo.voltage_mv;
        pdo.current_ma = (raw & 0x3FFU) * 10U;
        pdo.power_mw = pdo.voltage_mv * pdo.current_ma / 1000U;
    } else if (supply_type == 1U) {
        pdo.valid = true;
        pdo.min_voltage_mv = ((raw >> 10) & 0x3FFU) * 50U;
        pdo.max_voltage_mv = ((raw >> 20) & 0x3FFU) * 50U;
        pdo.voltage_mv = pdo.max_voltage_mv;
        pdo.power_mw = (raw & 0x3FFU) * 250U;
    } else if (supply_type == 2U) {
        pdo.valid = true;
        pdo.min_voltage_mv = ((raw >> 10) & 0x3FFU) * 50U;
        pdo.max_voltage_mv = ((raw >> 20) & 0x3FFU) * 50U;
        pdo.voltage_mv = pdo.max_voltage_mv;
        pdo.current_ma = (raw & 0x3FFU) * 10U;
        pdo.power_mw = pdo.voltage_mv * pdo.current_ma / 1000U;
    } else if (supply_type == 3U) {
        uint32_t apdo_type = (raw >> 28) & 0x03U;
        if (apdo_type == 0U) {
            pdo.valid = true;
            pdo.min_voltage_mv = ((raw >> 8) & 0xFFU) * 100U;
            pdo.max_voltage_mv = ((raw >> 17) & 0xFFU) * 100U;
            pdo.voltage_mv = pdo.max_voltage_mv;
            pdo.current_ma = (raw & 0x7FU) * 50U;
            pdo.power_mw = pdo.voltage_mv * pdo.current_ma / 1000U;
        } else if (apdo_type == 2U) {
            /* Current TPS image uses SPR AVS 9..20 V.  Store the 15..20 V
             * current limit, which determines its maximum advertised power. */
            pdo.valid = true;
            pdo.min_voltage_mv = 9000U;
            pdo.max_voltage_mv = 20000U;
            pdo.voltage_mv = pdo.max_voltage_mv;
            pdo.current_ma = (raw & 0x3FFU) * 10U;
            pdo.power_mw = 20000U * pdo.current_ma / 1000U;
        }
    }
    return pdo;
}

static bool TPS25751_DecodeCapabilitiesAt(
    TPS25751_Capabilities_t *capabilities,
    const uint8_t *data,
    uint8_t length,
    uint8_t pdo_offset)
{
    uint8_t count;
    uint8_t i;

    if ((capabilities == NULL) || (data == NULL) ||
        (length < pdo_offset)) {
        return false;
    }

    memset(capabilities, 0, sizeof(*capabilities));
    count = data[0] & 0x07U;
    if ((count > 7U) ||
        (length < (uint8_t)(pdo_offset + (count * 4U)))) {
        return false;
    }
    capabilities->count = count;

    for (i = 0U; i < count; ++i) {
        uint32_t raw = TPS25751_ReadLe32(&data[pdo_offset + (i * 4U)]);
        capabilities->pdo[i] = TPS25751_DecodePdo(raw);
        if (!capabilities->pdo[i].valid) {
            /* Never expose count=N with zero/invalid PDOs to role policy. */
            memset(capabilities, 0, sizeof(*capabilities));
            return false;
        }
        if (capabilities->pdo[i].max_voltage_mv >
            capabilities->max_voltage_mv) {
            capabilities->max_voltage_mv =
                capabilities->pdo[i].max_voltage_mv;
        }
    }

    if ((count > 0U) &&
        (((capabilities->pdo[0].raw >> 30) & 0x03U) == 0U)) {
        capabilities->first_pdo_dual_role_power =
            (capabilities->pdo[0].raw & (1UL << 29)) != 0U;
    }
    return true;
}

bool TPS25751_DecodeCapabilities(TPS25751_Capabilities_t *capabilities,
                                 const uint8_t *data,
                                 uint8_t length)
{
    /* RX_SOURCE_CAPS, RX_SINK_CAPS and TX_SINK_CAPS store PDO1 directly
     * after the one-byte object-count field.  Validate only the bytes
     * required by the advertised count, not the register's maximum size. */
    return TPS25751_DecodeCapabilitiesAt(capabilities, data, length, 1U);
}

bool TPS25751_DecodeTxSourceCapabilities(
    TPS25751_Capabilities_t *capabilities,
    const uint8_t *data,
    uint8_t length)
{
    /* TX_SOURCE_CAPS has two policy bytes between the object count and PDO1. */
    return TPS25751_DecodeCapabilitiesAt(capabilities, data, length, 3U);
}

bool TPS25751_DecodeAutoNegotiateSink(
    TPS25751_AutoNegotiateSink_t *policy,
    const uint8_t *data,
    uint8_t length)
{
    if ((policy == NULL) || (data == NULL) ||
        (length < TPS25751_AUTO_NEGOTIATE_SINK_LEN)) {
        return false;
    }

    memset(policy, 0, sizeof(*policy));
    memcpy(policy->raw, data, TPS25751_AUTO_NEGOTIATE_SINK_LEN);
    policy->capability_mismatch_power_mw =
        TPS25751_GetLeBitField(data, 52U, 10U) * 250U;
    policy->min_voltage_mv =
        TPS25751_GetLeBitField(data, 42U, 10U) * 50U;
    policy->max_voltage_mv =
        TPS25751_GetLeBitField(data, 32U, 10U) * 50U;
    policy->sink_min_required_power_mw =
        TPS25751_GetLeBitField(data, 22U, 10U) * 250U;
    policy->auto_disable_sink_on_mismatch =
        TPS25751_GetLeBitField(data, 6U, 1U) != 0U;
    policy->auto_compute_max_voltage =
        TPS25751_GetLeBitField(data, 5U, 1U) != 0U;
    policy->auto_compute_min_voltage =
        TPS25751_GetLeBitField(data, 4U, 1U) != 0U;
    policy->no_capability_mismatch =
        TPS25751_GetLeBitField(data, 3U, 1U) != 0U;
    policy->auto_compute_min_power =
        TPS25751_GetLeBitField(data, 2U, 1U) != 0U;
    return true;
}

bool TPS25751_PatchAutoNegotiateSinkMinPower(
    uint8_t data[TPS25751_AUTO_NEGOTIATE_SINK_LEN],
    uint32_t min_power_mw)
{
    uint32_t encoded_power;

    if ((data == NULL) || (min_power_mw > 255750U) ||
        ((min_power_mw % 250U) != 0U)) {
        return false;
    }

    encoded_power = min_power_mw / 250U;

    /* TPS25751 AUTO_NEGOTIATE_SINK: use the explicit requested power and
     * keep "No USB PD Capability Mismatch" enabled.  All other policy
     * fields, including the voltage limits, are preserved byte-for-byte. */
    TPS25751_SetLeBitField(data, 2U, 1U, 0U);
    TPS25751_SetLeBitField(data, 3U, 1U, 1U);
    TPS25751_SetLeBitField(data, 22U, 10U, encoded_power);
    return true;
}

TPS25751_Rdo_t TPS25751_DecodeRdo(
    uint32_t raw,
    const TPS25751_Pdo_t *pdo)
{
    TPS25751_Rdo_t rdo;

    memset(&rdo, 0, sizeof(rdo));
    rdo.raw = raw;
    rdo.valid = raw != 0U;
    if (rdo.valid) {
        rdo.object_position = (uint8_t)((raw >> 28) & 0x0FU);
        rdo.capability_mismatch = ((raw >> 26) & 1U) != 0U;
        if ((pdo != NULL) &&
            (((pdo->raw >> 30) & 0x03U) == 3U)) {
            rdo.requested_voltage_mv =
                ((raw >> 9) & 0x0FFFU) * 20U;
            rdo.operating_current_ma = (raw & 0x7FU) * 50U;
            rdo.maximum_current_ma = rdo.operating_current_ma;
        } else {
            rdo.operating_current_ma =
                ((raw >> 10) & 0x3FFU) * 10U;
            rdo.maximum_current_ma = (raw & 0x3FFU) * 10U;
        }
    }
    return rdo;
}

bool TPS25751_PatchPortMode(uint8_t port_config[TPS25751_PORT_CONFIG_LEN],
                            TPS25751_PortMode_t mode)
{
    uint8_t old_value;

    if ((port_config == NULL) || ((uint8_t)mode > 3U)) {
        return false;
    }
    old_value = port_config[0];
    port_config[0] = (uint8_t)((old_value & 0xFCU) | (uint8_t)mode);
    return port_config[0] != old_value;
}

const char *TPS25751_StatusToString(TPS25751_Status_t status)
{
    switch (status) {
        case TPS25751_OK: return "OK";
        case TPS25751_BUSY: return "BUSY";
        case TPS25751_I2C_ERROR: return "I2C_ERROR";
        case TPS25751_INVALID_ARG: return "INVALID_ARG";
        case TPS25751_BAD_LENGTH: return "BAD_LENGTH";
        case TPS25751_COMMAND_ERROR: return "COMMAND_ERROR";
        case TPS25751_NOT_AVAILABLE: return "NOT_AVAILABLE";
        case TPS25751_TIMEOUT: return "TIMEOUT";
        default: return "ERROR";
    }
}

const char *TPS25751_ModeToString(TPS25751_Mode_t mode)
{
    switch (mode) {
        case TPS25751_MODE_BOOT: return "BOOT";
        case TPS25751_MODE_PTCH: return "PTCH";
        case TPS25751_MODE_APP: return "APP";
        default: return "UNKNOWN";
    }
}
