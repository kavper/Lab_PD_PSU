#ifndef TPS25751_H
#define TPS25751_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define TPS25751_I2C_ADDR_DEFAULT       0x21U
#define TPS25751_MAX_PAYLOAD            64U

#define TPS25751_REG_MODE               0x03U
#define TPS25751_REG_CMD1               0x08U
#define TPS25751_REG_DATA1              0x09U
#define TPS25751_REG_INT_EVENT          0x14U
#define TPS25751_REG_INT_MASK           0x16U
#define TPS25751_REG_INT_CLEAR          0x18U
#define TPS25751_REG_STATUS             0x1AU
#define TPS25751_REG_POWER_PATH_STATUS  0x26U
#define TPS25751_REG_PORT_CONFIG        0x28U
#define TPS25751_REG_BOOT_FLAGS         0x2DU
#define TPS25751_REG_RX_SOURCE_CAPS     0x30U
#define TPS25751_REG_RX_SINK_CAPS       0x31U
#define TPS25751_REG_TX_SINK_CAPS       0x33U
#define TPS25751_REG_ACTIVE_PDO         0x34U
#define TPS25751_REG_ACTIVE_RDO         0x35U
#define TPS25751_REG_AUTO_NEGOTIATE_SINK 0x37U
#define TPS25751_REG_POWER_STATUS       0x3FU
#define TPS25751_REG_PD_STATUS          0x40U
#define TPS25751_REG_ADC_RESULTS        0x6AU

#define TPS25751_MODE_LEN               4U
#define TPS25751_STATUS_LEN             5U
#define TPS25751_POWER_PATH_LEN         5U
/* TPS25751 returns 17 payload bytes for PORT_CONFIG. Byte 16 is a reserved
 * implementation byte omitted from TRM table 3-14; preserve it during the
 * read-modify-write used to change TypeCStateMachine. */
#define TPS25751_PORT_CONFIG_LEN        17U
#define TPS25751_BOOT_FLAGS_LEN          5U
#define TPS25751_RX_CAPS_LEN            53U
#define TPS25751_TX_SINK_CAPS_LEN       53U
#define TPS25751_AUTO_NEGOTIATE_SINK_LEN 24U
#define TPS25751_ACTIVE_PDO_PREFIX_LEN  4U
#define TPS25751_ACTIVE_RDO_PREFIX_LEN  4U
#define TPS25751_POWER_STATUS_LEN       2U
#define TPS25751_PD_STATUS_LEN          4U
#define TPS25751_ADC_RESULTS_LEN        13U

typedef enum {
    TPS25751_OK = 0,
    TPS25751_BUSY,
    TPS25751_ERROR,
    TPS25751_I2C_ERROR,
    TPS25751_INVALID_ARG,
    TPS25751_BAD_LENGTH,
    TPS25751_COMMAND_ERROR,
    TPS25751_NOT_AVAILABLE,
    TPS25751_TIMEOUT
} TPS25751_Status_t;

typedef enum {
    TPS25751_MODE_UNKNOWN = 0,
    TPS25751_MODE_BOOT,
    TPS25751_MODE_PTCH,
    TPS25751_MODE_APP
} TPS25751_Mode_t;

/* PORT_CONFIG.TypeCStateMachine encoding from TPS25751 TRM table 3-12. */
typedef enum {
    TPS25751_PORT_SINK_ONLY = 0,
    TPS25751_PORT_SOURCE_ONLY = 1,
    TPS25751_PORT_DRP = 2,
    TPS25751_PORT_DISABLED = 3
} TPS25751_PortMode_t;

typedef enum {
    TPS25751_ROLE_UNKNOWN = 0,
    TPS25751_ROLE_SINK,
    TPS25751_ROLE_SOURCE
} TPS25751_PowerRole_t;

typedef struct {
    uint32_t raw;
    bool valid;
    uint32_t voltage_mv;
    uint32_t min_voltage_mv;
    uint32_t max_voltage_mv;
    uint32_t current_ma;
    uint32_t power_mw;
} TPS25751_Pdo_t;

typedef struct {
    uint8_t count;
    TPS25751_Pdo_t pdo[7];
    uint32_t max_voltage_mv;
    bool first_pdo_dual_role_power;
} TPS25751_Capabilities_t;

typedef struct {
    uint32_t raw;
    bool valid;
    uint8_t object_position;
    bool capability_mismatch;
    uint32_t requested_voltage_mv;
    uint32_t operating_current_ma;
    uint32_t maximum_current_ma;
} TPS25751_Rdo_t;

typedef struct {
    uint8_t raw[TPS25751_AUTO_NEGOTIATE_SINK_LEN];
    uint32_t min_voltage_mv;
    uint32_t max_voltage_mv;
    uint32_t sink_min_required_power_mw;
    uint32_t capability_mismatch_power_mw;
    bool auto_compute_min_voltage;
    bool auto_compute_max_voltage;
    bool auto_compute_min_power;
    bool no_capability_mismatch;
    bool auto_disable_sink_on_mismatch;
} TPS25751_AutoNegotiateSink_t;

typedef struct {
    TPS25751_Mode_t mode;
    char mode_ascii[5];

    bool attached;
    TPS25751_PowerRole_t role;
    bool data_role_dfp;
    uint8_t connection_state;
    uint8_t vbus_state;

    uint64_t status_raw;
    uint64_t boot_flags_raw;
    uint64_t power_path_raw;
    uint32_t power_status_raw;
    uint32_t pd_status_raw;

    uint8_t pp5v_state;
    uint8_t pphv_state;
    bool pp5v_overcurrent;
    bool ppcable_overcurrent;
    uint8_t hard_reset_reason;

    uint32_t vbus_mv;
    uint32_t active_pdo_raw;
    uint32_t active_rdo_raw;
    TPS25751_Pdo_t active_pdo;
    TPS25751_Rdo_t active_rdo;

    uint32_t updated_ms;
} TPS25751_Telemetry_t;

typedef enum {
    TPS25751_OP_NONE = 0,
    TPS25751_OP_READ_REGISTER,
    TPS25751_OP_WRITE_REGISTER,
    TPS25751_OP_COMMAND,
    TPS25751_OP_I2C_CONTROLLER_READ,
    TPS25751_OP_I2C_CONTROLLER_WRITE
} TPS25751_Operation_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address_7bit;

    TPS25751_Operation_t operation;
    TPS25751_Status_t operation_status;
    uint8_t operation_state;
    bool transfer_active;
    bool transfer_is_read;
    uint8_t register_address;
    uint8_t requested_length;
    uint8_t reported_length;
    uint8_t result_length;
    uint8_t task_return_code;
    uint32_t operation_started_ms;
    uint32_t transfer_started_ms;
    uint32_t next_action_ms;
    uint32_t command_raw;
    uint32_t hal_error;

    uint8_t tx_buffer[TPS25751_MAX_PAYLOAD + 1U];
    uint8_t rx_buffer[TPS25751_MAX_PAYLOAD + 1U];
    uint8_t result[TPS25751_MAX_PAYLOAD];
    uint8_t command_input[14U];
    uint8_t command_input_length;
    uint8_t command_output_length;
} TPS25751_Device_t;

TPS25751_Status_t TPS25751_Init(TPS25751_Device_t *dev,
                                I2C_HandleTypeDef *hi2c,
                                uint8_t address_7bit);

TPS25751_Status_t TPS25751_StartReadRegister(TPS25751_Device_t *dev,
                                             uint8_t reg,
                                             uint8_t length);
TPS25751_Status_t TPS25751_StartWriteRegister(TPS25751_Device_t *dev,
                                              uint8_t reg,
                                              const uint8_t *data,
                                              uint8_t length);
TPS25751_Status_t TPS25751_StartCommand(TPS25751_Device_t *dev,
                                        const char command[4],
                                        const uint8_t *input,
                                        uint8_t input_length,
                                        uint8_t output_length);
TPS25751_Status_t TPS25751_StartI2cControllerRead(TPS25751_Device_t *dev,
                                                  uint8_t target_addr_7bit,
                                                  uint8_t target_register,
                                                  uint8_t length);
TPS25751_Status_t TPS25751_StartI2cControllerWrite(TPS25751_Device_t *dev,
                                                   uint8_t target_addr_7bit,
                                                   uint8_t target_register,
                                                   const uint8_t *data,
                                                   uint8_t length);

/* Advances the current operation. It starts at most one physical I2C transfer. */
TPS25751_Status_t TPS25751_Task(TPS25751_Device_t *dev, uint32_t now_ms);
bool TPS25751_IsBusy(const TPS25751_Device_t *dev);
TPS25751_Status_t TPS25751_GetOperationStatus(const TPS25751_Device_t *dev);
const uint8_t *TPS25751_GetResult(const TPS25751_Device_t *dev,
                                 uint8_t *length);

TPS25751_Mode_t TPS25751_DecodeMode(const uint8_t data[4]);
void TPS25751_DecodeStatus(TPS25751_Telemetry_t *telemetry,
                           const uint8_t data[TPS25751_STATUS_LEN]);
void TPS25751_DecodePowerPath(TPS25751_Telemetry_t *telemetry,
                              const uint8_t data[TPS25751_POWER_PATH_LEN]);
void TPS25751_DecodePowerStatus(TPS25751_Telemetry_t *telemetry,
                                const uint8_t data[TPS25751_POWER_STATUS_LEN]);
void TPS25751_DecodePdStatus(TPS25751_Telemetry_t *telemetry,
                             const uint8_t data[TPS25751_PD_STATUS_LEN]);
void TPS25751_DecodeAdcResults(TPS25751_Telemetry_t *telemetry,
                               const uint8_t data[TPS25751_ADC_RESULTS_LEN]);
TPS25751_Pdo_t TPS25751_DecodePdo(uint32_t raw);
TPS25751_Rdo_t TPS25751_DecodeRdo(
    uint32_t raw,
    const TPS25751_Pdo_t *pdo);
bool TPS25751_DecodeCapabilities(TPS25751_Capabilities_t *capabilities,
                                 const uint8_t *data,
                                 uint8_t length);
bool TPS25751_DecodeAutoNegotiateSink(
    TPS25751_AutoNegotiateSink_t *policy,
    const uint8_t *data,
    uint8_t length);
bool TPS25751_PatchAutoNegotiateSinkMinPower(
    uint8_t data[TPS25751_AUTO_NEGOTIATE_SINK_LEN],
    uint32_t min_power_mw);
bool TPS25751_PatchPortMode(uint8_t port_config[TPS25751_PORT_CONFIG_LEN],
                            TPS25751_PortMode_t mode);

uint16_t TPS25751_ReadLe16(const uint8_t *data);
uint32_t TPS25751_ReadLe32(const uint8_t *data);

const char *TPS25751_StatusToString(TPS25751_Status_t status);
const char *TPS25751_ModeToString(TPS25751_Mode_t mode);

#endif
