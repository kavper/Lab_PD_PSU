#ifndef TPS25751_H
#define TPS25751_H

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

#define TPS25751_I2C_ADDR_DEFAULT              0x21U

#define TPS25751_MAX_REG_PAYLOAD              64U

#define TPS25751_ADC_RESULTS_LEN              12U

#define TPS25751_RX_SOURCE_CAPS_MAX_LEN       29U
#define TPS25751_RX_SINK_CAPS_MAX_LEN         29U
#define TPS25751_TX_SOURCE_CAPS_MAX_LEN       31U
#define TPS25751_TX_SINK_CAPS_MAX_LEN         29U

typedef enum {
    TPS25751_OK = 0,
    TPS25751_ERROR,
    TPS25751_I2C_ERROR,
    TPS25751_INVALID_ARG,
    TPS25751_BAD_LENGTH,
    TPS25751_COMMAND_ERROR
} TPS25751_Status_t;

typedef enum {
    TPS25751_MODE_UNKNOWN = 0,
    TPS25751_MODE_BOOT,
    TPS25751_MODE_PTCH,
    TPS25751_MODE_APP
} TPS25751_Mode_t;

typedef enum {
    TPS25751_SUPPLY_UNKNOWN = 0,
    TPS25751_SUPPLY_FIXED,
    TPS25751_SUPPLY_BATTERY,
    TPS25751_SUPPLY_VARIABLE,
    TPS25751_SUPPLY_APDO_PPS,
    TPS25751_SUPPLY_APDO_OTHER
} TPS25751_PdoSupply_t;

typedef enum {
    TPS25751_TYPEC_SM_DRP = 0,
    TPS25751_TYPEC_SM_SINK_ONLY = 1,
    TPS25751_TYPEC_SM_SOURCE_ONLY = 2,
    TPS25751_TYPEC_SM_DISABLED = 3
} TPS25751_TypecStateMachine_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address_7bit;

    uint8_t last_register;
    uint8_t last_requested_payload_len;
    uint8_t last_reported_payload_len;
    uint32_t last_error;

    char last_i2c_controller_command[5];
    uint8_t last_i2c_controller_target_addr;
    uint8_t last_i2c_controller_target_reg;
    uint8_t last_i2c_controller_length;
    uint8_t last_i2c_controller_task_return_code;
    uint8_t last_i2c_controller_data1_len;
    TPS25751_Status_t last_i2c_controller_error;
} TPS25751_Device_t;

typedef struct {
    uint32_t raw;

    TPS25751_PdoSupply_t type;

    uint32_t voltage_mv;
    uint32_t min_mv;
    uint32_t max_mv;
    uint32_t current_ma;
    uint32_t power_mw;
} TPS25751_PdoInfo_t;

typedef struct {
    uint8_t count;
    TPS25751_PdoInfo_t pdo[7];
} TPS25751_PdoList_t;

typedef struct {
    uint32_t raw;
    bool valid;

    uint8_t object_position;
    bool capability_mismatch;
    bool usb_comm_capable;
    bool no_usb_suspend;
    bool unchunked_supported;

    uint32_t operating_current_ma;
    uint32_t max_current_ma;
} TPS25751_RdoInfo_t;

typedef struct {
    TPS25751_Status_t status;

    char mode_ascii[5];
    TPS25751_Mode_t mode;
    bool app_ready;

    uint64_t status_raw;
    uint64_t power_path_raw;
    uint32_t power_status_raw;
    uint32_t pd_status_raw;
    uint32_t typec_raw;

    uint64_t active_pdo_raw;
    uint32_t active_rdo_raw;

    bool plug_present;
    uint8_t connection_state;
    bool orientation_cc2;
    bool port_role_source;
    bool data_role_dfp;
    uint8_t vbus_status;
    uint8_t usb_host_present;
    uint8_t legacy_status;

    uint8_t ppcable_switch;
    uint8_t pp1_switch;
    uint8_t pp3_switch;
    bool pp1_overcurrent;
    bool ppcable_overcurrent;
    uint8_t power_source;

    bool power_connection;
    bool power_status_source;
    uint8_t typec_current_status;

    uint8_t cc_pullup;
    uint8_t pd_port_type;
    bool pd_role_source;
    uint8_t soft_reset_reason;
    uint8_t hard_reset_reason;

    uint8_t cc_pin_for_pd;
    uint8_t cc1_state;
    uint8_t cc2_state;
    uint8_t typec_state;

    uint32_t adcin1_mv;
    uint32_t adcin2_mv;
    uint32_t ldo3v3_mv;
    uint32_t vbus_mv;
    uint32_t ibus_ma;
    uint32_t ibus_mean_ma;

    TPS25751_PdoList_t rx_source_caps;
    TPS25751_PdoList_t rx_sink_caps;
    TPS25751_PdoList_t tx_source_caps;
    TPS25751_PdoList_t tx_sink_caps;

    TPS25751_PdoInfo_t active_pdo;
    TPS25751_RdoInfo_t active_rdo;
} TPS25751_Telemetry_t;

TPS25751_Status_t TPS25751_Init(TPS25751_Device_t *dev,
                                I2C_HandleTypeDef *hi2c,
                                uint8_t address_7bit);

TPS25751_Status_t TPS25751_ReadPayload(TPS25751_Device_t *dev,
                                       uint8_t reg,
                                       uint8_t *payload,
                                       uint8_t payload_capacity,
                                       uint8_t *payload_len);

TPS25751_Status_t TPS25751_ReadTelemetry(TPS25751_Device_t *dev,
                                         TPS25751_Telemetry_t *t);

TPS25751_Status_t TPS25751_I2cControllerRead(TPS25751_Device_t *dev,
                                             uint8_t target_addr_7bit,
                                             uint8_t target_register,
                                             uint8_t *data,
                                             uint8_t length);

TPS25751_Status_t TPS25751_I2cControllerWrite(TPS25751_Device_t *dev,
                                              uint8_t target_addr_7bit,
                                              uint8_t target_register,
                                              const uint8_t *data,
                                              uint8_t length);

TPS25751_Status_t TPS25751_GetTypecStateMachine(TPS25751_Device_t *dev,
                                                TPS25751_TypecStateMachine_t *mode);

TPS25751_Status_t TPS25751_SetTypecStateMachine(TPS25751_Device_t *dev,
                                                TPS25751_TypecStateMachine_t mode);

const char *TPS25751_StatusToString(TPS25751_Status_t status);
const char *TPS25751_ModeToString(TPS25751_Mode_t mode);
const char *TPS25751_PdoTypeToString(TPS25751_PdoSupply_t type);
const char *TPS25751_ConnectionStateToString(uint8_t state);
const char *TPS25751_VbusStatusToString(uint8_t state);
const char *TPS25751_TypecStateToString(uint8_t state);
const char *TPS25751_CcStateToString(uint8_t state);
const char *TPS25751_PowerPathSwitchToString(uint8_t state);
const char *TPS25751_TypecCurrentToString(uint8_t value);

#endif
