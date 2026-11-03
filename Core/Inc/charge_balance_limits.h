#ifndef CHARGE_BALANCE_LIMITS_H
#define CHARGE_BALANCE_LIMITS_H
#include <stdbool.h>
#include <stdint.h>
/* Temporary balancing profile: round voltage DOWN to the 8mV register step.
 * RSR=5mOhm: charge current uses 128mA/LSB in register bits12:6. */
#define CHARGE_BALANCE_VOLTAGE_MV 17496U
#define CHARGE_BALANCE_CURRENT_MA 512U
#define CHARGE_BALANCE_VOLTAGE_RAW 17496U
#define CHARGE_BALANCE_CURRENT_RAW 0x0100U
static inline bool ChargeBalance_CurrentTooHigh(uint16_t raw) {
 return ((raw>>6)&0x7fU)>4U;
}
static inline bool ChargeBalance_VoltageTooHigh(uint16_t raw) {
 return (uint32_t)((raw>>3)&0x1fffU)*8U>CHARGE_BALANCE_VOLTAGE_MV;
}
static inline bool ChargeBalance_ReadbackSafe(uint16_t current, uint16_t voltage) {
 return !ChargeBalance_CurrentTooHigh(current)&&!ChargeBalance_VoltageTooHigh(voltage);
}
#endif
