#include "charge_balance_limits.h"
#include <assert.h>
#include <stdio.h>
int main(void){
 assert(CHARGE_BALANCE_LIMITS_ENABLED==0U);
 assert(CHARGE_BALANCE_VOLTAGE_RAW%8==0);
 assert(CHARGE_BALANCE_VOLTAGE_MV<=17500U);
 assert(CHARGE_BALANCE_CURRENT_RAW==(512U/128U)*64U);
 assert(ChargeBalance_CurrentTooHigh(0x0fc0));
 assert(!ChargeBalance_CurrentTooHigh(0x0100));
 assert(ChargeBalance_CurrentTooHigh(0x0140));
 assert(!ChargeBalance_CurrentTooHigh(0));
 assert(ChargeBalance_VoltageTooHigh(18000));
 assert(ChargeBalance_VoltageTooHigh(17504));
 assert(!ChargeBalance_VoltageTooHigh(17496));
 assert(!ChargeBalance_VoltageTooHigh(16504));
 assert(ChargeBalance_ReadbackSafe(0x100,17496));
 assert(!ChargeBalance_ReadbackSafe(0xfc0,17496));
 assert(!ChargeBalance_ReadbackSafe(0x100,18000));
 puts("PASS: production charge clamp disabled; optional bench caps, 5mOhm current units, voltage rounded down, lower limits preserved");
}
