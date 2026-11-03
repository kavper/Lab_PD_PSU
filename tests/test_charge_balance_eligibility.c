#include <assert.h>
#include <stdio.h>
#include "charge_balance_limits.h"
#include "bms_balance_policy.h"
int main(void){
 BmsBalancePolicy p={0};
 int16_t c[5]={3450,3540,3520,3600,3390};
 assert(CHARGE_BALANCE_VOLTAGE_MV/5>=3350);
 assert(CHARGE_BALANCE_CURRENT_MA<1000);
 assert(!BmsBalance_Select(&p,100,true,512,1000,c,0));
 assert(BmsBalance_Select(&p,5100,true,512,1000,c,0)==0x0A);
 assert(BmsBalance_SafetyAllows(8,0,0,false,true));
 assert(BmsBalance_Select(&p,5200,true,0,1000,c,0x0A)==0x0A);
 for(unsigned i=0;i<5;i++)c[i]=3499;
 assert(BmsBalance_Select(&p,5300,true,0,1000,c,0x0A)==0);
 puts("PASS: temporary17.496V/512mA profile allows existing near-full balancing and stops when equal; COV charge-off allowed");
}
