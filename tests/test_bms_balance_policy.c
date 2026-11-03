#include "bms_balance_policy.h"
#include <assert.h>
#include <stdio.h>
int main(void){
 assert(BmsBalance_SafetyAllows(0,0,0,true,true));
 assert(BmsBalance_SafetyAllows(8,0,0,false,true));
 assert(!BmsBalance_SafetyAllows(8,0,0,true,true));
 assert(!BmsBalance_SafetyAllows(8,0,0,false,false));
 for (unsigned bit=0;bit<8;bit++) {
   if (bit!=3) assert(!BmsBalance_SafetyAllows(8|(1U<<bit),0,0,false,true));
   assert(!BmsBalance_SafetyAllows(8,1U<<bit,0,false,true));
   assert(!BmsBalance_SafetyAllows(8,0,1U<<bit,false,true));
 }
 BmsBalancePolicy p={0};int16_t c[5]={3450,3450,3490,3450,3500};
 assert(!BmsBalance_Select(&p,0,true,500,950,c,0));
 assert(BmsBalance_Select(&p,5000,true,500,950,c,0)==0x14);
 assert(!BmsBalance_Select(&p,5001,true,1000,950,c,0x14));
 assert(!BmsBalance_Select(&p,6000,true,999,899,c,0));
 assert(!BmsBalance_Select(&p,7000,false,0,950,c,0));
 assert(!BmsBalance_Select(&p,8000,true,-1000,950,c,0));
 p.waiting=true;p.since_ms=0; c[4]=3459;
 assert(BmsBalance_Select(&p,5000,true,0,950,c,0x10)==0x14);
 c[4]=3457;assert(BmsBalance_Select(&p,5000,true,0,950,c,0x10)==4);
 c[0]=3300;assert(!BmsBalance_Select(&p,6000,true,0,950,c,0));
 p.waiting=true;p.since_ms=0xfffffff0U;c[0]=3450;
 assert(BmsBalance_Select(&p,6000,true,0,950,c,0)==4);
 puts("PASS: balance current/SOC/freshness gates, settling, hysteresis, tick wrap");
}
