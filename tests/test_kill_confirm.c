#include "ldo_ctrl_policy.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
 LdoKillConfirm s={0};
 assert(!Ldo_KillConfirmed(&s,true,100));
 assert(!Ldo_KillConfirmed(&s,true,149));
 assert(!Ldo_KillConfirmed(&s,false,150));
 assert(!Ldo_KillConfirmed(&s,true,200));
 assert(Ldo_KillConfirmed(&s,true,250));
 assert(!Ldo_KillConfirmed(&s,false,251));
 assert(!Ldo_KillConfirmed(&s,true,UINT32_MAX-24));
 assert(!Ldo_KillConfirmed(&s,true,24));
 assert(Ldo_KillConfirmed(&s,true,25));
 puts("PASS: isolated KILL sample is not a G4 latch; sustained 50 ms and tick wrap are confirmed");
}
