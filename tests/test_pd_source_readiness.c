#include "pd_source_readiness.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
 assert(PdSource_BqReady(100,true,5000,false,0,false,9999,0,20000));
 assert(PdSource_BqReady(100,true,19968,true,0,true,100,19894,20000));
 assert(PdSource_BqReady(100,true,9024,true,0,true,100,9016,9000));
 assert(!PdSource_BqReady(2600,true,5000,false,0,false,0,0,5000));
 assert(!PdSource_BqReady(100,false,5000,false,0,false,0,0,5000));
 assert(!PdSource_BqReady(100,true,19968,false,0,true,100,19894,20000));
 assert(!PdSource_BqReady(100,true,19968,true,0,false,100,19894,20000));
 assert(!PdSource_BqReady(100,true,19968,true,0,true,1600,19894,20000));
 assert(!PdSource_BqReady(100,true,19968,true,0,true,100,5000,20000));
 assert(!PdSource_BqReady(100,true,9024,true,0,true,100,9016,20000));
 for (unsigned bit=0;bit<8;bit++) {
  assert(!PdSource_BqReady(100,true,5000,false,1U<<bit,false,0,0,5000));
  assert(!PdSource_BqReady(100,true,19968,true,1U<<bit,true,100,19894,20000));
 }
 puts("PASS: initial5V and already-running source, fresh matching evidence, faults/invalid/stale/mismatched samples blocked");
}
