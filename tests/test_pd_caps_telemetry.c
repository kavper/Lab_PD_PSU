#include "pd_caps_telemetry.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
 PdCapsTelemetry s={0}; uint8_t p[44]={1};
 assert(PdCapsTelemetry_Due(&s,p,0));
 PdCapsTelemetry_Sent(&s,p,0);
 assert(!PdCapsTelemetry_Due(&s,p,100000)); /* quiet while detached */
 p[1]=3;p[2]=1;p[3]=1;p[4]=1;p[16]=42;
 assert(PdCapsTelemetry_Due(&s,p,100001));
 PdCapsTelemetry_Sent(&s,p,100001);
 assert(!PdCapsTelemetry_Due(&s,p,100201));
 assert(PdCapsTelemetry_Due(&s,p,101001)); /* H7 freshness/restart recovery */
 p[16]=43;assert(PdCapsTelemetry_Due(&s,p,100202));
 /* A failed UART submission must remain due until successfully committed. */
 assert(PdCapsTelemetry_Due(&s,p,100203));
 PdCapsTelemetry_Sent(&s,p,100203);
 p[1]=p[2]=p[3]=p[16]=0;
 assert(PdCapsTelemetry_Due(&s,p,100204));
 PdCapsTelemetry_Sent(&s,p,100204);
 assert(!PdCapsTelemetry_Due(&s,p,200000));
 p[1]=3;PdCapsTelemetry_Sent(&s,p,0xffffff00U);
 assert(!PdCapsTelemetry_Due(&s,p,0x100U));
 assert(PdCapsTelemetry_Due(&s,p,0x300U));
 puts("PASS caps detach invalidation, quiet detached, update/retry, attached freshness and tick wrap");
}
