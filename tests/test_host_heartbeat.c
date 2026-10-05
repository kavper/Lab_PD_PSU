#include "host_heartbeat.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    assert(!HostHeartbeat_Expired(false, 5000, 0));
    assert(!HostHeartbeat_Expired(true, 1000, 0));
    assert(HostHeartbeat_Expired(true, 1001, 0));
    assert(!HostHeartbeat_Expired(true, 1500, 1400));
    assert(!HostHeartbeat_Expired(true, 20, UINT32_MAX-50));
    assert(HostHeartbeat_Expired(true, 1100, UINT32_MAX-50));
    puts("PASS: H7 heartbeat deadline, OFF and tick wrap");
    return 0;
}
