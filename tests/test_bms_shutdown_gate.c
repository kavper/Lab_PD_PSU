#include <assert.h>
#include <stdio.h>
#include "bms_shutdown.h"

static BmsShutdownPoll_t poll(BmsShutdownWait *w, int ok, unsigned batt)
{
    return BmsShutdown_Poll(w, ok != 0, (uint16_t)batt, 5U);
}

int main(void)
{
    BmsShutdownWait w = {0};
    int i;

    /* Logged failure: Battery Status 0x2188 while the AFE still ACKs. */
    assert(BmsShutdown_SdCmd(0x2188U));
    assert(!BmsShutdown_SdCmd(0x0188U));

    assert(BmsShutdown_AllowSubcommand(false, BMS_SUBCMD_ALL_FETS_ON));
    assert(!BmsShutdown_AllowSubcommand(true, BMS_SUBCMD_ALL_FETS_ON));
    assert(!BmsShutdown_AllowSubcommand(true, BMS_SUBCMD_ALL_FETS_OFF));
    assert(!BmsShutdown_AllowSubcommand(true, BMS_SUBCMD_FET_ENABLE));
    assert(!BmsShutdown_AllowSubcommand(true, BMS_SUBCMD_SET_CFGUPDATE));
    assert(!BmsShutdown_AllowSubcommand(true, BMS_SUBCMD_EXIT_CFGUPDATE));
    assert(!BmsShutdown_AllowSubcommand(true, BMS_SUBCMD_SLEEP_DISABLE));
    assert(BmsShutdown_AllowSubcommand(true, 0x0010U));

    /* SD_CMD pending for a long time (LD held): never gone, never aborted. */
    for (i = 0; i < 1000; i++) {
        assert(poll(&w, 1, 0x2188U) == BMS_SHUT_POLL_WAIT);
    }

    /* Real shutdown: four NACKs wait, the fifth is gone. */
    for (i = 0; i < 4; i++) {
        assert(poll(&w, 0, 0) == BMS_SHUT_POLL_WAIT);
    }
    assert(poll(&w, 0, 0) == BMS_SHUT_POLL_GONE);

    /* A single ACK between NACKs restarts the count. */
    w = (BmsShutdownWait){0};
    for (i = 0; i < 4; i++) {
        assert(poll(&w, 0, 0) == BMS_SHUT_POLL_WAIT);
    }
    assert(poll(&w, 1, 0x2188U) == BMS_SHUT_POLL_WAIT);
    assert(poll(&w, 0, 0) == BMS_SHUT_POLL_WAIT);

    /* SHUTDOWN not taken: alive with SD_CMD clear for the idle limit. */
    w = (BmsShutdownWait){0};
    for (i = 0; i < (int)BMS_SHUT_IDLE_POLL_LIMIT - 1; i++) {
        assert(poll(&w, 1, 0x0188U) == BMS_SHUT_POLL_WAIT);
    }
    assert(poll(&w, 1, 0x0188U) == BMS_SHUT_POLL_ABORTED);

    /* SD_CMD reappearing resets the idle count. */
    w = (BmsShutdownWait){0};
    for (i = 0; i < (int)BMS_SHUT_IDLE_POLL_LIMIT - 1; i++) {
        assert(poll(&w, 1, 0x0188U) == BMS_SHUT_POLL_WAIT);
    }
    assert(poll(&w, 1, 0x2188U) == BMS_SHUT_POLL_WAIT);
    assert(poll(&w, 1, 0x0188U) == BMS_SHUT_POLL_WAIT);

    puts("BMS shutdown gate: PASS");
    return 0;
}
