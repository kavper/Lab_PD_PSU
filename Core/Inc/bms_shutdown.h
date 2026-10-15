#ifndef BMS_SHUTDOWN_H
#define BMS_SHUTDOWN_H

#include <stdbool.h>
#include <stdint.h>

/* BQ76922 TRM SLUUCG7, 0x12 Battery Status().
 * Bit 13 is SD_CMD in register chapter 12 and SDM in chapter 6:
 * shutdown is pending because SHUTDOWN() was received or RST_SHUT was held. */
#define BMS_BATT_SD_CMD                  (1U << 13)

#define BMS_SUBCMD_FET_ENABLE            0x0022U
#define BMS_SUBCMD_SET_CFGUPDATE         0x0090U
#define BMS_SUBCMD_EXIT_CFGUPDATE        0x0092U
#define BMS_SUBCMD_ALL_FETS_OFF          0x0095U
#define BMS_SUBCMD_ALL_FETS_ON           0x0096U
#define BMS_SUBCMD_SLEEP_DISABLE         0x009AU

/* Status polls with SD_CMD=0 and a live bus before the shutdown counts as
 * not accepted. At 100 ms per poll this is 3 s, well past the 4 s-window
 * second write that bypasses the shutdown delays. */
#define BMS_SHUT_IDLE_POLL_LIMIT         30U

static inline bool BmsShutdown_SdCmd(uint16_t battery_status)
{
    return (battery_status & BMS_BATT_SD_CMD) != 0U;
}

/* While SHUTDOWN is pending the AFE still ACKs. Those ACKs are not a TS2 wake,
 * so nothing that reconfigures or switches the FETs may be sent. */
static inline bool BmsShutdown_AllowSubcommand(bool shutdown_wait, uint16_t subcmd)
{
    if (!shutdown_wait) {
        return true;
    }
    switch (subcmd) {
    case BMS_SUBCMD_FET_ENABLE:
    case BMS_SUBCMD_SET_CFGUPDATE:
    case BMS_SUBCMD_EXIT_CFGUPDATE:
    case BMS_SUBCMD_ALL_FETS_OFF:
    case BMS_SUBCMD_ALL_FETS_ON:
    case BMS_SUBCMD_SLEEP_DISABLE:
        return false;
    default:
        return true;
    }
}

typedef enum {
    BMS_SHUT_POLL_WAIT = 0,
    BMS_SHUT_POLL_GONE,
    BMS_SHUT_POLL_ABORTED
} BmsShutdownPoll_t;

typedef struct {
    uint8_t fail_streak;
    uint8_t idle_polls;
} BmsShutdownWait;

/* One status poll. GONE: the AFE stopped answering (real SHUTDOWN).
 * ABORTED: it keeps answering with SD_CMD clear, so SHUTDOWN was not taken.
 * Any SD_CMD=1 read keeps waiting, however long LD holds the part up. */
static inline BmsShutdownPoll_t BmsShutdown_Poll(BmsShutdownWait *w,
                                                bool i2c_ok,
                                                uint16_t battery_status,
                                                uint8_t fail_limit)
{
    if (!i2c_ok) {
        w->idle_polls = 0U;
        if (w->fail_streak < 255U) {
            w->fail_streak++;
        }
        return (w->fail_streak >= fail_limit) ? BMS_SHUT_POLL_GONE
                                               : BMS_SHUT_POLL_WAIT;
    }
    w->fail_streak = 0U;
    if (BmsShutdown_SdCmd(battery_status)) {
        w->idle_polls = 0U;
        return BMS_SHUT_POLL_WAIT;
    }
    if (w->idle_polls < 255U) {
        w->idle_polls++;
    }
    return (w->idle_polls >= BMS_SHUT_IDLE_POLL_LIMIT) ? BMS_SHUT_POLL_ABORTED
                                                       : BMS_SHUT_POLL_WAIT;
}

#endif
