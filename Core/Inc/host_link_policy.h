#ifndef HOST_LINK_POLICY_H
#define HOST_LINK_POLICY_H

#include <ctype.h>
#include <stdbool.h>
#include <string.h>

/*
 * USART1 (H7 / PC) stays ASCII T/TB/TC. G0 TLM/ACK on USART2 is control
 * traffic — do not mirror it to the host unless VERBOSE is on.
 * NACK is rare and user-visible, so it still reaches the console.
 */

static inline bool HostLink_IsG0TlmLine(const char *line)
{
    return (line != NULL) &&
           (strncmp(line, "TLM", 3) == 0) &&
           ((line[3] == '\0') || (line[3] == ' '));
}

static inline bool HostLink_IsG0AckLine(const char *line)
{
    return (line != NULL) &&
           (strncmp(line, "ACK", 3) == 0) &&
           ((line[3] == '\0') || (line[3] == ' '));
}

static inline bool HostLink_IsG0NackLine(const char *line)
{
    return (line != NULL) &&
           (strncmp(line, "NACK", 4) == 0) &&
           ((line[4] == '\0') || (line[4] == ' '));
}

static inline bool HostLink_ShouldForwardG0Line(const char *line, bool verbose)
{
    if ((line == NULL) || (line[0] == '\0')) {
        return false;
    }
    if (verbose) {
        return true;
    }
    if (HostLink_IsG0TlmLine(line) || HostLink_IsG0AckLine(line)) {
        return false;
    }
    return HostLink_IsG0NackLine(line);
}

/* Ignore UART noise / shredded fragments; do not answer ERR CMD. */
static inline bool HostLink_LooksLikeHostCommand(const char *line)
{
    unsigned char c;

    if ((line == NULL) || (line[0] == '\0')) {
        return false;
    }
    c = (unsigned char)line[0];
    return (c == (unsigned char)'?') || (isalpha(c) != 0);
}

/* At 115200 8N1, full T+TB+TC (~1.3 kB) cannot run faster than ~150 ms.
 * Fast TEL keeps T at the requested period and spaces BMS/charger frames. */
static inline uint32_t HostLink_BmsPeriodMs(uint32_t tel_ms)
{
    const uint32_t bms_min_ms = 200U;

    if (tel_ms == 0U) {
        return 0U;
    }
    return (tel_ms >= bms_min_ms) ? tel_ms : bms_min_ms;
}

#endif /* HOST_LINK_POLICY_H */
