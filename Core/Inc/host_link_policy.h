#ifndef HOST_LINK_POLICY_H
#define HOST_LINK_POLICY_H

#include <ctype.h>
#include <stdbool.h>
#include <string.h>

/*
 * USART1 is the binary H7 link. G0 frames stay on USART2.
 * VERBOSE may forward a G0 text line inside a TEXT frame.
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

/* Production METER is binary and fixed. H7 does not choose the period.
 * BMS/PD stay on their own 200 ms timer as separate frames. */
static inline uint32_t HostLink_MeterPeriodMs(void)
{
    return 5U;
}

static inline uint32_t HostLink_BmsPeriodMs(void)
{
    return 200U;
}

#endif /* HOST_LINK_POLICY_H */
