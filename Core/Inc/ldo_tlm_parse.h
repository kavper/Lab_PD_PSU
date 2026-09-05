#ifndef LDO_TLM_PARSE_H
#define LDO_TLM_PARSE_H

#include <string.h>
#include <stdbool.h>

/* G0 TLM must start with "TLM out=" so "vout=" cannot be parsed as out=. */
static inline bool Ldo_TlmLooksComplete(const char *line)
{
    if (line == NULL) {
        return false;
    }

    return (strncmp(line, "TLM out=", 8) == 0) &&
           (strstr(line, "vset=") != NULL) &&
           (strstr(line, "vout=") != NULL) &&
           (strstr(line, "pgood=") != NULL) &&
           (strstr(line, "kill=") != NULL) &&
           (strstr(line, "fault=") != NULL);
}

#endif /* LDO_TLM_PARSE_H */
