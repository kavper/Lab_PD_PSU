#include "tlm_fields.h"

#include <stdio.h>
#include <string.h>

#define TLM_GRP_BMS  TLM_GROUP_BMS
#define TLM_GRP_BATT TLM_GROUP_BATT
#define TLM_GRP_PD   TLM_GROUP_PD
#define TLM_GRP_AUX  TLM_GROUP_AUX

static const TlmFieldInfo_t s_fields[] = {
#define TLM_INFO_(sym, id, grp, name, type) { (id), TLM_GRP_##grp, (type), (name) },
    TLM_FIELDS(TLM_INFO_)
#undef TLM_INFO_
};

const TlmFieldInfo_t *Tlm_Find(uint8_t id)
{
    unsigned i;

    for (i = 0U; i < sizeof(s_fields) / sizeof(s_fields[0]); i++) {
        if (s_fields[i].id == id) {
            return &s_fields[i];
        }
    }
    return NULL;
}

const char *Tlm_GroupName(uint8_t group)
{
    switch (group) {
    case TLM_GROUP_BMS:  return "BMS";
    case TLM_GROUP_BATT: return "BATT";
    case TLM_GROUP_PD:   return "PD";
    case TLM_GROUP_AUX:  return "AUX";
    default:             return "?";
    }
}

void Tlm_Begin(TlmBuilder_t *b, uint8_t group)
{
    b->group = group;
    b->count = 0U;
}

int Tlm_Put(TlmBuilder_t *b, uint8_t id, int32_t value)
{
    const TlmFieldInfo_t *f = Tlm_Find(id);

    if ((f == NULL) || (f->group != b->group) || (b->count >= TLM_MAX_ENTRIES)) {
        return 0;
    }
    b->id[b->count] = id;
    b->value[b->count] = value;
    b->count++;
    return 1;
}

static uint8_t entry_len(uint8_t id)
{
    const TlmFieldInfo_t *f = Tlm_Find(id);

    return (uint8_t)(2U + TLM_TYPE_LEN(f->type));
}

/* Greedy split; starts[] gets the first entry index of each part. */
static uint8_t split(const TlmBuilder_t *b, uint8_t starts[TLM_MAX_ENTRIES + 1U])
{
    uint8_t parts = 0U;
    unsigned used = TLM_MAX_PAYLOAD; /* force a new part on entry 0 */
    uint8_t i;

    for (i = 0U; i < b->count; i++) {
        uint8_t n = entry_len(b->id[i]);

        if (used + n > TLM_MAX_PAYLOAD) {
            starts[parts++] = i;
            used = TLM_HEADER_BYTES;
        }
        used += n;
    }
    if (parts == 0U) {
        starts[parts++] = 0U; /* empty snapshot still announces the group */
    }
    starts[parts] = b->count;
    return parts;
}

uint8_t Tlm_Parts(const TlmBuilder_t *b)
{
    uint8_t starts[TLM_MAX_ENTRIES + 1U];

    return split(b, starts);
}

uint8_t Tlm_EncodePart(const TlmBuilder_t *b, uint8_t part, uint8_t *out)
{
    uint8_t starts[TLM_MAX_ENTRIES + 1U];
    uint8_t parts = split(b, starts);
    uint8_t n = TLM_HEADER_BYTES;
    uint8_t i;

    if (part >= parts) {
        return 0U;
    }
    out[0] = TLM_FIELDS_VERSION;
    out[1] = b->group;
    out[2] = part;
    out[3] = parts;
    for (i = starts[part]; i < starts[part + 1U]; i++) {
        const TlmFieldInfo_t *f = Tlm_Find(b->id[i]);
        uint8_t len = TLM_TYPE_LEN(f->type);
        uint32_t v = (uint32_t)b->value[i];
        uint8_t k;

        out[n++] = f->id;
        out[n++] = f->type;
        for (k = 0U; k < len; k++) {
            out[n++] = (uint8_t)(v >> (8U * k));
        }
    }
    return n;
}

unsigned Tlm_Format(const TlmBuilder_t *b, char *out, unsigned out_len)
{
    unsigned pos = 0U;
    uint8_t i;
    int w;

    if (out_len == 0U) {
        return 0U;
    }
    w = snprintf(out, out_len, "%s", Tlm_GroupName(b->group));
    pos = (w > 0) ? (unsigned)w : 0U;
    for (i = 0U; (i < b->count) && (pos + 1U < out_len); i++) {
        const TlmFieldInfo_t *f = Tlm_Find(b->id[i]);

        if (TLM_TYPE_SIGNED(f->type)) {
            w = snprintf(out + pos, out_len - pos, " %s=%ld", f->name, (long)b->value[i]);
        } else {
            w = snprintf(out + pos, out_len - pos, " %s=%lu", f->name,
                         (unsigned long)(uint32_t)b->value[i]);
        }
        if (w < 0) {
            break;
        }
        pos += (unsigned)w;
    }
    if (pos >= out_len) {
        pos = out_len - 1U;
    }
    out[pos] = '\0';
    return pos;
}

int Tlm_Decode(const uint8_t *p, uint8_t len, uint8_t *group, uint8_t *part,
               uint8_t *parts, TlmFieldCb cb, void *ctx)
{
    uint8_t n = TLM_HEADER_BYTES;

    if ((len < TLM_HEADER_BYTES) || (p[0] != TLM_FIELDS_VERSION) ||
        (p[3] == 0U) || (p[2] >= p[3])) {
        return 0;
    }
    *group = p[1];
    *part = p[2];
    *parts = p[3];
    while (n < len) {
        uint8_t id;
        uint8_t type;
        uint8_t l;
        uint32_t v = 0U;
        uint8_t k;

        if ((uint8_t)(len - n) < 2U) {
            return 0;
        }
        id = p[n++];
        type = p[n++];
        l = TLM_TYPE_LEN(type);
        if (((l != 1U) && (l != 2U) && (l != 4U)) || ((uint8_t)(len - n) < l)) {
            return 0;
        }
        for (k = 0U; k < l; k++) {
            v |= (uint32_t)p[n++] << (8U * k);
        }
        if (TLM_TYPE_SIGNED(type) && (l < 4U)) {
            uint32_t sign = 1UL << (8U * l - 1U);

            if ((v & sign) != 0U) {
                v |= ~((sign << 1) - 1U);
            }
        }
        if (cb != NULL) {
            cb(ctx, id, (int32_t)v);
        }
    }
    return 1;
}
