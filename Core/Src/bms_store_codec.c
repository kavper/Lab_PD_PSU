#include "bms_store.h"

#include <string.h>

/*
 * Layout (little-endian):
 *  0  u32 magic        4  u32 seq        8  u8 version   9 u8 clean
 * 10  u16 reserved    12  i32 remaining_mah_x10          16 i32 fcc_mah
 * 20  u32 dsg_total   24  u32 chg_total  28  u16 soc      30 u16 rcell
 * 32  u16 flags       34  u8 anchor      35  u8 learn_count
 * 36  u32 reserved(0) 40  u32 reserved(0) 44  u32 crc32(0..43)
 */

static void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static uint16_t get16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint32_t BmsStore_Crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t i;
    uint8_t k;

    for (i = 0U; i < len; i++) {
        crc ^= data[i];
        for (k = 0U; k < 8U; k++) {
            crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
        }
    }
    return ~crc;
}

void BmsStore_Encode(const BmsStore_Record_t *rec, uint8_t out[BMS_STORE_RECORD_BYTES])
{
    const BmsGauge_Persist_t *g = &rec->gauge;

    memset(out, 0, BMS_STORE_RECORD_BYTES);
    put32(&out[0], BMS_STORE_MAGIC);
    put32(&out[4], rec->seq);
    out[8] = BMS_STORE_VERSION;
    out[9] = rec->clean ? 1U : 0U;
    put32(&out[12], (uint32_t)g->remaining_mah_x10);
    put32(&out[16], (uint32_t)g->fcc_mah);
    put32(&out[20], g->dsg_total_mah);
    put32(&out[24], g->chg_total_mah);
    put16(&out[28], g->soc_permille);
    put16(&out[30], g->rcell_mohm_x10);
    put16(&out[32], g->flags);
    out[34] = g->last_anchor;
    out[35] = g->learn_count;
    put32(&out[44], BmsStore_Crc32(out, 44U));
}

bool BmsStore_Decode(const uint8_t in[BMS_STORE_RECORD_BYTES], BmsStore_Record_t *rec)
{
    BmsGauge_Persist_t *g = &rec->gauge;

    if (get32(&in[0]) != BMS_STORE_MAGIC) {
        return false;
    }
    if (in[8] != BMS_STORE_VERSION) {
        return false;
    }
    if (get32(&in[44]) != BmsStore_Crc32(in, 44U)) {
        return false;
    }
    memset(rec, 0, sizeof(*rec));
    rec->seq = get32(&in[4]);
    rec->clean = (in[9] != 0U);
    g->remaining_mah_x10 = (int32_t)get32(&in[12]);
    g->fcc_mah = (int32_t)get32(&in[16]);
    g->dsg_total_mah = get32(&in[20]);
    g->chg_total_mah = get32(&in[24]);
    g->soc_permille = get16(&in[28]);
    g->rcell_mohm_x10 = get16(&in[30]);
    g->flags = get16(&in[32]);
    g->last_anchor = in[34];
    g->learn_count = in[35];
    return true;
}

bool BmsStore_IsErased(const uint8_t in[BMS_STORE_RECORD_BYTES])
{
    uint32_t i;

    for (i = 0U; i < BMS_STORE_RECORD_BYTES; i++) {
        if (in[i] != 0xFFU) {
            return false;
        }
    }
    return true;
}
