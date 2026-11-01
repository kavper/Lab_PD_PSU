#ifndef BMS_STORE_H
#define BMS_STORE_H

#include "bms_gauge.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * Gauge persistence in the last 8 KB of the G4 flash (two 4 KB pages,
 * 0x0803E000 / 0x0803F000; the linker region ends at 248 KB).
 *
 * Why here and not in the TPS25751 EEPROM: the TPS only reaches its EEPROM
 * through 4CC 'I2Cr'/'I2Cw' — 8-bit register offset (the 24C512 needs 16),
 * ≤10-byte writes that are only queued, and one 'I2Cr' per 5 s.  That is
 * not a storage path.  The G4 flash is fine as long as no erase/program
 * happens while the converter runs (CPU fetch stalls for ~20 ms on erase):
 * writes are therefore only allowed with the PSU output off — on BMS
 * SHUTDOWN and opportunistically while idle.
 *
 * Record: 48 bytes (6 double-words), magic + sequence + payload + CRC-32.
 * 85 records per page, ping-pong between the pages → one page erase per
 * 85 saves; 10 k erase cycles ≈ 1.7 M saves.  A power cut during a write
 * leaves an invalid (CRC / ECC) record that is skipped on the next scan.
 */

#define BMS_STORE_BASE          0x0803E000UL
#define BMS_STORE_PAGE_BYTES    0x1000UL
#define BMS_STORE_PAGES         2U
#define BMS_STORE_RECORD_BYTES  48U
#define BMS_STORE_MAGIC         0x31474D42UL  /* "BMG1" */
#define BMS_STORE_VERSION       1U

typedef struct {
    uint32_t seq;
    bool clean;          /* written by an orderly SHUTDOWN */
    BmsGauge_Persist_t gauge;
} BmsStore_Record_t;

/* ---- pure codec (host-tested) ---- */
void BmsStore_Encode(const BmsStore_Record_t *rec, uint8_t out[BMS_STORE_RECORD_BYTES]);
bool BmsStore_Decode(const uint8_t in[BMS_STORE_RECORD_BYTES], BmsStore_Record_t *rec);
bool BmsStore_IsErased(const uint8_t in[BMS_STORE_RECORD_BYTES]);
uint32_t BmsStore_Crc32(const uint8_t *data, uint32_t len);

/* ---- flash backend (target only) ---- */
/* Scan both pages; true + newest record when one is valid. */
bool BmsStore_Load(BmsStore_Record_t *out);
/* Append a record. Caller guarantees the output is off. */
bool BmsStore_Save(const BmsGauge_Persist_t *g, bool clean);
uint32_t BmsStore_LastSeq(void);
uint32_t BmsStore_SaveCount(void);
/* Called from NMI_Handler: true if the NMI was an ECC error in the store
 * region (flag cleared, record will read as invalid). */
bool BmsStore_HandleEccNmi(void);

#endif /* BMS_STORE_H */
