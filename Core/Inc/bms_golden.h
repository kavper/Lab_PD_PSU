#ifndef BMS_GOLDEN_H
#define BMS_GOLDEN_H

#include <stdbool.h>
#include <stdint.h>

/*
 * BQ76922 Data Memory "golden" image for the 5S LiFePO4 pack.
 *
 * ONE table drives three things, so what is tested in RAM is exactly what
 * gets burned:
 *   1. CONFIG_UPDATE on every wake when the chip is blank or carries a
 *      different OTP image (the old 4S Li-ion chip),
 *   2. BMS OTP CHECK / BURN (service build only),
 *   3. the post-burn readback after a device RESET.
 *
 * Entries flagged FINGERPRINT are read back on wake; if they all match the
 * chip already runs the golden image (OTP burned) and CONFIG_UPDATE — which
 * forces the FETs off — is skipped.
 */

typedef enum {
    BMS_DM_U1 = 1,
    BMS_DM_U2 = 2,
    BMS_DM_F4 = 4,
} BmsDmType_t;

#define BMS_DM_FLAG_FINGERPRINT  0x01U

typedef struct {
    uint16_t addr;
    uint8_t type;      /* BmsDmType_t, also the byte length */
    uint8_t flags;
    union {
        uint32_t u;
        float f;
    } value;
    const char *name;
} BmsDmEntry_t;

extern const BmsDmEntry_t g_bms_golden[];
extern const uint16_t g_bms_golden_count;

/* TI factory defaults of a few fields, used to recognise a blank chip. */
#define BMS_BLANK_VCELL_MODE      0x0000U
#define BMS_BLANK_FET_OPTIONS     0x0DU
#define BMS_BLANK_MFG_STATUS_INIT 0x0040U
#define BMS_BLANK_COV_THRESHOLD   86U
#define BMS_BLANK_PROT_B          0x00U

/* Little-endian bytes of an entry (length = type). */
void BmsGolden_Bytes(const BmsDmEntry_t *e, uint8_t out[4]);
/* CRC-32 over (addr,type,value) of every entry — printed by OTP STATUS so
 * the image on the bench can be compared with the documentation. */
uint32_t BmsGolden_Crc32(void);

#endif /* BMS_GOLDEN_H */
