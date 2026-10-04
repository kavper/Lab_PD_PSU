#ifndef BMS_SOC_H
#define BMS_SOC_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Session state of charge for a 4S1P pack. RAM only: nothing here is
 * written to flash, EEPROM, or BQ76922 OTP.
 *
 * Coulomb counting (CC2, mA) is the source of mAh. Open-circuit voltage
 * is applied only after a qualified rest, and the nominal capacity below
 * is a tuning start — not a measured cell fact. The AFE accumulated-charge
 * register is reported separately and is not the session integral.
 */

#define BMS_SOC_NOMINAL_CELL_MAH     2500
#define BMS_SOC_INVALID_PERMILLE     0xFFFFU

#define BMS_SOC_FLAG_VALID           0x01U
#define BMS_SOC_FLAG_RESTING         0x02U
#define BMS_SOC_FLAG_LEARNED         0x04U
#define BMS_SOC_FLAG_PASSQ_VALID     0x08U
#define BMS_SOC_FLAG_BALANCE         0x10U

typedef enum {
    BMS_SOC_CHEM_LIION = 0,
    BMS_SOC_CHEM_LFP = 1
} BmsSoc_Chem_t;

typedef struct {
    int32_t session_mah;
    int32_t passq_mah;
    uint16_t soc_permille;
    uint8_t flags;
    uint8_t balance_mask;   /* bit0 = cell 1; at most one bit */
    bool balance_write;     /* CB_ACTIVE_CELLS should be sent */
    uint16_t balance_cells; /* same mask, for the 16-bit subcommand */
} BmsSoc_Result_t;

void BmsSoc_Reset(void);
void BmsSoc_SetChemistry(BmsSoc_Chem_t chem);

/* DASTATUS6 userAh: signed whole plus 32-bit fraction. Nearest mAh. */
int32_t BmsSoc_UserAhToMah(int32_t whole, uint32_t frac);

/*
 * cell_mv has cell_count entries. cell_used_mask bit0 is cell 1.
 * A gap above 5 s skips the integral and restarts the rest timer.
 * Returns false when the arguments cannot be applied.
 */
bool BmsSoc_OnSample(uint32_t now_ms,
                     int16_t cc2_ma,
                     const int16_t *cell_mv,
                     uint8_t cell_count,
                     uint8_t cell_used_mask,
                     bool passq_valid,
                     int32_t passq_mah,
                     BmsSoc_Result_t *out);

/* Call after CB_ACTIVE_CELLS is accepted, with the same now_ms. */
void BmsSoc_NoteBalanceSent(uint32_t now_ms);

#endif /* BMS_SOC_H */
