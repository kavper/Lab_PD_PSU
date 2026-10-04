#ifndef SENSE_CHECK_H
#define SENSE_CHECK_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Self-test dividers on ADC_LOCAL_VOUT, ADC_REMOTE_P and ADC_REMOTE_N.
 * Each is 220 kΩ over 20 kΩ to ground, then 1 kΩ into the pin.
 * The 1 kΩ does not change the DC ratio. Vadc = Vin / 12.
 * Clamps are BAT54 to 3.3 V and ground. The ADC reference is 3.0 V,
 * so full scale is 36 V.
 *
 * The test runs with the relay released (local sense). K1 is a G6K-2G-DC5:
 * coil off connects V_SNS_P to local VOUT and V_SNS_N to local GND.
 * REMOTE_ON high turns Q14 on, the coil pulls in, and those nets move to
 * REMOTE_P / REMOTE_N. R112 and R115 (4.7 kΩ) stay across the local output
 * as a fallback if a remote lead is open, so an open lead does not wind
 * the loop up. Swapped leads would. The relay is clicked only after OK.
 *
 * An open negative lead reads the same ~0 V as a negative lead on the load
 * return at no current. That case is reported as OK. A negative lead on
 * the positive output, a swapped pair, and a missing positive lead are not.
 */

#define SENSE_DIVIDER_NUM            12U
#define SENSE_ADC_FULL_SCALE         4095U
#define SENSE_MIN_LOCAL_MV           2000U
#define SENSE_WINDOW_MIN_MV          1500U
#define SENSE_WINDOW_DIV             4U
#define SENSE_OK_STREAK              3U
#define SENSE_MV_MISSING             0xFFFFU

enum {
    SENSE_OK = 0,          /* positive matches Vout, negative is near ground */
    SENSE_NOT_READY = 1,   /* local Vout too low to tell an open lead from a real low output */
    SENSE_OPEN = 2,        /* both leads near ground: unplugged, or the positive lead is open */
    SENSE_OPEN_P = 3,      /* positive lead open, negative is neither ground nor Vout */
    SENSE_REVERSED = 4,    /* leads swapped: negative sees Vout, positive does not */
    SENSE_N_ON_POUT = 5,   /* both leads on the positive output; differential would be ~0 */
    SENSE_P_MISMATCH = 6,  /* positive lead is neither at Vout nor open */
    SENSE_N_HIGH = 7,      /* positive looks right, negative is well above ground */
    SENSE_NO_SAMPLE = 8    /* injected ADC has not produced a reading */
};

static inline uint32_t Sense_CountsToMv(uint16_t counts, uint32_t vref_mv)
{
    if (counts > SENSE_ADC_FULL_SCALE) {
        counts = (uint16_t)SENSE_ADC_FULL_SCALE;
    }
    return (uint32_t)(((uint64_t)counts * (uint64_t)vref_mv *
                       (uint64_t)SENSE_DIVIDER_NUM +
                       (SENSE_ADC_FULL_SCALE / 2U)) /
                      SENSE_ADC_FULL_SCALE);
}

static inline uint16_t Sense_MvToU16(uint32_t mv)
{
    if (mv >= SENSE_MV_MISSING) {
        return (uint16_t)(SENSE_MV_MISSING - 1U);
    }
    return (uint16_t)mv;
}

static inline uint32_t Sense_WindowMv(uint32_t local_mv)
{
    uint32_t frac = local_mv / SENSE_WINDOW_DIV;

    if (frac < SENSE_WINDOW_MIN_MV) {
        return SENSE_WINDOW_MIN_MV;
    }
    return frac;
}

static inline bool Sense_Near(uint32_t value_mv, uint32_t target_mv,
                              uint32_t window_mv)
{
    uint32_t diff = (value_mv > target_mv) ? (value_mv - target_mv)
                                           : (target_mv - value_mv);
    return diff <= window_mv;
}

static inline uint8_t Sense_Classify(uint32_t local_mv, uint32_t remote_p_mv,
                                    uint32_t remote_n_mv)
{
    uint32_t window;
    bool p_near;
    bool n_near;
    bool p_low;
    bool n_low;

    if (local_mv < SENSE_MIN_LOCAL_MV) {
        return SENSE_NOT_READY;
    }

    window = Sense_WindowMv(local_mv);
    p_near = Sense_Near(remote_p_mv, local_mv, window);
    n_near = Sense_Near(remote_n_mv, local_mv, window);
    p_low = (remote_p_mv <= window);
    n_low = (remote_n_mv <= window);

    if (n_near && p_near) {
        return SENSE_N_ON_POUT;
    }
    if (n_near) {
        return SENSE_REVERSED;
    }
    if (p_near && n_low) {
        return SENSE_OK;
    }
    if (p_near) {
        return SENSE_N_HIGH;
    }
    if (p_low && n_low) {
        return SENSE_OPEN;
    }
    if (p_low) {
        return SENSE_OPEN_P;
    }
    return SENSE_P_MISMATCH;
}

#endif /* SENSE_CHECK_H */
