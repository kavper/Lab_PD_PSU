#ifndef DCDC_PERMIT_H
#define DCDC_PERMIT_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Host ON starts the pre-regulator with the LDO output still off.
 * POWER_PERMIT is the LDO interlock. It stays off until that stage is
 * already switching and the rail is high enough for G0. Gating the stage
 * on permit, while permit requires the stage, never leaves the idle state.
 *
 * A tight band around the command held the pin in kill for the whole
 * slew and for any offset past that band, so G0 refused OUT ON and the
 * host start timed out with the output still at 0 V.
 *
 * A bench image (BOARD_BRINGUP_LOCAL_CV) still waits for permit, because
 * that build forces the permit itself.
 */

static inline bool Dcdc_StageStartAllowed(bool cv_requested,
                                         bool startup_hold_done,
                                         bool bringup_waits_for_permit,
                                         bool permit_granted)
{
    if (!cv_requested || !startup_hold_done) {
        return false;
    }
    if (bringup_waits_for_permit) {
        return permit_granted;
    }
    return true;
}

static inline bool Dcdc_PermitAllowed(bool enable_requested,
                                     bool dcdc_enabled,
                                     bool rail_ready,
                                     bool permit_override_off)
{
    return enable_requested && dcdc_enabled && rail_ready &&
           !permit_override_off;
}

typedef struct {
    bool timing;
    bool granted;
    uint32_t since_ms;
} DcdcPermitRail;

static inline void Dcdc_PermitRailInit(DcdcPermitRail *rail)
{
    rail->timing = false;
    rail->granted = false;
    rail->since_ms = 0U;
}

/*
 * Grant once measured Vout has sat at or above grant_v (the 6 V floor).
 * After that, hold the pin down to hold_v (G0's 4.5 V VIN_LOW). A dip
 * that is still a valid LDO input must not release POWER_KILL: the pin
 * is a hardware FET kill, not a regulation flag.
 */
static inline bool Dcdc_PermitRailUpdate(DcdcPermitRail *rail,
                                        bool stage_enabled,
                                        float measured_v,
                                        float grant_v,
                                        float hold_v,
                                        uint32_t now_ms,
                                        uint32_t hold_ms)
{
    bool level_ok;

    if ((rail == 0) || !stage_enabled) {
        if (rail != 0) {
            rail->timing = false;
            rail->granted = false;
        }
        return false;
    }

    level_ok = rail->granted ? (measured_v >= hold_v) : (measured_v >= grant_v);
    if (!level_ok) {
        rail->timing = false;
        rail->granted = false;
        return false;
    }
    if (rail->granted) {
        return true;
    }
    if (!rail->timing) {
        rail->timing = true;
        rail->since_ms = now_ms;
    }
    if ((uint32_t)(now_ms - rail->since_ms) >= hold_ms) {
        rail->granted = true;
        return true;
    }
    return false;
}

#endif /* DCDC_PERMIT_H */
