#ifndef DCDC_PERMIT_H
#define DCDC_PERMIT_H

#include <stdbool.h>

/*
 * Host ON starts the pre-regulator with the LDO output still off.
 * POWER_PERMIT is the LDO interlock and is asserted only after that
 * stage is already switching and has held regulation. Gating the stage
 * on permit, while permit requires the stage, never leaves the idle state.
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
                                     bool regulation_ok,
                                     bool permit_override_off)
{
    return enable_requested && dcdc_enabled && regulation_ok &&
           !permit_override_off;
}

#endif /* DCDC_PERMIT_H */
