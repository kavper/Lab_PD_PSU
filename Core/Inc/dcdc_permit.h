#ifndef DCDC_PERMIT_H
#define DCDC_PERMIT_H

#include <stdbool.h>

/*
 * Host ON starts the pre-regulator with the LDO output still off.
 * POWER_PERMIT clears G0's hardware POWER_KILL as part of that explicit
 * request. It does not turn the LDO output on: G0 still requires fresh
 * measurements, PGOOD, VIN >= 4.5 V, SETPOINT and SET_OUTPUT=1.
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
                                     bool permit_override_off)
{
    return enable_requested && !permit_override_off;
}

#endif /* DCDC_PERMIT_H */
