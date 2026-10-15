# Remote sense runtime correction — 2026-10-11

The relay supervisor compared the measured remote differential voltage with the final G0 voltage request, then tripped after three mismatches (300 ms). G0 ramps the applied DAC target; below-final-request operation is normal during startup, setpoint changes and CC transitions. This comparison is removed from the cable gate. The simultaneous local/remote wire-drop checks, local saturation and conversion-loss trips remain, as do G0 and G4 electrical protections. Legacy SENSE_CV_VD code and input fields remain reserved for compatibility.

A latched remote fault now preserves its first code after output shutdown. Previously the next zero-voltage sample overwrote that code with NOT_READY, concealing the reason. Explicit REMOTE OFF still clears the sense latch.

Validation: host tests include five seconds of legal wire drops with a different final CV request, preservation of the first fault through zero-output/OFF, and existing reversed/missing/overdrop/saturation/conversion-loss cases. Full G4 host suite, G0/H7 protocol checks and Release build pass. The user's particular trip code was not captured, so this removes a demonstrated false-trip path rather than proving it was the sole hardware cause.

At zero volts the requested remote mode remains stored; relay engagement still waits for three valid readings above approximately 0.6 V. Zero readings cannot establish continuity on the existing passive divider circuit. This change does not force an unverified remote feedback path at zero volts. Already validated remote remains engaged when voltage falls to zero while output remains requested.

## Consecutive sample confirmation

Moderate wire-drop violations now require three consecutive invalid samples (100 ms sampling period: up to 300 ms from event onset). A correct or low-voltage sample resets the drop counter. One missed injected ADC conversion is tolerated; two consecutive misses stop the output (up to 200 ms from onset). A missed conversion resets the wire-drop counter. Reversal, both leads on plus, short classification, local saturation and gross drop errors remain immediate. Gross means an excess greater than 1 V beyond the existing individual-leg error/drop budget. This extra 1 V defines which faults are immediate; it does not enlarge the steady-state accepted drop.

Tests exercise twenty isolated spikes interleaved with valid readings, persistent moderate drop requiring three samples, two consecutive missed conversions, and existing immediate gross/reversed faults. All host tests and the Release build pass. Noise on the actual board has not been captured, so this mitigates an observed source-level sensitivity without proving the cause of the reported shutdown.
