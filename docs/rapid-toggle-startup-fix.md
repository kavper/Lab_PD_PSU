# Rapid ON/OFF: startup retry pacing

Live G0 capture 2026-10-10-203554 recorded two successful starts and stops,
then four OUT ON NACKs (TYPE 03, reason 04 UNSAFE) at approximately 63.9582,
63.9582, 63.9583 and 63.9592 s. G0 telemetry stayed healthy, fault flags zero.
The fourth retry exhausted G4's four-attempt startup budget, which latches
G4 control FAULT. G0's binary UNSAFE code does not identify the exact preflight
condition, so this trace does not establish whether the first refusal was
VIN, KILL, PGOOD, VOUT zero, ADC freshness or temperature.

The definite scheduling defect is that all four attempts occurred before a
new 5 ms G0 telemetry snapshot. G4 classified the refusal from an older
snapshot and repeated the same action in a tight loop.

After an OUT ON NACK, wait at least 20 ms AND a telemetry-count advance before
classifying/refiring the next attempt. Use the latest snapshot for existing
WAIT_PERMIT, WAIT_VIN and WAIT_VOUT_ZERO recovery. Four retry budget, actual
500 ms lost-link shutdown and G0 preflight protections remain unchanged.
OFF is still immediate at the permit pin. This does not introduce automatic
restart after a latched fault.

A production state-machine regression failed on the previous code and passes
after pacing: no retry before the settle deadline or without a newer frame;
then a retry may be ACKed and become RUNNING. Existing tests for real loss,
confirmed KILL, faults, startup failure and explicit CLEAR continue to pass.
G4 Release builds. Physical repeated-toggle validation after flashing remains
outstanding; the exact first UNSAFE condition is still not identified.
