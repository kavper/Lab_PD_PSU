# LDO headroom correction

The former 6 V operating floor and 4.5 V LDO-input threshold contradicted
low-voltage output operation. Replace both G0 and G4 together:

- CV: request applied output setpoint + 1500 mV (G4 retains the host startup setpoint while G0 is off).
- Confirmed CC: request measured final output + 1500 mV, capped at CV headroom.
- Minimum preregulator request: 1500 mV, including zero-output CC.
- G0 preflight/runtime VIN sanity threshold and G4 WAIT_VIN threshold: 1000 mV.
- G4 downward reference slew: 5 V/s instead of 0.3 V/s, both CV and CC.
- CC uses the existing filtered mode and 100 ms confirmation before folding.

The margin is a voltage **request**, not a claim that physical VIN follows
instantaneously or exactly. DCDC slew, loop response, ADC calibration and LDO
hardware limits still apply. The 1 V sanity threshold is a software policy;
minimum analog operating voltage has not been measured on the board.

PGOOD, temperature, overcurrent, output overvoltage, measurement validity,
hardware KILL, host/G0 link watchdogs and explicit OFF remain active.
H7 406896e protocol is unchanged. Prior firmware versions using the 6 V floor
must not be mixed with this G0/G4 pair.

Validation: G0 six host tests; G4 host tests and real supervisor startup at
2.5 V; actual G0/G4 request policies using production configuration across
0–30 V in CV/CC; real G0 pack/G4 forward/H7 parser; Release builds of both.
No analog-load or physical-board validation has been performed.
