# Software shutdown and TS2 wake — 2026-10-10

The previous shutdown routine sent ALL_FETS_OFF before SHUTDOWN. On a pack-powered host, the first command can remove the host supply before it writes SHUTDOWN. The AFE then remains awake with the FETs off, so a TS2 button press is not a shutdown wake event. This is a source-level explanation consistent with the reported balance-harness power-cycle recovery, not yet a measured confirmation on this board.

The routine now sends SHUTDOWN directly. TI SLUUCG7, command-only subcommands: sealed mode needs two commands within four seconds; in unsealed mode the second skips shutdown delays. There is no preceding pack-FET-off write. The host handler still stops the PSU output before invoking this routine. No OTP is changed.

Reference: https://www.ti.com/lit/ug/sluucg7/sluucg7.pdf

Validation: production function extracted and executed with a command boundary that rejects any pre-shutdown FET-off command; first/second-write failure paths; complete G4 host test suite including G0/H7 protocol and headroom checks; G4 Release build. Hardware shutdown/wake remains to be verified.

Bench: keep balance harness attached; disconnect charging power for the initial test (LD held high can delay true shutdown per TI); issue BMS SHUTDOWN with TS2 released, then a short TS2 press. Repeat at least three times and verify the GUI and pack-powered rails return without disconnecting cells. If it fails, measure TS2 and LD after shutdown before changing OTP or wake settings.

Remote sense audit: ADC1 ranks are PB14/IN5 local, PB0/IN15 plus and PB1/IN12 minus; relay PB6 active high. H7 uses command type 9 and AUX sense_flags bit 1 for requested state. The gate needs output requested, samples above about 0.6 V and three stable valid readings. Missing plus, reversed wiring, or a previous sense latch can hold the relay local. No protection thresholds were relaxed without the board readings. Obtain local/plus/minus voltages, sense code, requested state and relay state at 5 V. J8 pin 2 is sense plus, pin 3 sense minus; pins 1 and 4 are power plus and ground. Both sense leads must reach the load terminals.
