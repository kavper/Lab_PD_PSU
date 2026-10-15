# BMS predischarge guard, 2026-10-15

Observed on the connected STM32G47x: flash matches 573fad6 byte for byte; BQ snapshot FET Status 0x05 (CHG+DSG, PDSG off), no safety flags, pack about 14.18 V, stack 14.39 V. The reported smoking state was not reproduced.

Source issues: init treated PDSG as completed DSG; runtime reissued ALL_FETS_ON after failures indefinitely; OTP fast path skipped timeout/delta verification. This change checks actual RAM timeout/delta even on the fast path (50*10 ms=500 ms, delta 50*10 mV=500 mV), reads both back after RAM programming, and requires CHG+DSG with PDSG off to finish startup. Init polls for completion instead of retriggering. Repeated host ALL_FETS_ON within a bound driver session is blocked and latched. Runtime no longer drives an endless host recovery loop through the predischarge resistor. AFE hardware protections continue to operate.

An independent software ceiling latches after PDSG is observed continuously for 1000 ms, allowing scan latency beyond the 500 ms hardware setting. It requests PSU shutdown and retries ALL_FETS_OFF until acknowledged. Fault bit 0x100 means FET startup/guard failure. A hardware driver restart clears RAM latch; ordinary CLEAR/BMS reinit and I2C absence do not clear it. This deliberately requires inspection after a failed startup, not repeated attempts. The software timer requires a running, powered MCU and cooperative task; it is not an independent hardware thermal fuse. A shorted MOSFET, incorrect resistor energy rating, or autonomous OTP recovery can still cause physical overheating. OTP is not programmed or changed.

Validation: all G4 host tests including start/completion, continuous PDSG timeout, timer wrap and repeated-start blocking; ARM Release build. Board status and programmed-flash verification are recorded separately in output/pdsg-2026-10-15. No short/load/smoking test is performed.

TI reference: BQ76922 TRM SLUUCG7, predischarge mode 5.2.3.2.2. Timeout exits predischarge and turns DSG on; zero disables the time criterion. https://www.ti.com/lit/ug/sluucg7/sluucg7.pdf
