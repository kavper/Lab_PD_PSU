# Current TPS EEPROM candidate

Use MAX_CHARGE/ for the requested normal charging ceiling, 18 V / 8.064 A at 5 mOhm, with PPS, fixed 12 V and SINK negotiation updates. G4 must have CHARGE_BALANCE_LIMITS_ENABLED=0.

The pps_12v_17v5_512ma/ image is historical, with a fixed 512 mA charging limit, and is not the requested normal-charge configuration.

MAX_CHARGE/SOURCE_5V_3A is the normal candidate. SOURCE_5V_5A_BENCH_ONLY additionally advertises 5 V / 5 A and requires load validation. Neither image has been programmed by this session. The candidate limit does not certify the cells' permitted charge current.
