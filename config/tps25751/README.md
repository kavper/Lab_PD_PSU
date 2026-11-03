# Current TPS EEPROM: 17.50 V / full current

Use 17v5_FULL_CURRENT/SOURCE_5V_3A for 17.496 V (17.50 V rounded down) and the earlier 8.064 A charging ceiling, with SOURCE PPS/fixed12V and updated SINK profiles. G4 must not reapply the old temporary 512 mA clamp.

17v5_FULL_CURRENT/SOURCE_5V_5A_BENCH_ONLY additionally advertises 5V/5A for load testing and remains unvalidated under that load.

MAX_CHARGE/ is superseded: its 18V target was incorrect for the requested 17.50V target. pps_12v_17v5_512ma/ is also historical because it applies a fixed 512mA limit. Do not use either older image as the current configuration.

Neither current image has been programmed in this session. This update changes EEPROM artifacts and documentation only, without accessing G4 hardware.
