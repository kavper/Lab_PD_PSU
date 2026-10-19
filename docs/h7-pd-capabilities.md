# Read-only H7 PDO/RDO telemetry

Branch `codex/instrument-ui-telemetry` extends the existing binary firmware from
`cursor/board-flash-0ef3`. It does not replace it with the older ASCII main branch.

New frame type `0x14`, 44-byte payload, little endian:

| Offset | Size | Meaning |
| --- | --- | --- |
| 0 | 1 | Schema version 1 |
| 1 | 1 | Flags: attached=1, caps valid=2, RDO valid=4, local source list=8 |
| 2 | 1 | Actual role: 0 unknown, 1 sink, 2 source |
| 3 | 1 | PDO count, 0..7 |
| 4 | 4 | Attach generation |
| 8 | 4 | Active raw RDO |
| 12 | 4 | Active raw PDO |
| 16 | 28 | Seven raw source PDO slots; unused slots zero |

Reserved byte 11 of the existing `0x12` PD frame is now the low byte of the same
attach generation. Existing payload sizes/offsets and METER timing remain unchanged.

Sink exports only fresh, current-session cached partner source capabilities. Source
exports the existing validated local TX source list. Reset/detach suppresses the
list and RDO. RDO is exported only from the existing coherent snapshot matching
the current role. This getter performs no new I2C transactions or policy writes.

The added frame uses slow telemetry slot 3 at 200 ms. UART slow slot capacity is
four; newer samples replace pending slow samples. METER remains 5 ms; existing
safety/command/ACK priority is retained. The H7 companion implementation validates
length/schema/flags/role/count, CRC and session age before displaying a list.

BMS charge/SOC was already present in this base: DASTATUS6 passQ and validity are
sent at BMS offsets 56 and 71. SOC is an estimate using the existing configuration,
not a measured remaining capacity. The reserved session mAh field stays zero.

Validation: Release build succeeds (97,192 bytes FLASH, 16,816 bytes RAM reported by
the linker); existing `test_bms_soc` passes. Added cross-project test extracts the
actual export getter and payload packing statements and feeds the production H7
binary parser. It covers seven PDOs, active RDO/PDO, source-local list, current
generation, stale partner cache and PD reset. Run with a native C compiler:

```
python tests/test_pd_capabilities.py /path/to/GUI_Lab_PD_PSU
```

`CC` may specify the compiler path. Hardware flashing and physical telemetry
verification are deferred; build/test success is not programming confirmation.
