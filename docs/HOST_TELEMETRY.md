# Host USART1 telemetry (H7 / PC parser)

Physical: **USART1** PC4 TX / PC5 RX, **115200 8N1**, no flow control.
One ASCII line per message, `\n` terminated (`\r` ignored).

**Default:** only `T` / `TB` / `TC` (+ `OK` / `ERR` / `HELP` replies).  
`[APP]` / `[PM]` / `[MON]` spam is **off**. Turn on only for bring-up: `VERBOSE 1`.

OTP on BQ76922 is **not required**. Blank OTP boots “all cells”; G4 writes **4S `VCell Mode=0x0017` (skip VC4)** in RAM on every wake, then `SLEEP_DISABLE` + `ALL_FETS_ON`. OTP is a factory option only (one-way); do not burn it from this firmware.

---

## How to read (H7 or Python)

1. Open serial 115200.
2. Optionally send `TEL 500` (periodic) or `?` for one frame.
3. Split input on `\n`.
4. Keep lines whose first token is `T`, `TB`, or `TC`.
5. Ignore everything else (`OK…`, `ERR…`, `HELP…`, forwarded `TLM…` from G0).
6. Parse `key=value` pairs (space-separated). Values are decimal ints or `0x` hex.

Minimal Python sketch:

```python
import serial

ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.2)
ser.write(b"TEL 500\n")

def parse_line(line: str) -> dict:
    toks = line.strip().split()
    if not toks:
        return {}
    out = {"_type": toks[0]}
    for t in toks[1:]:
        if "=" not in t:
            continue
        k, v = t.split("=", 1)
        if v.startswith("0x") or v.startswith("0X"):
            out[k] = int(v, 16)
        else:
            try:
                out[k] = int(v)
            except ValueError:
                out[k] = v
    return out

while True:
    raw = ser.readline().decode("ascii", errors="ignore")
    if not raw:
        continue
    if raw[0] not in "T":
        continue
    d = parse_line(raw)
    if d.get("_type") == "TB":
        print("pack", d.get("pack_mv"), "mV  I", d.get("i_pack_ma"),
              "mA  fets", d.get("fets"), "cells",
              d.get("c1_mv"), d.get("c2_mv"), d.get("c3_mv"), d.get("c5_mv"))
    if d.get("_type") == "TC":
        print("BQ VBAT", d.get("bq_vbat_mv"), "IBAT", d.get("bq_ibat_ma"))
```

---

## Line `T` — PSU / G0 / PD

| key | meaning |
|---|---|
| `vin_mv` `vout_mv` `iout_ma` | G4 ADC input / DCDC out / current |
| `set_mv` `ilim_ma` | G0 setpoint / current limit |
| `duty_ppm` | PWM duty A × 1e6 |
| `run` `mode` `fault` | running, `IDLE\|CV\|CC`, fault bitmask |
| `pd` `pd_mv` `pd_ma` `pd_mw` | USB-PD contract |
| `permit` `rem_sense` | POWER_PERMIT / remote sense |
| `g0*` | G0 link / OUT sequencer state |
| `vpre_req_mv` `vpre_cmd_mv` `reg_ok` | pre-regulator |
| `pm_st` | power-manager state enum |
| `fmt` | always `0` (machine) |

## Line `TB` — BMS (BQ76922)

| key | meaning |
|---|---|
| `bms` `cfg` `st` | present / configured / state enum |
| `c1_mv`…`c5_mv` | cell voltages; **`c4_mv=-1` expected** (4S skip VC4) |
| `min_mv` `max_mv` `dV_mv` `sum_mv` | among used cells |
| `pack_mv` | PACK pin (after FETs), mV |
| `stack_mv` | top-of-stack, mV |
| `i_pack_ma` / `i_cc2_ma` | pack current (CC2), mA (+ charge / − discharge) |
| `chg` `dsg` `fets` | CHG FET / DSG FET / either on |
| `sa` `sb` `sc` | Safety Status A/B/C |
| `alarm` `alert` `fault` | alarm / alert pin / fault flags |
| `manuf` | Manufacturing Status (`FET_EN`=bit4; **must be set**, never trust `0x0017`) |
| `fet` | FET Status raw (`CHG`=bit0 `DSG`=bit2) |
| `init_step` | init FSM step (0=WAIT_READY … done); stuck value diagnoses wake |
| `vcell_rb` | VCell Mode readback (expect `0x0017`) |
| `batt` | Battery Status `0x12` (`CFGUPDATE`=bit0, `SEC` in bits 9:8) |
| `series` | `4` |

Healthy after button/USB wake: `cfg=1`, `fets=1`, `vcell_rb=0x0017`, `manuf` bit4 set (not `0x0017`), `pack_mv` ≈ `stack_mv` ≈ `sum_mv`, `c4_mv=-1`, `min_mv` ~3700.

## Line `TC` — charger (BQ25731) + TPS

| key | meaning |
|---|---|
| `bq_vbat_mv` `bq_ibat_ma` | battery node V / I (charger ADC) |
| `bq_vbus_mv` `bq_iin_ma` | USB input |
| `bq_vreg_mv` `bq_ichg_set_ma` | charge targets |
| `bq_fast` `bq_pre` `bq_in` | phase flags |
| `tps_vbus_mv` `cc1` `cc2` `role` `conn` | Type-C / PD path |
| `pd_role` `pd_mv` `pd_ma` | PM snapshot contract |

---

## Commands (host → G4)

| cmd | action |
|---|---|
| `TEL [ms]` | period; `0` = stop; default 500 |
| `?` / `STATUS` | one `T`+`TB`+`TC` now |
| `BMS` | re-run 4S config + `ALL_FETS_ON` (RAM only, **no OTP**) |
| `VERBOSE 0\|1` | debug logs on same UART (keep `0` for parsers) |
| `ON` / `OFF` / `SET` / `ILIM` / `PERMIT` / `REMOTE` | power / LDO |
| `CLR` | clear sticky PSU fault latch |

---

## Button / charger wake (no OTP)

Hardware:
1. **Button → TS2** to VSS exits SHUTDOWN; **USB-C/charger → LD** > ~1.45 V also wakes.  
2. **G4 stays on BAT/REG1** with FETs off; **PACK** needs CHG/DSG open.

Firmware (RAM every wake, no OTP):
1. Hold I2C4 briefly; wait `Battery Status` ready; `SET_CFGUPDATE` until `CFGUPDATE=1`.  
2. Write `VCell Mode=0x0017`, exit CFGUPDATE, verify `vcell_rb`, `SLEEP_DISABLE`.  
3. Read Manufacturing Status — **reject stale `0x0017`** (VCell Mode bit4 == `FET_EN`).  
4. `FET_ENABLE` if `FET_EN=0` (blank OTP = FET Test Mode), confirm bit4, then `ALL_FETS_ON`.  
5. Prot B OT/UT left off (TS2 is the wake button).

If FETs stay off: `BMS` then `?` — check `TB` for `cfg=1 vcell_rb=0x0017 manuf` (bit4) `fets=1` and `init_step`/`batt`.
