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
| `series` | `4` |

Healthy after button/USB wake: `cfg=1`, `fets=1`, `pack_mv` ≈ `stack_mv` ≈ `sum_mv`, `c4_mv=-1`, `min_mv` ~3700.

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

1. Button / TS2 wakes BQ76922 from SHUTDOWN → G4 boots.  
2. Firmware writes `VCell Mode=0x0017`, clears alarms, `SLEEP_DISABLE`, `ALL_FETS_ON`.  
3. If MCU is powered from the **PACK** side, FETs must open before the rail collapses — that looks like a **bootloop** if config never sticks.  
4. OTP is optional factory programming so the AFE would boot already as 4S without the host; this project **re-applies RAM config every wake** instead.

If FETs stay off: send `BMS`, then `?`, check `TB` for `cfg=1 fets=1 pack_mv=…`.
