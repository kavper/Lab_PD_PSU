# Host telemetry (USART1 → H7 / PC parser)

USART1 **PC4 TX / PC5 RX**, 115200 8N1. Default: machine frames only (`VERBOSE 0`).

OTP on BQ76922 is **not required**. Blank OTP boots “all cells”; G4 writes **4S `VCell Mode=0x0017` (skip VC4)** in RAM on every wake, then `SLEEP_DISABLE` + `ALL_FETS_ON`. OTP is a factory option only (one-way); do not burn it from this firmware.

## Boot banner

```
=== Lab_PD_PSU G4 host ready (USART1 115200) ===
boot rcc_csr=0x........ (PIN=… POR=… SFT=… IWDG=… WWDG=… LPWR=…)
```

Use `boot rcc_csr` to see why the MCU restarted (pin / BOR / software / IWDG).

## Frame set

On `TEL [ms]` or `?` / `STATUS`:

1. `T …` — PSU / G0 / PD / pre-reg  
2. `TB …` — BMS  
3. `TC …` — BQ25731 + TPS  

Parse: lines starting with `T` / `TB` / `TC`, then `key=value` integers.

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
| `init_step` | init FSM step (0=WAIT_READY …); stuck value diagnoses wake |
| `vcell_rb` | VCell Mode readback (expect `0x0017`) |
| `batt` | Battery Status `0x12` (`CFGUPDATE`=bit0, `SEC` in bits 9:8) |
| `cfg_fail` | failed CONFIG_UPDATE attempts (rising = never entered CFGUPDATE) |
| `manuf` `fet` | Manufacturing Status / FET Status |
| `series` | `4` |

Healthy after button/USB wake: `cfg=1`, `fets=1`, `vcell_rb=0x0017`, `manuf` bit4 set (**not** `0x0017`), `pack_mv` ≈ `stack_mv` ≈ `sum_mv`, `c4_mv=-1`, `min_mv` ~3700.

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

### Schematic (HW rev2, BMS.SchDoc)

| Net | Function |
|---|---|
| **S1** → **TS2**–**BATT-** (+R61) | Button wake from SHUTDOWN |
| **LD** (TP27) | Charger/USB wake when LD > ~1.45 V |
| **RST_SHUT** (**TP28**) | Must stay **LOW** in normal run. High &lt;1 s = AFE digital reset; high ≥1 s = SHUTDOWN (FETs off, REG1 off). Floating/pull-up here blocks `CFGUPDATE` and looks like a wake/reset loop. |
| **CFETOFF / DFETOFF** (TP29 / TP30) | If asserted, FETs stay forced off even after `ALL_FETS_ON` |
| **+VBAT** → LMR33620 → 3V3 | G4 MCU keep-alive (independent of PACK/FETs) |
| **PACK** | After CHG/DSG; `pack_mv≈0` with FETs off is expected |

### Firmware sequence

1. Button (TS2→VSS) or charger (LD) exits SHUTDOWN → G4 boots from +VBAT.  
2. Hold I2C4, settle ~300 ms, `SLEEP_DISABLE`, clear alarms.  
3. `SET_CFGUPDATE` until `batt` bit0=1, write `VCell Mode=0x0017`, exit, verify `vcell_rb`.  
4. `FET_ENABLE` (reject stale `manuf==0x0017`) then `ALL_FETS_ON`.  
5. Prot B OT/UT left off (TS2 is the wake button).

If FETs stay off: measure **TP28 ≈ 0 V**, then `BMS` / `?` — expect `cfg=1 vcell_rb=0x0017 fets=1`. Rising `cfg_fail` with `batt=0x0184` and `init_step` stuck low almost always means **RST_SHUT not held low** or CFETOFF/DFETOFF asserted.
