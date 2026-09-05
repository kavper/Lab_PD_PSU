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
| `pd_role` `pd_mv` `pd_ma` | PM snapshot contract (`pd_role`: 1=sink charge, 2=source) |
| `bq_otg` | BQ25731 `IN_OTG`. `1` only after a real Source **contract** (`conn` 6/7 + PDO/RDO). Stays `0` in AttachWait, SINK ONLY, and AUTO sink. |

Default USB-C: **AUTO = DRP + Try.SRC** with a deterministic session/AUTO state machine.

### Session / attach_generation
Every plug and unplug bumps `attach_generation`. PDO/RDO, partner Source Caps, AUTO decisions, and 4CC command results are valid only for the current generation. Soft detach invalidates them without writing PORT_CONFIG `0x28`.

### Normal unplug vs stuck reset
- **Normal detach**: TPS Type-C SM handles it. Firmware soft-invalidates session data and clears INITIATE bits only. **No** Disabled→vSafe0V→tSrcRecover window (so fast replug is not blacked out for ~800 ms).
- **Destructive reset** (`0x28` Disabled → wait vSafe0V → tSrcRecover → restore): only after a **confirmed stuck** state (several agreeing coherent STATUS/TYPEC/ADC samples + separate debounce). Used for leftover VBUS / AttachWait stuck, and for explicit user mode changes.
- If vSafe0V is **not** reached before the give-up timeout: stay Disabled, latch `session_reset_faulted`, report TPS/BQ VBUS + TYPEC diagnostics — **do not** restore Type-C.

### Source Caps freshness
After `GSrC`, wait for `source_caps_received` for this generation before trusting `RX_SOURCE_CAPS`. A cold register read alone never sets `partner_source_caps_current/fresh`. GSrC fail/timeout does not fall through to stale RX contents.

### PR_SWAP single initiation path
`PORT_CONTROL 0x29` carries **PROCESS_SWAP_*** acceptance policy only. Role change is initiated **once** via `SWSk` (Source→Sink yield). Never set `INITIATE_SWAP_*` and send SWSk/SWSr together. If already at the target role, do neither. PORT_CONTROL logs all four bits: `process_snk`, `initiate_snk`, `process_src`, `initiate_src`.

### AUTO decision (locked once per plug)
Inputs: Type-C initial role, fresh Source Caps (+ Dual-Role Power), live sink contract, user mode.
- Attached.SNK / live sink contract (incl. 5 V powerbank) → **SINK**
- Attached.SRC + strong fresh caps (V>5 V or >15 W) → **SINK** yield (one SWSk)
- Attached.SRC otherwise / ambiguous weak DRP → **keep Type-C role** (documented preference; no oscillate)
- User SINK ONLY / SOURCE ONLY overrides heuristics

### Source power path
Explicit SM: wait PD contract → confirm BQ ADC VBUS ~5 V → then raise OTG pin / treat path READY. Valid PDO/RDO alone is not enough. Do not drop 5 V while BQ starts.

### BQ ownership
STM may program/verify static Option0/1/4/ADC (+ BIGCAP) before port activation. During a live contract **TPS owns runtime** Option3 / EN_HIZ / EN_OTG. STM only touches Option3 in stuck-reset or an explicit emergency shutdown (RMW + readback).


---

## Commands (host → G4)

| cmd | action |
|---|---|
| `TEL [ms]` | period; `0` = stop; default 500 |
| `?` / `STATUS` | one `T`+`TB`+`TC` now |
| `BMS` | **soft**: if already `cfg=1` + CHG/DSG on + `vcell_rb=0x0017`, skip CFGUPDATE (clear sticky only). Otherwise full reinit. |
| `BMS FORCE` / `BMSREINIT` | full CONFIG_UPDATE + `ALL_FETS_ON` (RAM only, **no OTP**). Can hold I2C4 (starve BQ/TPS) and PACK inrush may PIN/POR-reboot the G4 — do not click casually while running. |
| `VERBOSE 0\|1` | debug logs on same UART (keep `0` for parsers) |
| `ON` / `OFF` / `SET` / `ILIM` / `PERMIT` / `REMOTE` | power / LDO |
| `USB AUTO` / `USB SINK` / `USB SOURCE` | Type-C role. **AUTO** = DRP+Try.SRC; if already Attached.SNK (incl. 5 V powerbank) stay SINK; sink-only gadget → SOURCE; strong partner Source Caps while we are Source → SINK. `USB SOURCE` / `USB SINK` are true single-role (reject the opposite PR_Swap). |
| `CLR` | clear sticky PSU fault latch |

---

## Button / charger wake (no OTP)

### Schematic (HW rev2, BMS.SchDoc)

| Net | Function |
|---|---|
| **S1** → **TS2**–**BATT-** (+R61) | Button wake from SHUTDOWN |
| **LD** (TP27) | Charger/USB wake when LD > ~1.45 V |
| **RST_SHUT** (**TP28**) | Must stay **LOW** in normal run. To force SHUTDOWN for button tests: drive **TP28 to 3.3 V (REG1 / MCU logic — never pack VBAT)** for **≥1 s**, then release to GND. High &lt;1 s = AFE digital reset only. Floating/pull-up here blocks `CFGUPDATE` and looks like a wake/reset loop. Prefer UART `BMS SHUTDOWN` when the link is up. |
| **CFETOFF / DFETOFF** (TP29 / TP30) | Leave floating OK now: CFGUPDATE writes Pin Config **0x00** (unused). If OTP had CFETOFF (`0x02`) and the pin floated high → `fet=0x14` (`DSG`+`DCHG_PIN`), `chg=0` forever |
| **+VBAT** → LMR33620 → 3V3 | G4 MCU keep-alive (independent of PACK/FETs) |
| **PACK** | After CHG/DSG; `pack_mv≈0` with FETs off is expected |

### Firmware sequence

1. Button (TS2→VSS) or charger (LD) exits SHUTDOWN → G4 boots from +VBAT.  
2. Hold I2C4, settle ~300 ms, `SLEEP_DISABLE`, clear alarms.  
3. `SET_CFGUPDATE` until `batt` bit0=1, write `VCell Mode=0x0017`, **OCC/OCD1 thresholds**, **CFETOFF/DFETOFF Pin Config=0x00**, CC Gain for 5 mΩ, exit, verify `vcell_rb`.  
4. `FET_ENABLE` (reject stale `manuf==0x0017`) then `ALL_FETS_ON` with **PDSG** soft-start (FET Options `PDSG_EN`, SCD threshold raised, body-diode threshold 2 A). Init verifies CHG+DSG and retries after clearing SCD/OCC.  
5. Prot B OT/UT left off (TS2 is the wake button). Runtime: if CHG or DSG drops (charger plug/unplug / OCC latch), clear alarms + `ALL_FETS_ON`; sticky current faults re-issue FET commands without latching `FAULT_BMS`.

If FETs stay off: measure **TP28 ≈ 0 V**, then `BMS` / `?` — expect `cfg=1 vcell_rb=0x0017 fets=1` with `chg=1 dsg=1` and `sa` without SCD (`sa&0x80==0`) or OCC (`sa&0x10==0`). Rising `cfg_fail` with `batt=0x0184` and `init_step` stuck low almost always means **RST_SHUT not held low**. Stuck `chg=0 dsg=1 fet=0x14 sa=0x10` was **real OCC** (default ~0.8 A @ 5 mΩ vs ~8 A charger) mislabeled as COV by a reversed Safety A bit map — fixed by OCC/OCD1 thresholds + TI bit map; CFETOFF/DFETOFF also forced unused. Reboot loops with `sa=0x90` (SCD|OCC) / `vin` dip after `cfg=1` were capacitive PACK inrush — fixed by PDSG + FET verify retry.

### Sense resistors / current limits (do not mix paths)

| Path | Shunt | Where set | Notes |
|---|---|---|---|
| **BQ76922 pack** (`i_pack_ma` / `i_cc2_ma`) | **5 mΩ** SRP–SRN | `BMS_SENSE_MOHM` → `CC Gain` / `Capacity Gain` in CFGUPDATE | OTP default is ~1 mΩ → readings were ~**5× too high** until `CC Gain=7.4768/5`. **Already in this FW** — schematic 5 mΩ is correct; not the `chg=0` root cause |
| **BQ25731 charger** | **5 mΩ** RAC/RSR | `power_manager` Option1 `FAST_5MOHM` | Already programmed at BQ init. Also at init via TPS I2C passthrough `0x6B`: Option0 `EN_OOA|PWM_FREQ=400kHz` (`0x0400|0x0200`), Option4 dither ±6% (`0x1800`), Option3 `EN_OTG_BIGCAP` (no HW VBUS slew bit — TPS owns `OTGVoltage`) |
| **DCDC / INA296** (`iout` on T line) | **1 mΩ** in `board_rev.h` | `BOARD_INA296_SHUNT_OHM` | Separate from BMS pack sense |

BMS protection numbers after CFGUPDATE:

| Limit | Value | Meaning @ 5 mΩ |
|---|---|---|
| SCD threshold code `0x05` | ~100 mV | ~**20 A** short (hardware mV, not CC Gain) |
| OCC threshold | ~50 mV (code 25) | ~**10 A** charge (default was ~0.8 A) |
| OCD1 threshold | ~75 mV (code 38) | ~**15 A** discharge tier 1 |
| Body diode | 2000 mA | After CC Gain fix = real ~2 A |
| COV / CUV | 4250 / 2800 mV/cell | Unchanged |
| Safety Status A | TI bits 7..2 | SCD=0x80 … OCC=0x10 COV=0x08 CUV=0x04 |
