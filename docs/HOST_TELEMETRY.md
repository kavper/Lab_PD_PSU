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

Default USB-C: **AUTO = DRP + Try.SRC**. Firmware follows the Type-C attach role; it does **not** software-PR_SWAP into SOURCE (that OTG-discharged the pack into 5 V powerbanks).

- **Attached.SNK / live sink contract** (including a **5 V-only powerbank**) → **SINK**, charge the pack, `bq_otg=0`.
- **Attached.SRC** + fresh partner caps with V>5 V or >15 W → **SINK** (yield to charger).
- **Attached.SRC** otherwise → **SOURCE** (stay; phone/iPad via Try.SRC). PORT_CONTROL uses `PROCESS_SWAP_SRC` only — no `INITIATE_SWAP_SRC`.
- Role not stable yet → wait; do not lock a guessed mode.

`USB SOURCE` is source-only (`0x28` Source SM, `0x29` reject swap to sink). `USB SINK` is sink-only (`0x28` Sink SM, `0x29` reject swap to source). 9 V source PDOs stay in the TPS image.

`cc1`/`cc2` are TPS `0x69` pin states (TRM 3.2.27): `1`=Ra and `2`=Rd are **Source-only** detections. `conn` is `STATUS 0x1A` bits 3:1 (`0`=no connection, `6`/`7`=attached). `plug` is `STATUS` bit 0 PlugPresent. `typec` is `0x69` TypeC Port State (`0x60` Attached.SRC, `0x61` Attached.SNK, `0x64` AttachWait.SRC, `0x67` Unattached.SRC, `0x00` Disabled). `rst` increments only when firmware actually starts a silicon reset; `rst_busy=1` while `0x28` is held Disabled waiting vSafe0V.

Expected `TC` after flash:

- **iPhone (good, keep)**: later `conn=6/7`, `typec=0x60`, PD `5V/3A` then `9V/3A` (`pd_mv=9000 pd_ma=3000`), `bq_otg=1` only after that contract.
- **iPad + USB SOURCE**: must leave `conn=0 typec=0x64` + VBUS 5↔0. Next log: `typec=0x60`, `conn=6/7`, `pd_role=2`, `pd_mv=5000` or `9000`, `bq_otg=1` only after the contract. `[PD-RESET]` on unplug/mode change restores **source-only** (`sm=1`), not DRP.
- **iPad + AUTO**: Try.SRC → Attached.SRC → stay SOURCE (`pd_role=2`, `bq_otg=1` after contract). No software swap to SOURCE.
- **Charger / 5 V powerbank + AUTO**: `typec=0x61`, `conn=6/7`, `pd_role=1`, `bq_otg=0`, charge on `bq_ichg_ma` / `bq_ibat_ma`.
- Accidental OTG: PA4 stays low until Source contract.

Unplug/reattach or `USB SOURCE`/`USB SINK`/`USB AUTO`: PA4 LOW, BQ `ChargeOption3` EN_HIZ=1 EN_OTG=0, `0x28` Disabled until TPS VBUS `<0.8 V` and `tSrcRecover` 800 ms, then `0x28` for **that** mode (DRP+Try.SRC / Source-only / Sink-only), `0x29` swap bits for that mode. Do not treat `AttachWait.SRC` (`typec=0x64`) with vSafe0V as leftover — that is the iPad SOURCE attach. Kick only if CC is live in Unattached, or AttachWait with VBUS still high for 1.5 s.

**Sticky / OTG regressions (fixed):** stale `RX_SOURCE_CAPS` are ignored without a fresh plug event; Attached.SNK always stays SINK; AUTO never initiates PR_SWAP to SOURCE after SINK ONLY → AUTO.

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
| **RST_SHUT** (**TP28**) | Must stay **LOW** in normal run. High &lt;1 s = AFE digital reset; high ≥1 s = SHUTDOWN (FETs off, REG1 off). Floating/pull-up here blocks `CFGUPDATE` and looks like a wake/reset loop. |
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
