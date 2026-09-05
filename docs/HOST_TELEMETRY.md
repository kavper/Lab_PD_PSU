# Host telemetry (USART1 → H7 / PC parser)

USART1 **PC4 TX / PC5 RX**, 115200 8N1. Default: machine frames only (`VERBOSE 0`).

Do **not** binary-compress `T`/`TB`/`TC`. The H7 parser is ASCII `key=value`. The win is: stop duplicating G0 `TLM`, stop blocking USART1 TX, debounce live `SET`, then raise baud only if a sequencer needs <100 ms `T`.

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
   Extra DCDC keys: `i_buck_ma` / `i_boost_ma` (INA296 high-side averages), `ucc_a` / `ucc_c` (UCC33420 EN, HS duty ≥ 97%).  
   `duty_a_x10` / `duty_c_x10` are switch duties in percent×10 (802 = 80.2%). `duty_a` = buck HS (TA1), `duty_c` = boost LS (TC2). There is no `duty_ppm`.  
   `g0_vout_mv` is the G0 output (do **not** parse forwarded `TLM` — G4 no longer mirrors it).  
   On high-side OCP the G4 also emits a one-shot `E OCP …` line with both branch currents.  
2. `TB …` — BMS (cell voltages, pack/stack, FETs)  
3. `TC …` — BQ25731 + TPS  

Parse: lines starting with `T` / `TB` / `TC`, then `key=value` integers. Ignore `TLM`, `ACK`, `NACK` (except log NACK), `OK`, `ERR`, `HELP`, and banner lines.

**Period:** `TEL` sets the `T` period (default 500 ms). At 115200, a full `T`+`TB`+`TC` burst is ~1.3 kB (~135 ms on the wire). If `TEL < 200`, G4 still sends `T` at `TEL` but spaces `TB`/`TC` to **200 ms** so cell voltages stay on the wire without saturating USART1. `?` / `STATUS` always sends all three.

| `TEL` | `T` | `TB` + `TC` (cells, charger) | Fits 115200? |
|---|---|---|---|
| 500 ms (default) | 500 | 500 | yes, lots of headroom |
| 200 ms | 200 | 200 | yes |
| 100 ms | 100 | 200 | yes |
| 50 ms | 50 | 200 | tight; raise baud before a sequencer |

## H7 / PC console (UI 1.8.1+)

Keep every dashboard field. Change **how** you read and write the UART:

1. **Parse only `T` / `TB` / `TC`.** Do not decode `TLM out=…`. G4 no longer forwards G0 TLM (unless `VERBOSE 1`). Use `g0_vout_mv`, `g0_out`, `set_mv`, `c1_mv`…`c5_mv` from machine frames.  
2. **Do not send `SET` on every slider tick.** Debounce ≥80–100 ms, or send on release / “Wyślij oba”. Live SET while output is ON is applied on G4 without `OUT OFF`. Slider spam was shredding `SET 17.300` → `OK SET 1300 mV` and blinking the rail.  
3. **Stay at 115200** until you bump G4 + H7 together. Next step for a sequencer is **230400 or 460800 8N1**, still ASCII, still `T`/`TB`/`TC`. Do not invent a binary frame until the parser is rewritten.  
4. `TEL 100` is the practical floor at 115200. Sequencer at 50 ms `T` wants 460800 (or a later compact `T`).  
5. Treat `ERR CMD` / `ERR LINE` as a lost command, not a protocol change. Re-send the last SET/ILIM once, debounce first.  
6. `VERBOSE 0` on the G4. Verbose dumps debug ASCII onto the same USART and will break the parser.

## G0 firmware (USART2, not the H7 port)

G4 still **needs** G0 `TLM` on USART2 for `vout` / `pgood` / `kill` / `outoff`. Do not delete TLM. Do not echo it to the host.

| Change | Why |
|---|---|
| Accept **live `SET V=x I=y` while `OUT=ON`** | G4 no longer drops the rail (`OUT OFF` / wait Vout≈0) for slider SET. If G0 NACK `VOUT_NOT_ZERO` on SET-while-ON, the slider will stall. |
| Do **not** require `OUT OFF` before SET | Same. First-start zero-V wait on G4 is only for the initial `OUT ON`. |
| TLM **10 Hz is enough** (20 Hz max) | USART2 is 115200 and only G4 listens. Faster TLM does not make the H7 UI faster. |
| Do **not** echo the `SET` command line back | Echo + TLM + ACK was filling G4 RX. Reply **one** `ACK SET V=… I=… OUT=…`. |
| One ACK per command, no `OK SET` plus `ACK SET` spam if you can | G4 keys off `ACK SET`. Extra `OK SET` is ignored (not `ACK` / `TLM` / `NACK`). |
| Keep TLM ASCII `TLM out=… vset=… vout=… pgood=… kill=… fault=…` | G4 rejects shredded lines that miss those tokens. |

G0↔G4 “junk” to drop: SET echo, extra OK lines, TLM faster than 10 Hz, any debug on USART2. Do **not** drop cell voltages — those are **not** on G0; they are `TB` from the G4 BQ76922.

## Why not compression first

A G0 `TLM` line is already on USART2. Mirroring it to USART1 doubled the host traffic and blocked `HAL_UART_Transmit` so host `SET` lost bytes (`ERR CMD`, `SET 1.300`). Binary frames would require an H7 rewrite and would hide the console. Cut the duplicate, use a TX ring, then raise baud.

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

## Button wake (no OTP, no host command)

Product sleep = BQ76922 **SHUTDOWN**. Wake = **TS2 button only** (short press → VSS). No UART/`BMS`/`ON` needed for wake — firmware re-probes the AFE and runs `FET_ENABLE` + `ALL_FETS_ON` automatically. Charger on **LD** is an alternate wake for plug-in.

### Schematic (HW rev2, BMS.SchDoc)

| Net | Function |
|---|---|
| **S1** → **TS2**–**BATT-** (+R61) | **Only** product wake from SHUTDOWN |
| **LD** (TP27) | Charger/USB wake when LD > ~1.45 V |
| **RST_SHUT** (**TP28**) | Must stay **LOW** in normal run. Lab enter-SHUTDOWN: drive **TP28 to 3.3 V (logic — never pack VBAT)** ≥1 s, then GND. Or UART `BMS SHUTDOWN` once to *enter* sleep (not to wake). |
| **CFETOFF / DFETOFF** (TP29 / TP30) | Leave floating OK: CFGUPDATE writes Pin Config **0x00** (unused). |
| **+VBAT** → LMR33620 → 3V3 | MCU rail; AFE REG18 dies in SHUTDOWN. Driver treats AFE I2C loss as absent and re-inits FETs on button wake with **zero** host commands. |
| **PACK** | After CHG/DSG; `pack_mv≈0` with FETs off is expected |

### Firmware sequence (automatic on TS2 wake)

1. TS2→VSS exits SHUTDOWN → AFE returns on I2C (probe).  
2. Hold I2C4, settle ~300 ms, `SLEEP_DISABLE`, clear alarms.  
3. `SET_CFGUPDATE` … `VCell Mode=0x0017`, protections, CFETOFF=0, CC Gain, exit, verify `vcell_rb`.  
4. `FET_ENABLE` then `ALL_FETS_ON` (PDSG soft-start).  
5. Runtime keeps CHG+DSG up after recoverable SCD/OCC/false-COV — still no host command.

If FETs stay off after a button wake: **TP28 ≈ 0 V**, release the button fully (held TS2 = soft-SHUTDOWN), then check `TB`/`?` for `cfg=1 fets=1 vcell_rb=0x0017`. Rising `cfg_fail` with `init_step` stuck low → **RST_SHUT not low**.

### Sense resistors / current limits (do not mix paths)

| Path | Shunt | Where set | Notes |
|---|---|---|---|
| **BQ76922 pack** (`i_pack_ma` / `i_cc2_ma`) | **5 mΩ** SRP–SRN | `BMS_SENSE_MOHM` → `CC Gain` / `Capacity Gain` in CFGUPDATE | OTP default is ~1 mΩ → readings were ~**5× too high** until `CC Gain=7.5684/5=1.51368`. **Already in this FW** — schematic 5 mΩ is correct; not the `chg=0` root cause |
| **BQ25731 charger** | **5 mΩ** RAC/RSR | `power_manager` Option1 `FAST_5MOHM` | Already programmed at BQ init. Also at init via TPS I2C passthrough `0x6B`: Option0 `EN_OOA|PWM_FREQ=400kHz` (`0x0400|0x0200`), Option4 dither ±6% (`0x1800`), Option3 `EN_OTG_BIGCAP` (no HW VBUS slew bit — TPS owns `OTGVoltage`) |
| **DCDC INA296 high-side** (`iout_ma` / `i_buck_ma` / `i_boost_ma` on T) | **1 mΩ** in `board_rev.h` | `BOARD_INA296_SHUNT_OHM` | Buck HS = `I_IN_BUCK`, boost HS = `I_OUT_BOOST`. ACS37100 series `I_L_MEAS` is ignored. |

BMS protection numbers after CFGUPDATE:

| Limit | Value | Meaning @ 5 mΩ |
|---|---|---|
| SCD threshold code `0x05` | ~100 mV | ~**20 A** short (hardware mV, not CC Gain) |
| OCC threshold | ~50 mV (code 25) | ~**10 A** charge (default was ~0.8 A) |
| OCD1 threshold | ~75 mV (code 38) | ~**15 A** discharge tier 1 |
| Body diode | 2000 mA | After CC Gain fix = real ~2 A |
| COV / CUV | 4250 / 2800 mV/cell | Unchanged |
| Safety Status A | TI bits 7..2 | SCD=0x80 … OCC=0x10 COV=0x08 CUV=0x04 |
