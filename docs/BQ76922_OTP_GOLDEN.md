# BQ76922 OTP golden (4S, button wake → FETs on, no MCU)

Goal: after `SHUTDOWN`, a short **TS2** press brings CHG/DSG up **without G4**.
OTP is loaded on wake; RAM does not survive SHUTDOWN.

**Do not burn OTP until this exact map works in RAM** for ≥20–30 cycles:
`SHUTDOWN → TS2 → REG18/REG1 → PDSG → DSG/CHG`.

Sources: BQ76922 DS (TS2 wake), TRM SLUUCG7, TI E2E (FET_EN + FET_INIT_OFF=0).

## Required for autonomous FETs

| Addr | Name | Value | Notes |
|------|------|-------|-------|
| `0x9343` | Mfg Status Init | **`0x0050`** | `0x0040` PF_EN + **`0x0010` FET_EN** |
| `0x9308` | FET Options | **`0x1D`** | PDSG_EN\|FET_CTRL_EN\|HOST_FET_EN\|SFET; **FET_INIT_OFF=0** |
| `0x9304` | Vcell Mode | **`0x0017`** | Cells 1/2/3/5, skip VC4 |

Without FET_EN or with FET_INIT_OFF=1, host must still send FET commands.
Wrong Vcell Mode → CUV on empty VC4 → FETs stay off.

## Protections (match current FW)

| Addr | Name | Value | Notes |
|------|------|-------|-------|
| `0x9261` | Enabled Protections A | **`0xBC`** | SCD\|OCD1\|OCC\|COV\|CUV |
| `0x9262` | Enabled Protections B | **`0x00`** | No OT/UT (TS2 = button). Prototype OK; product needs a thermistor plan |
| `0x9265` | CHG FET Protections A | **`0x98`** | TI default: SCD\|OCC\|COV (only 0x98/0x18 legal) |
| `0x9269` | DSG FET Protections A | **`0xE4`** | TI default: SCD\|OCD1\|CUV (only 0xE4/0x80 legal) |
| `0x9275` | CUV Threshold | **55** | ~2.80 V |
| `0x9278` | COV Threshold | **84** | ~4.25 V |
| `0x9280` | OCC Threshold | **25** | ~10 A @ 5 mΩ |
| `0x9282` | OCD1 Threshold | **38** | ~15 A @ 5 mΩ |
| `0x9286` | SCD Threshold | **`0x05`** | ~100 mV → ~20 A @ 5 mΩ |
| `0x9288` | OCC Recovery Threshold | **`+100` mA** | I2; default −200 mA never clears at I≈0 |
| `0x9273` | Body Diode Threshold | **2000** | mA |

## Current sense (5 mΩ) — TRM formula

BQ76922 TRM: `CC Gain = 7.5684 / Rsense[mΩ]` (VREF2 = 1.24 V).  
(Older BQ769x2 notes used 7.4768 — do **not** use that for this part.)

| Addr | Name | Value |
|------|------|-------|
| `0x91A8` | CC Gain (F4) | **`1.51368`** |
| `0x91AC` | Capacity Gain (F4) | **`451472.6456`** (= CC Gain × 298261.6178) |

bqStudio: enter decimals. Manual I2C: IEEE-754 little-endian.

## Pins / PDSG / shutdown policy

| Addr | Name | Value | Notes |
|------|------|-------|-------|
| `0x92FA` | CFETOFF Pin Config | **`0x00`** | Unused (TP29) |
| `0x92FB` | DFETOFF Pin Config | **`0x00`** | Unused (TP30) |
| `0x930E` | Predischarge Timeout | **`0x32`** | ~500 ms — only if PDSG FET+R fitted |
| `0x930F` | Predischarge Stop Delta | **50** | ×10 mV = 500 mV |
| `0x92FE` | TS2 Config | **`0x00`** | Not a thermistor; TS2 wake still works |
| `0x9254` | Auto Shutdown Time | **`0`** | Else wake without I2C may re-enter SHUTDOWN |

Confirm in bqStudio: `Power:Shutdown:Auto Shutdown Time` = **0**.

## Burn procedure (UART — never automatic)

Normal boot / wake **only writes RAM**. OTP is burned **once** via USART1:

1. Pack on balance cable. Apply **BAT 10–12 V** (OTP requirement).
2. `BMS OTP STATUS` → expect `check=0x80`, `full=1`, `otpb=0`.
3. `BMS OTP BURN I-UNDERSTAND-OTP` → one-shot; refused again this boot.
4. `BMS SHUTDOWN` → short TS2 (MCU held in reset / no I2C) → FETs on.

Do **not** re-run BURN. Bits are one-way; each write also burns a signature slot (~7 useful updates max).

Alternate (bqStudio): same map in Data Memory, then Program OTP at BAT 10–12 V.

## 4S → 5S later

- `Vcell Mode = 0x001F` (+ wire cell 4).
- Options: MCU writes RAM each boot; or partial OTP `0x0017→0x001F` (set VC4 bit) if signature budget remains; or new IC if OTP exhausted.
- Re-check caps / FET / converter voltage ratings for 5S.

## Firmware parity

`Core/Inc/bms_board.h` + CFGUPDATE path write this set to **RAM every wake** while OTP is blank — that is **not** an OTP burn.

One-shot OTP from the host UART:

- `BMS OTP STATUS`
- `BMS OTP BURN I-UNDERSTAND-OTP`

After a successful burn, G4 is optional for FET-on; MCU is still used for PD/PSU.
