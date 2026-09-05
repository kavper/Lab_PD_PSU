# G4 ↔ G0 LDO UART (G474 firmware)

Canonical copy: [LDO_controller `docs/G4_LDO_UART.md`](https://github.com/kavper/LDO_controller/blob/cursor/g0-hw-rev-cubemx-19b5/docs/G4_LDO_UART.md)

## Split of roles

| MCU | Role |
|---|---|
| **G0** | Final **CC/CV** on LDO output, bleed/fan policy, `vpre_request` computation |
| **G4** | **Pre-regulator** DCDC: holds `Vin_LDO` at headroom above G0 output, fan PWM, BLEED_ON, POWER_PERMIT_G4 |

G4 **does not** run CC/CV on the user output. It regulates **ADC_VOUT (PB2)** = DCDC rail feeding the LDO.

## Pre-regulator policy (`ldo_prereg.c`)

Mirrors G0 `control_update_vpre_request()`, with a **VIN floor** so CC collapse / OUT-off / stale TLM cannot starve the LDO:

| G0 state | DCDC target |
|---|---|
| `out=0`, host idle (`g0_want=0`) | disable DCDC, ramp command → 3 V |
| `out=0`, host wants ON | `max(host_vset, tlm_vset) + 3 V`, floored at **6 V** |
| CV (`cccv=0`) | same floor (or `vpre=` from TLM if present) |
| CC (`cccv=1`) | `max(vout + 3 V, vset + 3 V, 6 V)` — do **not** follow collapsed `vout` |
| Stale TLM while `g0_want=1` | **hold** CV/VIN floor (do not dive to 3 V) |

`fault=VIN_LOW` does **not** disable the pre-reg DCDC (that fault is caused by a low rail; killing DCDC worsens the spiral). Other faults still drop enable.

Constants (match G0 `app_config.h`): min 3 V, max 36 V, margin 3 V, **VIN floor 6 V** (`BOARD_VPRE_VIN_FLOOR_V` = G0 `CONSOLE_MINIMUM_VIN_MV`).

Slew: up 10 V/s, down 0.3 V/s (command never below 6 V while output wanted/on). **POWER_PERMIT_G4** asserts only after DCDC is within 0.5 V of command for 150 ms (`pgood=1`, non-blocking fault, G0 `out=1` or want).

## Pin / module map (schematic U7)

| Interface | Pins | Module |
|---|---|---|
| G0 isolated UART | USART2 **PB3 TX / PB4 RX** AF7 | `ldo_link.c` |
| H7 / PC host | USART1 PC4/PC5 | `host_link.c` |
| Fan PWM | **PA7** TIM17_CH1 AF1 | `fan_pwm.c` (PA6 is NC) |
| BLEED_ON | **PB5** | `ldo_link.c` |
| REMOTE_ON | **PB6** | `ldo_link.c` (default LOW) |
| POWER_PERMIT_G4 | **PB7** | `ldo_prereg.c` → `ldo_link.c` (HIGH=ena, Low/reset=LDO zabity) |
| I2C_USBPD_IRQ | **PB9** | EXTI |
| Local Vout sense (DCDC) | PB2 `ADC_VOUT` | `measurements.c` (CV) |
| ADC_LOCAL_VOUT | **PB14** | analog (no DMA rank yet) |
| I_L_ZERO | **PB15** | analog (no DMA rank yet) |
| Remote Kelvin sense | PB0/PB1 `ADC_REMOTE_P/N` | analog inputs (no DMA ranks yet) |
| PA2 | NC | — |

## Local vs remote sense

- **Default at boot:** local only (`REMOTE_ON` = LOW). CV regulation always uses `ADC_VOUT` (PB2).
- Host `REMOTE ON` / `REMOTE 1` asserts `REMOTE_ON` (**PB6**) to switch the hardware remote sense path.
- Host `REMOTE OFF` / `REMOTE 0` returns to local.
- Telemetry `T` line includes `rem_sense=0|1`.
- Open-lead / differential remote ADC readback needs PB0/PB1 added to ADC DMA ranks later; until then remote is a GPIO path switch only.

## Host UART commands (USART1)

| Command | Action |
|---|---|
| `ON` / `OFF` | Enable / disable DCDC |
| `CLR` / `CLEAR` | Clear sticky fault latch |
| `SET <v>` | Set voltage |
| `ILIM <a>` | Set current limit |
| `USB …` | USB PD mode |
| `PERMIT 0\|1` | Force G0 kill assert / clear (**PB7**) |
| `REMOTE 0\|OFF` | Local sense (default) |
| `REMOTE 1\|ON` | Enable remote sense path |
| `TEL` / `?` / `STATUS` | One or periodic `T`/`TB`/`TC` machine frame |
| `BMS` | Soft: skip CFGUPDATE if already healthy; else full 4S reinit |
| `BMS FORCE` / `BMSREINIT` | Full CFGUPDATE + ALL_FETS_ON (may bus-hold + reboot) |
| `VERBOSE 0\|1` | Debug spam on USART1 (default **0** — keep clean for H7) |

## BMS (4S pack, skip VC4)

Hardware is a **4S** Li-ion pack on the BQ76922 (5-channel AFE). Firmware writes `VCell Mode = 0x0017` (cells 1/2/3/5, **skip VC4**) in **RAM on every wake**. Blank OTP defaults to “all cells”; that is why a skipped VC4 used to look like CUV and blocked FETs. **OTP burn is not required** and is not done by this firmware (OTP is one-way / production-line only).

See [HOST_TELEMETRY.md](HOST_TELEMETRY.md) for USART1 `T`/`TB`/`TC` parsing. Charger `BOARD_CHARGER_CELL_COUNT` is already 4.

With `BMS_ENABLE=1`: used cells between CUV (2.8 V) and COV (4.25 V); unused `c4_mv=-1` is expected. Pack warn window is 11–17 V. Telemetry `fets=1` means CHG/DSG are on. **Wake is TS2 button only** (no host command): firmware writes CFGUPDATE (VCell Mode `0x0017`, CFETOFF=0, **CC Gain=7.5684/5**, OCC Recovery=+100 mA, CHG/DSG FET Prot A) then `FET_ENABLE` + `ALL_FETS_ON`. Optional OTP burn so BQ enables FETs alone — see `docs/BQ76922_OTP_GOLDEN.md`. `BOARD_BRINGUP_AUTO_ON=0` — host `ON` is only for the PSU output path. Check `TB`: `init_step`, `vcell_rb=0x0017`, `manuf` bit4, `fet`/`fets`.

## Bring-up sequence

1. Flash G0 + G4. Connect isolator UART (115200).
2. PC on USART1: `SET 5.0`, `ILIM 0.1`, then **`ON`**.
3. G4 asserts `POWER_PERMIT` (**PB7** HIGH) → waits `kill=0` / `pgood=1` / `vin≥4500` → sends `SET V=… I=…` → `OUT ON` to G0.
4. Watch forwarded `TLM` (`out=1 kill=0 outoff=0`) and host `T` (`g0_want=1 g0_ctrl=… g0_out=1`).

Host **`ON`** starts G4 DCDC pre-reg **and** the G0 ASCII sequencer (default `V=5.000 I=0.100` until `SET`/`ILIM`). Host **`OFF`** / **`PERMIT 0`** sends `OUT OFF`, forces **PB7** low (LDO zabity), and stops DCDC.

`g0_ctrl` states: 0 idle, 1 wait link, 2 wait permit, 3 wait VIN, 4–8 SET/OUT handshake, 9 running, 10–11 OFF, 12 fault.

`g0_want=0` means the host has not started the G0 sequencer — send **`ON`** after permit is on the correct pad.

## G0 link triage (`g0_*` on host `T` line)

| Symptom | Meaning |
|---|---|
| `vout_mv≈8000`, `mode=CV`, `g0_tlm=0` | DCDC OK; final LDO not talking / not ON |
| `g0_rx` stuck at 1, `g0_age_ms` climbing | One noise byte then silence — isolator / TX-RX / G0 not streaming |
| `g0_err>0`, `g0_uart=0x…` | Framing/noise (`0x4` FE, `0x8` NE, `0x1` ORE) |
| `g0_tlm` rising, forwarded `TLM …` lines | Link OK — check G0 LED / `kill=` / `pgood=` / `out=` |
| `permit=1` but G0 `kill=1` | Firmware was driving PERMIT on wrong pad (was PB6/REMOTE_ON); must be **PB7** |

Hardware checks: G4 **PB3↔G0 RX**, **PB4↔G0 TX** via ISO6721; J6 sniffer at 115200; G0 LED double-blink = KILL/!PGOOD; meter on LDO Vout (not DCDC rail on PB2).
