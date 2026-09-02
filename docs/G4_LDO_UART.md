# G4 ↔ G0 LDO UART (G474 firmware)

Canonical copy: [LDO_controller `docs/G4_LDO_UART.md`](https://github.com/kavper/LDO_controller/blob/cursor/g0-hw-rev-cubemx-19b5/docs/G4_LDO_UART.md)

## Split of roles

| MCU | Role |
|---|---|
| **G0** | Final **CC/CV** on LDO output, bleed/fan policy, `vpre_request` computation |
| **G4** | **Pre-regulator** DCDC: holds `Vin_LDO` at headroom above G0 output, fan PWM, BLEED_ON, POWER_PERMIT_G4 |

G4 **does not** run CC/CV on the user output. It regulates **ADC_VOUT (PB2)** = DCDC rail feeding the LDO.

## Pre-regulator policy (`ldo_prereg.c`)

Mirrors G0 `control_update_vpre_request()`:

| G0 state | DCDC target |
|---|---|
| `out=0` | disable DCDC, ramp command → 3 V |
| CV (`cccv=0`) | `vset + 3 V` (or `vpre=` from TLM if present) |
| CC (`cccv=1`) | `vout + 3 V` (tracks measured LDO output) |

Constants (match G0 `app_config.h`): min 3 V, max 36 V, margin 3 V.

Slew: up 10 V/s, down 0.3 V/s. **POWER_PERMIT_G4** asserts only after DCDC is within 0.5 V of command for 150 ms (`pgood=1`, `fault=NONE`, G0 `out=1`).

## Pin / module map

| Interface | Pins | Module |
|---|---|---|
| G0 isolated UART | USART3 PB14/PB15 | `ldo_link.c` |
| H7 / PC host | USART1 PC4/PC5 | `host_link.c` |
| Fan PWM | PA6 TIM16 | `fan_pwm.c` |
| BLEED_ON | PB4 | `ldo_link.c` |
| POWER_PERMIT_G4 | PB6 | `ldo_prereg.c` → `ldo_link.c` |
| Local Vout sense | PB2 `ADC_VOUT` | `measurements.c` (CV) |
| Remote sense enable | PB5 `REMOTE_ON` | `ldo_link.c` (default LOW) |
| Remote Kelvin sense | PB0/PB1 `ADC_REMOTE_P/N` | analog inputs (no DMA ranks yet) |

## Local vs remote sense

- **Default at boot:** local only (`REMOTE_ON` = LOW). CV regulation always uses `ADC_VOUT` (PB2).
- Host `REMOTE ON` / `REMOTE 1` asserts `REMOTE_ON` (PB5) to switch the hardware remote sense path.
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
| `PERMIT 0\|1` | Force G0 kill assert / clear |
| `REMOTE 0\|OFF` | Local sense (default) |
| `REMOTE 1\|ON` | Enable remote sense path |
| `TEL` / `?` | Status dump |

## BMS (5S bring-up)

With `BMS_ENABLE=1` and `BMS_CELL_COUNT=5`, equal **5×68 Ω** ladder on a bench supply: pack ≈ 14–21 V keeps cells between CUV (2.8 V) and COV (4.25 V). `BOARD_BRINGUP_AUTO_ON=0` — send host `ON` after BMS comes up. Front buttons off (`BOARD_HAS_FRONT_BUTTONS=0`); ON/OFF is USART1.

## Bring-up sequence

1. Flash G0 + G4. Connect isolator UART (115200).
2. On G0: `OUT ON`, `SET V=… I=…` (or G0 `BOARD_BRINGUP_AUTO_OUT=1`).
3. G4 parses `TLM` @ 5 Hz → enables DCDC → ramps to `vpre` → asserts PERMIT.
4. PC on USART1: forwarded `TLM` lines + `T … vpre_req_mv= … permit= …` from G4.

With `BOARD_BRINGUP_LOCAL_CV` / `BOARD_BRINGUP_AUTO_ON`, G4 can run DCDC CV without G0 TLM (pre-reg rail only). **User LDO output still requires a live G0.**

Host **OFF** / **PERMIT 0** = emergency: force DCDC off and PERMIT low (even if G0 still reports on). Clears when G0 `out=0` or host **ON** + **PERMIT 1**.

## G0 link triage (`g0_*` on host `T` line)

| Symptom | Meaning |
|---|---|
| `vout_mv≈8000`, `mode=CV`, `g0_tlm=0` | DCDC OK; final LDO not talking / not ON |
| `g0_rx` stuck at 1, `g0_age_ms` climbing | One noise byte then silence — isolator / TX-RX / G0 not streaming |
| `g0_err>0`, `g0_uart=0x…` | Framing/noise (`0x4` FE, `0x8` NE, `0x1` ORE) |
| `g0_tlm` rising, forwarded `TLM …` lines | Link OK — check G0 LED / `kill=` / `pgood=` / `out=` |

Hardware checks: G4 **PB14↔G0 RX**, **PB15↔G0 TX** via ISO6721; J6 sniffer at 115200; G0 LED double-blink = KILL/!PGOOD; meter on LDO Vout (not DCDC rail on PB2).
