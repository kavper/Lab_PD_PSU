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
| G0 isolated UART | USART3 **PB9 TX / PB8 RX** | `ldo_link.c` — **not** silk PB14/PB15 (invalid AF on G474) |
| H7 / PC host | USART1 PC4/PC5 | `host_link.c` |
| Fan PWM | PA6 TIM16 | `fan_pwm.c` |
| BLEED_ON | PB4 | `ldo_link.c` |
| POWER_PERMIT_G4 | PB6 | `ldo_prereg.c` → `ldo_link.c` (HIGH=ena, Low/reset=LDO zabity) |
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
2. PC on USART1: `SET 5.0`, `ILIM 0.1`, then **`ON`**.
3. G4 asserts `POWER_PERMIT` (PB6 HIGH) → waits `kill=0` / `pgood=1` / `vin≥4500` → sends `SET V=… I=…` → `OUT ON` to G0.
4. Watch forwarded `TLM` (`out=1 kill=0 outoff=0`) and host `T` (`g0_want=1 g0_ctrl=… g0_out=1`).

Host **`ON`** starts G4 DCDC pre-reg **and** the G0 ASCII sequencer (default `V=5.000 I=0.100` until `SET`/`ILIM`). Host **`OFF`** / **`PERMIT 0`** sends `OUT OFF`, forces PB6 low (LDO zabity), and stops DCDC.

`g0_ctrl` states: 0 idle, 1 wait link, 2 wait permit, 3 wait VIN, 4–8 SET/OUT handshake, 9 running, 10–11 OFF, 12 fault.

## G0 link triage (`g0_*` on host `T` line)

| Symptom | Meaning |
|---|---|
| `vout_mv≈8000`, `mode=CV`, `g0_tlm=0` | DCDC OK; final LDO not talking / not ON |
| `g0_rx` stuck at 1, `g0_age_ms` climbing | One noise byte then silence — isolator / TX-RX / G0 not streaming |
| `g0_err>0`, `g0_uart=0x…` | Framing/noise (`0x4` FE, `0x8` NE, `0x1` ORE) |
| `g0_tlm` rising, forwarded `TLM …` lines | Link OK — check G0 LED / `kill=` / `pgood=` / `out=` |

Hardware checks: G4 **PB9↔G0 RX**, **PB8↔G0 TX** via ISO6721 (rev2 flywire / next PCB); J6 sniffer at 115200; G0 LED double-blink = KILL/!PGOOD; meter on LDO Vout (not DCDC rail on PB2).

## Rev2 UART pin rework (required)

STM32G474 **cannot** map USART3_TX/RX onto PB14/PB15 (PB14 is USART3_RTS only; PB15 has no USART3 data AF). Silk/nets `USART2_*_G0` on those pads were a layout mistake.

| Net | Old pad | New pad | AF |
|---|---|---|---|
| `USART2_TX_G0` (G4→G0) | PB14 | **PB9** | USART3_TX |
| `USART2_RX_G0` (G0→G4) | PB15 | **PB8** | USART3_RX |
| `I2C_USBPD_IRQ` | PB9 | **PB3** | EXTI3 |

Bench flywire: lift isolator MCU-side from PB14/15 → PB9/PB8; move USB-PD IRQ wire PB9→PB3. Leave PB14/PB15 NC.
