# G4 ↔ G0 LDO UART (G474 firmware)

Canonical wire format: matching G0 repository
`docs/G4_G0_UART_PROTOCOL_V2.md`. Production G4↔G0 traffic is binary with
CRC-16, sequence matching and ACK/NACK; old ASCII `TLM` is obsolete.

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
| `out=0`, host wants ON | `max(host_vset, tlm_vset) + 1.5 V`, floored at **6 V** |
| CV (`mode=1`) | `max(vpre, vset + 1.5 V, 6 V)` |
| CC (`mode=2`, filtered) | G0 `vpre` = measured `vout + 1.5 V`, clamped to `[6 V, vset + 1.5 V]`. Do **not** lift that request back to `vset + 1.5 V` — the difference would sit on the LDO. A raw `cccv` blip does not fold. |
| Stale TLM while `g0_want=1` | **hold** CV/VIN floor (do not dive to 3 V) |

`fault=VIN_LOW` does **not** disable the pre-reg DCDC (that fault is caused by a low rail; killing DCDC worsens the spiral). Other faults still drop enable.

Constants (match G0 `app_config.h`): min 3 V, max 36 V, margin 1.5 V, **VIN floor 6 V**.

Slew: up 10 V/s, down 0.3 V/s while leaving the rail or holding the CV floor. Confirmed CC folds down at 5 V/s and still never commands below 6 V while output is wanted or on. Host ON starts the DCDC with the LDO output still off. **POWER_PERMIT_G4** asserts only after that stage is already running and has stayed within 0.5 V of command for 150 ms. The WAIT_PERMIT state waits for that grant; it does not drive PB7 itself.

## Pin / module map (schematic U7)

| Interface | Pins | Module |
|---|---|---|
| G0 isolated UART | USART2 **PB3 TX / PB4 RX** AF7 | `ldo_link.c` |
| H7 / PC host | USART1 PC4/PC5 | `host_link.c` |
| Fan PWM | **PA7** TIM17_CH1 AF1 | `fan_pwm.c` (Q9 inverts; PA6 is NC) |
| Fan tach | **PA5** TIM2_CH1 AF1 | `fan_tach.c`, 2 pulses/rev |
| BLEED_ON | **PB5** | `ldo_link.c` |
| REMOTE_ON | **PB6** | `ldo_link.c` (default LOW) |
| POWER_PERMIT_G4 | **PB7** | `ldo_prereg.c` → `ldo_link.c` (HIGH=ena, Low/reset=LDO zabity) |
| I2C_USBPD_IRQ | **PB9** | EXTI |
| Local Vout sense (DCDC) | PB2 `ADC_VOUT` | `measurements.c` (CV) |
| ADC_LOCAL_VOUT | **PB14** ADC1_IN5 | injected self-test, 220 kΩ / 20 kΩ |
| I_L_ZERO | **PB15** | analog (no DMA rank yet) |
| Remote Kelvin sense | PB0 ADC1_IN15 / PB1 ADC1_IN12 | injected self-test, same divider |
| PA2 | NC | — |

## Local vs remote sense

- **Default at boot:** local only (`REMOTE_ON` = LOW). The DCDC still regulates from `ADC_VOUT` (PB2). K1 switches the LDO Kelvin sense, not that ADC.
- Host `REMOTE ON` / `REMOTE 1` requests remote sense. PB0/PB1 stay on the remote wires, so the check runs while K1 is still local. The relay stays off until three samples, 100 ms apart, read OK and agree within the ADC error.
- The input filter is about 0.19 ms, so a 100 ms sample is settled. The check is DP = local − remote_p, DN = remote_n, and DP+DN, plus a saturated local reading. Cable limits (500 mV per wire, 1000 mV total) and the per-channel error (1.85% and 40 mV, no stored trim) are separate. The 500/1000 mV pair is a bench starting point. The low-voltage floor falls out of that budget (592 mV at the 500 mV wire limit).
- R112 (4.7 kΩ) holds an open positive lead at about 98.1% of Vout. R115 holds an open negative lead at ground. That pair is reported as OK. It is not a detected break, before or after K1 closes.
- A wiring fault before close leaves the relay local and leaves PERMIT alone. After close, a drop fault, a missed conversion, or (in CV only) VD missing the setpoint for three samples opens the relay, drops PERMIT, and latches remote off until the host sends REMOTE 0 and then REMOTE 1. CC does not compare VD with the voltage setpoint.
- Host `REMOTE OFF` / `REMOTE 0` releases the relay immediately and clears the latch. The next host ON clears the permit override.

## Fan

G0 sends `fan` as 0..100. That number is already the higher of the output-power map (0 W → 0 %, 150 W → 100 %) and the NTC map (25 °C → 0 %, 60 °C → 100 %). G4 copies it onto PA7 and does not draw its own curve. If G0 telemetry is older than 500 ms the pin is forced to 40 %.

Q9 inverts PA7 onto J7 pin 4. A 4-wire fan runs while that pin is high, so 0 % holds PA7 high (fan stopped) and 100 % holds PA7 low (fan pull-up, full speed).

PA5 counts falling edges on the open-collector tach (two per revolution) for one second. RPM goes out on AUX byte 28 (`0xFFFF` until the first second closes, `0` when the fan is stopped or the tach wire is open). Bytes 30..31 stay zero.

## Host UART commands (USART1)

| Command | Action |
|---|---|
| `ON` / `OFF` | Enable / disable DCDC |
| `CLR` / `CLEAR` | Clear sticky fault latch |
| `SET <v>` | Legacy/manual voltage command, strict 0..27 V |
| `ILIM <a>` | Legacy/manual current command, strict 0..5 A |
| `SET V=<v> I=<a>` | Atomic GUI voltage/current, three decimal places |
| `USB …` | USB PD mode |
| `PERMIT 0\|1` | Force G0 kill assert / clear (**PB7**) |
| `REMOTE 0\|OFF` | Local sense (default) |
| `REMOTE 1\|ON` | Enable remote sense path |
| `TEL` / `?` / `STATUS` | One or periodic `T`/`TB`/`TC` machine frame. `TEL < 200` keeps `T` fast and `TB`/`TC` at 200 ms. |
| `BMS` | Soft: skip CFGUPDATE if already healthy; else full 4S reinit |
| `BMS FORCE` / `BMSREINIT` | Full CFGUPDATE + ALL_FETS_ON (may bus-hold + reboot) |
| `VERBOSE 0\|1` | Debug spam on USART1 (default **0** — keep clean for H7). `VERBOSE 1` also mirrors G0 TLM/ACK onto USART1. |

## BMS (4S pack, skip VC4)

Hardware is a **4S** Li-ion pack on the BQ76922 (5-channel AFE). Firmware writes `VCell Mode = 0x0017` (cells 1/2/3/5, **skip VC4**) in **RAM on every wake**. Blank OTP defaults to “all cells”; that is why a skipped VC4 used to look like CUV and blocked FETs. **OTP burn is not required** and is not done by this firmware (OTP is one-way / production-line only).

See [HOST_TELEMETRY.md](HOST_TELEMETRY.md) for USART1 `T`/`TB`/`TC` parsing. Charger `BOARD_CHARGER_CELL_COUNT` is already 4.

With `BMS_ENABLE=1`: used cells between CUV (2.8 V) and COV (4.25 V); unused `c4_mv=-1` is expected. Pack warn window is 11–17 V. Telemetry `fets=1` means CHG/DSG are on. **Wake is TS2 button only** (no host command): firmware writes CFGUPDATE (VCell Mode `0x0017`, CFETOFF=0, **CC Gain=7.5684/5**, OCC Recovery=+100 mA, CHG/DSG FET Prot A) then `FET_ENABLE` + `ALL_FETS_ON`. Optional OTP burn so BQ enables FETs alone — see `docs/BQ76922_OTP_GOLDEN.md`. `BOARD_BRINGUP_AUTO_ON=0` — host `ON` is only for the PSU output path. Check `TB`: `init_step`, `vcell_rb=0x0017`, `manuf` bit4, `fet`/`fets`.

## Bring-up sequence

1. Flash G0 + G4 together. Isolator UART is 460800 8N1, DMA, telemetry every 5 ms.
2. PC on USART1: `SET 5.0`, `ILIM 0.1`, then **`ON`**.
3. G4 starts the DCDC with the LDO output off. After the rail stays within 0.5 V of command for 150 ms, G4 asserts `POWER_PERMIT` (**PB7** HIGH), then waits for `kill=0` / `pgood=1` / `vin≥4500` before binary SETPOINT and SET_OUTPUT=1.
4. Watch host `T` (`g0_vout_mv`, `g0_want=1 g0_ctrl=… g0_out=1`). G0 `TLM` stays on USART2 and is **not** forwarded to USART1 unless `VERBOSE 1`.

Host **`ON`** starts G4 DCDC pre-reg and the G0 binary sequencer. Host **`OFF`** / **`PERMIT 0`** sends binary output-off, forces **PB7** low, and stops DCDC.

`g0_ctrl` states: 0 idle, 1 wait link, 2 wait permit, 3 wait VIN, 4–8 SET/OUT handshake, 9 running, 10–11 OFF, 12 fault.

`g0_want=0` means the host has not started the G0 sequencer — send **`ON`** after permit is on the correct pad.

## Fan pins and HRTIM FLT3

`FanPwm_Init` and `FanTach_Init` program the timers. Cube must not also emit `MX_TIM17_Init` or `MX_TIM2_Init`.

| Pin | Code | `.ioc` |
|---|---|---|
| PA7 `FAN_PWM` | TIM17_CH1, AF1, 25 kHz, inverted by Q9 | `TIM17_CH1`, `GPIO_AF1_TIM17` |
| PA5 `FAN_TACH` | TIM2_CH1, AF1, falling edges, pull-up | `TIM2_CH1`, `GPIO_AF1_TIM2`, pull-up |

PB10 stays `HRTIM1_FLT3` in the `.ioc` (digital input, pull-up, polarity active-low, fault armed on timers A and C). `BoardMx_ApplyHrtimFault()` then disables fault mode and clears `FLT3EN` on timers A and C. The pin is the ACS37100 series-inductor FAULT. The net and the active level were not measured on this board, so FLT3 stays off. High-side INA296 OCP remains the current protection. Do not enable FLT3 until that source and polarity are checked.

## G0 link triage (`g0_*` on host `T` line)

| Symptom | Meaning |
|---|---|
| `vout_mv≈8000`, `mode=CV`, `g0_tlm=0` | DCDC OK; final LDO not talking / not ON |
| `g0_rx` stuck at 1, `g0_age_ms` climbing | One noise byte then silence — isolator / TX-RX / G0 not streaming |
| `g0_err>0`, `g0_uart=0x…` | HAL UART error latch: `0x1` PE, `0x2` NE, `0x4` FE, `0x8` ORE |
| `g0_tlm` rising, `g0_vout_mv` tracking | Link OK — check G0 LED / `g0_kill` / `pgood` via G0 TLM on USART2, `g0_out` on `T` |
| `permit=1` but G0 `kill=1` | Firmware was driving PERMIT on wrong pad (was PB6/REMOTE_ON); must be **PB7** |

Hardware checks: G4 **PB3↔G0 RX**, **PB4↔G0 TX** via ISO6721; J6 sniffer at 460800; G0 LED double-blink = KILL/!PGOOD; meter on LDO Vout (not DCDC rail on PB2).
