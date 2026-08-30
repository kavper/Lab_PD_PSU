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

## Bring-up sequence

1. Flash G0 + G4. Connect isolator UART (115200).
2. On G0: `OUT ON`, `SET V=… I=…`.
3. G4 parses `TLM` @ 5 Hz → enables DCDC → ramps to `vpre` → asserts PERMIT.
4. PC on USART1: forwarded `TLM` lines + `T … vpre_req_mv= … permit= …` from G4.

Host **OFF** / **PERMIT 0** = emergency: force DCDC off and PERMIT low (even if G0 still reports on). Clears when G0 `out=0` or host **ON** + **PERMIT 1**.
