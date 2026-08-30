# G4 ↔ G0 LDO UART (G474 firmware)

Canonical copy: [LDO_controller `docs/G4_LDO_UART.md`](https://github.com/kavper/LDO_controller/blob/cursor/g0-hw-rev-cubemx-19b5/docs/G4_LDO_UART.md)

G474 implements the **G4 side** in this repo (`Lab_PD_PSU`, branch `cursor/hw-rev2-gan-bms-0e60`):

| Interface | Pins | Firmware |
|---|---|---|
| G0 isolated UART | **USART3** PB14 TX / PB15 RX | `ldo_link.c` |
| H7 / PC host | **USART1** PC4 / PC5 | `host_link.c` |
| Fan PWM | **PA6** TIM16 | `fan_pwm.c` |
| Fan tach | **PA5** input | future IC |
| BLEED_ON | **PB4** | `ldo_link.c` ← G0 `bleed` |
| REMOTE_ON | **PB5** | held low |
| POWER_PERMIT_G4 | **PB6** | `ldo_link.c` ← DCDC policy |

G0 streams ASCII `TLM ...` every 200 ms (stage 6). G4 parses `bleed` / `fan`, forwards the **raw line unchanged** to USART1 (H7/PC), drives actuators. Do not re-implement bleeder/fan curves on G4.
