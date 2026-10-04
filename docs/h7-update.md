# H7: binarny UART7 zamiast ASCII 115200

Dotyczy `GUI_Lab_PD_PSU` (commit bazowy `b5b317a`). Firmware H7 nie jest w tym zadaniu zmieniane. G4 (`Lab_PD_PSU`, USART1 PC4 TX / PC5 RX) już nadaje i odbiera dokładnie ten format. Poniższy opis wystarcza, żeby podmienić łączność bez czytania diffa G4.

G4 publikuje METER co 5 ms własnym timerem. H7 nie ustawia tego okresu i nie wysyła `TEL`.

## Co wyłączyć

Produkcyjny tor przestaje być ASCII. Przestań składać i parsować linie `SET V=… I=…`, `ON`, `OFF`, `TEL`, `STATUS` oraz `T` / `TB` / `TC`.

W `Appli/Core/Inc/g4_ascii.h` i `g4_ascii.c` kontrakt 115200 jest nieaktualny. Nie używaj go jako szybkiej ścieżki:

- `g4_rx_bytes` — parser linii
- `g4_pop_tx` / `g4_process` — kolejka tekstu i sesja startowa, która prosi o telemetrię
- `g4_set_limits`, `g4_set_mv`, `g4_ilim_ma` — osobne komendy ASCII; zastępuje je jedna ramka `SET`
- `g4_on`, `g4_off`, `g4_permit`, `g4_remote`, `g4_usb_role`, `g4_simple`, `g4_enqueue`
- `G4_SETPOINT_GAP_MS` (120) — nie ma odstępu między SET
- `G4_CMD_TIMEOUT_MS` (2000) — timeout SET to 800 ms
- `G4_LINE_MAX` / `G4_RX_LINE_MAX` jako granica ramki telemetrycznej

W `g4_uart.c` wyłącz odbiór `HAL_UARTEx_ReceiveToIdle_IT` i nadawanie `HAL_UART_Transmit_IT` całej linii. Zostaw nazwy `G4_UartInit`, `G4_UartProcess`, `G4_UartRxEvent`, `G4_UartTxComplete`, `G4_UartError`, ale niech wołają DMA opisane niżej. Callbacki w `main.c` (`HAL_UARTEx_RxEventCallback`, `HAL_UART_TxCpltCallback`, `HAL_UART_ErrorCallback`, ok. linii 689–702) mogą zostać; zdarzenie RX jest tylko kopnięciem, parser czyta bufor w zadaniu.

W `psu_app_tick` przestań składać `G4Telemetry` z linii `T` (`g4_record_value` na `G4_RECORD_T/TB/TC`). `ldo_protocol.h` to starsza, inna struktura LDO — nie jest tym łączem.

`psu_app_set_limits` ma wysłać jeden binarny `SET`. `psu_app_set_output` ma wysłać `ON` albo `OFF`, a nie `g4_on` / `g4_off`. `psu_app_clear_fault` → `CLEAR`. `psu_app_bms_cmd` → `BMS`. `psu_app_usb_role` → `USB`.

Krótki ASCII może zostać tylko jako ramka `TEXT` / `TEXT_CMD` (diagnostyka na żądanie). Nie może jechać co 5 ms i nie może blokować DMA.

## Warstwa fizyczna

- UART7, piny bez zmian: PE7 RX, PE8 TX, AF7 (`GPIO_AF7_UART7`), zegar PCLK1, jak w `HAL_UART_MspInit`.
- 460800 8N1, oversampling 16, preskaler 1, bez flow control, pełny dupleks.
- G4 (170 MHz, BRR 369) ma błąd ok. −0,021%. G0 (64 MHz, BRR 139) ok. −0,080%. Na H7 policz `UARTDIV = f_PCLK1 / 460800` i zaokrąglij BRR tak, żeby błąd został poniżej 2%.
- G4 i G0 nie mają D-cache. H7 ma (`SCB_EnableDCache()` w `main.c`).

## Koperta ramki

Little-endian. Każda ramka jest kompletna i ma co najwyżej 120 bajtów.

| Offset | Bajty | Znaczenie |
|---:|---:|---|
| 0 | `A5` | SOF1 |
| 1 | `5A` | SOF2 |
| 2 | `LEN` | liczba bajtów `TYPE + SEQ + PAYLOAD` (2…115) |
| 3 | `TYPE` | typ |
| 4 | `SEQ` | numer transakcji, 0…255, zawija się |
| 5 … 4+LEN-2 | payload | `LEN - 2` bajtów |
| ostatnie dwa | CRC16 LE | CRC-16/CCITT-FALSE, poly `0x1021`, init `0xFFFF` |

CRC liczone od bajtu `LEN` włącznie do ostatniego bajtu payloadu (bez SOF i bez samego CRC). Złe CRC albo urwana ramka nie ma żadnego skutku: brak ACK, brak NACK, brak zmiany wyjścia.

Całkowita długość = `LEN + 5` ≤ 120, więc payload ≤ 113. Szybkie ramki produkcyjne są krótsze (METER ma 79 B).

### Przykład SET

`SET`, SEQ `7`, 5000 mV, 100 mA. 15 bajtów:

```
A5 5A 0A 01 07 88 13 00 00 64 00 00 00 5D 97
```

| Bajty | Pole |
|---|---|
| `A5 5A` | SOF |
| `0A` | LEN = 10 |
| `01` | TYPE = SET |
| `07` | SEQ |
| `88 13 00 00` | 5000 mV |
| `64 00 00 00` | 100 mA |
| `5D 97` | CRC16 LE |

ACK tego SET (8 B): `A5 5A 03 81 07 01 C0 9A` — payload to jeden bajt `01` (typ, który potwierdzamy).

NACK timeout (9 B): `A5 5A 04 82 07 01 06 3D 7E` — payload `01 06` = SET + reason TIMEOUT.

OFF, SEQ 4, pusty payload (7 B): `A5 5A 02 03 04 2B B7`.

## Typy H7 → G4

| TYPE | Nazwa | Payload |
|---:|---|---|
| `0x01` | SET | `u32 mV`, `u32 mA` (dokładnie 8 B). Zakres 0…27000 mV i 0…5000 mA. Poza zakresem NACK reason 3, bez zapamiętania efektu. |
| `0x02` | ON | pusty |
| `0x03` | OFF | pusty |
| `0x04` | CLEAR | pusty |
| `0x05` | PING | pusty |
| `0x07` | DIAG | pusty. Odpowiedź to TEXT, nie ACK. |
| `0x08` | PERMIT | `u8`: 0 wyłącza (jak OFF), 1 zdejmuje lokalną blokadę permit |
| `0x09` | REMOTE | `u8`: 0 local, 1 remote sense |
| `0x0A` | BMS | `u8`: 0 soft (pomija CFGUPDATE, gdy AFE jest zdrowe), 1 force |
| `0x0B` | USB | `u8`: 0 auto, 1 sink, 2 source |
| `0x21` | TEXT_CMD | ASCII bez wymaganego CR/LF, 1…96 B. G4 wykonuje linię serwisową (HELP, OTP, G0DIAG, G0SWAP) i zwraca ACK tego typu plus ewentualne TEXT. To nie jest tor 5 ms. OTP/SHUTDOWN na G4 może na chwilę zablokować zadanie (`HAL_Delay`). |

Zły rozmiar payloadu → NACK reason 2. Nieznany typ → NACK reason 1.

## Typy G4 → H7

| TYPE | Nazwa | Payload | Okres |
|---:|---|---:|---|
| `0x10` | METER | 72 B, ramka 79 B | 5 ms, timer G4 |
| `0x11` | BMS_TLM | 72 B | 200 ms |
| `0x12` | PD_TLM | 64 B | 200 ms |
| `0x13` | AUX_TLM | 32 B | 200 ms |
| `0x20` | TEXT | 1…96 B ASCII | najniższy priorytet, diagnostyka |
| `0x81` | ACK | `u8` typ | po przyjęciu komendy |
| `0x82` | NACK | `u8` typ, `u8` reason | j.w. |

Reason NACK: `1` UNKNOWN, `2` BAD_PAYLOAD, `3` RANGE, `4` UNSAFE, `5` BUSY, `6` TIMEOUT, `7` LINK.

G4 nie nadrabia pominiętych okresów. Gdy TX jest zajęty, nowszy METER nadpisuje starszy, jeszcze nieoddany do DMA. Rozpoczęte DMA nie jest przerywane. Kolejność nadawania G4: safety/OFF, potem ACK/NACK, potem najnowszy METER, potem BMS, potem PD, potem AUX, na końcu TEXT.

### METER, payload 72 B

Wiek `g0_age_ms` liczy się od ostatniej poprawnej ramki telemetrycznej G0, nie od retransmisji. `0xFFFF` = G0 jeszcze nic nie przysłał. Wartość jest nasycona do `0xFFFE`.

| Offset | Typ | Pole |
|---:|---|---|
| 0 | u32 | `vin_mv` — wejście DCDC G4 |
| 4 | u32 | `vout_mv` — szyna DCDC G4 (to nie jest wyjście LDO) |
| 8 | i32 | `i_buck_ma` |
| 12 | i32 | `i_boost_ma` |
| 16 | u32 | `set_mv` — cel G0 trzymany przez G4 |
| 20 | u32 | `ilim_ma` — limit G0 trzymany przez G4 |
| 24 | u32 | `g0_vout_mv` — zmierzone wyjście LDO |
| 28 | u32 | `g0_iout_ma` |
| 32 | u32 | `g0_vin_mv` |
| 36 | u32 | `g0_vset_mv` |
| 40 | u32 | `g0_iset_ma` |
| 44 | u32 | `vpre_req_mv` |
| 48 | u32 | `vpre_cmd_mv` |
| 52 | u16 | `g0_age_ms` |
| 54 | u16 | `duty_a_x10` — buck, procent×10 (410 = 41,0%) |
| 56 | u16 | `duty_c_x10` — boost |
| 58 | u32 | `g0_fault` |
| 62 | u32 | `psu_fault` |
| 66 | u8 | `flags0` |
| 67 | u8 | `flags1` |
| 68 | u8 | `g0_ctrl` |
| 69 | u8 | `g0_mode` — 0 OFF, 1 CV, 2 CC |
| 70 | u8 | `set_phase` — 0 idle, 1 komenda u G0, 2 jeden SET czeka |
| 71 | u8 | `g0_stale` — 1 gdy brak ważnej telemetrii G0 albo wiek > 500 ms |

`flags0`: bit0 `g0_out`, 1 `g0_want`, 2 `g0_kill`, 3 `g0_outoff`, 4 `g0_cc`, 5 `permit`, 6 `run`, 7 `reg_ok`.

`flags1`: bit0 `stage_en`, 1 `ps_en`, 2 `ps_fault`, 3 `rem_sense`, 4 `ucc_a`, 5 `ucc_c`, 6 `g0_valid`, 7 `fault_latch` (`g0_ctrl == FAULT`).

`g0_ctrl`: 0 IDLE, 1 WAIT_LINK, 2 WAIT_PERMIT, 3 WAIT_VIN, 4 SEND_SET, 5 WAIT_SET_ACK, 6 WAIT_VOUT_ZERO, 7 SEND_OUT_ON, 8 WAIT_OUT_ON_ACK, 9 RUNNING, 10 SEND_OUT_OFF, 11 WAIT_OUT_OFF_ACK, 12 FAULT.

Bity `g0_fault`: 0 HW_INIT, 1 PGOOD_LOST, 2 POWER_KILL, 3 VIN_LOW, 4 VOUT_HARD, 5 VOUT_HIGH, 6 TEMP_HIGH, 7 IOUT_HARD.

Przykład METER, SEQ 3, ramka 79 B. Szyna 6,5 V, cel 5,000 V / 1,000 A, `g0_age_ms = 4`, duty 41,0% i 12,0%, `flags0 = 0xE3` (out, want, permit, run, reg_ok), `flags1 = 0x43` (stage, ps_en, g0_valid), `g0_ctrl = 9` (RUNNING), `g0_mode = 1` (CV), faza 0, nie stale:

```
A5 5A 4A 10 03
E0 2E 00 00  64 19 00 00  FA 00 00 00  00 00 00 00
88 13 00 00  E8 03 00 00  74 13 00 00  C8 00 00 00
00 19 00 00  88 13 00 00  E8 03 00 00  64 19 00 00
64 19 00 00  04 00  9A 01  78 00
00 00 00 00  00 00 00 00
E3 43 09 01 00 00
06 1F
```

Ostatnie dwa bajty to CRC. `vout_mv` UI bierz z `g0_vout_mv` (offset 24), nie z offsetu 4.

### BMS_TLM, payload 72 B

| Offset | Typ | Pole |
|---:|---|---|
| 0 | u8 | present (AFE włączone i widoczne) |
| 1 | u8 | configured |
| 2 | u8 | state |
| 3 | u8 | alert |
| 4 | u32 | fault |
| 8 | u16 | alarm_status |
| 10 | u8 | safety A |
| 11 | u8 | safety B |
| 12 | u8 | safety C |
| 13 | u8 | fet_status |
| 14 | u16 | manuf_status |
| 16 | u8 | init_step |
| 17 | u8 | cfg_fail_count |
| 18 | u16 | vcell_mode |
| 20 | u16 | battery_status |
| 22 | u8 | chg FET |
| 23 | u8 | dsg FET |
| 24 | u8 | fets enabled |
| 25 | u8 | series |
| 26, 28, 30, 32, 34 | i16 LE | c1…c5 mV. Bitowy wzorzec; c4 = −1 to `FF FF` (ogniwo pominięte w 4S) |
| 36 | i16 | min_mv |
| 38 | i16 | max_mv |
| 40 | i16 | dV_mv |
| 42 | u16 | sum_mv |
| 44 | u16 | pack_mv |
| 46 | u16 | stack_mv |
| 48 | i32 | i_pack_ma (CC2) |
| 52 | u8 | sample_valid |
| 53…55 | — | zera |
| 56 | i32 | passq_mah — odczyt DASTATUS6, mAh ze znakiem. To jest licznik ładunku |
| 60 | i32 | zero. G4 nie prowadzi osobnej całki CC2 |
| 64 | u16 | soc_permille. `0xFFFF` = jeszcze nieoszacowany. Po kalibracji napięciem SOC rusza się różnicą passQ |
| 66 | i16 | cc1_ma |
| 68 | i16 | int_temp_dK — temperatura struktury AFE, 0,1 K; `0` = nieodczytana |
| 70 | u8 | balance_mask. Bit0 = ogniwo 1. Jedno ogniwo, tylko RAM `CB_ACTIVE_CELLS` |
| 71 | u8 | soc_flags |

`soc_flags`: bit0 SOC ważny, 1 odpoczynek, 2 pojemność nauczona, 3 passQ ważny, 4 balans włączony.

UI czterech cel: tak jak dziś, pomiń ogniwo ≤ 0 (fizyczne odczepy 1, 2, 3, 5).

### AUX_TLM, payload 32 B

Bajty 0…19 są takie jak przy payloadzie 24 B. Potem jest self-test pomiaru zdalnego, a na bajtach 28…29 obroty wentylatora. Bajty 30…31 zostają zerami. Ramka ma 39 B (`LEN` = 34).

Temperatury to już przeliczone NTC z G0, int16 w dziesiątych stopnia Celsjusza (`253` = 25,3 °C). To nie są kody ADC. `INT16_MIN` (`0x8000`) = brak pomiaru. Źródło na G0: payload telemetrii `0x80`, offsety 56, 58, 60, 62 (po surowym i filtrowanym ADC, których H7 nie dostaje).

Napięcia sense są po dzielniku 220 kΩ / 20 kΩ (×12) i referencji 3,0 V. `0xFFFF` = brak próbki. Przekaźnik K1 klika dopiero po trzech zgodnych próbkach OK. `flags1` bitu remote w METER oznacza, że cewka już jest załączona, nie że H7 o to poprosiło.

| Offset | Typ | Pole |
|---:|---|---|
| 0 | u32 | `dac_cv_mv` — odczyt DAC CV z G0 |
| 4 | u32 | `dac_cc_mv` — odczyt DAC CC z G0 |
| 8 | i16 | T1 MOSFET, °C×10 |
| 10 | i16 | T2 otoczenie, °C×10 |
| 12 | i16 | T3 bleeder, °C×10 |
| 14 | i16 | T4 okolica 3,3 V LDO / 15→5 V, °C×10 |
| 16 | u8 | fan, procent. Wyższe z mapy mocy (0 W = 0 %, 150 W = 100 %) i mapy temperatury (25 °C = 0 %, 60 °C = 100 %) |
| 17 | u8 | PGOOD |
| 18 | u8 | bleed — 1 także przy wyłączonym wyjściu, gdy Vout < 4 V, i przez cały czas gdy wyjście jest off |
| 19 | u8 | valid — 1 gdy telemetria G0 jest żywa |
| 20 | u16 | `local_mv` — ADC_LOCAL_VOUT (PB14) |
| 22 | u16 | `remote_p_mv` — REMOTE_P (PB0) |
| 24 | u16 | `remote_n_mv` — REMOTE_N (PB1) |
| 26 | u8 | `sense_code` |
| 27 | u8 | `sense_flags`: bit0 cewka K1 załączona, bit1 H7/host prosi o remote |
| 28 | u16 | `fan_rpm`. Dwa zbocza opadające na obrót. `0` = stoi albo tachometr nie daje impulsów. `0xFFFF` = pierwsza sekunda pomiaru jeszcze nie minęła |
| 30…31 | — | zera |

`fan_rpm` jest liczone na G4 z PA5, bramka 1 s, i wkładane w tę samą wolną ramkę AUX (200 ms) co przekaźnik. Między bramkami H7 dostaje ostatnią wartość. PWM na PA7 jest odwrócony tranzystorem Q9: 0 % zatrzymuje wentylator, 100 % puszcza go na pełne obroty.

`sense_code`:

| Kod | Nazwa | Co pokazać |
|---:|---|---|
| 0 | OK | przewody wyglądają dobrze; po trzech próbkach wolno kliknąć przekaźnik |
| 1 | NOT_READY | Vout poniżej 2 V, za mało żeby ocenić przewody |
| 2 | OPEN | oba przewody przy masie: odpięte albo brak plusa |
| 3 | OPEN_P | plus odpięty, minus nie jest ani przy masie, ani przy Vout |
| 4 | REVERSED | przewody zamienione (minus widzi Vout) |
| 5 | N_ON_POUT | oba przewody na plusie wyjścia |
| 6 | P_MISMATCH | plus nie jest ani przy Vout, ani odpięty |
| 7 | N_HIGH | plus pasuje, minus jest wyraźnie nad masą |
| 8 | NO_SAMPLE | ADC jeszcze nic nie zmierzył |

Odpięty minus przy braku prądu czyta się tak samo jak minus podłączony do masy obciążenia, więc ten jeden przypadek wchodzi w OK. Zamiana przewodów i brak plusa przekaźnika nie puszczają.

### PD_TLM, payload 64 B

| Offset | Typ | Pole |
|---:|---|---|
| 0 | u8 | bq_ok (online i próbka ADC ważna) |
| 1 | u8 | flagi: bit0 input, 1 precharge, 2 fast, 3 otg, 4 iindpm, 5 vindpm, 6 reset_busy, 7 attached |
| 2 | u16 | charger_status |
| 4 | u8 | fault |
| 5 | u8 | cc1 |
| 6 | u8 | cc2 |
| 7 | u8 | role TPS |
| 8 | u8 | connection_state |
| 9 | u8 | typec_port_state |
| 10 | u8 | pd power_role |
| 11 | u8 | 0 |
| 12 | u32 | vbat_mv |
| 16 | u32 | vsys_mv |
| 20 | i32 | ibat_ma |
| 24 | u32 | ichg_ma |
| 28 | u32 | idchg_ma |
| 32 | u32 | vbus_mv |
| 36 | u32 | iin_ma |
| 40 | u32 | vreg_mv |
| 44 | u32 | ichg_set_ma |
| 48 | u32 | iin_set_ma |
| 52 | u32 | tps_vbus_mv |
| 56 | u32 | pd_mv |
| 60 | u32 | pd_ma |

## DMA i cache na H7

Zostaw piny i AF. Zmień tylko baud i transport.

1. Bufor RX: DMA kołowy, rozmiar wielokrotność 32 (np. 256 lub 512). Wyrównanie 32 B (`aligned(32)`). Adres i długość przekazane do czyszczenia cache też wyrównane do 32 w górę.
2. Bufor TX: zwykły (nie kołowy) DMA, ten sam wymóg wyrównania. Jedna ramka ≤ 120 B, więc bufor 128 B jest wygodny.
3. Start: `HAL_UARTEx_ReceiveToIdle_DMA`. Zostaw przerwania HT i TC (`DMA_IT_HT | DMA_IT_TC`) oraz IDLE w UART. W przerwaniu tylko ustaw flagę. Nie parsuj w ISR.
4. W zadaniu, zanim odczytasz nowe bajty: `SCB_InvalidateDCache_by_Addr` na zakres obejmujący te bajty (wyrównany). Potem pozycja zapisu = `rozmiar - NDTR`. Parser jest poza ISR.
5. Zanim wystartujesz TX DMA: zapisz ramkę do bufora, potem `SCB_CleanDCache_by_Addr`, potem `HAL_UART_Transmit_DMA`. Nie wołaj `HAL_UART_Transmit` (blokuje 5 ms).
6. Nie przerywaj DMA, które już wystartowało (`HAL_UART_AbortTransmit` nie służy do wepchnięcia nowszego SET).
7. Błąd UART: zlicz, wyczyść flagi, uruchom RX od nowa. Jeśli TX nadal jest `BUSY_TX`, nie zeruj mu stanu — dokończy się.
8. AXI SRAM `0x24000000` w `MPU_Config` (region 5) jest cacheowalny. Bufory DMA leżące tam wymagają invalidate/clean. Nie wyłączaj D-cache globalnie.

G4 nie ma D-cache, więc po swojej stronie nie robi tych operacji. H7 musi.

## Maszyna SET

Jedna transakcja SET w locie i jeden nadpisany oczekujący komplet V+I. Nie ma `LDO_LIVE_SET_MIN_MS` ani `G4_SETPOINT_GAP_MS`.

1. Użytkownik zmienia napięcie i prąd. Złóż jedno 8-bajtowe V+I. Jeśli nic nie leci, nadaj `SET` z nowym SEQ i zapamiętaj go jako in-flight.
2. Jeśli SET już leci, zapisz nową parę V+I w jednym slocie pending razem z jego SEQ. Kolejny ruch suwaka nadpisuje ten slot. Nie dokładaj kolejki.
3. ACK/NACK przychodzi z tym samym SEQ. ACK znaczy: G0 przyjął cel rampy. Nie znaczy, że napięcie na wyjściu już doszło. Rampy zostają 50 mV/ms i 10 mA/ms po stronie G0; H7 ich nie egzekwuje.
4. Po ACK/NACK, jeśli pending jest niepusty, wyślij go (to już inny SEQ).
5. Timeout 800 ms bez ACK/NACK tego SEQ to awaria (reason 6), nie przerwa między suwakami. Wolno retransmitować ten sam SEQ. G4 odda zapamiętany wynik i nie zastosuje setpointu drugi raz. Nie wolno w tym celu brać nowego SEQ — nowy SEQ to nowy efekt.
6. G4 wysyła ACK do H7 dopiero po ACK z G0. Do tego czasu `set_phase` w METER jest 1 (w locie) albo 2 (pending czeka, a na drucie jest starszy SET).

ON, OFF, CLEAR i bezpieczeństwo nie są latest-wins.

- OFF: wyślij od razu, także gdy SET leci. G4 zdejmuje permit lokalnie natychmiast i ACK OFF oznacza przyjęcie wyłączenia, nie `vout == 0`.
- ON: jeden w locie. ACK, gdy `g0_ctrl == 9` (RUNNING); jeśli już RUNNING, ACK wraca od razu. G4 czeka na to do 8 s, potem NACK TIMEOUT i sam gasi wyjście. Nie kolejkuj kilku ON. Retry = ten sam SEQ.
- PERMIT 0 działa jak lokalne wyłączenie i dostaje ACK od razu.
- Ten sam SEQ zakończonej komendy (ON, OFF, CLEAR, PING, PERMIT, REMOTE, BMS, USB, SET) wraca jako zapamiętany ACK/NACK bez powtórzenia efektu.

## UI, stale, reconnect

- Rysuj UI co około 33 ms z ostatniego poprawnego snapshotu (`psu_snapshot` już jest). Nie czekaj na UART w wątku rysowania i nie parsuj `T` przy każdym odświeżeniu.
- Wiek danych = czas od ostatniej ramki z dobrym CRC, nie od powtórki tego samego SEQ.
- METER starszy niż 50 ms: łącz G4↔H7 jest stale (wygaszenie żywych mierników). BMS/PD/AUX starsze niż 1000 ms: wygaszenie tych paneli, METER może dalej być żywy.
- `g0_stale == 1` albo `g0_age_ms > 500`: dane LDO nieaktualne. G4 po 500 ms ważnej, a potem urwanej telemetrii G0 sam zdejmuje permit i nie włącza wyjścia ponownie.
- Po powrocie ramek nie wysyłaj `ON`. Nie ustawiaj `output_requested` z powrotem na 1 tylko dlatego, że łącze wróciło. `flags0.g0_want` będzie 0, dopóki użytkownik nie wyda nowego ON.
- Dzisiejsze `psu_app_tick` przy utracie G0 woła `psu_app_set_output(0, …)` (ok. linii 611). Zostaw to jako OFF. Usuń każdą ścieżkę, która po reconnect albo po nowej linii `T` sama woła `g4_on` / `psu_app_set_output(1)`. Jednorazowy `cold_output_off` przy starcie (OFF, nie ON) może zostać.
- Suwak nie jest „zrobiony”, gdy `g0_vout_mv` zrówna się z celem. Zrobiony jest, gdy przyszedł ACK SET.

## Podmiana w obecnym drzewie

| Dziś (`b5b317a`) | Zastąp |
|---|---|
| `MX_UART7_Init` baud 115200 | 460800 8N1, piny PE7/PE8 zostają |
| `G4_UartInit` / `arm_rx` (`ReceiveToIdle_IT`) | DMA kołowy + IDLE/HT/TC |
| `G4_UartProcess` + `g4_pop_tx` + `Transmit_IT` | priorytet TX DMA, parser poza ISR |
| `G4_UartRxEvent` kopiuje chunk do pierścienia linii | tylko flaga; zadanie czyta NDTR po invalidate |
| `g4_rx_bytes` | parser binarny tej koperty |
| `g4_set_limits` / `g4_set_mv` / `g4_ilim_ma` | jedna ramka `0x01` |
| `g4_on` / `g4_off` | `0x02` / `0x03` |
| `g4_process` i rekordy `T/TB/TC` | dekodowanie `0x10` / `0x11` / `0x12` / `0x13` do snapshotu |
| `psu_app_tick` czyta `g4.telemetry` z ASCII | czyta ostatni METER; BMS/PD/AUX z wolnych ramek |
| `psu_app_set_limits`, `psu_app_set_output`, `psu_app_clear_fault`, `psu_app_bms_cmd`, `psu_app_usb_role` | budują typy powyżej |
| `G4_CMD_TIMEOUT_MS` 2000 i `G4_SETPOINT_GAP_MS` 120 | 800 ms na SET, bez szczeliny; ON do 8 s |

Sekwencer i ładowarka dalej wołają `psu_app_set_limits` / `psu_app_set_output`. Wystarczy, że te dwie funkcje mówią już binarnie — nie dubluj w nich starego `g4_set_limits`.
