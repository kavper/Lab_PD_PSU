# Protokół UART dla HMI (G4 USART1)

Ten plik jest kontraktem dla HMI, które steruje płytą. Implementacja jest w `Core/Src/host_link.c`. Szczegóły ramek po angielsku: [HOST_TELEMETRY.md](HOST_TELEMETRY.md). Link G4↔G0 (nie ten port): [G4_LDO_UART.md](G4_LDO_UART.md).

HMI steruje **wyjściem LDO na G0**. Ramki `T` / `TB` / `TC` pokazują stan G4. Suwak prądu **nie** ogranicza przetwornicy DCDC. Awaryjne 10 A DCDC jest zaszyte w G4.

## Warstwa fizyczna

| | |
|---|---|
| Port | USART1, G4 **PC4 = TX**, **PC5 = RX** |
| Format | **115200 8N1**, ASCII, jedna komenda na linię |
| Koniec linii | HMI wysyła `\r\n`. G4 kończy komendę na `\r` albo `\n` |
| Wielkość liter | komendy są case-insensitive |
| Długość | maks. 95 znaków. Dłuższa linia → `ERR LINE` |
| Start | po resecie G4 samo leci z `TEL 500` (ramka `T` co 500 ms) |

Ignoruj linie, które nie są `T`, `TB`, `TC`, `E`, `OK`, `ERR`, `WARN`, `HELP` albo banerem. **Nie parsuj `TLM`** — to jest ruch G0 na USART2 i na ten port nie wychodzi, dopóki ktoś nie włączy `VERBOSE 1`. `VERBOSE` zostaw na 0. `NACK` z G0 może się pojawić i warto go zalogować.

Parser: najpierw rozpoznaj prefiks linii `TB`, `TC`, `E`, `OK`, `ERR`, `WARN`, dopiero potem `T` (inaczej `TB` wpadnie w parser `T`). Potem `key=value`. Kolejność kluczy jest stała, ale bierz po nazwie.

Pusta linia jest ignorowana. Linia, która nie zaczyna się od litery albo `?`, jest ignorowana (bez `ERR CMD`).

Baner po starcie:

```
=== Lab_PD_PSU G4 host ready (USART1 115200) ===
boot rcc_csr=0x........ (PIN=… POR=… SFT=… IWDG=… WWDG=… LPWR=…)
USART1 = T/TB/TC only; G0 TLM not forwarded (VERBOSE 0). Send HELP.
```

`boot rcc_csr` mówi, czemu MCU wstało (pin / BOR / software / IWDG).

## Komendy, które HMI wysyła

### Wyjście

```
ON
OFF
SET V=12.000 I=1.500
CLR
```

- `ON` → `OK ON set=<mV> ilim=<mA> (wait g0_out=1)`. Potem czekaj, aż w `T` będzie `g0_out=1`. Jak G0 milczy, dodatkowo przyjdzie `WARN g0_rx=0 — DCDC may start; LDO needs G0 TLM`.
- `OFF` → `OK OFF`. Gasi LDO i DCDC.
- `SET V=<wolty> I=<ampery>` to **jedyna** komenda suwaków. V: `0.000..27.000`, I: `0.000..5.000`. Odpowiedź: `OK SET V=12.000 I=1.500` (część całkowita i trzy cyfry po kropce, zero-padded). Zły zakres: `ERR SET use: SET V=0.000..27.000 I=0.000..5.000`.
- `I=` to limit prądu **LDO na G0**, nie przetwornicy. Awaryjne 10 A DCDC jest zaszyte w G4 i z HMI się go nie ustawia.
- `V=` to napięcie na wyjściu LDO, które widzi użytkownik. Szyna DCDC jedzie wyżej (`vpre_*`, zapas 1,5 V) i schodzi w dół 0,3 V/s. Suwak w dół nie skacze od razu na buck.
- `SET` przy włączonym wyjściu **nie gasi** raila. Nie wysyłaj `OFF` przed każdym `SET`.
- Nie wysyłaj `SET` na każdy tick suwaka. Debounce **≥ 100 ms** albo wyślij po puszczeniu. Spam urywa linię i dostajesz `ERR CMD` / `ERR LINE`.
- Po `ERR CMD — send HELP` albo `ERR LINE` wyślij ostatnie `SET` jeszcze raz, jeden raz, po debounce.
- `CLR` / `CLEAR` → `OK CLR`. Zdejmuje zatrzask błędu PSU.

### Odczyt

```
?
TEL
TEL 500
TEL 0
```

- `?` albo `STATUS` albo `ST` — jedna pełna trójka `T` + `TB` + `TC` od razu.
- `TEL` bez liczby wraca do 500 ms → `OK TEL 500 ms`.
- `TEL 0` stopuje telemetrię.
- `TEL 100` to praktyczne dno przy 115200: `T` co 100 ms, `TB`/`TC` i tak co 200 ms. Zostań na 500, chyba że UI naprawdę potrzebuje szybciej.
- Zły argument: `ERR TEL`.
- Przy 115200 pełna trójka `T`+`TB`+`TC` to ~1,3 kB (~135 ms na drucie).

| `TEL` | `T` | `TB` + `TC` |
|---|---|---|
| 500 ms (domyślnie) | 500 | 500 |
| 200 ms | 200 | 200 |
| 100 ms | 100 | 200 |
| 0 | stop | stop |

### Opcje w UI

| Komenda | Odpowiedź | Znaczenie |
|---|---|---|
| `PERMIT 0` | `OK PERMIT 0 (LDO zabity)` | gasi wyjście, PB7 w dół |
| `PERMIT 1` | `OK PERMIT 1` | zdejmuje kill; samo z siebie nie włącza wyjścia |
| `REMOTE ON` albo `REMOTE 1` | `OK REMOTE` | pomiar zdalny (PB6) |
| `REMOTE OFF` albo `REMOTE 0` | `OK LOCAL` | pomiar lokalny (domyślnie) |
| `USB AUTO` | `OK USB` | DRP |
| `USB SINK` | `OK USB` | tylko sink |
| `USB SOURCE` | `OK USB` | tylko source |

Zły argument: `ERR USB`, `ERR PERMIT`, `ERR REMOTE`.

Nie wystawiaj w HMI: `VERBOSE 1`, `BMS FORCE`, `BMS SHUTDOWN`, `BMS OTP BURN`, `G0SWAP`. `BMS` bez argumentu jest miękkie (jak BMS już zdrowy, G4 nic nie rekonfiguruje). Wybudzenie z uśpienia baterii to przycisk TS2, nie UART.

Stare formy, których GUI nie musi używać: `SET 12.0` → `OK SET 12000 mV` (zakres `0.000..27.000`, błąd `ERR SET range: 0.000..27.000 V`), `ILIM 1.5` → `OK ILIM 1500 mA` (zakres `0.000..5.000`, błąd `ERR ILIM range: 0.000..5.000 A`).

Nieznana komenda: `ERR CMD — send HELP`.

## Ramka `T` — główny ekran

Jedna linia. Wszystkie wartości są liczbami całkowitymi poza `mode` (`IDLE`, `CV`, `CC`).

```
T vin_mv=14820 vout_mv=13500 iout_ma=420 i_buck_ma=410 i_boost_ma=5 set_mv=12000 ilim_ma=1500 duty_a_x10=802 duty_c_x10=0 ucc_a=0 ucc_c=0 run=1 mode=CV fault=0 pd=1 pd_mv=20000 pd_ma=3000 pd_mw=60000 permit=1 rem_sense=0 g0=1 g0_out=1 g0_want=1 g0_ctrl=9 g0_kill=0 g0_outoff=0 g0_fault=0x0 g0_vout_mv=12010 g0_iout_ma=410 vpre_req_mv=13500 vpre_cmd_mv=13500 reg_ok=1 stage_en=1 ps_en=1 flt=0 hold_ms=0 ps_err=0 g0_rx=12000 g0_tlm=400 g0_age_ms=80 g0_err=0 g0_uart=0x0 pm_st=2 fmt=0
```

Co pokazać użytkownikowi:

| Klucz | UI |
|---|---|
| `g0_vout_mv` | napięcie na wyjściu LDO, mV. To jest „Vout” |
| `g0_iout_ma` | prąd wyjścia LDO, mA |
| `set_mv` | zadane V, mV. G0, gdy wyjście jest chciane albo G0 żyje; inaczej lokalny setpoint CV |
| `ilim_ma` | zadany limit LDO, mA (0..5000) |
| `g0_out` | 1 = LDO naprawdę włączone. Na to czekaj po `ON` |
| `g0_want` | 1 = HMI kazało włączyć |
| `run` | 1 = ścieżka PSU chodzi |
| `mode` | `IDLE`, `CV` albo `CC`. Tryb LDO, nie buck/boost |
| `vin_mv` | wejście DCDC, mV |
| `vout_mv` | szyna DCDC przed LDO, mV. Nie mylić z `g0_vout_mv` |
| `vpre_req_mv` | cel szyny DCDC, mV |
| `vpre_cmd_mv` | aktualne polecenie DCDC po slew, mV |
| `iout_ma` | prąd DCDC, mA |
| `i_buck_ma` `i_boost_ma` | prądy high-side INA296, mA |
| `duty_a_x10` | wypełnienie HS bucka (TA1), procent×10. `802` = 80,2% |
| `duty_c_x10` | wypełnienie LS boosta (TC2), procent×10 |
| `ucc_a` | 1 = aux buck (PC10) włączony. Włącza się przy HS ≥ 98%, gaśnie poniżej 96% |
| `ucc_c` | 1 = aux boost (PC12), ta sama histereza |
| `fault` | maska bitowa, niżej |
| `permit` | 1 = PB7 puszcza LDO |
| `rem_sense` | 0 lokalny, 1 zdalny |
| `reg_ok` | 1 = DCDC w okolicy `vpre_cmd` |
| `flt` | 1 = fault UCC (noga włączona i FLT trzymany nisko) |
| `pd` `pd_mv` `pd_ma` `pd_mw` | kontrakt USB-PD: jest / mV / mA / mW |
| `g0` | 1 = G0 aktywne w pre-reg |
| `g0_kill` `g0_outoff` | flagi z G0, 0/1 |
| `g0_fault` | flagi błędu G0, hex |
| `stage_en` `ps_en` | stopień / power stage włączony, 0/1 |
| `hold_ms` | pozostały czas holdu startu, ms |
| `ps_err` | ostatni błąd power stage |
| `g0_rx` | liczba bajtów odebranych z G0 |
| `g0_tlm` | liczba ramek telemetrii G0 |
| `g0_age_ms` | wiek ostatniego bajtu z G0. `4294967295` = jeszcze nic nie przyszło |
| `g0_err` `g0_uart` | błędy UART do G0. `g0_uart`: 1 PE, 2 NE, 4 FE, 8 ORE |
| `pm_st` | stan power managera |
| `fmt=0` | wersja ramki |

`g0_ctrl`:

| wartość | stan |
|---|---|
| 0 | idle |
| 1 | czeka na link |
| 2 | czeka na permit |
| 3 | czeka na VIN |
| 4 | wysyła SET |
| 5 | czeka na ACK SET |
| 6 | czeka aż Vout zejdzie do zera (tylko pierwsze `ON`) |
| 7 | wysyła OUT ON |
| 8 | czeka na ACK OUT ON |
| 9 | chodzi |
| 10 | wysyła OUT OFF |
| 11 | czeka na ACK OUT OFF |
| 12 | fault |

`fault` na `T` (OR bitów):

| bit | nazwa |
|---|---|
| 1 | DRIVER |
| 2 | OVP |
| 4 | OCP |
| 8 | UVIN |
| 16 | ADC |
| 32 | BMS |

Jednorazowa linia po przekroczeniu 10 A przez kilka cykli sterowania. To nie jest limit suwaka. `ilim_ma` tutaj wynosi 10000:

```
E OCP src=HS_INA296 i_buck_ma=… i_boost_ma=… iout_ma=… ilim_ma=10000 hits=… vin_mv=… vout_mv=…
```

## Ramka `TB` — bateria (BQ76922)

```
TB bms=1 cfg=1 st=… fault=0x00000000 alert=0 alarm=0x0000 sa=0x00 sb=0x00 sc=0x00 fet=0x.. manuf=0x.... init_step=… vcell_rb=0x0017 batt=0x.... cfg_fail=0 chg=1 dsg=1 fets=1 series=4 c1_mv=… c2_mv=… c3_mv=… c4_mv=-1 c5_mv=… min_mv=… max_mv=… dV_mv=… sum_mv=… pack_mv=… stack_mv=… i_pack_ma=… i_cc2_ma=… sample=1 alerts=… i2c_err=…
```

Zdrowa 4S: `bms=1 cfg=1 fets=1 chg=1 dsg=1 vcell_rb=0x0017 series=4`. `c4_mv=-1` jest poprawne (ogniwo 4 pominięte). `c1_mv`..`c5_mv`, `min_mv`, `max_mv`, `pack_mv`, `stack_mv` w mV. `i_pack_ma` i `i_cc2_ma` to ten sam prąd paczki (CC2), mA, plus = ładowanie, minus = rozładowanie. `sa` / `sb` / `sc` to Safety Status A/B/C.

| klucz | znaczenie |
|---|---|
| `bms` `cfg` `st` | obecny / skonfigurowany / stan automatu |
| `fault` | flagi błędu BMS, hex |
| `alert` | 1 = alert |
| `alarm` | Alarm Status |
| `fet` `manuf` | FET Status / Manufacturing Status |
| `init_step` | krok inicjalizacji; 0 = WAIT_READY |
| `vcell_rb` | odczyt VCell Mode, oczekuj `0x0017` |
| `batt` | Battery Status |
| `cfg_fail` | nieudane wejścia w CFGUPDATE |
| `chg` `dsg` `fets` | FET ładowania / rozładowania / którykolwiek |
| `sample` | 1 = próbka ważna |
| `alerts` `i2c_err` | liczniki |

## Ramka `TC` — ładowarka (BQ25731) i USB-C

```
TC bq_ok=1 bq_vbat_mv=… bq_vsys_mv=… bq_ibat_ma=… bq_ichg_ma=… bq_idchg_ma=… bq_vbus_mv=… bq_iin_ma=… bq_vreg_mv=… bq_ichg_set_ma=… bq_iin_set_ma=… bq_st=0x.... bq_fault=0x.. bq_in=… bq_pre=… bq_fast=… bq_otg=… bq_iindpm=… bq_vindpm=… tps_vbus_mv=… cc1=… cc2=… role=… conn=… plug=… typec=0x.. rst=… rst_busy=… pd_role=… pd_mv=… pd_ma=…
```

`pd_role`: 1 = sink, 2 = source. `bq_otg=1` dopiero po prawdziwym kontrakcie source. `plug=1` = kabel wpięty. `bq_ok=1` = ładowarka online i ADC ważne. Napięcia w mV, prądy w mA.

| klucz | znaczenie |
|---|---|
| `bq_vbat_mv` `bq_ibat_ma` | napięcie / prąd węzła baterii |
| `bq_vsys_mv` | VSYS |
| `bq_ichg_ma` `bq_idchg_ma` | prąd ładowania / rozładowania (ADC) |
| `bq_vbus_mv` `bq_iin_ma` | wejście USB |
| `bq_vreg_mv` `bq_ichg_set_ma` `bq_iin_set_ma` | zadane Vbat / Ichg / Iin |
| `bq_st` `bq_fault` | Charger Status / fault, hex |
| `bq_in` `bq_pre` `bq_fast` | wejście jest / precharge / fast charge |
| `bq_iindpm` `bq_vindpm` | ograniczenie prądu / napięcia wejścia |
| `tps_vbus_mv` `cc1` `cc2` `role` `conn` | ścieżka Type-C |
| `plug` | 1 = attached |
| `typec` | stan portu Type-C, hex |
| `rst` `rst_busy` | licznik resetu PD / reset w toku |
| `pd_mv` `pd_ma` | kontrakt PD z power managera |

## Minimalna pętla HMI

1. Otwórz 115200 8N1. Poczekaj na baner `=== Lab_PD_PSU G4 host ready (USART1 115200) ===`.
2. Czytaj `T` co 500 ms. Ekran główny bierz z `g0_vout_mv`, `g0_iout_ma`, `set_mv`, `ilim_ma`, `g0_out`, `mode`, `fault`.
3. Suwak: po 100 ms ciszy wyślij `SET V=x.xxx I=y.yyy` i czekaj na `OK SET`.
4. Przycisk włącz: `ON`, czekaj na `g0_out=1`. Wyłącz: `OFF`.
5. `fault≠0` albo linia `E OCP` → pokaż błąd, daj przycisk `CLR`.
