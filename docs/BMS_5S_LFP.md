# BMS 5S LiFePO4 — BQ76922, estymacja SOC/SOH, zapis stanu, OTP

Gałąź: `feature/bms-5s-lfp` (bazuje na `codex/instrument-ui-telemetry`).
Pakiet: **5S1P LiFePO4, 6,7 Ah**, układ BQ76922 (HW rev2), bocznik 5 mΩ.

## 1. Co się zmieniło

| Obszar | Było (4S Li-ion) | Jest (5S LFP) |
|---|---|---|
| Konfiguracja AFE | ~25 osobnych kroków init + osobna lista dla OTP | **jedna tabela** `Core/Src/bms_golden.c` dla RAM, OTP i weryfikacji |
| VCell Mode | 0x0017 (pominięte VC4) | **0x001F** (5 ogniw) |
| COV / CUV | 4,25 V / 2,80 V | **3,694 V / 2,479 V** (kody 73 / 49) |
| Temperatura | wyłączona | TS1 = NTC 103AT: **ładowanie 0…55 °C**, rozładowanie −20…60 °C, die 85 °C |
| Balansowanie | ręczne z MCU (1 ogniwo, RAM) | **autonomiczne w BQ**, tylko góra krzywej (min. ogniwo ≥ 3,35 V, start 20 mV, stop 8 mV) |
| Głęboki rozład | brak | **SHUTDOWN przy 2,30 V/ogniwo** (ładowarka na PACK/LD to przerywa) |
| SOC | sesyjny, tylko RAM | **gauge LFP** (licznik BQ + kotwice napięciowe), **0,1 %**, uczona pojemność, SOH, cykle |
| Zapis stanu | brak | **flash G4** (ostatnie 8 KB), przy `BMS SHUTDOWN` i na postoju |
| Telemetria | stała ramka BMS 72 B + osobna lista nazw w H7 | **jeden słownik** `tlm_fields.h` (binarne FIELDS + tekst `TLM`) |
| OTP | komendy zawsze w firmware | **tylko build serwisowy** (`OtpService`), BURN wyłącznie na czystym układzie |

## 2. Okno napięć (dlaczego takie)

LFP ma płaski środek krzywej (≈3,27–3,33 V), więc napięcie mówi coś o SOC tylko na końcach.

| Poziom | Wartość / ogniwo | Kto pilnuje |
|---|---|---|
| Ładowanie (CV) | **3,60 V** (18,00 V pakiet) | ładowarka BQ25731 — *do zrobienia, patrz §9* |
| Ostrzeżenie górne | 3,60 V | G4 (stan WARN) |
| COV (odcięcie CHG) | **3,694 V**, powrót ≈3,59 V | BQ76922, autonomicznie (OTP) |
| Ostrzeżenie dolne | 3,00 V | G4 + gauge (flaga LOW) |
| Odcięcie obciążenia | 2,85 V pod obciążeniem (kompensacja IR) | gauge (flaga EMPTY) |
| CUV (odcięcie DSG) | **2,479 V**, powrót ≈2,68 V | BQ76922 |
| SHUTDOWN | **2,30 V** lub stos < 10,0 V | BQ76922 |

Cel CV: 3,60 V/ogniwo (18,00 V dla 5S). Próg wykrywania pełnego naładowania podąża za celem CV; rzeczywista pojemność zależy od ogniw i zakończenia ładowania.
Domyślne napięcie ładowarki z pinu CELL (4S → 16,8 V = 3,36 V/ogn.) jest dla 5S LFP bezpieczne;
**nie przestawiać rezystora CELL na 5S** (21 V = 4,2 V/ogn.).

## 3. Złoty obraz Data Memory (to trafia do RAM i do OTP)

Generowane narzędziem `tools/print_golden.c` z tej samej tabeli, którą wykonuje firmware:

| Adres | Nazwa (TRM) | Typ | Wartość | Surowe bajty (LE) | FP |
|---|---|---|---|---|---|
| 0x9236 | REG1 Config | U1 | 13 (0x0D, i8 13) | 0D |  |
| 0x9237 | REG0 Config | U1 | 1 (0x01, i8 1) | 01 |  |
| 0x9304 | Vcell Mode | U2 | 31 (0x001F, i16 31) | 1F 00 | tak |
| 0x92FA | CFETOFF Pin Config | U1 | 0 (0x00, i8 0) | 00 |  |
| 0x92FB | DFETOFF Pin Config | U1 | 0 (0x00, i8 0) | 00 |  |
| 0x92FD | TS1 Config | U1 | 7 (0x07, i8 7) | 07 |  |
| 0x92FE | TS2 Config | U1 | 0 (0x00, i8 0) | 00 |  |
| 0x9343 | Mfg Status Init | U2 | 80 (0x0050, i16 80) | 50 00 | tak |
| 0x91A8 | CC Gain | F4 | 1.51368 | 44 C0 C1 3F |  |
| 0x91AC | Capacity Gain | F4 | 451472.65625 | 15 72 DC 48 |  |
| 0x9261 | Enabled Protections A | U1 | 188 (0xBC, i8 -68) | BC | tak |
| 0x9262 | Enabled Protections B | U1 | 115 (0x73, i8 115) | 73 | tak |
| 0x9265 | CHG FET Protections A | U1 | 152 (0x98, i8 -104) | 98 |  |
| 0x9266 | CHG FET Protections B | U1 | 213 (0xD5, i8 -43) | D5 |  |
| 0x9269 | DSG FET Protections A | U1 | 228 (0xE4, i8 -28) | E4 |  |
| 0x926A | DSG FET Protections B | U1 | 230 (0xE6, i8 -26) | E6 |  |
| 0x9273 | Body Diode Threshold | U2 | 2000 (0x07D0, i16 2000) | D0 07 |  |
| 0x9275 | CUV Threshold | U1 | 49 (0x31, i8 49) | 31 | tak |
| 0x927B | CUV Recovery Hysteresis | U1 | 4 (0x04, i8 4) | 04 |  |
| 0x9278 | COV Threshold | U1 | 73 (0x49, i8 73) | 49 | tak |
| 0x927C | COV Recovery Hysteresis | U1 | 2 (0x02, i8 2) | 02 |  |
| 0x9280 | OCC Threshold | U1 | 25 (0x19, i8 25) | 19 |  |
| 0x9288 | OCC Recovery Threshold | U2 | 100 (0x0064, i16 100) | 64 00 |  |
| 0x9282 | OCD1 Threshold | U1 | 38 (0x26, i8 38) | 26 |  |
| 0x9286 | SCD Threshold | U1 | 5 (0x05, i8 5) | 05 |  |
| 0x929A | OTC Threshold | U1 | 55 (0x37, i8 55) | 37 |  |
| 0x929C | OTC Recovery | U1 | 40 (0x28, i8 40) | 28 |  |
| 0x929D | OTD Threshold | U1 | 60 (0x3C, i8 60) | 3C |  |
| 0x929F | OTD Recovery | U1 | 55 (0x37, i8 55) | 37 |  |
| 0x92A3 | OTINT Threshold | U1 | 85 (0x55, i8 85) | 55 |  |
| 0x92A5 | OTINT Recovery | U1 | 80 (0x50, i8 80) | 50 |  |
| 0x92A6 | UTC Threshold | U1 | 0 (0x00, i8 0) | 00 |  |
| 0x92A8 | UTC Recovery | U1 | 5 (0x05, i8 5) | 05 |  |
| 0x92A9 | UTD Threshold | U1 | 236 (0xEC, i8 -20) | EC |  |
| 0x92AB | UTD Recovery | U1 | 241 (0xF1, i8 -15) | F1 |  |
| 0x9308 | FET Options | U1 | 29 (0x1D, i8 29) | 1D | tak |
| 0x930A | Precharge Start Voltage | U2 | 0 (0x0000, i16 0) | 00 00 |  |
| 0x930C | Precharge Stop Voltage | U2 | 0 (0x0000, i16 0) | 00 00 |  |
| 0x930E | Predischarge Timeout | U1 | 50 (0x32, i8 50) | 32 | tak |
| 0x930F | Predischarge Stop Delta | U1 | 50 (0x32, i8 50) | 32 | tak |
| 0x9254 | Auto Shutdown Time | U1 | 0 (0x00, i8 0) | 00 |  |
| 0x923F | Shutdown Cell Voltage | U2 | 2300 (0x08FC, i16 2300) | FC 08 |  |
| 0x9241 | Shutdown Stack Voltage | U2 | 1000 (0x03E8, i16 1000) | E8 03 |  |
| 0x9335 | Balancing Configuration | U1 | 15 (0x0F, i8 15) | 0F | tak |
| 0x9336 | Min Cell Temp | U1 | 0 (0x00, i8 0) | 00 |  |
| 0x9337 | Max Cell Temp | U1 | 50 (0x32, i8 50) | 32 |  |
| 0x9338 | Max Internal Temp | U1 | 70 (0x46, i8 70) | 46 |  |
| 0x9339 | Cell Balance Interval | U1 | 20 (0x14, i8 20) | 14 |  |
| 0x933A | Cell Balance Max Cells | U1 | 2 (0x02, i8 2) | 02 |  |
| 0x933B | Cell Balance Min Cell V (Charge) | U2 | 3350 (0x0D16, i16 3350) | 16 0D | tak |
| 0x933D | Cell Balance Min Delta (Charge) | U1 | 20 (0x14, i8 20) | 14 |  |
| 0x933E | Cell Balance Stop Delta (Charge) | U1 | 8 (0x08, i8 8) | 08 |  |
| 0x933F | Cell Balance Min Cell V (Relax) | U2 | 3350 (0x0D16, i16 3350) | 16 0D |  |
| 0x9341 | Cell Balance Min Delta (Relax) | U1 | 20 (0x14, i8 20) | 14 |  |
| 0x9342 | Cell Balance Stop Delta (Relax) | U1 | 8 (0x08, i8 8) | 08 |  |

Wpisów: 55, CRC-32 obrazu: 0x8EB86164


FP = pole odcisku. Jeżeli przy wybudzeniu wszystkie pola FP w RAM już mają złote wartości
(OTP wypalone albo poprzednia sesja), CONFIG_UPDATE jest pomijane — FET-y nie gasną.

Prądy (6,7 Ah): OCC 10 A (1,5 C), OCD1 15 A (2,2 C), SCD ≈20 A (100 mV), body diode 2 A.

## 4. Estymacja SOC / SOH / cykli (`bms_gauge.c`)

Zasada jak w TI Impedance Track / Apple:

1. **Licznik ładunku** — rejestr BQ76922 `DASTATUS6` (64 bity, 2⁻³² mAh). G4 bierze tylko różnice,
   więc jego pętla nie wpływa na dokładność. Reset licznika (SHUTDOWN, reset AFE) wykrywany jest po
   cofnięciu się jego zegara — wtedy nowa baza, bez skoku SOC.
2. **Kotwice napięciowe** (tylko tam, gdzie krzywa LFP coś mówi):
   - **FULL**: średnie ogniwo ≥ 3,45 V i prąd ładowania < C/20 (335 mA) przez 60 s → 100 %,
   - **knee 1 / knee 2** pod obciążeniem, najsłabsze ogniwo z kompensacją IR: ≤ 3,05 V → 6 %, ≤ 2,85 V → 1 %,
   - **OCV po relaksacji** (≥ 30 min, |I| < 400 mA): tylko ≥ 3,36 V lub ≤ 3,22 V, osobne tablice
     dla gałęzi ładowania i rozładowania (histereza LFP),
   - **plateau**: wartość spoza pasma (np. 5 % przy 3,30 V) jest przycinana i oznaczana jako niepewna.
3. **Pojemność (FCC)**: ładunek zliczony między dwiema prawdziwymi kotwicami odległymi o ≥ 50 % SOC
   → pomiar pojemności; filtr z wagą ∝ rozpiętości, max ±5 % na aktualizację, zakres 50–110 % nominalnej.
   **SOH = FCC / 6700 mAh.**
4. **Cykle** = suma rozładowanego ładunku / 6700 mAh (cykle ekwiwalentne, rozdzielczość 0,01).
5. **Rezystancja ogniwa** z kroków prądu ≥ 1 A (ΔV/ΔI, filtr 1/8) — używana do kompensacji IR.
6. **Po ponownym podłączeniu** pakietu: stan z flash; jeśli napięcie spoczynkowe na stromym końcu
   różni się o > 15 % — przyjmowane jest napięcie (pakiet ładowany/rozładowany poza urządzeniem
   albo wymieniony). SOC jest „niepewny” (`~` w GUI) do następnej kotwicy.

Test na PC (`tests/test_bms_gauge.c`) symuluje pakiet o prawdziwej pojemności 6000 mAh:
po 12 cyklach FCC = 6028 mAh (SOH 89,9 %), SOC w połowie rozładowania różni się od prawdziwego o 0,6 %.

## 5. Gdzie zapisywany jest stan (`bms_store.c`)

**Flash G4, ostatnie 8 KB (0x0803E000, 2 × 4 KB).** Linker kończy kod na 248 KB.

Dlaczego nie EEPROM TPS25751: TPS dostaje się do swojego EEPROM tylko przez 4CC `I2Cr`/`I2Cw`
(8-bitowy offset przy 16-bitowym adresie 24C512, zapis ≤ 10 B tylko kolejkowany, `I2Cr` raz na 5 s).
Dlaczego nie tylko H7: G4 musi znać stan baterii zaraz po starcie, także bez GUI.

- rekord 48 B: magic, numer, flaga „czysty zapis”, stan gauge, CRC-32; 85 rekordów na stronę, ping-pong
  → jedno kasowanie na 85 zapisów (≈1,7 mln zapisów przy 10 k cykli),
- **zapis tylko przy wyłączonym wyjściu** (kasowanie strony zatrzymuje CPU na ~20 ms):
  - `BMS SHUTDOWN` — zawsze, rekord „czysty”,
  - na postoju — po zdarzeniu (FULL, nauczona pojemność) najwcześniej co 30 s, a po zmianie SOC ≥ 0,1 % co 10 min,
  - `BMS SAVE` — ręcznie,
- rozdzielczość zapisu 0,1 mAh; przerwany zapis (CRC / błąd ECC → NMI obsłużone) jest pomijany,
- **`flash`/CubeProgrammer nie może robić „full chip erase”** — skasuje historię baterii
  (cel `flash` w CMake już nie robi `-e all`).

## 6. Telemetria (`tlm_fields.h`)

Jeden słownik pól dla wszystkich: ramka binarna `0x15 FIELDS` do H7 i tekst `TLM` / `?`.
Grupy: BMS (AFE), BATT (gauge), PD (ładowarka/USB-C), AUX (G0, wentylator, sense).
Ramka: `ver | grupa | część | części | (id, typ, wartość)…`; brak pola = brak danych (bez wartości
specjalnych), nieznane id są pomijane, więc dodanie pola to jedna linia w słowniku + `Tlm_Put()` w G4.
Stała ramka METER (5 ms) zostaje binarna i stała. Stare ramki 0x11–0x13 są nadal wysyłane
(`H7_LINK_LEGACY_SLOW=1`) dla starszego H7; nowy H7 je ignoruje po pierwszym FIELDS.

Nowe pola BATT: `soc_permille, rem_mah, fcc_mah, design_mah, soh_permille, cycles_x100, rcell_x10,
ocv_mv, tte_min, ttf_min, anchor, learn_n, learn_mah, chg_total_mah, dsg_total_mah, store_seq,
store_ok, unsaved_mah, temp_dC, chem, cell_n, soc_flags`. W BMS doszły `ts1_dK` i `golden`.

## 7. Komendy (USART1 / TEXT_CMD)

| Komenda | Działanie |
|---|---|
| `TLM` / `?` | wszystkie grupy jako tekst |
| `BMS GAUGE` | SOC, FCC, SOH, cykle, R, kotwica, flagi |
| `BMS GAUGE RESET` | nowy pakiet: FCC = nominalna, cykle 0 |
| `BMS SAVE` | zapis stanu (tylko przy wyłączonym wyjściu) |
| `BMS SHUTDOWN` | wyłącza tor, **zapisuje stan**, BQ → SHUTDOWN; wybudzenie TS2 |
| `BMS OTP STATUS` | CRC złotego obrazu, zgodność RAM, odcisk przy wybudzeniu |
| `BMS OTP CHECK` | *(OtpService)* RESET + raport „czysty?”, obraz → RAM, odczyt, `OTP_WR_CHECK` |
| `BMS OTP BURN I-UNDERSTAND-OTP` | *(OtpService)* jak CHECK + wypalenie; **odmowa, jeśli układ nie jest czysty** |

## 8. Procedura: test na starym układzie → wypalenie nowego

### Etap A — obecny układ (OTP 4S Li-ion), firmware produkcyjny
1. Zasilanie USB-C podłączone (w trakcie CONFIG_UPDATE FET-y są chwilowo wyłączone).
2. Wgrać `Debug` (bez full erase). Przy każdym wybudzeniu MCU zapisuje cały złoty obraz do RAM.
   Stary OTP nie przeszkadza: jego progi (4,25 V / 2,8 V) i tak nie zadziałają przy LFP.
3. Sprawdzić (`TLM`, `BMS GAUGE`, `BMS OTP STATUS` → `ram_match=1`):
   - 5 napięć ogniw, `vcell_rb=0x001F`, `golden` bit0 = 1,
   - `ts1_dK` ≈ 2930–3030 (20–30 °C) — **jeśli termistor nie działa, nie wypalać**,
   - COV/CUV na drabince/symulatorze (podnieść/obniżyć napięcie, obserwować `sa` 0x08 / 0x04),
   - OT/UT: ogrzać / schłodzić NTC, `sb` (0x10 OTC, 0x01 UTC),
   - balansowanie: rozstroić ogniwo > 20 mV przy ≥ 3,35 V → `balance_mask` ≠ 0,
   - `BMS SHUTDOWN` → TS2 → FET-y wracają (na starym układzie przez MCU, bo OTP jest inny).

### Etap B — nowy, czysty BQ76922, firmware `OtpService`
1. Wlutować układ. Na wejścia ogniw **10–12 V** (np. drabinka 5 × 2,2 V z zasilacza; pakiet 5S LFP
   ≈16 V jest **za wysoki** dla OTP). Temperatura ≤ 45 °C. USB-C podłączone.
2. Wgrać build `OtpService`. `BMS OTP CHECK` → oczekiwane `blank=1 check=0x80 fail_addr=0x0000`.
   (CUV będzie aktywne przy 2,2 V/ogn. — to normalne.)
3. `BMS OTP BURN I-UNDERSTAND-OTP` → `OK BMS OTP BURNED … crc=0x8EB86164`.
4. Wgrać z powrotem firmware produkcyjny (`Debug`/`Release`) — komend BURN już nie ma.
5. Podłączyć pakiet, `BMS SHUTDOWN`, krótkie TS2 → FET-y załączają się **bez MCU**;
   `BMS OTP STATUS` → `fp_match=1`, `otp_at_wake=1`.

Zabezpieczenia: BURN robi najpierw RESET i sprawdza, że w OTP są fabryczne wartości
(Vcell 0x0000, FET Options 0x0D, Mfg 0x0040, COV 86, Prot B 0x00); w przeciwnym razie odmawia
(`REFUSED (chip not blank)`). Po wypaleniu RESET i porównanie całej tabeli z odczytem.

## 9. Otwarte punkty

- **Ładowarka BQ25731**: napięcie ładowania nadal ustawia TPS (16,8 V z konfiguracji 4S). Do zmiany
  na **18,00 V**, wyłączenie górnego prechargu i zakończenie ładowania przy C/20. Dopóki ładowarka
  kończy na 16,8 V (3,36 V/ogn.), kotwica FULL się nie pojawi i pojemność uczy się tylko z kotwic
  dolnych/OCV.
- Tablice OCV są typowe dla LFP; po kilku pełnych cyklach warto je skorygować pomiarem.


OTC zmienione na 55 °C na żądanie użytkownika (2026-11-02); recovery pozostaje 40 °C. OTC Threshold i Recovery należą do fingerprintu, więc zmiana wymusza aktualizację RAM na starcie. Poprzedni zapis CRC tabeli w tym dokumencie jest historyczny. OTP nie programowano.

## NTC correction pending (2026-11-02)

User confirmed BQ76922RSNR and Vishay NTCLE100E103JB0A (10k, B3977),
mounted on PCB. The existing Semitec 103AT polynomial is not a verified
conversion for this part. TS1 is not a measurement of cell temperature.
OTP BURN is therefore blocked by BMS_THERMISTOR_MODEL_VALIDATED=0.
A manufacturer R(T) input archive has been prepared for TI GPC per SLUUCP2.
Fit/validate the hardware coefficients and resolve cell temperature sensing
before changing this gate. Current RAM model and thresholds were not changed
by this audit; the 55 C raw-model threshold is not yet a calibrated 55 C trip.

## Balancing policy update (2026-11-02)

Supersedes autonomous balancing description above. Balancing Configuration
0x0C disables autonomous charge/relax balancing; host commands remain enabled.
G4 commands at most two non-adjacent cells only with fresh valid data, no
reported safety flags, |I| strictly below 1000 mA, SOC>=900/1000, minimum
cell voltage>=3350mV, and five seconds continuously within those gates.
Start delta20mV; retain active cells to8mV. Existing BMS thermal gates remain.
Command watchdog is5s. Current crossing is detected at the next BMS scan;
TI permits up to about1s for CB_ACTIVE_CELLS writes to take effect.
Telemetry continues to read CB_ACTIVE_CELLS; requested mask is never used
as confirmation. Firmware is required for this policy; hostless BMS has
autonomous balancing disabled. OTP remains blocked pending NTC validation.
Only imbalance warning changed100->300mV. COV/CUV unchanged.

## Vishay 18K model applied (2026-11-02)

TI GPC report accepted: A=[-16652,29010,-22024,26321,1343], B=[-3958,5551,-4365,4554], Adc0=11703. Reported maximum nominal fitting error0.5C over supplied -40..125C points. These overwrite the18K model (0x91EA..0x91FA,0x91FE), with TS1 still selecting18K. Ten coefficient fields and TS1 configuration are fingerprinted and included in full RAM verification. OTC remains60C, recovery40C. This supersedes the earlier missing-polynomial blocker. Physical measurement validation and PCB-vs-cell placement remain unresolved; OTP gate remains0. Sensor tolerance, pull-up/pad errors and placement are not included in the0.5C fit error.

User confirmed loose cell-terminal screws caused the large current-dependent spread. Imbalance warning restored300->100mV (2026-11-02). Balancing policy remains |I|<1A and SOC>=90%; temperature model remains Vishay TI GPC, OTC60C/recovery40C. This supersedes the earlier300mV warning update.


## COV recovery — 2026-11-02
User-approved host policy: COV remains reported until cleared by AFE hysteresis.
COV disables CHG autonomously, without a host rail shutdown; monitoring stays alive.
Balancing during COV requires CHG-off, DSG-on, no other Safety A/B/C, no startup
guard latch, and all existing current/SOC/freshness/temperature gates.
Start verification accepts DSG-on/PDSG-off with CHG-off only for COV alone.
No repeated FET starts, COV overrides or protection threshold changes.
CUV still requests host shutdown. TI SLUUCG7 sections5.2.4 and10.

## Profil ładowania G4 — 2026-11-03
Tymczasowy profil 17,496 V / 512 mA usunięto. Obowiązują limity z EEPROM TPS:
18 V / 8,064 A (średnio 3,6 V/ogniwo). COV i pozostałe zabezpieczenia BMS
pozostają aktywne. Balansowanie nadal wymaga |I|<1A, SOC>=90%, min 3350 mV,
poprawnych temperatur i danych, więc przy pełnym prądzie rusza dopiero przy
końcu ładowania.
