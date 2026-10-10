# Przegląd współpracy H7, G4 i G0 — 2026-10-10

## Zakres i status

Sprawdzone źródła: G4 bazowy `1e4116c`, G0 `f974390`, H7 bazowy `13a18d1`.
Poprawiony H7: `9e9c9eb` na `codex/unify-main-header-controls` (zawiera także
równoległe zmiany wyglądu `6afc899`). Poprawki G4 są w tym commicie na gałęzi
`cursor/board-flash-0ef3`. G0 pozostaje `f974390`.

Znaleziono i poprawiono błędy logiki komunikacji i stanów. Nie ustalono jeszcze,
który warunek rzeczywiście wyłączył fizyczną płytkę po 2–3 sekundach: brak ramki
z pierwszym błędem i pomiarów na sprzęcie. Testy opisane niżej nie stanowią
potwierdzenia pracy całej elektroniki. Pełnego obrazu H7 nie zlinkowano tutaj
(brak SDK TouchGFX i wygenerowanych zasobów); trzeba go zbudować w środowisku
użytkownika na Windowsie.

## Podział odpowiedzialności

* H7 wysyła atomowe V/I, ON/OFF/CLEAR i heartbeat. Pokazuje stan G4. Nie ma
  niezależnego wyłączania działającego wyjścia po wieku METER. Błędy potwierdzone
  przez G4 zatrzymują także sekwencer/ładowanie w GUI. Timeout komendy/startu
  powoduje jawny OFF, aby niedokończony start nie uruchomił później wyjścia.
* G4 steruje DCDC i optoizolowanym POWER_PERMIT. Nadzoruje łączność H7/G0 oraz
  własną przetwornicę. Rozstrzyga zatrzymanie całego toru i wystawia stan FAULT.
* G0 steruje końcowym LDO, DAC i OUT_OFF. Lokalnie wykrywa awarie pomiarów,
  napięcia, prądu, temperatury oraz zasilania. Analogue POWER_KILL fizycznie
  odcina wyjście niezależnie od oprogramowania.

Nie usunięto rzeczywistych zabezpieczeń analogowych ani lokalnych progów G0.

## Potwierdzone błędy i poprawki

| Błąd źródłowy | Skutek | Poprawka |
|---|---|---|
| H7 wysyłał PING tylko przy METER młodszym niż 200 ms i małej kolejce | Utrata odbioru/obciążenie kolejki zatrzymywały heartbeat pracującego H7; G4 po 1000 ms odcinał tor | PING niezależny od METER, co 100 ms, osobny pojedynczy slot; kolejne PING są scalane; OFF zachowuje pierwszeństwo |
| RX H7 odrzucał zawartość DMA po przerwie ponad 25 ms bez sprawdzenia rzeczywistego przepełnienia | Możliwa utrata poprawnych ramek przy opóźnieniu zadania | Przepełnienie wynika z liczby nieodczytanych bajtów DMA |
| Zadanie UART/control H7 miało niższy priorytet od GUI | Renderowanie mogło opóźniać serwis UART | Priorytet High1 ponad GUI High; zapisano także w projekcie CubeMX; fizyczne zachowanie GUI/DMA wymaga sprawdzenia |
| G4 traktował pojedynczy raw KILL z telemetrii jako natychmiastowy zatrzask, G0 potwierdzał go przez 50 ms | G4 mógł zatrzasnąć cały tor po krótkim sygnale, zanim G0 go potwierdził | Raw KILL wymaga 50 ms. Zgłoszone fault_flags G0 nadal są obsługiwane od razu; analogowe odcięcie pozostaje natychmiastowe |
| Nieudany SET/OUT przechodził do FAULT z pozostawionym żądaniem ON | Następny tick mógł automatycznie rozpocząć nowy start | FAULT wycofuje żądanie ON, usuwa PERMIT, blokuje DCDC i zatrzymuje tor; brak samoczynnego restartu |
| Utrata G0 zatrzymywała tor bez jednoznacznego trwałego kodu nadzorcy | GUI mogło pokazać głównie skutek lub odrzucenie kolejnego ON | Utrata G0 przechodzi do FAULT; kod przyczyny jest zachowany do CLEAR/nowego jawnego ON |
| H7 używał 50 ms również jako progu gotowości nowego ON/CLEAR | Krótkie opóźnienie mogło blokować kliknięcie | Jeden próg METER 200 ms dla gotowości/prezentacji; brak runtime OFF od samego wieku |

H7 `13a18d1` usunął wcześniejszy niezależny runtime OFF od wieku METER 50 ms.
Samo usunięcie tego warunku nie usuwało zależności heartbeat od odbioru METER.

## Ramki, piny i nastawy

Sprawdzono w źródłach:

* H7 UART7: PE8 TX / PE7 RX; G4 USART1: PC4 TX / PC5 RX.
* G4 USART2: PB3 TX / PB4 RX, SWAP=0; G0 USART2: PA2 TX / PA3 RX.
* Obie magistrale: 460800, 8N1. TX jednej strony musi być fizycznie połączony
  z RX drugiej oraz wspólnym odniesieniem masy; ciągłość PCB/przewodów nie była
  mierzona tym przeglądem.
* G4 PB7 POWER_PERMIT jest aktywny HIGH; opto ma ściągać G0 PB1 POWER_KILL LOW.
  G0 PA15 OUT_OFF HIGH wyłącza wyjście. Nie zamieniono polaryzacji.
* Format obu protokołów: A5 5A LEN TYPE SEQ payload CRC16/CCITT-FALSE LE.
  Numery TYPE H7↔G4 i G4↔G0 są różne — G4 tłumaczy komendy.
* SET H7 zawiera mV i mA; G4 przekazuje G0 atomowe SETPOINT mV/mA i czeka na ACK.
* Telemetria G0 ma 68 bajtów; METER G4 ma 72, AUX 32. Test przepływu przez
  rzeczywiste funkcje sprawdza kolejność i jednostki napięć, prądu, temperatur
  i maski MEAS_LOST. Temperatury na łączu są °C×10, wewnętrznie G0 °C×100.

## VIN 7 V przy nastawie 3 V

Trzeba rozróżnić trzy węzły:

| Pole | Co oznacza |
|---|---|
| VIN na ekranie głównym / snapshot.vin_mv | G0 VIN: wejście LDO, za przetwornicą |
| Diagnostics METER `vin_mv` | Wejście DCDC G4, np. akumulator/PD |
| Diagnostics METER `vout_mv` | Wyjście DCDC G4, przed LDO |
| `g0_vout_mv` / VOUT na ekranie głównym | Końcowe wyjście zasilacza |
| `vpre_req_mv` / `vpre_cmd_mv` | Żądane / ograniczone szybkością zmian napięcie przed LDO |

Dla nastawy 3 V kod nie żąda 3 V przed LDO. Dolna granica wynosi 6 V, margines
CV 1,5 V. Przy starszym odczycie nastawy G0 5 V podłoga może chwilowo wynosić
6,5 V, a zejście w CV jest ograniczone do 0,3 V/s. To wyjaśnia, dlaczego wejście
LDO może być wyższe niż końcowe 3 V; nie potwierdza dokładności odczytu 7 V.
Jeśli miernik na końcowych zaciskach pokaże 7 V, jest to osobny problem toru
regulacji/pomiaru, którego nie można uznać za poprawny na podstawie VIN.

G4 ma UVLO wejścia DCDC 7 V. Jeśli właśnie Diagnostics `vin_mv` spada pod 7 V,
bit FAULT_UVIN=0x08 jest właściwym tropem. Nie jest to próg G0 VIN.
G0 VIN_LOW to 4,5 V. Nie obniżono tych progów bez pomiaru sprzętu.

## Trwała diagnostyka

AUX byte 30 (poprzednio zero) zawiera:

| Kod | Znaczenie |
|---|---|
| 0 | Brak kodu nadzorcy / starszy firmware; własny fault G4 nadal sprawdzaj oddzielnie |
| 1 | Brak poprawnych ramek H7 przez ponad 1000 ms |
| 2 | Telemetria G0 starsza niż 500 ms |
| 3 | Raw KILL G0 potwierdzony przez 50 ms |
| 4 | Zgłoszone fault_flags G0 |
| 5 | Nieudany start lub komenda G0 |

Byte 31 pozostaje zerem; długość AUX nie zmieniła się. H7 pokazuje `Stop:`
obok masek błędów. Przy G0 FAULT dokładny powód wynika z g0_fault/event context
(np. bit 8 MEAS_LOST, bit 4 VOUT_HARD, bit 5 VOUT_HIGH). Stop reason jest
pierwszą przyczyną nadzorcy; OFF i późniejsze ramki nie nadpisują go.

## Walidacja i ograniczenia

Przeszły 21 programów testowych: 9 jednostek G4, rzeczywiste funkcje CLEAR i
supervisora G4, test między repozytoriami, 3 programy H7, 6 jednostek G0.

Test między repozytoriami kompiluje rzeczywiste pakowanie G0, dekodowanie G4,
przekazywanie METER G4 i parser H7. Przez 10 s bez METER testuje PING H7 przeciw
parserowi i watchdogowi G4. Wejścia analogowe/GPIO i transmisja DMA są zastąpione
kontrolowanymi danymi; nie jest to symulacja analogowej pętli ani test 3 płytek.

G4 Release i Debug kompilują się. Release ma 96488 B FLASH, 16544 B RAM.
Zmodyfikowane H7 C (g4_ascii, psu_app, g4_uart, main) kompilują się dla Cortex-M7
z -Wall -Wextra -Werror. Pełny build GUI wymaga środowiska TouchGFX użytkownika.

Uruchomienie testów G4 z katalogami peerów:

```sh
bash scripts/run_host_tests.sh /path/to/LDO_controller /path/to/GUI_Lab_PD_PSU
```

Przed uznaniem płytki za sprawną trzeba jeszcze odczytać pierwszą przyczynę
zatrzymania, porównać końcowe VOUT i VIN_LDO z miernikiem i sprawdzić pracę
pod obciążeniem. Bez tego nie ma dowodu, że OVP/OCP/MEAS_LOST albo KILL
zadziałały fałszywie. Zmiany powyżej usuwają błędy znalezione w kodzie,
a diagnostyka rozróżnia dalsze przyczyny zamiast wymuszać zgadywanie.
