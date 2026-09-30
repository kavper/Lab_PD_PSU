# Pomocnicze zasilanie bootstrapu UCC33420

Każda gałąź ma osobne sterowanie: PC10 → BUCK, PC12 → BOOST.
PC11 i PA15 odczytują EN/FLT przez EXTI na zboczu opadającym, bez pull-up/pull-down.
Konfiguracja jest w IOC; PowerStage_Init ustawia ją również w obecnym firmware.

## Sekwencja

- EN załącza się przy żądanym HS ≥95%, wyłącza poniżej 93%.
- Dla BOOST HS = 1 − duty dolnego tranzystora. W BUCK zasilanie pomocnicze
  BOOST jest więc potrzebne również przy małym wypełnieniu tranzystora BUCK.
- Przez 20 ms od zbocza EN fizyczne HS danej gałęzi jest ograniczone do 90%.
  Zasilanie jest sterowane z żądania, nie z ograniczonego duty.
- Po tym czasie, przy braku zatrzaśniętej awarii, dopuszczane jest żądane
  wypełnienie, w tym static-high od 99,5%.
- Regulator CV nie akumuluje błędu podczas rozruchu UCC. Punkt próbkowania
  ADC korzysta z faktycznie zastosowanego duty.
- FLT jest zatrzaskiwany również podczas rozruchu. Przerwanie zatrzymuje
  wyjścia HRTIM; pętla aplikacji wyłącza stopień i EN. Wyłączenie EN przez
  program nie jest awarią. Kasowanie błędu wymaga wyłączonego stopnia i EN.
- Zatrzymanie nie wywołuje SetDuty(0,0), bo to oznacza HS BOOST = 100%.
- Rozładowanie nie rozpoczyna się, dopóki nie upłynie czas rozruchu BOOST UCC.

Progi i czas są w Core/Inc/dcdc_hs_policy.h. Dawne definicje progów są
aliasami tego samego ustawienia; sprzeczne ustawienia zatrzymują kompilację.
Wyłączenie impulsów odświeżania bootstrapu nie wyłącza zamontowanych UCC.
Log PWM zawiera `tr_en_a`, `tr_en_c` i `ucc_starting`.

## Weryfikacja

`sh tests/run_host_tests.sh` uruchamia istniejące testy polityk oraz rzeczywisty
power_stage.c z rejestrami w RAM i atrapą HAL: BUCK/BOOST/mixed, niezależne
czasy rozruchu, histereza, brak impulsów EN przy inicjalizacji, start PWM,
FLT w rozruchu i podczas startu wyjść, stan niski EN/FLT, zatrzymanie,
kasowanie błędu, ponowny start i rozładowanie. Osobny przebieg sprawdza
konfigurację z wyłączonym odświeżaniem impulsowym.

Kompilacja firmware: `cmake --build build/Debug --target Lab_PD_PSU`.

## Do potwierdzenia na płytce

EN/FLT nie jest power-good; upływ 20 ms nie jest pomiarem napięcia.
Czas 20 ms uwzględnia opisany przez TI typowy timeout soft-start 16 ms z marginesem;
nie jest gwarantowanym przez producenta czasem osiągnięcia gotowości.
Sprawdzić oscyloskopem EN, EN/FLT i BST względem SW przy starcie oraz zmianach
BUCK ↔ BOOST. Potwierdzić wystarczające napięcie drivera przed końcem ograniczenia,
ładowanie bootstrapu przy limicie 90%, progi 95/93% oraz przejściowe napięcie wyjścia.
Testy komputerowe nie symulują drivera, bootstrapu ani stopnia mocy.

Dokumentacja TI: https://www.ti.com/lit/ds/symlink/ucc33420.pdf,
sekcje 7.3.2 (soft-start) i 7.3.4.5 (FLT).
