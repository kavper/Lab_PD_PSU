# Normalne limity ładowania — usunięcie stałych 512 mA

Docelowe ustawienie TPS: ChargeVoltage 18,000 V, ChargeCurrent 8,064 A przy shuncie 5 mΩ (raw 0x0FC0). Jest to przywrócenie wcześniejszego skonfigurowanego limitu, nie zwiększanie do maksymalnego prądu układu. Rzeczywisty prąd zależy od kontraktu USB, ograniczeń wejścia, regulacji CV, temperatury i odcięć BMS. Nie zmieniono zabezpieczeń BMS.

G4 musi mieć wyłączony dodatkowy limit: CHARGE_BALANCE_LIMITS_ENABLED=0. Sam nowy G4 nie podniesie 512 mA zapisanych w starym EEPROM. Sam nowy EEPROM też nie wystarczy ze starym G4, które ponownie narzuci 512 mA.

SOURCE_5V_3A to podstawowy obraz: PPS 5–21 V / 5 A, stałe 5/9/12/15/20 V i dotychczasowy AVS. SINK zawiera 12 V i preferencję wyższego napięcia przy równej mocy i typie. SOURCE_5V_5A_BENCH_ONLY to oddzielny wariant testowy 5 V / 5 A, bez potwierdzenia obciążeniowego.

Obrazy przygotowane i sprawdzone poza sprzętem. EEPROM nie został jeszcze zapisany. Otwórz właściwy szkic .ino; odpowiadający full.h znajduje się obok. ESP rozpoczyna zapis automatycznie po uruchomieniu. Wymagane sprawdzenie odczytu oraz ponowny start TPS po odłączeniu ESP. PREVIOUS_512mA.bin to kopia wcześniejszego pliku, nie odczyt fizycznej pamięci.

Nie wgrywać wcześniejszych obrazów z nazwami 512mA lub 17v5, jeśli celem jest powrót do normalnego ładowania. OTP BMS nie zmieniano. Model ogniw i ich dopuszczalny prąd nadal wymagają zatwierdzenia przed OTP.
