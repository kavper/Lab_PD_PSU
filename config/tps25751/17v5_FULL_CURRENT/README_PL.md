# Aktualny obraz: 17,50 V / pełny wcześniejszy limit prądu

Żądane ustawienie ładowania pakietu: 17,496 V (17,50 V zaokrąglone w dół do kroku 8 mV), 8,064 A przy 5 mΩ. To limit, nie wymuszenie prądu niezależnie od napięcia lub zabezpieczeń.

SOURCE_5V_3A: 5V/3A,9V/3A,12V/3A,15V/3A,20V/5A, dotychczasowy AVS, PPS5–21V/5A. SINK ma też 12V/3A i preferencję wyższego napięcia przy równej mocy. PPS SINK pozostaje wyłączony. SOURCE_5V_5A_BENCH_ONLY to osobny wariant, zwiększający wyjście 5V do5A; nie został zatwierdzony obciążeniowo.

Nie używać poprzednich wersji 18V ani wersji ze stałym 512mA. Każdy wariant ma kompletny szkic ESP i full.h. Szkic zapisuje EEPROM automatycznie po uruchomieniu. Pliki zweryfikowane poza sprzętem: oba CRC niezależną implementacją bitową, wszystkie cztery operandy napięcia/prądu, brak zmian w PDO i innych ustawieniach, zgodność full.h z BIN. EEPROM nie został jeszcze zapisany.

G4 nie jest potrzebny do przygotowania ani zapisania tego EEPROM przez ESP. Jednak wcześniejszy firmware G4 z włączonym dodatkowym ograniczeniem będzie obniżał prąd do512mA niezależnie od obrazu EEPROM. Poprawka G4 wyłączająca tę funkcję jest osobnym elementem; nie wgrywano jej podczas przygotowania tego obrazu.
