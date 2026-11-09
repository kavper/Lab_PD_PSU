# TPS EEPROM — PPS oraz stałe 12 V

Gotowy szkic: esp32_tps25751_memorybin_writer/esp32_tps25751_memorybin_writer.ino. W tym samym katalogu znajduje się nowy full.h. Oryginalny katalog programatora nie został zmieniony.

Profile SOURCE w obu regionach pamięci:
1. 5 V / 3 A
2. 9 V / 3 A
3. 12 V / 3 A — dodany
4. 15 V / 3 A
5. 20 V / 5 A
6. Dotychczasowy SPR AVS — zachowany dokładnie 0xE004B1F4
7. PPS 5–21 V / 5 A — przywrócony dokładnie 0xC1A43264 z archiwum

PPS 21 V × 5 A oznacza nominalnie 105 W na górnym końcu zakresu; jest to odtworzenie wcześniejszego APDO, a nie potwierdzenie wydajności sprzętu przy tym obciążeniu. Prąd faktycznej sesji zależy także od rozpoznania kabla i negocjacji.

Ładowanie pakietu zachowuje tymczasowy profil do wyrównywania: 17,496 V / 512 mA przy 5 mΩ. Te wartości dotyczą ładowania pakietu, a nie dostępnej mocy SOURCE. Inne ustawienia, precharge, polityka portu i kod poprawki TPS pozostają identyczne. OTP BMS bez zmian.

Sprawdzenia poza sprzętem zakończone poprawnie, szczegóły CHECKS.log i manifest.json. CRC obu regionów sprawdzone niezależną implementacją bitową, także wobec wcześniejszego obrazu PPS. EEPROM nie został jeszcze zapisany. Po zapisie potrzebne potwierdzenie startu TPS, siedmiu profili i negocjacji rzeczywistego PPS oraz 12 V; samo wyświetlenie profilu nie dowodzi stabilnej regulacji.

Programator ESP: SDA21/SCL22, EEPROM 0x50, monitor 115200. Szkic automatycznie rozpoczyna zapis po uruchomieniu ESP i sprawdza odczyt każdego fragmentu. Użyj dotychczasowego sprawdzonego podłączenia, bez jednoczesnego dostępu TPS do magistrali. Wymagany komunikat końcowy: [OK] Full flash programmed and verified. Odłącz ESP i uruchom ponownie TPS. Pliki PREVIOUS to kopie wcześniejszych plików, nie odczyt fizycznego EEPROM.
