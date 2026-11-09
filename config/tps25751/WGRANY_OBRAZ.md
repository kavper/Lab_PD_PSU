# Obraz konfiguracyjny TPS25751 w urządzeniu

Bieżący obraz (wg `README.md`): `17v5_FULL_CURRENT/SOURCE_5V_3A` – napięcie ładowania
17,496 V (3,50 V/ogniwo), sufit prądu ładowania 8,064 A, PDO źródła: 5/9/12/15 V przy 3 A,
20 V przy 5 A oraz PPS 5–21 V przy 5 A. Według autora w urządzeniu wgrany jest najnowszy obraz.
`MAX_CHARGE/` i `pps_12v_17v5_512ma/` to warianty historyczne. Programator EEPROM:
`esp32_tps25751_memorybin_writer` w katalogu wariantu.

Oprogramowanie G4 w urządzeniu: tag `v1.0-praca` (kod z d5d0cde, kompilacja G4_PD_RST300, 4.11.2026).
