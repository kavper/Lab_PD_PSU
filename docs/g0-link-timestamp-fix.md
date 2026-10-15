# False G0 telemetry timeout in G4

Live G0 UART capture on 2026-10-10 showed uninterrupted telemetry during an
approximately 1.8 s output attempt. G0 requested 2500 mV, reported no faults,
then observed KILL before disabling its output approximately 50 ms later.
H7 diagnostics recorded G4 stop reason G0 TELEMETRY LOST.

LdoLink_Task sampled HAL_GetTick before LinkUart_Poll. The parser stamped
last_tlm_ms and last_rx_ms with HAL_GetTick while processing incoming data.
If SysTick advanced during parsing, the saved now_ms preceded last_tlm_ms.
Unsigned subtraction then produced a value near UINT32_MAX rather than zero,
triggering the 500 ms watchdog immediately despite a fresh frame.

Sample now_ms after Poll and the first-RX diagnostic. Preserve the actual
500 ms disconnect watchdog. The production-task regression advances SysTick
inside Poll, including UINT32_MAX wrap: it failed before the fix and passes
afterwards. Genuine 500 ms telemetry loss still stops output in the tests.

The observed G0 VIN around 7.26 V despite a 2.5 V request remains a separate
unresolved regulation/measurement issue. This patch does not claim to fix it.
