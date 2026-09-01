#!/usr/bin/env bash
# Flash Lab_PD_PSU (STM32G474) via ST-Link / st-flash.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${ROOT}/build/Debug"
ELF="${BUILD_DIR}/Lab_PD_PSU.elf"
BIN="${BUILD_DIR}/Lab_PD_PSU.bin"
FLASH_ADDR=0x08000000

if ! command -v st-flash >/dev/null 2>&1; then
    echo "error: st-flash not found (install stlink-tools)" >&2
    exit 1
fi

if [[ ! -f "${ELF}" ]]; then
    echo "error: ${ELF} missing — run: cmake --preset Debug && cmake --build --preset Debug" >&2
    exit 1
fi

if [[ ! -f "${BIN}" ]]; then
    arm-none-eabi-objcopy -O binary "${ELF}" "${BIN}"
fi

echo "Probing ST-Link..."
PROBE_OUT="$(st-info --probe 2>&1)"
echo "${PROBE_OUT}"
if echo "${PROBE_OUT}" | grep -qE 'Found 0 stlink'; then
    echo "error: no ST-Link programmer detected" >&2
    exit 1
fi

echo "Flashing ${BIN} -> ${FLASH_ADDR}"
st-flash --reset write "${BIN}" "${FLASH_ADDR}"
echo "Done."
