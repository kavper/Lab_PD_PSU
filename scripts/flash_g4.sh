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

if ! command -v arm-none-eabi-objcopy >/dev/null 2>&1; then
    echo "error: arm-none-eabi-objcopy not found" >&2
    exit 1
fi

echo "Building Debug..."
cmake --preset Debug -S "${ROOT}" -B "${BUILD_DIR%/*}/Debug" >/dev/null 2>&1 || cmake --preset Debug
cmake --build --preset Debug

if [[ ! -f "${ELF}" ]]; then
    echo "error: ${ELF} missing after build" >&2
    exit 1
fi

arm-none-eabi-objcopy -O binary "${ELF}" "${BIN}"
BIN_SIZE="$(wc -c < "${BIN}" | tr -d ' ')"
echo "BIN size: ${BIN_SIZE} bytes"

echo "Probing ST-Link..."
PROBE_OUT="$(st-info --probe --connect-under-reset 2>&1 || true)"
echo "${PROBE_OUT}"
if echo "${PROBE_OUT}" | grep -qE 'Found 0 stlink'; then
    echo "error: no ST-Link programmer detected" >&2
    exit 1
fi

echo "Flashing ${BIN} -> ${FLASH_ADDR}"
st-flash --connect-under-reset --reset write "${BIN}" "${FLASH_ADDR}"

VERIFY_BIN="$(mktemp)"
trap 'rm -f "${VERIFY_BIN}"' EXIT
st-flash --connect-under-reset read "${FLASH_ADDR}" "${BIN_SIZE}" "${VERIFY_BIN}" >/dev/null
if cmp -s "${BIN}" "${VERIFY_BIN}"; then
    echo "Verify OK."
else
    echo "error: flash verify failed" >&2
    exit 1
fi

echo "Resetting target..."
st-flash --connect-under-reset reset >/dev/null
echo "Done."
