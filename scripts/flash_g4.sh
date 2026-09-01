#!/usr/bin/env bash
# Flash Lab_PD_PSU (STM32G474) via ST-Link.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${ROOT}/build/Debug"
ELF="${BUILD_DIR}/Lab_PD_PSU.elf"
BIN="${BUILD_DIR}/Lab_PD_PSU.bin"
FLASH_ADDR=0x08000000

find_stm32_programmer_cli() {
    if [[ -n "${STM32_PROGRAMMER_CLI:-}" ]] && [[ -x "${STM32_PROGRAMMER_CLI}" ]]; then
        echo "${STM32_PROGRAMMER_CLI}"
        return 0
    fi
  local candidate
  for candidate in \
    "/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.*/tools/bin/STM32_Programmer_CLI" \
    "${HOME}/Library/Application Support/stm32cube/bundles/programmer/"*/bin/STM32_Programmer_CLI \
    "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI"; do
    # shellcheck disable=SC2086
    for path in ${candidate}; do
      if [[ -x "${path}" ]]; then
        echo "${path}"
        return 0
      fi
    done
  done
  return 1
}

echo "Building Debug..."
cmake --preset Debug -S "${ROOT}" -B "${BUILD_DIR%/*}/Debug" >/dev/null 2>&1 || cmake --preset Debug
cmake --build --preset Debug

if [[ ! -f "${ELF}" ]]; then
    echo "error: ${ELF} missing after build" >&2
    exit 1
fi

if command -v arm-none-eabi-objcopy >/dev/null 2>&1; then
    arm-none-eabi-objcopy -O binary "${ELF}" "${BIN}"
elif command -v arm-none-eabi-objcopy >/dev/null 2>&1; then
    arm-none-eabi-objcopy -O binary "${ELF}" "${BIN}"
else
    echo "error: arm-none-eabi-objcopy not found" >&2
    exit 1
fi

BIN_SIZE="$(wc -c < "${BIN}" | tr -d ' ')"
echo "BIN size: ${BIN_SIZE} bytes"

if command -v st-flash >/dev/null 2>&1; then
    echo "Probing ST-Link (st-flash)..."
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
    exit 0
fi

STM32_CLI="$(find_stm32_programmer_cli || true)"
if [[ -n "${STM32_CLI}" ]]; then
    echo "Flashing with STM32_Programmer_CLI: ${STM32_CLI}"
    "${STM32_CLI}" -c port=SWD mode=UR reset=HWrst \
        -w "${ELF}" -v -rst
    echo "Done."
    exit 0
fi

echo "error: need st-flash (brew install stlink) or STM32_Programmer_CLI" >&2
exit 1
