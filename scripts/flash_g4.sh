#!/usr/bin/env bash
# Flash Lab_PD_PSU (STM32G474RCTx, 256KB) via ST-Link.
#
# CubeProgrammer "Operation exceeds memory limits" on images ~>128KB usually
# means the chip is still in factory dual-bank mode (DBANK=1): Bank2 lives at
# 0x08040000, so 0x08020000..0x0803FFFF is not programmable. This project's
# linker map is contiguous 256KB and requires DBANK=0 (single-bank).
#
# Prefer .bin over .elf: ELF PT_LOAD MemSiz can include .bss and make the tool
# think flash extends past a bank boundary even when FileSiz fits.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${ROOT}/build/Debug"
ELF="${BUILD_DIR}/Lab_PD_PSU.elf"
BIN="${BUILD_DIR}/Lab_PD_PSU.bin"
HEX="${BUILD_DIR}/Lab_PD_PSU.hex"
FLASH_ADDR=0x08000000
FLASH_SIZE=$((256 * 1024))
BANK1_SIZE=$((128 * 1024))

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

ensure_single_bank_ob() {
    local cli="$1"
    echo "Ensuring option byte DBANK=0 (single-bank 256KB contiguous flash)..."
    # Changing DBANK requires a mass-erase on G4; then program image.
    "${cli}" -c port=SWD mode=UR reset=HWrst -ob DBANK=0 || {
        echo "warning: could not write DBANK=0; if erase fails, set it in CubeProgrammer GUI:" >&2
        echo "  Option bytes → User Configuration → DBANK = 0 (unchecked), Apply, power-cycle." >&2
        return 0
    }
    echo "Mass-erase after bank-mode change..."
    "${cli}" -c port=SWD mode=UR reset=HWrst -e all || true
}

echo "Building Debug..."
cmake --preset Debug -S "${ROOT}" -B "${BUILD_DIR}" >/dev/null 2>&1 || cmake --preset Debug
cmake --build --preset Debug

if [[ ! -f "${ELF}" ]]; then
    echo "error: ${ELF} missing after build" >&2
    exit 1
fi

if ! command -v arm-none-eabi-objcopy >/dev/null 2>&1; then
    echo "error: arm-none-eabi-objcopy not found (brew install arm-gcc-none-eabi)" >&2
    exit 1
fi

arm-none-eabi-objcopy -O binary "${ELF}" "${BIN}"
arm-none-eabi-objcopy -O ihex "${ELF}" "${HEX}"

BIN_SIZE="$(wc -c < "${BIN}" | tr -d ' ')"
echo "BIN size: ${BIN_SIZE} bytes ($((BIN_SIZE * 100 / FLASH_SIZE))% of 256KB)"

if (( BIN_SIZE > FLASH_SIZE )); then
    echo "error: image ${BIN_SIZE} exceeds 256KB flash" >&2
    exit 1
fi

if (( BIN_SIZE > BANK1_SIZE )); then
    echo "note: image >128KB — requires DBANK=0 (single-bank). flash_g4 will set it via CubeProgrammer."
fi

STM32_CLI="$(find_stm32_programmer_cli || true)"

if [[ -n "${STM32_CLI}" ]]; then
    echo "Flashing with STM32_Programmer_CLI: ${STM32_CLI}"
    ensure_single_bank_ob "${STM32_CLI}"
    # Program BIN (not ELF) at flash base — avoids MemSiz/BSS false ranges.
    "${STM32_CLI}" -c port=SWD mode=UR reset=HWrst \
        -w "${BIN}" "${FLASH_ADDR}" -v -rst
    echo "Done."
    exit 0
fi

if command -v st-flash >/dev/null 2>&1; then
    echo "Probing ST-Link (st-flash)..."
    PROBE_OUT="$(st-info --probe --connect-under-reset 2>&1 || true)"
    echo "${PROBE_OUT}"
    if echo "${PROBE_OUT}" | grep -qE 'Found 0 stlink'; then
        echo "error: no ST-Link programmer detected" >&2
        exit 1
    fi

    if (( BIN_SIZE > BANK1_SIZE )); then
        echo "warning: st-flash cannot clear DBANK. If write fails past 128KB, run CubeProgrammer once:" >&2
        echo "  Option bytes → DBANK=0, Apply; or install STM32_Programmer_CLI and re-run." >&2
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

echo "error: need STM32_Programmer_CLI (preferred) or st-flash (brew install stlink)" >&2
exit 1
