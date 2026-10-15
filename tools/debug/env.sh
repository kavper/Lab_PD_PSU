#!/usr/bin/env bash
# Shared paths for the STM32CubeIDE / ST-LINK GDB workflow.
# Firmware sources, CMake presets, and the linker script stay untouched.
set -euo pipefail

DEBUG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${DEBUG_DIR}/../.." && pwd)"

CUBE_BUNDLES="${CUBE_BUNDLE_PATH:-${HOME}/Library/Application Support/stm32cube/bundles}"
MCU="STM32G474RCT6"
CORE="Cortex-M4"
GDB_HOST="127.0.0.1"
GDB_PORT="${GDB_PORT:-61234}"
# Release matches what runs on the board; build.sh adds symbols without changing code.
BUILD_TYPE="${BUILD_TYPE:-Release}"
BUILD_DIR="${BUILD_DIR:-${ROOT}/build/${BUILD_TYPE}}"
ELF="${ELF:-${BUILD_DIR}/Lab_PD_PSU.elf}"
BIN="${BIN:-${BUILD_DIR}/Lab_PD_PSU.bin}"
LOG_DIR="${DEBUG_DIR}/logs"
FLASH_BASE="0x08000000"
SRAM_BASE="0x20000000"
FLASH_SIZE="$((256 * 1024))"
SRAM_SIZE="$((128 * 1024))"

# Versions locked by .settings/bundles-lock.store.json (STM32CubeIDE).
GCC_BIN="${CUBE_BUNDLES}/gnu-tools-for-stm32/14.3.1+st.2/bin/arm-none-eabi-gcc"
GDB_BIN="${CUBE_BUNDLES}/gnu-gdb-for-stm32/14.3.1+st.2/bin/arm-none-eabi-gdb"
OBJDUMP_BIN="${CUBE_BUNDLES}/gnu-tools-for-stm32/14.3.1+st.2/bin/arm-none-eabi-objdump"
GDBSERVER_BIN="${CUBE_BUNDLES}/stlink-gdbserver/7.13.0+st.3/bin/ST-LINK_gdbserver"
PROGRAMMER_BIN="${CUBE_BUNDLES}/programmer/2.22.0+st.1/bin"
CMAKE_BIN="${CUBE_BUNDLES}/cmake/4.3.1+st.1/CMake.app/Contents/bin/cmake"
NINJA_BIN="${CUBE_BUNDLES}/ninja/1.13.2+st.1/bin/ninja"

if [[ ! -x "${GDB_BIN}" ]]; then
    echo "error: CubeIDE GDB not found at ${GDB_BIN}" >&2
    exit 1
fi

mkdir -p "${LOG_DIR}"

debug_port_open() {
    nc -z -G 1 "${GDB_HOST}" "${GDB_PORT}" >/dev/null 2>&1
}

debug_require_elf() {
    if [[ ! -f "${ELF}" ]]; then
        echo "error: ${ELF} missing. Run tools/debug/build.sh first." >&2
        exit 1
    fi
}
