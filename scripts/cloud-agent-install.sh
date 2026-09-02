#!/usr/bin/env bash
# Idempotent Cloud Agent bootstrap for the Lab_PD_PSU STM32G474 firmware.
# Installs the ARM cross-toolchain + build tools, then configures and builds
# the Debug preset so the firmware image is ready for inspection.
#
# Note: flashing/UART (scripts/flash-and-monitor.sh) requires a physical
# ST-Link + STM32G474 board and is not runnable in the Cloud Agent VM.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "${ROOT}"

REQUIRED_PKGS=(cmake ninja-build gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi)

need_install=0
for cmd in arm-none-eabi-gcc ninja cmake; do
    command -v "${cmd}" >/dev/null 2>&1 || need_install=1
done

if [[ "${need_install}" -eq 1 ]]; then
    echo "== Installing ARM cross-toolchain and build tools =="
    sudo apt-get update -qq
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "${REQUIRED_PKGS[@]}"
else
    echo "== Toolchain already present, skipping apt install =="
fi

echo "== Tool versions =="
arm-none-eabi-gcc --version | head -1
cmake --version | head -1
ninja --version

echo "== Configure + build (Debug preset) =="
cmake --preset Debug
cmake --build --preset Debug

echo "== Build complete =="
ls -la "${ROOT}/build/Debug/Lab_PD_PSU."{elf,bin,hex}
