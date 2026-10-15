#!/usr/bin/env bash
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=env.sh
source "${DIR}/env.sh"

echo "MCU ${MCU} (${CORE})"
echo "GCC  $("${GCC_BIN}" --version | head -1)"
echo "GDB  $("${GDB_BIN}" --version | head -1)"
echo "GDB server $("${GDBSERVER_BIN}" --version 2>&1 | awk 'NF {print; exit}')"
echo "CMake $("${CMAKE_BIN}" --version | head -1)"
echo "Ninja $("${NINJA_BIN}" --version)"
echo
echo "ST-LINK probes:"
"${GDBSERVER_BIN}" -q -cp "${PROGRAMMER_BIN}" || true
echo
if debug_port_open; then
    echo "GDB port ${GDB_PORT}: open"
else
    echo "GDB port ${GDB_PORT}: closed"
fi
