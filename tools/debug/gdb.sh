#!/usr/bin/env bash
# Run one GDB batch against ST-LINK GDB Server.
# Without --hold the core is resumed with detach, so a PSU loop is not left stopped.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=env.sh
source "${DIR}/env.sh"

HOLD=0
if [[ "${1:-}" == "--hold" ]]; then
    HOLD=1
    shift
fi
if [[ $# -lt 1 ]]; then
    echo "Usage: $0 [--hold] <gdb-command> [gdb-command...]" >&2
    exit 2
fi

"${DIR}/gdbserver.sh" start
debug_require_elf

args=(
    -nx -batch
    -ex "set pagination off"
    -ex "set confirm off"
    -ex "set mem inaccessible-by-default off"
    -ex "set print elements 16"
    -ex "set tcp connect-timeout 8"
    -ex "set remotetimeout 8"
    -ex "file ${ELF}"
    -ex "target remote ${GDB_HOST}:${GDB_PORT}"
)
local_cmd=""
for local_cmd in "$@"; do
    args+=(-ex "${local_cmd}")
done
if [[ "${HOLD}" -eq 0 ]]; then
    args+=(-ex "detach")
fi

"${GDB_BIN}" "${args[@]}"
