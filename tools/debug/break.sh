#!/usr/bin/env bash
# Plant a breakpoint, show it, then remove it and resume.
# --arm leaves the breakpoint in place and the core halted.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
ARM=0
if [[ "${1:-}" == "--arm" ]]; then
    ARM=1
    shift
fi
if [[ $# -lt 1 ]]; then
    echo "Usage: $0 [--arm] <location>" >&2
    echo "Example: $0 App_Run" >&2
    exit 2
fi
LOC="$1"
if [[ "${ARM}" -eq 1 ]]; then
    exec "${DIR}/gdb.sh" --hold "break ${LOC}" "info break"
fi
exec "${DIR}/gdb.sh" "break ${LOC}" "info break" "delete breakpoints" "printf \"breakpoint removed\\n\""
