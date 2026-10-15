#!/usr/bin/env bash
# Read registers, a variable, memory, or Cortex-M fault registers. Resumes afterwards.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
cmd="${1:-}"
case "${cmd}" in
    reg)
        exec "${DIR}/gdb.sh" "info registers"
        ;;
    var)
        [[ $# -eq 2 ]] || { echo "Usage: $0 var <symbol>" >&2; exit 2; }
        exec "${DIR}/gdb.sh" "print/x &${2}" "x/8wx &${2}" "info address ${2}"
        ;;
    mem)
        [[ $# -ge 2 ]] || { echo "Usage: $0 mem <address> [words]" >&2; exit 2; }
        words="${3:-4}"
        exec "${DIR}/gdb.sh" "x/${words}wx ${2}"
        ;;
    fault)
        exec "${DIR}/gdb.sh" \
            "printf \"CFSR \"" "x/wx 0xE000ED28" \
            "printf \"HFSR \"" "x/wx 0xE000ED2C" \
            "printf \"MMFAR \"" "x/wx 0xE000ED34" \
            "printf \"BFAR \"" "x/wx 0xE000ED38"
        ;;
    *)
        echo "Usage: $0 {reg|var <symbol>|mem <address> [words]|fault}" >&2
        exit 2
        ;;
esac
