#!/usr/bin/env bash
# Halt the core and leave it halted. Resume with continue.sh.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
exec "${DIR}/gdb.sh" --hold "monitor halt" "printf \"HALTED\\n\"" "info registers"
