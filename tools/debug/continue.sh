#!/usr/bin/env bash
# Resume the core. Connecting halts briefly; detach lets it run again.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
exec "${DIR}/gdb.sh" "printf \"RESUMED\\n\"" "print/x \$pc"
