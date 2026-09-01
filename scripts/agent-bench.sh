#!/usr/bin/env bash
# Entry point for cloud agent on My Machines worker: flash + UART verify.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "${ROOT}"
exec "${ROOT}/scripts/flash-and-monitor.sh" "$@"
