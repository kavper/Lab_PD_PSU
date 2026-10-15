#!/usr/bin/env bash
# Build, then collect a debug snapshot. Programs only with --approve-flash.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
"${DIR}/build.sh"
if [[ "${1:-}" == "--approve-flash" ]]; then
    "${DIR}/flash.sh" --approve
else
    echo "Programowanie pominięte. Podaj --approve-flash, jeśli nowy obraz ma trafić do mikrokontrolera."
fi
"${DIR}/snapshot.sh"
