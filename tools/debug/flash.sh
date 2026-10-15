#!/usr/bin/env bash
# Program the existing .bin only after an explicit --approve.
# Writes the image and verifies it. Does not mass-erase and does not change option bytes.
# The CMake target named "flash" is different: it sets DBANK and erases the whole device.
# Do not use that target from here.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=env.sh
source "${DIR}/env.sh"

if [[ "${1:-}" != "--approve" ]]; then
    echo "Programowanie nie zostało uruchomione." >&2
    echo "To nadpisze firmware w pamięci Flash od 0x08000000 i zresetuje rdzeń." >&2
    echo "Nie kasuje całej pamięci i nie zmienia option bytes." >&2
    echo "Uruchom świadomie: $0 --approve" >&2
    exit 2
fi

debug_require_elf
if [[ ! -f "${BIN}" ]]; then
    echo "error: ${BIN} missing" >&2
    exit 1
fi

cli="${PROGRAMMER_BIN}/STM32_Programmer_CLI"
if [[ ! -x "${cli}" ]]; then
    echo "error: ${cli} missing" >&2
    exit 1
fi

bin_size="$(wc -c < "${BIN}" | tr -d ' ')"
echo "image ${bin_size} bytes"
if (( bin_size > FLASH_SIZE )); then
    echo "error: image exceeds 256KB" >&2
    exit 1
fi
if (( bin_size > 128 * 1024 )); then
    echo "note: image is larger than 128KB. STM32G474 must already be in single-bank mode (DBANK=0)."
    echo "note: this script will not change option bytes. If programming reports a memory limit, stop and ask."
fi

# Stop our GDB server so CubeProgrammer can open ST-LINK. Do not reset option bytes.
"${DIR}/gdbserver.sh" stop || true

cmd=(
    "${cli}"
    -c "port=SWD" "mode=UR" "reset=HWrst"
    -w "${BIN}" "${FLASH_BASE}"
    -v
    -rst
)
printf 'command:'
printf ' %q' "${cmd[@]}"
printf '\n'
for arg in "${cmd[@]}"; do
    if [[ "${arg}" == "-e" || "${arg}" == "all" || "${arg}" == "-ob" || "${arg}" == "--erase-all" ]]; then
        echo "error: refused mass erase or option-byte change" >&2
        exit 1
    fi
done
"${cmd[@]}"
echo "Programowanie zakończone. GDB server został zatrzymany; uruchom tools/debug/gdbserver.sh start."
