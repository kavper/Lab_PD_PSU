#!/usr/bin/env bash
# Build, flash G4, read USART1 telemetry. For agent / bench use.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MONITOR_SEC="${MONITOR_SEC:-12}"
SERIAL_PORT="${SERIAL_PORT:-auto}"
BAUD="${BAUD:-115200}"
OUT="${OUT:-/tmp/lab_pd_psu_uart_capture.txt}"

usage() {
    echo "Usage: $0 [--no-flash] [--port /dev/cu.usbmodemXXX] [--seconds N]" >&2
    exit 1
}

DO_FLASH=1
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-flash) DO_FLASH=0; shift ;;
        --port) SERIAL_PORT="$2"; shift 2 ;;
        --seconds) MONITOR_SEC="$2"; shift 2 ;;
        -h|--help) usage ;;
        *) echo "unknown arg: $1" >&2; usage ;;
    esac
done

detect_port() {
    local p
    for p in /dev/cu.usbmodem* /dev/cu.usbserial* /dev/ttyUSB* /dev/ttyACM*; do
        [[ -e "$p" ]] || continue
        echo "$p"
        return 0
    done
    return 1
}

if [[ "${SERIAL_PORT}" == "auto" ]]; then
    SERIAL_PORT="$(detect_port || true)"
    if [[ -z "${SERIAL_PORT}" ]]; then
        echo "error: no USB serial found; set SERIAL_PORT=/dev/cu...." >&2
        exit 1
    fi
fi

echo "== flash-and-monitor =="
echo "repo:   ${ROOT}"
echo "port:   ${SERIAL_PORT}"
echo "baud:   ${BAUD}"
echo "out:    ${OUT}"
echo

if [[ "${DO_FLASH}" -eq 1 ]]; then
    "${ROOT}/scripts/flash_g4.sh"
    echo "Waiting 2 s after reset..."
    sleep 2
fi

if ! python3 -c "import serial" 2>/dev/null; then
    if [[ "$(uname -s)" == "Darwin" ]] && command -v brew >/dev/null 2>&1; then
        brew list python3 >/dev/null 2>&1 || brew install python3
        pip3 install --user pyserial 2>/dev/null || python3 -m pip install --user pyserial
    else
        python3 -m pip install --user pyserial
    fi
fi

python3 - "${SERIAL_PORT}" "${BAUD}" "${MONITOR_SEC}" "${OUT}" <<'PY'
import sys
import time

port, baud, seconds, out_path = sys.argv[1:5]
seconds = float(seconds)

try:
    import serial
except ImportError:
    print("error: pyserial missing (pip3 install pyserial)", file=sys.stderr)
    sys.exit(1)

lines = []
print(f"Reading {port} @ {baud} for {seconds}s ...")
with serial.Serial(port, int(baud), timeout=0.5) as ser:
    end = time.time() + seconds
    while time.time() < end:
        raw = ser.readline()
        if not raw:
            continue
        try:
            line = raw.decode("utf-8", errors="replace").rstrip()
        except Exception:
            line = repr(raw)
        print(line)
        lines.append(line)

with open(out_path, "w", encoding="utf-8") as f:
    f.write("\n".join(lines))
    if lines:
        f.write("\n")

ok = False
last_t = ""
for line in lines:
    if line.startswith("T "):
        last_t = line
        if "run=1" in line and "permit=1" in line and "stage_en=1" in line:
            ok = True

print()
print("=== SUMMARY ===")
print(f"lines={len(lines)}")
if last_t:
    print(f"last_T={last_t}")
else:
    print("last_T=(none)")
print(f"PASS={'yes' if ok else 'no'}")
print(f"saved={out_path}")
sys.exit(0 if ok else 2)
PY
