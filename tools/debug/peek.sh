#!/usr/bin/env bash
# Read firmware variables without halting the core.
# Addresses come from the ELF offline; memory is read over SWD in HOTPLUG mode
# (no reset, no halt), so the MCU keeps serving UART/I2C while sampled.
#
#   peek.sh [--elf path] [--loop N] [--period S] expr [expr...]
#   peek.sh g_bq76922.shutdown_wait g_bq76922.snapshot.battery_status
#
# Only scalar expressions (integers, bools, enums) are decoded.
source "$(dirname "$0")/env.sh"

elf="${ELF}"
loops=1
period=1
exprs=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --elf) elf="$2"; shift 2 ;;
        --loop) loops="$2"; shift 2 ;;
        --period) period="$2"; shift 2 ;;
        *) exprs+=("$1"); shift ;;
    esac
done
[[ ${#exprs[@]} -gt 0 ]] || { echo "usage: $0 [--elf f] [--loop N] [--period S] expr..." >&2; exit 2; }

if debug_port_open; then
    echo "error: ST-LINK GDB Server holds the probe; run gdbserver.sh stop first" >&2
    exit 1
fi

gdb_args=(-batch -nx "${elf}")
for e in "${exprs[@]}"; do
    gdb_args+=(-ex "printf \"%u %u\\n\", (unsigned)&(${e}), (unsigned)sizeof(${e})")
done
layout="$("${GDB_BIN}" "${gdb_args[@]}" 2>&1)" || { echo "${layout}" >&2; exit 1; }

lo=""; hi=""
while read -r addr size; do
    [[ "${addr}" =~ ^[0-9]+$ ]] || { echo "error: cannot resolve: ${addr} ${size}" >&2; exit 1; }
    end=$((addr + size))
    [[ -z "${lo}" || ${addr} -lt ${lo} ]] && lo=${addr}
    [[ -z "${hi}" || ${end} -gt ${hi} ]] && hi=${end}
done <<< "${layout}"
lo=$((lo & ~3))
len=$(((hi - lo + 3) & ~3))

cli="${PROGRAMMER_BIN}/STM32_Programmer_CLI"
for ((i = 1; i <= loops; i++)); do
    raw="$("${cli}" -c port=SWD mode=HOTPLUG -q -r32 "$(printf '0x%08X' "${lo}")" "${len}" 2>&1)" || {
        echo "${raw}" | tail -5 >&2; exit 1; }
    python3 - "${lo}" "${layout}" "${raw}" "${exprs[@]}" <<'EOF'
import re, sys, time
lo = int(sys.argv[1]); layout = sys.argv[2].split("\n"); raw = sys.argv[3]
exprs = sys.argv[4:]
mem = {}
for m in re.finditer(r"0x([0-9A-Fa-f]{8})\s*:((?:\s+[0-9A-Fa-f]{8})+)", raw):
    base = int(m.group(1), 16)
    for k, w in enumerate(m.group(2).split()):
        v = int(w, 16)
        for b in range(4):
            mem[base + 4 * k + b] = (v >> (8 * b)) & 0xFF
out = [time.strftime("%H:%M:%S")]
for e, line in zip(exprs, layout):
    addr, size = map(int, line.split())
    try:
        val = sum(mem[addr + b] << (8 * b) for b in range(size))
    except KeyError:
        out.append(f"{e}=?"); continue
    out.append(f"{e.split('.')[-1]}=0x{val:0{size*2}X}({val})" if size > 1 else f"{e.split('.')[-1]}={val}")
print(" ".join(out))
EOF
    if [[ ${i} -lt ${loops} ]]; then sleep "${period}"; fi
done
