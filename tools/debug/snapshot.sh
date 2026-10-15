#!/usr/bin/env bash
# Halt, collect registers, stack, fault status, and a temporary breakpoint, then resume.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=env.sh
source "${DIR}/env.sh"
debug_require_elf

stamp="$(date +%Y%m%d-%H%M%S)"
out="${LOG_DIR}/snapshot-${stamp}.txt"
elf_vec="$(python3 - "${OBJDUMP_BIN}" "${ELF}" <<'PY'
import subprocess, sys
text = subprocess.check_output([sys.argv[1], "-s", "-j", ".isr_vector", sys.argv[2]], text=True)
for line in text.splitlines():
    parts = line.split()
    if parts and parts[0] == "8000000":
        words = []
        for chunk in parts[1:5]:
            words.append(f"0x{int.from_bytes(bytes.fromhex(chunk), 'little'):08x}")
        print(" ".join(words))
        break
PY
)"

{
    echo "MCU ${MCU}"
    echo "ELF ${ELF}"
    echo "ELF_VECTOR ${elf_vec}"
    echo
    "${DIR}/gdb.sh" \
        "printf \"CPUID \"" "x/wx 0xE000ED00" \
        "printf \"CFSR \"" "x/wx 0xE000ED28" \
        "printf \"HFSR \"" "x/wx 0xE000ED2C" \
        "printf \"MMFAR \"" "x/wx 0xE000ED34" \
        "printf \"BFAR \"" "x/wx 0xE000ED38" \
        "printf \"\\n=== REGISTERS ===\\n\"" "info registers" \
        "printf \"\\n=== BACKTRACE ===\\n\"" "bt" \
        "printf \"\\n=== TARGET FLASH ===\\n\"" "x/4wx 0x08000000" \
        "printf \"\\n=== SYMBOL ===\\n\"" "info address App_Run" "print/x &g_bq76922" \
        "printf \"\\n=== BREAK ===\\n\"" "break App_Run" "info break" "delete breakpoints" \
        "printf \"breakpoint removed\\n\""
} | tee "${out}"

python3 - "${out}" <<'PY' | tee -a "${out}"
import re, sys
from pathlib import Path
text = Path(sys.argv[1]).read_text(encoding="utf-8", errors="replace")
match = re.search(
    r"0x0*8000000\s+\S+:\s+((?:0x[0-9a-fA-F]+\s+){3}0x[0-9a-fA-F]+)",
    text,
)
print()
print("TARGET_VECTOR " + (match.group(1).lower() if match else "missing"))
PY

echo
python3 "${DIR}/analyze.py" "${out}" | tee -a "${out}"
ln -sfn "$(basename "${out}")" "${LOG_DIR}/snapshot-latest.txt"
echo "saved ${out}"
