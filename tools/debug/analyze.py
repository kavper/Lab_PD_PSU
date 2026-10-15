#!/usr/bin/env python3
"""Decode a tools/debug snapshot: fault registers, stack pointer, ELF match."""
import re
import sys
from pathlib import Path

CFSR_BITS = {
    0: "IACCVIOL",
    1: "DACCVIOL",
    3: "MUNSTKERR",
    4: "MSTKERR",
    5: "MLSPERR",
    7: "MMARVALID",
    8: "IBUSERR",
    9: "PRECISERR",
    10: "IMPRECISERR",
    11: "UNSTKERR",
    12: "STKERR",
    13: "LSPERR",
    15: "BFARVALID",
    16: "UNDEFINSTR",
    17: "INVSTATE",
    18: "INVPC",
    19: "NOCP",
    24: "UNALIGNED",
    25: "DIVBYZERO",
}
HFSR_BITS = {1: "VECTTBL", 30: "FORCED", 31: "DEBUGEVT"}


def words(label, text):
    match = re.search(rf"{label}\s+0x[0-9a-fA-F]+:\s*(0x[0-9a-fA-F]+)", text)
    return int(match.group(1), 16) if match else None


def reg(name, text):
    match = re.search(rf"^{name}\s+(0x[0-9a-fA-F]+)", text, re.M)
    return int(match.group(1), 16) if match else None


def bits(value, table):
    return [name for bit, name in table.items() if value & (1 << bit)]


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    text = path.read_text(encoding="utf-8", errors="replace") if path else sys.stdin.read()
    findings = []

    cpuid = words("CPUID", text)
    if cpuid == 0x410FC241:
        findings.append("CPUID 0x410FC241 matches Cortex-M4 (STM32G474).")
    elif cpuid is None:
        findings.append("CPUID was not read. The debug session did not reach the core.")
    else:
        findings.append(f"CPUID 0x{cpuid:08X} is not Cortex-M4 0x410FC241.")

    cfsr = words("CFSR", text)
    hfsr = words("HFSR", text)
    if cfsr == 0 and hfsr == 0:
        findings.append("CFSR and HFSR are clear. No latched MemManage, BusFault, UsageFault, or forced HardFault.")
    if cfsr:
        findings.append("CFSR " + ", ".join(bits(cfsr, CFSR_BITS)) + f" (0x{cfsr:08X}).")
    if hfsr:
        findings.append("HFSR " + ", ".join(bits(hfsr, HFSR_BITS)) + f" (0x{hfsr:08X}).")

    pc = reg("pc", text)
    sp = reg("sp", text)
    if pc is not None:
        if 0x08000000 <= pc < 0x08040000:
            findings.append(f"PC 0x{pc:08X} is inside the 256KB flash window.")
        else:
            findings.append(f"PC 0x{pc:08X} is outside flash. The core is not executing the application image.")
    if sp is not None:
        if 0x20000000 <= sp < 0x20020000:
            findings.append(f"SP 0x{sp:08X} is inside SRAM.")
        else:
            findings.append(f"SP 0x{sp:08X} is outside SRAM. The stack is corrupt or the core is not halted cleanly.")

    if "corrupt stack" in text or "previous frame identical" in text:
        findings.append("GDB stopped the backtrace on a corrupt frame. The loaded ELF may not match flash, or the frame has no unwind info.")
    if re.search(r"#0\s+0x[0-9a-fA-F]+ in \?\?", text):
        findings.append("Frame #0 has no symbol. Rebuild the matching ELF or the PC is outside this image.")

    elf_vec = re.search(r"ELF_VECTOR\s+((?:0x[0-9a-fA-F]+\s*)+)", text)
    tgt_vec = re.search(r"TARGET_VECTOR\s+((?:0x[0-9a-fA-F]+\s*)+)", text)
    if elf_vec and tgt_vec:
        elf_words = [w.lower() for w in elf_vec.group(1).split()[:2]]
        tgt_words = [w.lower() for w in tgt_vec.group(1).split()[:2]]
        if elf_words == tgt_words:
            findings.append("Flash vector table matches this ELF. Source-level symbols are valid.")
        else:
            findings.append("Flash vector table does not match this ELF. Do not trust variable names or source lines until this image is programmed.")

    if "hardware breakpoints" in text or "breakpoint" in text.lower():
        if "breakpoint removed" in text:
            findings.append("A hardware breakpoint was planted and then removed. The core was not left sitting on it.")

    print("=== debug analysis ===")
    if not findings:
        print("No recognizable debug fields in the snapshot.")
        return 1
    for line in findings:
        print("- " + line)
    hard = any(s in text for s in ("HardFault", "FORCED", "IACCVIOL", "UNDEFINSTR", "INVSTATE"))
    return 2 if (cfsr or hfsr or hard) else 0


if __name__ == "__main__":
    sys.exit(main())
