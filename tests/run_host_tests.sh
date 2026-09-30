#!/bin/sh
set -eu
cd "$(dirname "$0")/.."
test_dir=$(mktemp -d "${TMPDIR:-/tmp}/psu-ucc-tests.XXXXXX")
trap 'rm -rf "$test_dir"' EXIT HUP INT TERM
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -I Core/Inc \
    tests/test_dcdc_hs_policy.c -lm -o "$test_dir/policy"
"$test_dir/policy"
# ST's unused register helpers assume 32-bit pointers; this host can be 64-bit.
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror \
    -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast \
    -DSTM32G474xx -DUSE_HAL_DRIVER \
    -I Core/Inc -I Drivers/STM32G4xx_HAL_Driver/Inc \
    -I Drivers/CMSIS/Include -I Drivers/CMSIS/Device/ST/STM32G4xx/Include \
    tests/test_power_stage_ucc.c -o "$test_dir/power_stage"
"$test_dir/power_stage"
# Turning off fallback refresh pulses must not disable installed UCC supplies.
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror \
    -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast \
    -DSTM32G474xx -DUSE_HAL_DRIVER -DPOWER_STAGE_BOOTSTRAP_REFRESH_ENABLE=0 \
    -I Core/Inc -I Drivers/STM32G4xx_HAL_Driver/Inc \
    -I Drivers/CMSIS/Include -I Drivers/CMSIS/Device/ST/STM32G4xx/Include \
    tests/test_power_stage_ucc.c -o "$test_dir/no_refresh"
"$test_dir/no_refresh"
