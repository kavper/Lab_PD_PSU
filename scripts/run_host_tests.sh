#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
output=$(mktemp -d "${TMPDIR:-/tmp}/g4-host-tests.XXXXXX")
trap 'rm -rf "$output"' EXIT
for test_source in tests/*.c; do
  sources=("$test_source")
  if [[ "$test_source" == tests/test_bms_soc.c ]]; then sources+=(Core/Src/bms_soc.c); fi
  "${CC:-cc}" -std=c11 -Wall -Wextra -Werror -ICore/Inc "${sources[@]}" -lm -o "$output/test"
  "$output/test"
done
python3 tests/test_bms_shutdown.py
python3 tests/test_clear_recovery.py
python3 tests/test_runtime_supervisor.py
if [[ $# == 2 ]]; then
  python3 tests/test_headroom_pair.py "$1"
  python3 tests/test_peer_protocol.py "$1" "$2"
fi
