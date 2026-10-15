#!/usr/bin/env bash
# Toolchain, host tests, build, and a live ST-LINK snapshot when the probe is free.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=env.sh
source "${DIR}/env.sh"

echo "======== tools ========"
"${DIR}/probe.sh"

echo
echo "======== host tests ========"
set +e
"${DIR}/run-host-tests.sh"
host_rc=$?
set -e
echo "host test exit ${host_rc}"

echo
echo "======== build ========"
"${DIR}/build.sh"

echo
echo "======== flash gate ========"
if "${DIR}/flash.sh"; then
    echo "error: flash.sh ran without approval" >&2
    exit 1
else
    echo "flash.sh correctly refused to program without --approve"
fi

echo
echo "======== hardware ========"
if "${DIR}/gdbserver.sh" start; then
    "${DIR}/snapshot.sh"
else
    echo "ST-LINK GDB Server did not start. Plug in the probe and stop any other debugger, then run tools/debug/snapshot.sh"
    exit 1
fi

exit "${host_rc}"
