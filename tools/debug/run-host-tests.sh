#!/usr/bin/env bash
# Host-side unit tests (project runner). These do not touch ST-LINK.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
exec bash "${DIR}/../../scripts/run_host_tests.sh" "$@"
