#!/usr/bin/env bash
# One-time Mac/Linux setup so Cursor Cloud Agent can flash via My Machines worker.
# Run on the bench PC (ST-Link + USB attached). Keep the worker terminal open.
set -euo pipefail

REPO_URL="${REPO_URL:-https://github.com/kavper/Lab_PD_PSU.git}"
BRANCH="${BRANCH:-cursor/hw-rev2-gan-bms-0e60}"
WORKER_NAME="${WORKER_NAME:-lab-psu}"
INSTALL_DIR="${INSTALL_DIR:-${HOME}/Lab_PD_PSU}"

echo "== Lab_PD_PSU agent flash setup =="
echo "Install dir: ${INSTALL_DIR}"
echo "Branch:      ${BRANCH}"
echo "Worker name: ${WORKER_NAME}"
echo

if [[ "$(uname -s)" == "Darwin" ]]; then
    if ! command -v brew >/dev/null 2>&1; then
        echo "error: Homebrew required on macOS — https://brew.sh" >&2
        exit 1
    fi
    echo "Installing build/flash tools (brew)..."
    brew list stlink >/dev/null 2>&1 || brew install stlink
    brew list cmake >/dev/null 2>&1 || brew install cmake
    brew list ninja >/dev/null 2>&1 || brew install ninja
    brew list arm-gcc-none-eabi >/dev/null 2>&1 || brew install arm-gcc-none-eabi
    brew list git >/dev/null 2>&1 || true
fi

if ! command -v agent >/dev/null 2>&1; then
    echo "Installing Cursor CLI (agent)..."
    curl -fsSL https://cursor.com/install | bash
    export PATH="${HOME}/.local/bin:${PATH}"
fi

if ! command -v agent >/dev/null 2>&1; then
    echo "error: 'agent' not on PATH after install. Add ~/.local/bin to PATH and re-run." >&2
    exit 1
fi

if [[ -d "${INSTALL_DIR}/.git" ]]; then
    echo "Updating ${INSTALL_DIR}..."
    git -C "${INSTALL_DIR}" fetch origin "${BRANCH}"
    git -C "${INSTALL_DIR}" checkout "${BRANCH}"
    git -C "${INSTALL_DIR}" pull --ff-only origin "${BRANCH}" || true
else
    echo "Cloning ${REPO_URL} -> ${INSTALL_DIR}..."
    git clone --branch "${BRANCH}" --single-branch "${REPO_URL}" "${INSTALL_DIR}"
fi

echo
echo "Probing ST-Link..."
if command -v st-info >/dev/null 2>&1; then
    st-info --probe --connect-under-reset || true
else
    echo "warn: st-info missing"
fi

echo
echo "============================================================"
echo "NEXT (one time in Cursor):"
echo "  1. Keep THIS terminal open."
echo "  2. In Cursor: Agents -> Run on -> ${WORKER_NAME}"
echo "  3. Tell the agent: wgraj firmware"
echo
echo "Starting worker (Ctrl+C to stop)..."
echo "============================================================"
cd "${INSTALL_DIR}"
exec agent worker start --name "${WORKER_NAME}" --worker-dir "${INSTALL_DIR}"
