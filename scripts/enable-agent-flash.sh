#!/usr/bin/env bash
# One-time Mac/Linux bench setup: repo + tools + Cursor worker for agent flash/UART.
set -euo pipefail

REPO_URL="${REPO_URL:-https://github.com/kavper/Lab_PD_PSU.git}"
BRANCH="${BRANCH:-cursor/hw-rev2-gan-bms-0e60}"
WORKER_NAME="${WORKER_NAME:-lab-psu}"
INSTALL_DIR="${INSTALL_DIR:-${HOME}/Lab_PD_PSU}"

echo "== Lab_PD_PSU agent bench setup =="
echo "Install dir: ${INSTALL_DIR}"
echo "Branch:      ${BRANCH}"
echo "Worker name: ${WORKER_NAME}"
echo

if [[ "$(uname -s)" == "Darwin" ]]; then
    if ! command -v brew >/dev/null 2>&1; then
        echo "error: Homebrew required — https://brew.sh" >&2
        exit 1
    fi
    echo "Installing tools (brew)..."
    for pkg in stlink cmake ninja arm-gcc-none-eabi python3; do
        brew list "${pkg}" >/dev/null 2>&1 || brew install "${pkg}"
    done
    python3 -m pip install --user pyserial 2>/dev/null || pip3 install --user pyserial 2>/dev/null || true
fi

if ! command -v agent >/dev/null 2>&1; then
    echo "Installing Cursor CLI..."
    curl -fsSL https://cursor.com/install | bash
    export PATH="${HOME}/.local/bin:${PATH}"
fi

if ! command -v agent >/dev/null 2>&1; then
    echo "error: 'agent' not on PATH. Add ~/.local/bin to ~/.zshrc:" >&2
    echo '  export PATH="$HOME/.local/bin:$PATH"' >&2
    exit 1
fi

if [[ -d "${INSTALL_DIR}/.git" ]]; then
    echo "Updating ${INSTALL_DIR}..."
    git -C "${INSTALL_DIR}" fetch origin "${BRANCH}"
    git -C "${INSTALL_DIR}" checkout "${BRANCH}" 2>/dev/null || \
        git -C "${INSTALL_DIR}" checkout -B "${BRANCH}" "origin/${BRANCH}"
    git -C "${INSTALL_DIR}" pull --ff-only origin "${BRANCH}" || true
else
    echo "Cloning ${REPO_URL}..."
    git clone --branch "${BRANCH}" --single-branch "${REPO_URL}" "${INSTALL_DIR}"
fi

chmod +x "${INSTALL_DIR}/scripts/"*.sh 2>/dev/null || true

echo
echo "Probing ST-Link..."
st-info --probe --connect-under-reset 2>&1 || true

echo
echo "USB serial ports:"
ls /dev/cu.usb* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "(none yet — plug USB-UART)"
echo
echo "============================================================"
echo "KEEP THIS TERMINAL OPEN."
echo ""
echo "In Cursor (THIS chat): Run on -> ${WORKER_NAME}"
echo "Then write: wgraj i sprawdz uart"
echo ""
echo "Agent will run: ./scripts/flash-and-monitor.sh"
echo "============================================================"
echo

cd "${INSTALL_DIR}"
exec agent worker start --name "${WORKER_NAME}" --worker-dir "${INSTALL_DIR}"
