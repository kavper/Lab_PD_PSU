#!/usr/bin/env bash
# Start or stop STM32CubeIDE ST-LINK GDB Server.
# Attach mode: no reset, no mass erase, no option bytes.
set -euo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=env.sh
source "${DIR}/env.sh"

usage() {
    echo "Usage: $0 {start|stop|status}" >&2
    exit 2
}

listener_pid() {
    lsof -nP -iTCP:"${GDB_PORT}" -sTCP:LISTEN -t 2>/dev/null | head -1 || true
}

status() {
    local pid
    pid="$(listener_pid)"
    if [[ -n "${pid}" ]]; then
        echo "ST-LINK GDB Server listening on ${GDB_HOST}:${GDB_PORT} (pid ${pid})"
        ps -p "${pid}" -o pid=,command= || true
        return 0
    fi
    echo "ST-LINK GDB Server is not listening on ${GDB_PORT}"
    return 1
}

stop() {
    local pid
    pid="$(listener_pid)"
    if [[ -z "${pid}" ]]; then
        echo "ST-LINK GDB Server already stopped"
        return 0
    fi
    echo "Stopping pid ${pid}"
    kill "${pid}" 2>/dev/null || true
    sleep 0.5
    if [[ -n "$(listener_pid)" ]]; then
        echo "error: server still listening" >&2
        return 1
    fi
    echo "stopped"
}

start() {
    if status >/dev/null; then
        status
        return 0
    fi
    if [[ ! -x "${GDBSERVER_BIN}" ]]; then
        echo "error: ${GDBSERVER_BIN} missing" >&2
        exit 1
    fi
    local stdout_log="${LOG_DIR}/gdbserver.stdout"
    local server_log="${LOG_DIR}/gdbserver.log"
    : >"${stdout_log}"
    # -e persistent, -d SWD, -g attach (do not reset the running firmware)
    # -m 0 single core. Never pass --erase-all.
    nohup "${GDBSERVER_BIN}" \
        -e -d -g -p "${GDB_PORT}" -l 2 -m 0 \
        -f "${server_log}" \
        -cp "${PROGRAMMER_BIN}" \
        >>"${stdout_log}" 2>&1 &
    local i
    for i in 1 2 3 4 5 6 7 8 9 10; do
        if debug_port_open; then
            echo "ST-LINK GDB Server ready on ${GDB_HOST}:${GDB_PORT}"
            return 0
        fi
        if grep -q "Error in initializing ST-LINK device" "${stdout_log}"; then
            echo "error: ST-LINK GDB Server could not open the probe" >&2
            cat "${stdout_log}" >&2
            return 1
        fi
        sleep 0.4
    done
    echo "error: server did not open port ${GDB_PORT}" >&2
    cat "${stdout_log}" >&2
    return 1
}

[[ $# -eq 1 ]] || usage
case "$1" in
    start) start ;;
    stop) stop ;;
    status) status ;;
    *) usage ;;
esac
