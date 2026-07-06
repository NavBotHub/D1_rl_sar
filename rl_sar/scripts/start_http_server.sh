#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RL_SAR_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
RUN_DIR="${RUN_DIR:-/tmp/navbot-webtest}"
LOG_FILE="${LOG_FILE:-${RUN_DIR}/server.log}"
PID_FILE="${PID_FILE:-${RUN_DIR}/robot_http_control_server.pid}"
HOST="${HOST:-0.0.0.0}"
PORT="${PORT:-8080}"

if [[ "${EUID}" -ne 0 ]]; then
  exec sudo -E bash "$0" "$@"
fi

mkdir -p "${RUN_DIR}"

if [[ -f "${PID_FILE}" ]]; then
  existing_pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
  if [[ -n "${existing_pid}" ]] && kill -0 "${existing_pid}" 2>/dev/null; then
    echo "robot_http_control_server.py is already running: pid ${existing_pid}"
    exit 0
  fi
fi

existing_pid="$(pgrep -f "robot_http_control_server.py --host ${HOST} --port ${PORT}" | head -n 1 || true)"
if [[ -n "${existing_pid}" ]]; then
  echo "${existing_pid}" > "${PID_FILE}"
  echo "robot_http_control_server.py is already running: pid ${existing_pid}"
  exit 0
fi

cd "${RL_SAR_DIR}"

nohup bash -lc "
  cd '${RL_SAR_DIR}' &&
  source /opt/ros/humble/setup.bash &&
  source install/setup.bash &&
  exec python3 scripts/robot_http_control_server.py --host '${HOST}' --port '${PORT}'
" >"${LOG_FILE}" 2>&1 &

pid="$!"
echo "${pid}" > "${PID_FILE}"
sleep 1

if ! kill -0 "${pid}" 2>/dev/null; then
  echo "Failed to start robot_http_control_server.py. Log:"
  tail -80 "${LOG_FILE}" || true
  exit 1
fi

echo "Started robot_http_control_server.py on ${HOST}:${PORT}, pid ${pid}"
echo "Log: ${LOG_FILE}"
