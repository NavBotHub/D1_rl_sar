#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${RUN_DIR:-/tmp/navbot-webtest}"
PID_FILE="${PID_FILE:-${RUN_DIR}/robot_http_control_server.pid}"
PORT="${PORT:-8080}"

if [[ "${EUID}" -ne 0 ]]; then
  exec sudo -E bash "$0" "$@"
fi

stopped=0

if [[ -f "${PID_FILE}" ]]; then
  pid="$(cat "${PID_FILE}" 2>/dev/null || true)"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
    for _ in {1..20}; do
      kill -0 "${pid}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "${pid}" 2>/dev/null; then
      kill -9 "${pid}" 2>/dev/null || true
    fi
    stopped=1
  fi
  rm -f "${PID_FILE}"
fi

mapfile -t pids < <(
  ps -eo pid=,args= |
    awk -v port="${PORT}" \
      '$0 ~ /robot_http_control_server[.]py/ && $0 ~ "--port " port {print $1}'
)
for pid in "${pids[@]}"; do
  [[ -z "${pid}" || "${pid}" == "$$" ]] && continue
  kill "${pid}" 2>/dev/null || true
  for _ in {1..20}; do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "${pid}" 2>/dev/null; then
    kill -9 "${pid}" 2>/dev/null || true
  fi
  stopped=1
done

if [[ "${stopped}" -eq 1 ]]; then
  echo "Stopped robot_http_control_server.py on port ${PORT}"
else
  echo "robot_http_control_server.py is not running on port ${PORT}"
fi
