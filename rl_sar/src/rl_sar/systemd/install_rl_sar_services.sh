#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: sudo bash install/share/rl_sar/systemd/install_rl_sar_services.sh [--root PATH] [--no-restart]

Installs rl-sar-main.service and rl-sar-trigger.service for the current
workspace. The detected workspace path is written to /etc/default/rl-sar.

Recommended from the built rl_sar workspace:
  sudo bash install/share/rl_sar/systemd/install_rl_sar_services.sh --root "$(pwd)"
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
rl_sar_root=""
restart_service=true

while [[ $# -gt 0 ]]; do
    case "$1" in
        --root)
            if [[ $# -lt 2 ]]; then
                echo "Missing value for --root" >&2
                exit 2
            fi
            rl_sar_root="$2"
            shift 2
            ;;
        --no-restart)
            restart_service=false
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ "${EUID}" -ne 0 ]]; then
    echo "Please run with sudo." >&2
    exit 1
fi

if [[ -z "${rl_sar_root}" ]]; then
    if [[ -f "${script_dir}/../../../setup.bash" ]]; then
        rl_sar_root="$(cd "${script_dir}/../../../.." && pwd)"
    elif [[ -f "${PWD}/install/setup.bash" ]]; then
        rl_sar_root="${PWD}"
    else
        echo "Cannot detect rl_sar workspace root. Run from the built rl_sar workspace or pass --root /path/to/rl_sar." >&2
        exit 1
    fi
fi

rl_sar_root="$(cd "${rl_sar_root}" && pwd)"

if [[ ! -f "${rl_sar_root}/install/setup.bash" ]]; then
    echo "Missing ${rl_sar_root}/install/setup.bash. Build first from that workspace with ./build.sh rl_sar." >&2
    exit 1
fi

if [[ ! -f "${script_dir}/rl-sar-main.service" || ! -f "${script_dir}/rl-sar-trigger.service" ]]; then
    echo "Missing rl-sar service templates in ${script_dir}." >&2
    exit 1
fi

install -m 0644 "${script_dir}/rl-sar-main.service" /etc/systemd/system/rl-sar-main.service
install -m 0644 "${script_dir}/rl-sar-trigger.service" /etc/systemd/system/rl-sar-trigger.service

cat >/etc/default/rl-sar <<EOF
RL_SAR_ROOT="${rl_sar_root}"
EOF

systemctl daemon-reload
systemctl reset-failed rl-sar-trigger.service rl-sar-main.service >/dev/null 2>&1 || true
systemctl enable rl-sar-trigger.service

if [[ "${restart_service}" == true ]]; then
    systemctl restart rl-sar-trigger.service
fi

echo "Installed rl-sar systemd services."
echo "RL_SAR_ROOT=${rl_sar_root}"
echo "Check logs with:"
echo "  journalctl -u rl-sar-trigger.service -u rl-sar-main.service -f"
