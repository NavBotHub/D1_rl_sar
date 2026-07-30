#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: sudo bash docs/dtb/install_dmgo_dtb.sh [options]

Installs the repository DMGO carrier-board DTB and points an extlinux label at it.

Options:
  --label NAME          extlinux LABEL to update (default: primary)
  --boot-dtb-dir PATH   target boot DTB directory (default: /boot/dtb)
  --extlinux PATH       extlinux config path (default: /boot/extlinux/extlinux.conf)
  --dry-run             print planned actions without writing system files
  -h, --help            show this help
EOF
}

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
dtb_name="kernel_tegra234-p3768-0000+p3767-0003-nv.dtb"
source_dtb="${script_dir}/${dtb_name}"
boot_dtb_dir="/boot/dtb"
extlinux="/boot/extlinux/extlinux.conf"
label="primary"
dry_run=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --label)
            if [[ $# -lt 2 ]]; then
                echo "Missing value for --label" >&2
                exit 2
            fi
            label="$2"
            shift 2
            ;;
        --boot-dtb-dir)
            if [[ $# -lt 2 ]]; then
                echo "Missing value for --boot-dtb-dir" >&2
                exit 2
            fi
            boot_dtb_dir="$2"
            shift 2
            ;;
        --extlinux)
            if [[ $# -lt 2 ]]; then
                echo "Missing value for --extlinux" >&2
                exit 2
            fi
            extlinux="$2"
            shift 2
            ;;
        --dry-run)
            dry_run=true
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

if [[ ! -f "${source_dtb}" ]]; then
    echo "Missing repository DTB: ${source_dtb}" >&2
    exit 1
fi

if [[ ! -f "${extlinux}" ]]; then
    echo "Missing extlinux config: ${extlinux}" >&2
    exit 1
fi

if [[ ! -d "${boot_dtb_dir}" ]]; then
    echo "Missing boot DTB directory: ${boot_dtb_dir}" >&2
    exit 1
fi

if ! grep -Eq "^[[:space:]]*LABEL[[:space:]]+${label}([[:space:]]|\$)" "${extlinux}"; then
    echo "Cannot find LABEL ${label} in ${extlinux}. Pass --label NAME if needed." >&2
    exit 1
fi

target_dtb="${boot_dtb_dir}/${dtb_name}"
fdt_path="${target_dtb}"
timestamp="$(date +%Y%m%d-%H%M%S)"

echo "Source DTB: ${source_dtb}"
echo "Target DTB: ${target_dtb}"
echo "Extlinux:   ${extlinux}"
echo "Label:      ${label}"
echo "FDT path:   ${fdt_path}"

if [[ "${dry_run}" == true ]]; then
    echo "[dry-run] Would copy DTB to ${target_dtb}"
    if [[ -f "${target_dtb}" ]]; then
        echo "[dry-run] Would backup existing DTB to ${target_dtb}.bak.${timestamp}"
    fi
    echo "[dry-run] Would backup extlinux to ${extlinux}.bak.${timestamp}"
    echo "[dry-run] Would set LABEL ${label} FDT to ${fdt_path}"
    exit 0
fi

if [[ -f "${target_dtb}" ]]; then
    cp -a "${target_dtb}" "${target_dtb}.bak.${timestamp}"
fi
install -m 0644 "${source_dtb}" "${target_dtb}"

cp -a "${extlinux}" "${extlinux}.bak.${timestamp}"

tmp_extlinux="$(mktemp)"
cleanup() {
    rm -f "${tmp_extlinux}"
}
trap cleanup EXIT

awk -v label="${label}" -v fdt="${fdt_path}" '
BEGIN {
    in_label = 0
    found = 0
    fdt_done = 0
    fdt_line = "      FDT " fdt
}
function maybe_add_fdt() {
    if (in_label && !fdt_done) {
        print fdt_line
        fdt_done = 1
    }
}
$1 == "LABEL" {
    maybe_add_fdt()
    in_label = ($2 == label)
    if (in_label) {
        found = 1
        fdt_done = 0
    }
    print
    next
}
in_label && $1 == "FDT" {
    if (!fdt_done) {
        print fdt_line
        fdt_done = 1
    }
    next
}
in_label && $1 == "APPEND" && !fdt_done {
    print fdt_line
    fdt_done = 1
    print
    next
}
{
    print
    if (in_label && $1 == "INITRD" && !fdt_done) {
        print fdt_line
        fdt_done = 1
    }
}
END {
    maybe_add_fdt()
    if (!found) {
        exit 3
    }
}
' "${extlinux}" > "${tmp_extlinux}"

extlinux_mode="$(stat -c "%a" "${extlinux}")"
extlinux_owner="$(stat -c "%u" "${extlinux}")"
extlinux_group="$(stat -c "%g" "${extlinux}")"
install -m "${extlinux_mode}" -o "${extlinux_owner}" -g "${extlinux_group}" "${tmp_extlinux}" "${extlinux}"

sync

echo "Installed DMGO DTB and updated LABEL ${label}."
echo "Reboot to apply:"
echo "  sudo reboot"
echo "After reboot, verify:"
cat <<'EOF'
  tr -d '\0' < /proc/device-tree/compatible; echo
  for s in 3100000 3110000 3140000; do
    echo "===== serial@$s ====="
    tr -d '\0' < /proc/device-tree/bus@0/serial@$s/status 2>/dev/null || echo "no status"
    echo
  done
EOF
