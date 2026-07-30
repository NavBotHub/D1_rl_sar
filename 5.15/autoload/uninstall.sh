#!/usr/bin/env bash
# Uninstall D1 USB2CANFD_Dual power-on autoload configuration
# Usage: sudo bash uninstall.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Root privileges are required. Use: sudo bash $0" >&2
  exit 1
fi

echo "[1/4] Stopping and disabling services"
for IF in can1 can2; do
  systemctl stop "d1-canfd@${IF}.service" 2>/dev/null || true
done
systemctl disable --now d1-gs-usb.service 2>/dev/null || true

echo "[2/4] Removing unit and config files"
rm -f /etc/systemd/system/d1-gs-usb.service
rm -f /etc/systemd/system/d1-canfd@.service
rm -f /etc/udev/rules.d/85-d1-canfd.rules
rm -f /etc/modules-load.d/d1-can.conf

echo "[3/4] Reloading systemd and udev"
systemctl daemon-reload
udevadm control --reload-rules

echo "[4/4] Unloading kernel modules (optional)"
modprobe -r gs_usb 2>/dev/null || true

echo "Done."
