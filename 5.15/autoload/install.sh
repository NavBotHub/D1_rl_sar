#!/usr/bin/env bash
# D1 USB2CANFD_Dual power-on autoload installer
# Usage: sudo bash install.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "Root privileges are required. Use: sudo bash $0" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KO_PATH="$(cd "$SCRIPT_DIR/.." && pwd)/gs_usb.ko"

if [[ ! -f "$KO_PATH" ]]; then
  echo "Could not find $KO_PATH. Run make in the 5.15 directory first to build gs_usb.ko" >&2
  exit 1
fi

echo "[1/6] Installing modules-load config -> /etc/modules-load.d/d1-can.conf"
install -m 0644 "$SCRIPT_DIR/d1-can.conf" /etc/modules-load.d/d1-can.conf

echo "[2/6] Generating d1-gs-usb.service (bound path: $KO_PATH) -> /etc/systemd/system/"
sed "s|__GS_USB_KO__|$KO_PATH|g" "$SCRIPT_DIR/d1-gs-usb.service" \
  > /etc/systemd/system/d1-gs-usb.service
chmod 0644 /etc/systemd/system/d1-gs-usb.service

echo "[3/6] Installing d1-canfd@.service template -> /etc/systemd/system/"
install -m 0644 "$SCRIPT_DIR/d1-canfd@.service" /etc/systemd/system/d1-canfd@.service

echo "[4/6] Installing udev rules -> /etc/udev/rules.d/85-d1-canfd.rules"
install -m 0644 "$SCRIPT_DIR/85-d1-canfd.rules" /etc/udev/rules.d/85-d1-canfd.rules

echo "[5/6] Reloading systemd and udev"
systemctl daemon-reload
systemctl enable systemd-modules-load.service >/dev/null 2>&1 || true
systemctl enable d1-gs-usb.service
udevadm control --reload-rules

echo "[6/6] Applying now (load CAN modules, load gs_usb, trigger udev)"
modprobe can || true
modprobe can_raw || true
modprobe can_dev || true
systemctl start d1-gs-usb.service
udevadm trigger --subsystem-match=net --action=add
udevadm settle --timeout=10 || true
for IF in can1 can2; do
  if [[ -d /sys/class/net/$IF ]]; then
    systemctl reset-failed "d1-canfd@${IF}.service" 2>/dev/null || true
    systemctl restart "d1-canfd@${IF}.service" || true
  fi
done

echo
echo "Done. Status preview:"
systemctl --no-pager --full status d1-gs-usb.service | sed -n '1,5p' || true
echo "Current CAN devices:"
ip -brief link show type can 2>/dev/null || true
for IF in can1 can2; do
  if [[ -d /sys/class/net/$IF ]]; then
    systemctl --no-pager --full status "d1-canfd@${IF}.service" 2>/dev/null | sed -n '1,5p' || true
    ip -d link show "$IF" | sed -n '1,3p' || true
  else
    echo "[$IF] is not present yet (USB-CAN is unplugged or not recognized); it will be configured automatically when connected"
  fi
done
