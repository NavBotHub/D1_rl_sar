#!/usr/bin/env bash
# 卸载 D1 USB2CANFD_Dual 上电自动加载配置
# 用法: sudo bash uninstall.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "需要 root 权限，请用: sudo bash $0" >&2
  exit 1
fi

echo "[1/4] 停止并 disable 服务"
for IF in can1 can2; do
  systemctl stop "d1-canfd@${IF}.service" 2>/dev/null || true
done
systemctl disable --now d1-gs-usb.service 2>/dev/null || true

echo "[2/4] 删除 unit 与配置文件"
rm -f /etc/systemd/system/d1-gs-usb.service
rm -f /etc/systemd/system/d1-canfd@.service
rm -f /etc/udev/rules.d/85-d1-canfd.rules
rm -f /etc/modules-load.d/d1-can.conf

echo "[3/4] 重新加载 systemd 与 udev"
systemctl daemon-reload
udevadm control --reload-rules

echo "[4/4] 卸载内核模块（可选）"
modprobe -r gs_usb 2>/dev/null || true

echo "完成。"
