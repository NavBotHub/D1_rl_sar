#!/usr/bin/env bash
# D1 USB2CANFD_Dual 上电自动加载安装脚本
# 用法: sudo bash install.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "需要 root 权限，请用: sudo bash $0" >&2
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KO_PATH="$(cd "$SCRIPT_DIR/.." && pwd)/gs_usb.ko"

if [[ ! -f "$KO_PATH" ]]; then
  echo "未找到 $KO_PATH，请先在 5.15 目录跑 make 编译 gs_usb.ko" >&2
  exit 1
fi

echo "[1/6] 安装 modules-load 配置 -> /etc/modules-load.d/d1-can.conf"
install -m 0644 "$SCRIPT_DIR/d1-can.conf" /etc/modules-load.d/d1-can.conf

echo "[2/6] 生成 d1-gs-usb.service（绑定路径: $KO_PATH）-> /etc/systemd/system/"
sed "s|__GS_USB_KO__|$KO_PATH|g" "$SCRIPT_DIR/d1-gs-usb.service" \
  > /etc/systemd/system/d1-gs-usb.service
chmod 0644 /etc/systemd/system/d1-gs-usb.service

echo "[3/6] 安装 d1-canfd@.service 模板 -> /etc/systemd/system/"
install -m 0644 "$SCRIPT_DIR/d1-canfd@.service" /etc/systemd/system/d1-canfd@.service

echo "[4/6] 安装 udev 规则 -> /etc/udev/rules.d/85-d1-canfd.rules"
install -m 0644 "$SCRIPT_DIR/85-d1-canfd.rules" /etc/udev/rules.d/85-d1-canfd.rules

echo "[5/6] 重新加载 systemd 与 udev"
systemctl daemon-reload
systemctl enable systemd-modules-load.service >/dev/null 2>&1 || true
systemctl enable d1-gs-usb.service
udevadm control --reload-rules

echo "[6/6] 立即生效（加载 can 系列模块、加载 gs_usb、触发 udev）"
modprobe can || true
modprobe can_raw || true
modprobe can_dev || true
systemctl start d1-gs-usb.service
udevadm trigger --subsystem-match=net --action=add

echo
echo "完成。状态预览："
systemctl --no-pager --full status d1-gs-usb.service | sed -n '1,5p' || true
for IF in can1 can2; do
  if [[ -d /sys/class/net/$IF ]]; then
    systemctl --no-pager --full status "d1-canfd@${IF}.service" 2>/dev/null | sed -n '1,5p' || true
    ip -d link show "$IF" | sed -n '1,3p' || true
  else
    echo "[$IF] 当前不存在（USB-CAN 未插或未识别），插上后会自动配置"
  fi
done
