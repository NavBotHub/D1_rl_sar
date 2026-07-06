#!/usr/bin/env python3
"""
Start the Jetson WiFi adapter as a gateway/access point.

Default behavior matches the D1 field setup:
  - interface: wlP1p1s0
  - SSID: D1-Robot-<first 6 lowercase hex chars of WiFi MAC>
  - password: 12345678
  - band: 5 GHz
  - channel: 149
  - gateway IP: 192.168.4.100/24

The script uses NetworkManager's nmcli and must be run as root.
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path


def run(command):
    print("+", " ".join(command), flush=True)
    result = subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.stdout.strip():
        print(result.stdout.strip(), flush=True)
    if result.returncode != 0:
        if result.stderr.strip():
            print(result.stderr.strip(), file=sys.stderr, flush=True)
        raise RuntimeError("command failed: " + " ".join(command))
    return result.stdout.strip()


def connection_exists(name):
    result = subprocess.run(
        ["nmcli", "-t", "-f", "NAME", "connection", "show"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return name in result.stdout.splitlines()


def mac_suffix(interface):
    address_path = Path("/sys/class/net") / interface / "address"
    mac = address_path.read_text(encoding="utf-8").strip().replace(":", "")
    if len(mac) < 6:
        raise RuntimeError(f"invalid MAC address for {interface}: {mac}")
    return mac[:6].lower()


def default_ssid(interface):
    return "D1-Robot-" + mac_suffix(interface)


def start_gateway(args):
    if len(args.password) < 8:
        raise ValueError("WiFi password must be at least 8 characters")

    ssid = args.ssid or default_ssid(args.interface)
    band = "a" if args.band == "5g" else "bg"

    run(["nmcli", "radio", "wifi", "on"])

    if args.recreate and connection_exists(args.connection):
        run(["nmcli", "connection", "delete", args.connection])

    if not connection_exists(args.connection):
        run(
            [
                "nmcli",
                "connection",
                "add",
                "type",
                "wifi",
                "ifname",
                args.interface,
                "con-name",
                args.connection,
                "autoconnect",
                "yes",
                "ssid",
                ssid,
            ]
        )

    run(
        [
            "nmcli",
            "connection",
            "modify",
            args.connection,
            "connection.interface-name",
            args.interface,
            "connection.autoconnect",
            "yes",
            "802-11-wireless.ssid",
            ssid,
            "802-11-wireless.mode",
            "ap",
            "802-11-wireless.band",
            band,
            "802-11-wireless.channel",
            str(args.channel),
            "802-11-wireless-security.key-mgmt",
            "wpa-psk",
            "802-11-wireless-security.proto",
            "rsn",
            "802-11-wireless-security.pairwise",
            "ccmp",
            "802-11-wireless-security.group",
            "ccmp",
            "802-11-wireless-security.psk",
            args.password,
            "ipv4.method",
            "shared",
            "ipv4.addresses",
            args.gateway,
            "ipv4.gateway",
            "",
            "ipv4.dns",
            "",
            "ipv6.method",
            "ignore",
        ]
    )

    subprocess.run(["nmcli", "connection", "down", args.connection], check=False)
    run(["nmcli", "connection", "up", args.connection])

    print("", flush=True)
    print(f"WiFi gateway started: ssid={ssid}", flush=True)
    print(f"interface={args.interface} gateway={args.gateway}", flush=True)
    run(["ip", "-brief", "addr", "show", args.interface])


def parse_args():
    parser = argparse.ArgumentParser(description="Start D1 WiFi gateway mode.")
    parser.add_argument("--interface", default="wlP1p1s0", help="WiFi interface name")
    parser.add_argument("--connection", default="d1-gateway", help="NetworkManager connection name")
    parser.add_argument("--ssid", default="", help="SSID; default is D1-Robot-<mac prefix>")
    parser.add_argument("--password", default="12345678", help="WPA2 password")
    parser.add_argument("--gateway", default="192.168.4.100/24", help="gateway CIDR address")
    parser.add_argument("--band", choices=["5g", "2g"], default="5g", help="WiFi band")
    parser.add_argument("--channel", type=int, default=149, help="WiFi channel")
    parser.add_argument("--recreate", action="store_true", help="delete and recreate the nmcli connection")
    return parser.parse_args()


def main():
    if os.geteuid() != 0:
        raise SystemExit("Run with sudo.")
    try:
        start_gateway(parse_args())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
