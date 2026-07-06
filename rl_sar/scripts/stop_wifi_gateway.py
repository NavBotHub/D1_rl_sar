#!/usr/bin/env python3
"""
Stop the Jetson WiFi gateway/access-point mode.

By default this script:
  - brings down the NetworkManager connection named d1-gateway
  - disables autoconnect for that gateway connection
  - keeps WiFi radio enabled so the adapter returns to normal client mode

Optionally pass --ssid and --password to reconnect to a normal WiFi network.
"""

import argparse
import os
import subprocess
import sys


def run(command, check=True):
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
    if result.returncode != 0 and check:
        if result.stderr.strip():
            print(result.stderr.strip(), file=sys.stderr, flush=True)
        raise RuntimeError("command failed: " + " ".join(command))
    return result


def connection_exists(name):
    result = subprocess.run(
        ["nmcli", "-t", "-f", "NAME", "connection", "show"],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return name in result.stdout.splitlines()


def stop_gateway(args):
    run(["nmcli", "radio", "wifi", "on"])

    if connection_exists(args.connection):
        run(["nmcli", "connection", "modify", args.connection, "connection.autoconnect", "no"])
        run(["nmcli", "connection", "down", args.connection], check=False)
        if args.delete:
            run(["nmcli", "connection", "delete", args.connection])
    else:
        print(f"Gateway connection does not exist: {args.connection}", flush=True)

    if args.ssid:
        command = [
            "nmcli",
            "device",
            "wifi",
            "connect",
            args.ssid,
            "ifname",
            args.interface,
        ]
        if args.password:
            command.extend(["password", args.password])
        run(command)
        run(["nmcli", "connection", "modify", args.ssid, "connection.autoconnect", "yes"], check=False)
    else:
        run(["nmcli", "device", "set", args.interface, "managed", "yes"])
        run(["nmcli", "device", "wifi", "rescan", "ifname", args.interface], check=False)

    print("", flush=True)
    print("WiFi gateway stopped.", flush=True)
    run(["nmcli", "device", "status"])


def parse_args():
    parser = argparse.ArgumentParser(description="Stop D1 WiFi gateway mode.")
    parser.add_argument("--interface", default="wlP1p1s0", help="WiFi interface name")
    parser.add_argument("--connection", default="d1-gateway", help="NetworkManager gateway connection name")
    parser.add_argument("--delete", action="store_true", help="delete the gateway connection")
    parser.add_argument("--ssid", default="", help="normal WiFi SSID to reconnect to")
    parser.add_argument("--password", default="", help="normal WiFi password")
    return parser.parse_args()


def main():
    if os.geteuid() != 0:
        raise SystemExit("Run with sudo.")
    try:
        stop_gateway(parse_args())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
