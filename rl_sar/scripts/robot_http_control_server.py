#!/usr/bin/env python3
"""
HTTP control API for D1 robot field testing and mobile/web clients.

Run on the Jetson after sourcing ROS 2 and the workspace:

  source /opt/ros/humble/setup.bash
  source install/setup.bash
  sudo -E python3 rl_sar/scripts/robot_http_control_server.py

Endpoints:
  GET  /api/status
  POST /api/start
  POST /api/stop
  POST /api/standup
  POST /api/sitdown
  POST /api/locomotion
  POST /api/cmd_vel       {"x": 0.2, "y": 0.0, "yaw": 0.1}
  POST /api/cmd_vel/zero
"""

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger


def clamp(value, lower, upper):
    return max(lower, min(upper, float(value)))


class RobotHttpController:
    def __init__(self, args):
        self.args = args
        self.last_cmd_at = 0.0
        self.last_cmd = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self.lock = threading.Lock()
        self.running = True

        rclpy.init(args=None)
        self.node = rclpy.create_node("d1_robot_http_control_server")
        self.cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.standup_client = self.node.create_client(Trigger, "/rl_sar/standup")
        self.sitdown_client = self.node.create_client(Trigger, "/rl_sar/sitdown")
        self.locomotion_client = self.node.create_client(Trigger, "/rl_sar/locomotion")

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.spin_thread = threading.Thread(target=self._spin, daemon=True)
        self.spin_thread.start()

        self.watchdog_thread = threading.Thread(target=self._watchdog, daemon=True)
        self.watchdog_thread.start()

    def shutdown(self):
        self.running = False
        self.publish_cmd_vel(0.0, 0.0, 0.0, update_watchdog=False)
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()

    def _spin(self):
        while self.running and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)

    def _watchdog(self):
        while self.running:
            time.sleep(0.05)
            with self.lock:
                age = time.monotonic() - self.last_cmd_at if self.last_cmd_at > 0 else 0.0
                should_zero = self.last_cmd_at > 0 and age > self.args.cmd_timeout
                already_zero = (
                    self.last_cmd["x"] == 0.0
                    and self.last_cmd["y"] == 0.0
                    and self.last_cmd["yaw"] == 0.0
                )
            if should_zero and not already_zero:
                self.publish_cmd_vel(0.0, 0.0, 0.0, update_watchdog=False)

    def run_systemctl(self, action):
        command = ["systemctl", action, self.args.service]
        if os.geteuid() != 0:
            command = ["sudo", "-n"] + command
        proc = subprocess.run(
            command,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        return {
            "ok": proc.returncode == 0,
            "returncode": proc.returncode,
            "stdout": proc.stdout.strip(),
            "stderr": proc.stderr.strip(),
        }

    def service_active(self):
        proc = subprocess.run(
            ["systemctl", "is-active", self.args.service],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        return proc.stdout.strip()

    def service_enabled(self):
        proc = subprocess.run(
            ["systemctl", "is-enabled", self.args.service],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        return proc.stdout.strip()

    def standup_ready(self):
        return self.standup_client.wait_for_service(timeout_sec=0.0)

    def sitdown_ready(self):
        return self.sitdown_client.wait_for_service(timeout_sec=0.0)

    def locomotion_ready(self):
        return self.locomotion_client.wait_for_service(timeout_sec=0.0)

    def wait_for_standup(self, timeout):
        return self.standup_client.wait_for_service(timeout_sec=timeout)

    def wait_for_sitdown(self, timeout):
        return self.sitdown_client.wait_for_service(timeout_sec=timeout)

    def wait_for_locomotion(self, timeout):
        return self.locomotion_client.wait_for_service(timeout_sec=timeout)

    def call_standup(self):
        if not self.wait_for_standup(self.args.service_timeout):
            return {"ok": False, "message": "/rl_sar/standup service not ready"}
        future = self.standup_client.call_async(Trigger.Request())
        deadline = time.monotonic() + self.args.service_timeout
        while time.monotonic() < deadline:
            if future.done():
                response = future.result()
                return {"ok": bool(response.success), "message": response.message}
            time.sleep(0.05)
        return {"ok": False, "message": "/rl_sar/standup call timed out"}

    def call_sitdown(self):
        if not self.wait_for_sitdown(self.args.service_timeout):
            return {"ok": False, "message": "/rl_sar/sitdown service not ready"}
        future = self.sitdown_client.call_async(Trigger.Request())
        deadline = time.monotonic() + self.args.service_timeout
        while time.monotonic() < deadline:
            if future.done():
                response = future.result()
                return {"ok": bool(response.success), "message": response.message}
            time.sleep(0.05)
        return {"ok": False, "message": "/rl_sar/sitdown call timed out"}

    def call_locomotion(self):
        if not self.wait_for_locomotion(self.args.service_timeout):
            return {"ok": False, "message": "/rl_sar/locomotion service not ready"}
        future = self.locomotion_client.call_async(Trigger.Request())
        deadline = time.monotonic() + self.args.service_timeout
        while time.monotonic() < deadline:
            if future.done():
                response = future.result()
                return {"ok": bool(response.success), "message": response.message}
            time.sleep(0.05)
        return {"ok": False, "message": "/rl_sar/locomotion call timed out"}

    def publish_cmd_vel(self, x, y, yaw, update_watchdog=True):
        x = clamp(x, -self.args.max_x, self.args.max_x)
        y = clamp(y, -self.args.max_y, self.args.max_y)
        yaw = clamp(yaw, -self.args.max_yaw, self.args.max_yaw)

        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        msg.angular.z = yaw
        self.cmd_vel_pub.publish(msg)

        with self.lock:
            self.last_cmd = {"x": x, "y": y, "yaw": yaw}
            if update_watchdog:
                self.last_cmd_at = time.monotonic()
        return {"ok": True, "cmd_vel": self.last_cmd}

    def status(self):
        with self.lock:
            age = None if self.last_cmd_at <= 0 else int((time.monotonic() - self.last_cmd_at) * 1000)
            last_cmd = dict(self.last_cmd)
        return {
            "ok": True,
            "service": self.args.service,
            "service_active": self.service_active(),
            "service_enabled": self.service_enabled(),
            "standup_ready": self.standup_ready(),
            "sitdown_ready": self.sitdown_ready(),
            "locomotion_ready": self.locomotion_ready(),
            "last_cmd": last_cmd,
            "last_cmd_age_ms": age,
            "limits": {
                "x": self.args.max_x,
                "y": self.args.max_y,
                "yaw": self.args.max_yaw,
                "cmd_timeout": self.args.cmd_timeout,
            },
        }


class ControlHandler(BaseHTTPRequestHandler):
    controller = None

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Authorization")
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(204)
        self.end_headers()

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self.send_text(self.page(), "text/html; charset=utf-8")
            return
        if self.path == "/api/status":
            self.send_json(200, self.controller.status())
            return
        self.send_json(404, {"ok": False, "error": "not found"})

    def do_POST(self):
        if self.path == "/api/start":
            result = self.controller.run_systemctl("start")
            self.send_json(200 if result["ok"] else 500, {"ok": result["ok"], "systemctl": result, "status": self.controller.status()})
            return
        if self.path == "/api/stop":
            zero = self.controller.publish_cmd_vel(0.0, 0.0, 0.0, update_watchdog=False)
            result = self.controller.run_systemctl("stop")
            self.send_json(200 if result["ok"] else 500, {"ok": result["ok"], "zero": zero, "systemctl": result, "status": self.controller.status()})
            return
        if self.path == "/api/standup":
            start = self.controller.run_systemctl("start")
            standup = self.controller.call_standup() if start["ok"] else {"ok": False, "message": "failed to start service"}
            ok = start["ok"] and standup["ok"]
            self.send_json(200 if ok else 500, {"ok": ok, "start": start, "standup": standup, "status": self.controller.status()})
            return
        if self.path == "/api/sitdown":
            sitdown = self.controller.call_sitdown()
            self.send_json(200 if sitdown["ok"] else 500, {"ok": sitdown["ok"], "sitdown": sitdown, "status": self.controller.status()})
            return
        if self.path == "/api/locomotion":
            start = self.controller.run_systemctl("start")
            locomotion = self.controller.call_locomotion() if start["ok"] else {"ok": False, "message": "failed to start service"}
            ok = start["ok"] and locomotion["ok"]
            self.send_json(200 if ok else 500, {"ok": ok, "start": start, "locomotion": locomotion, "status": self.controller.status()})
            return
        if self.path == "/api/cmd_vel":
            payload = self.read_json()
            result = self.controller.publish_cmd_vel(
                payload.get("x", 0.0),
                payload.get("y", 0.0),
                payload.get("yaw", 0.0),
            )
            self.send_json(200, result)
            return
        if self.path == "/api/cmd_vel/zero":
            result = self.controller.publish_cmd_vel(0.0, 0.0, 0.0)
            self.send_json(200, result)
            return
        self.send_json(404, {"ok": False, "error": "not found"})

    def read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        raw = self.rfile.read(length).decode("utf-8")
        return json.loads(raw) if raw.strip() else {}

    def send_json(self, status, payload):
        self.send_text(json.dumps(payload, separators=(",", ":")), "application/json", status)

    def send_text(self, body, content_type, status=200):
        data = body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt, *args):
        print("%s - %s" % (self.address_string(), fmt % args), flush=True)

    def page(self):
        return """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>D1 Robot HTTP Control</title>
  <style>
    :root {
      color-scheme: dark;
      --bg: #101316;
      --panel: #191f24;
      --panel2: #20272d;
      --line: #33404a;
      --text: #f4f7f8;
      --muted: #a9b5bc;
      --blue: #2f7df6;
      --green: #18a058;
      --red: #dc3f3f;
      --amber: #d99a18;
      --purple: #7a5cff;
    }
    * { box-sizing: border-box; }
    html, body { min-height: 100%; }
    body {
      margin: 0;
      background: radial-gradient(circle at 50% 0%, #1d2a31 0, var(--bg) 42%);
      color: var(--text);
      font-family: system-ui, -apple-system, Segoe UI, sans-serif;
      user-select: none;
      -webkit-user-select: none;
      -webkit-touch-callout: none;
      overscroll-behavior: none;
    }
    main {
      width: min(100%, 520px);
      min-height: 100vh;
      margin: 0 auto;
      padding: 18px 16px 24px;
      display: flex;
      flex-direction: column;
      gap: 14px;
    }
    .topbar {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
    }
    h1 { margin: 0; font-size: 24px; line-height: 1.1; letter-spacing: 0; }
    .subtitle { margin-top: 4px; color: var(--muted); font-size: 13px; }
    .state-pill {
      min-width: 88px;
      padding: 8px 10px;
      border-radius: 8px;
      border: 1px solid var(--line);
      background: var(--panel);
      text-align: center;
      font-size: 12px;
      color: var(--muted);
    }
    .state-pill b { display: block; color: var(--text); font-size: 15px; margin-top: 2px; }
    .status {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
      padding: 10px;
      background: rgba(25, 31, 36, 0.92);
      border: 1px solid var(--line);
      border-radius: 8px;
    }
    .metric {
      min-height: 56px;
      padding: 8px;
      border-radius: 6px;
      background: var(--panel2);
      text-align: center;
      font-size: 12px;
      color: var(--muted);
    }
    .metric b { display: block; color: var(--text); font-size: 14px; margin-top: 4px; }
    .panel {
      padding: 14px;
      border: 1px solid var(--line);
      border-radius: 8px;
      background: rgba(25, 31, 36, 0.94);
      box-shadow: 0 18px 48px rgba(0,0,0,0.22);
    }
    .section-title {
      margin: 0 0 12px;
      color: var(--muted);
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 0;
    }
    button {
      min-height: 48px;
      border-radius: 8px;
      border: 0;
      cursor: pointer;
      color: var(--text);
      font-size: 15px;
      font-weight: 700;
      letter-spacing: 0;
      user-select: none;
      -webkit-user-select: none;
      -webkit-touch-callout: none;
      -webkit-tap-highlight-color: transparent;
      touch-action: manipulation;
    }
    button:active { transform: translateY(1px) scale(0.99); filter: brightness(1.08); }
    .primary { background: linear-gradient(180deg, #3f8dff, var(--blue)); }
    .standup { background: linear-gradient(180deg, #24bd70, var(--green)); }
    .danger { background: linear-gradient(180deg, #ee5555, var(--red)); }
    .motion { background: linear-gradient(180deg, #907bff, var(--purple)); }
    .neutral { background: linear-gradient(180deg, #53616d, #39454f); }
    .actions {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    .remote {
      display: grid;
      grid-template-columns: 1fr;
      gap: 14px;
      touch-action: none;
    }
    .dpad {
      width: min(100%, 330px);
      margin: 0 auto;
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      grid-template-rows: repeat(3, 76px);
      gap: 10px;
    }
    .drive-btn {
      background: linear-gradient(180deg, #52616d, #323c45);
      border: 1px solid #64737e;
      font-size: 16px;
    }
    .drive-btn.forward { grid-column: 2; grid-row: 1; }
    .drive-btn.left { grid-column: 1; grid-row: 2; }
    .drive-btn.stop { grid-column: 2; grid-row: 2; background: linear-gradient(180deg, #f0ab2d, var(--amber)); color: #18130a; }
    .drive-btn.right { grid-column: 3; grid-row: 2; }
    .drive-btn.back { grid-column: 2; grid-row: 3; }
    .turn-row {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }
    .turn-row .drive-btn { min-height: 64px; }
    .readout {
      min-height: 62px;
      margin: 0;
      padding: 10px 12px;
      border-radius: 8px;
      border: 1px solid var(--line);
      background: #0d1114;
      color: #c7d0d6;
      white-space: pre-wrap;
      font-size: 12px;
      overflow: auto;
    }
    @supports (min-height: 100dvh) {
      main { min-height: 100dvh; }
    }
    @media (max-width: 420px) {
      main { padding: 10px 12px 12px; gap: 8px; }
      h1 { font-size: 21px; }
      .subtitle { font-size: 12px; }
      .state-pill { min-width: 76px; padding: 6px 8px; }
      .state-pill b { font-size: 14px; }
      .status { gap: 6px; padding: 8px; }
      .metric { min-height: 44px; padding: 6px; font-size: 11px; }
      .metric b { font-size: 13px; margin-top: 2px; }
      .panel { padding: 10px; }
      .section-title { margin-bottom: 8px; font-size: 11px; }
      .actions { grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 8px; }
      .remote { gap: 10px; }
      .dpad { width: min(100%, 292px); grid-template-rows: repeat(3, 60px); gap: 8px; }
      .turn-row { gap: 8px; }
      .turn-row .drive-btn { min-height: 54px; }
      button { min-height: 42px; font-size: 13px; }
      .drive-btn { font-size: 14px; }
      .readout { display: none; }
    }
    @media (max-width: 420px) and (max-height: 760px) {
      main { padding-top: 8px; gap: 7px; }
      .topbar { gap: 8px; }
      h1 { font-size: 20px; }
      .status { display: none; }
      .panel { padding: 9px; }
      .actions { gap: 7px; }
      .dpad { grid-template-rows: repeat(3, 56px); gap: 7px; }
      .turn-row .drive-btn { min-height: 50px; }
      button { min-height: 40px; }
    }
  </style>
</head>
<body>
<main>
  <div class="topbar">
    <div>
      <h1>D1 Robot</h1>
      <div class="subtitle">HTTP Remote Control</div>
    </div>
    <div class="state-pill">Main<b id="state">-</b></div>
  </div>

  <div class="status">
    <div class="metric">Stand Up<b id="standupReady">-</b></div>
    <div class="metric">Sit Down<b id="sitdownReady">-</b></div>
    <div class="metric">Work Mode<b id="locomotionReady">-</b></div>
  </div>

  <section class="panel">
    <p class="section-title">Robot State</p>
    <div class="actions">
      <button class="primary" onclick="post('/api/start')">Start</button>
      <button class="danger" onclick="confirmPost('/api/stop','Stop motor control service?')">Stop</button>
      <button class="standup" onclick="confirmPost('/api/standup','Stand up the robot? Confirm it is in a safe test area.')">Stand Up</button>
      <button class="danger" onclick="confirmPost('/api/sitdown','Sit down the robot? Confirm it is in a safe test area.')">Sit Down</button>
      <button class="motion" onclick="confirmPost('/api/locomotion','Enter work mode? Confirm the robot is standing and clear to move.')">Work Mode</button>
      <button class="neutral" onclick="zero()">Zero</button>
    </div>
  </section>

  <section class="panel remote">
    <p class="section-title">Drive</p>
    <div class="dpad">
      <button class="drive-btn forward" oncontextmenu="return false" onpointerdown="hold(event,0.2,0,0)" onpointerup="release(event)" onpointercancel="release(event)" onpointerleave="release(event)">FWD</button>
      <button class="drive-btn left" oncontextmenu="return false" onpointerdown="hold(event,0,0.15,0)" onpointerup="release(event)" onpointercancel="release(event)" onpointerleave="release(event)">LEFT</button>
      <button class="drive-btn stop" onclick="zero()">STOP</button>
      <button class="drive-btn right" oncontextmenu="return false" onpointerdown="hold(event,0,-0.15,0)" onpointerup="release(event)" onpointercancel="release(event)" onpointerleave="release(event)">RIGHT</button>
      <button class="drive-btn back" oncontextmenu="return false" onpointerdown="hold(event,-0.2,0,0)" onpointerup="release(event)" onpointercancel="release(event)" onpointerleave="release(event)">BACK</button>
    </div>
    <div class="turn-row">
      <button class="drive-btn" oncontextmenu="return false" onpointerdown="hold(event,0,0,0.3)" onpointerup="release(event)" onpointercancel="release(event)" onpointerleave="release(event)">TURN L</button>
      <button class="drive-btn" oncontextmenu="return false" onpointerdown="hold(event,0,0,-0.3)" onpointerup="release(event)" onpointercancel="release(event)" onpointerleave="release(event)">TURN R</button>
    </div>
  </section>

  <pre class="readout" id="log"></pre>
</main>
<script>
async function post(path, body) {
  const res = await fetch(path, {method:'POST', headers:{'Content-Type':'application/json'}, body: body ? JSON.stringify(body) : '{}'});
  const text = await res.text();
  document.getElementById('log').textContent = text;
  refresh();
}
function confirmPost(path, message) { if (confirm(message)) post(path); }
function drive(x, y, yaw) { post('/api/cmd_vel', {x, y, yaw}); }
function zero() { post('/api/cmd_vel/zero'); }
let driveTimer = null;
function hold(event, x, y, yaw) {
  event.preventDefault();
  event.currentTarget.setPointerCapture(event.pointerId);
  drive(x, y, yaw);
  if (driveTimer) clearInterval(driveTimer);
  driveTimer = setInterval(() => drive(x, y, yaw), 150);
}
function release(event) {
  event.preventDefault();
  if (driveTimer) {
    clearInterval(driveTimer);
    driveTimer = null;
  }
  if (event.currentTarget.hasPointerCapture(event.pointerId)) {
    event.currentTarget.releasePointerCapture(event.pointerId);
  }
  zero();
}
async function refresh() {
  const res = await fetch('/api/status');
  const data = await res.json();
  document.getElementById('state').textContent = data.service_active;
  document.getElementById('standupReady').textContent = data.standup_ready;
  document.getElementById('sitdownReady').textContent = data.sitdown_ready;
  document.getElementById('locomotionReady').textContent = data.locomotion_ready;
}
setInterval(refresh, 1000);
refresh();
</script>
</body>
</html>"""


def parse_args():
    parser = argparse.ArgumentParser(description="D1 robot HTTP control server.")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--service", default="rl-sar-main.service")
    parser.add_argument("--service-timeout", type=float, default=12.0)
    parser.add_argument("--cmd-timeout", type=float, default=0.35)
    parser.add_argument("--max-x", type=float, default=0.5)
    parser.add_argument("--max-y", type=float, default=0.3)
    parser.add_argument("--max-yaw", type=float, default=0.8)
    return parser.parse_args()


def main():
    args = parse_args()
    controller = RobotHttpController(args)
    ControlHandler.controller = controller
    server = ThreadingHTTPServer((args.host, args.port), ControlHandler)
    print(f"Serving D1 robot HTTP control on {args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        controller.shutdown()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
