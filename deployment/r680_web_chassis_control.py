#!/usr/bin/env python3
"""Dead-man web controller for the R680 C50C V4 chassis."""

from __future__ import annotations

import argparse
import hmac
import json
import math
import os
from pathlib import Path
import secrets
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional
from urllib.parse import parse_qs, urlparse

from r680_ackermann_bridge import (
    ControlFileLock,
    encode_command,
    format_frame,
    open_serial,
)


DEVICE = "/dev/serial/by-id/YOUR_CHASSIS_SERIAL"
CONTROL_LOCK = "/root/r680_nav/run/r680_chassis_control.lock"
MAX_SPEED = 0.26
MIN_SPEED = 0.05
DEFAULT_SPEED = 0.09
MINIMUM_TURNING_RADIUS = 1.40
MAX_ANGULAR = 1.00
DEFAULT_PORT = 8765
HEARTBEAT_TIMEOUT = 0.80
TX_RATE = 20.0
ZERO_FRAME = encode_command(0.0, 0.0, 0.0)
WEB_UI_VERSION = "R680-WEB-A2-20260919"


class CommandRejected(ValueError):
    """A command failed token, lease, sequence, or range validation."""


def _finite(value: object, default: Optional[float] = 0.0) -> float:
    if isinstance(value, bool):
        raise ValueError("numeric value must not be boolean")
    try:
        result = float(value)
    except (TypeError, ValueError):
        if default is not None:
            return float(default)
        raise ValueError("numeric value is invalid")
    if not math.isfinite(result):
        if default is not None:
            return float(default)
        raise ValueError("numeric value must be finite")
    return result


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def action_command(action: str, speed: float, steering: float) -> tuple[float, float]:
    """Convert a whitelisted page action into ``(vx, wz)`` SI units."""

    action = str(action)
    signs = {
        "forward": (1.0, 0.0),
        "forward_left": (1.0, 1.0),
        "forward_right": (1.0, -1.0),
        "backward": (-1.0, 0.0),
        "backward_left": (-1.0, -1.0),
        "backward_right": (-1.0, 1.0),
    }
    if action not in signs:
        raise ValueError(f"unknown action: {action}")
    # Malformed numeric input must fail closed; it must not become the minimum
    # speed after a NaN/None-to-zero conversion.
    speed = _clamp(abs(_finite(speed, default=None)), MIN_SPEED, MAX_SPEED)
    steering = _clamp(abs(_finite(steering, default=None)), 0.0, 1.0)
    vx_sign, wz_sign = signs[action]
    wz = (speed / MINIMUM_TURNING_RADIUS) * steering * wz_sign
    wz = _clamp(wz, -MAX_ANGULAR, MAX_ANGULAR)
    return vx_sign * speed, wz


def command_frame(action: str, speed: float, steering: float) -> bytes:
    vx, wz = action_command(action, speed, steering)
    return encode_command(
        vx,
        0.0,
        wz,
        max_linear_x=MAX_SPEED,
        max_angular_z=MAX_ANGULAR,
        minimum_turning_radius=MINIMUM_TURNING_RADIUS,
        allow_reverse=True,
    )


class ControlState:
    """Single-client dead-man state, independent of HTTP worker threads."""

    def __init__(self, heartbeat_timeout: float = HEARTBEAT_TIMEOUT) -> None:
        self.heartbeat_timeout = float(heartbeat_timeout)
        self._lock = threading.Lock()
        self._client: Optional[str] = None
        self._seq = -1
        self._last_seen = 0.0
        self._command = (0.0, 0.0)
        self._last_seq_by_client: dict[str, int] = {}
        self._retired_clients: set[str] = set()
        # A heartbeat timeout is a dead-man event.  Keep the old client from
        # silently resuming if delayed browser requests arrive after timeout;
        # it must release the button and press again (or a new client may
        # claim the lease).
        self._needs_rearm = False

    def accept(
        self,
        client_id: str,
        seq: int,
        action: str,
        speed: float,
        steering: float,
        *,
        now: Optional[float] = None,
    ) -> None:
        now = time.monotonic() if now is None else float(now)
        try:
            seq = int(seq)
        except (TypeError, ValueError) as exc:
            raise CommandRejected("sequence must be an integer") from exc
        if not client_id or len(str(client_id)) > 96:
            raise CommandRejected("invalid client id")
        command = action_command(action, speed, steering)
        client_id = str(client_id)
        with self._lock:
            if client_id in self._retired_clients:
                raise CommandRejected("retired browser cannot reclaim control; refresh the page")
            expired = self._client is not None and now - self._last_seen > self.heartbeat_timeout
            if expired:
                was_moving = self._command != (0.0, 0.0)
                self._command = (0.0, 0.0)
                if was_moving:
                    self._needs_rearm = True
            if seq <= self._last_seq_by_client.get(client_id, -1):
                raise CommandRejected("stale command sequence")
            # Record an authenticated browser's monotonic sequence before
            # ownership checks.  If a competing page releases while another
            # page owns the chassis, its higher stop sequence must still fence
            # its own older in-flight motion requests.
            self._last_seq_by_client[client_id] = seq
            if self._needs_rearm:
                if self._client == client_id:
                    raise CommandRejected("heartbeat expired; release and press again")
                if self._client is not None:
                    self._retired_clients.add(self._client)
                self._client = client_id
                self._seq = -1
                self._needs_rearm = False
            elif self._client is None:
                self._client = client_id
                self._seq = -1
            elif self._client != client_id:
                if self._command == (0.0, 0.0):
                    # An explicit stop is a hand-off point.  A refreshed page
                    # has a new client id and must not lose its first press to
                    # the old page's otherwise harmless zero-command lease.
                    self._retired_clients.add(self._client)
                    self._client = client_id
                    self._seq = -1
                else:
                    raise CommandRejected("another browser already controls the chassis")
            self._client = client_id
            self._seq = seq
            self._last_seen = now
            self._command = command
            self._needs_rearm = False

    def stop(self, client_id: str, seq: int, *, now: Optional[float] = None) -> None:
        now = time.monotonic() if now is None else float(now)
        client_id = str(client_id)
        try:
            seq = int(seq)
        except (TypeError, ValueError) as exc:
            raise CommandRejected("sequence must be an integer") from exc
        with self._lock:
            if client_id in self._retired_clients:
                raise CommandRejected("retired browser cannot reclaim control; refresh the page")
            previous_seq = self._last_seq_by_client.get(client_id, -1)
            if seq < previous_seq:
                raise CommandRejected("stale stop sequence")
            if seq == previous_seq:
                if self._client == client_id and self._command == (0.0, 0.0):
                    return
                raise CommandRejected("stale stop sequence")
            # Preserve this browser-local release fence even if another page
            # currently owns the chassis.  It must not stop that owner, but
            # delayed lower-sequence commands from this page stay stale.
            self._last_seq_by_client[client_id] = seq
            if self._client not in (None, client_id):
                raise CommandRejected("another browser already controls the chassis")
            self._client = client_id
            self._seq = seq
            self._last_seen = now
            self._command = (0.0, 0.0)
            # An explicit stop is the release/re-arm acknowledgement after a
            # timeout; the next pointer-down may start a fresh command.
            self._needs_rearm = False

    def snapshot(self, *, now: Optional[float] = None) -> tuple[float, float]:
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            if self._client is None or now - self._last_seen > self.heartbeat_timeout:
                was_moving = self._command != (0.0, 0.0)
                self._command = (0.0, 0.0)
                if self._client is not None and was_moving:
                    self._needs_rearm = True
            return self._command

    def clear(self) -> None:
        with self._lock:
            self._client = None
            self._seq = -1
            self._last_seen = 0.0
            self._command = (0.0, 0.0)
            self._needs_rearm = False
            self._last_seq_by_client.clear()
            self._retired_clients.clear()


def _proc_cmdline(pid: str) -> str:
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
    except (OSError, ValueError):
        return ""


def reject_nav2_activity(run_dir: str = "/root/r680_nav/run") -> None:
    """Fail closed if an existing real bridge or Nav2 process is present."""

    pid_file = Path(run_dir) / "r680_nav2_chassis_bridge.pid"
    if pid_file.is_file():
        try:
            pid = int(pid_file.read_text(encoding="utf-8").strip())
        except (OSError, ValueError):
            raise RuntimeError("invalid Nav2 bridge PID record")
        if Path(f"/proc/{pid}").exists():
            raise RuntimeError("Nav2 chassis bridge is already running")

    forbidden = (
        "r680_ackermann_bridge.py",
        "controller_server",
        "planner_server",
        "bt_navigator",
        "velocity_smoother",
        "r680_nav2_chassis_start.sh",
        "r680_nav2_start.sh",
        "map_server",
        "amcl",
        "behavior_server",
        "r680_web_chassis_control.py",
    )
    proc_root = Path("/proc")
    # Unit tests and development checks may run on Windows.  The authoritative
    # PID/lock checks above still apply; process argv scanning is Linux-only.
    if not proc_root.is_dir():
        return
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        command = _proc_cmdline(entry.name)
        if any(token in command for token in forbidden):
            if str(os.getpid()) != entry.name:
                raise RuntimeError(f"control process already active: {command.strip()}")


def render_page(token: str) -> str:
    token_json = json.dumps(token, ensure_ascii=False)
    version_json = json.dumps(WEB_UI_VERSION, ensure_ascii=False)
    return f'''<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>R680 底盘控制</title>
<style>
body{{font-family:system-ui,sans-serif;background:#10141b;color:#f3f4f6;margin:0;padding:18px;touch-action:manipulation}}
main{{max-width:560px;margin:auto}}h1{{font-size:24px;margin:0 0 8px}}.warning{{color:#ffd166;font-size:14px;margin-bottom:16px}}
.pad{{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;max-width:430px;margin:20px auto}}
button{{min-height:68px;border:0;border-radius:14px;background:#263244;color:white;font-size:18px;font-weight:700;touch-action:none;user-select:none}}
button:active,.active{{background:#2563eb;transform:scale(.98)}}.stop{{background:#d62828;grid-column:1/4}}.stop:active{{background:#a4161a}}
label{{display:block;margin:14px 0 4px}}input[type=range]{{width:100%;accent-color:#3b82f6}}#status{{min-height:24px;text-align:center;color:#a7f3d0;margin-top:16px}}.version{{text-align:center;color:#64748b;font-size:12px;margin-top:8px}}
</style></head><body><main><h1>R680 底盘人工控制</h1>
<div class="warning">必须按住动作按钮；松开、失焦或通信中断会自动停车并回中。</div>
<label>速度：<output id="speedValue">0.09</output> m/s</label><input id="speed" type="range" min="0.05" max="0.26" step="0.01" value="0.09">
<label>转向幅度：<output id="steerValue">50</output>%</label><input id="steer" type="range" min="0" max="1" step="0.05" value="0.5">
<div class="pad">
<button data-action="forward_left">前左转</button><button data-action="forward">前进</button><button data-action="forward_right">前右转</button>
<button data-action="backward_left">后左转</button><button data-action="backward">后退</button><button data-action="backward_right">后右转</button>
<button class="stop" id="stop">停止 / 舵机回中</button></div><div id="status">未开始控制</div><div class="version">{WEB_UI_VERSION}</div></main>
<script>
const TOKEN={token_json},UI_VERSION={version_json}; const clientId=crypto.randomUUID?crypto.randomUUID():String(Date.now())+'-'+Math.random();
let seq=0,activeAction=null,inputKind=null,activeTouchId=null,statusEpoch=0;
const motionRequests=new Set(),MAX_PENDING_MOTION=4;
const speed=document.querySelector('#speed'),steer=document.querySelector('#steer'),status=document.querySelector('#status');
function updateLabels(){{document.querySelector('#speedValue').textContent=speed.value;document.querySelector('#steerValue').textContent=Math.round(Number(steer.value)*100)}}
speed.oninput=steer.oninput=updateLabels; updateLabels();
function clearUi(){{document.querySelectorAll('button').forEach(b=>b.classList.remove('active'))}}
function request(path,payload,keepalive=false,signal=undefined){{const body=Object.assign({{token:TOKEN,client_id:clientId,seq:++seq}},payload),options={{method:'POST',headers:{{'Content-Type':'application/json'}},body:JSON.stringify(body),keepalive}};if(signal)options.signal=signal;return fetch(path,options).then(async r=>{{if(!r.ok){{const e=await r.json().catch(()=>({{}}));throw Error(e.error||r.status)}}return r}})}}
function sendCommand(){{const action=activeAction,epoch=statusEpoch;if(!action)return;const canAbort=typeof AbortController==='function';if(!canAbort&&motionRequests.size>=MAX_PENDING_MOTION)return;while(canAbort&&motionRequests.size>=MAX_PENDING_MOTION){{const oldest=motionRequests.values().next().value;oldest.controller.abort();motionRequests.delete(oldest)}}const controller=canAbort?new AbortController():null,entry={{controller}};motionRequests.add(entry);request('/api/command',{{action,speed:Number(speed.value),steering:Number(steer.value)}},false,controller?controller.signal:undefined).then(()=>{{if(activeAction===action&&statusEpoch===epoch)status.textContent='控制中：'+action}}).catch(e=>{{if(e.name!=='AbortError'&&activeAction===action&&statusEpoch===epoch)status.textContent='通信暂时异常，仍在重试；松手会停车：'+e.message}}).finally(()=>motionRequests.delete(entry))}}
function stopNow(force=false){{if(!force&&!activeAction)return;statusEpoch++;activeAction=null;inputKind=null;activeTouchId=null;clearUi();for(const entry of motionRequests)if(entry.controller)entry.controller.abort();motionRequests.clear();const epoch=statusEpoch;const result=request('/api/stop',{{}},true);result.then(()=>{{if(statusEpoch===epoch&&!activeAction)status.textContent='已停止，舵机回中'}}).catch(e=>{{if(statusEpoch===epoch&&!activeAction)status.textContent='停止请求异常，后端死手超时将停车：'+e.message}});return result}}
function begin(b,kind,touchId=null){{if(activeAction)return;statusEpoch++;activeAction=b.dataset.action;inputKind=kind;activeTouchId=touchId;clearUi();b.classList.add('active');sendCommand()}}
function changedTouchContains(e,id){{return Array.from(e.changedTouches||[]).some(t=>t.identifier===id)}}
document.querySelectorAll('[data-action]').forEach(b=>{{
  b.addEventListener('mousedown',e=>{{if(e.button!==0)return;e.preventDefault();begin(b,'mouse')}});
  b.addEventListener('touchstart',e=>{{e.preventDefault();const touch=(e.changedTouches||[])[0];if(touch)begin(b,'touch',touch.identifier)}},{{passive:false}});
}});
window.addEventListener('mouseup',e=>{{if(inputKind==='mouse')stopNow()}},true);
window.addEventListener('mouseout',e=>{{if(inputKind==='mouse'&&!e.relatedTarget)stopNow()}},true);
window.addEventListener('touchend',e=>{{if(inputKind==='touch'&&changedTouchContains(e,activeTouchId)){{e.preventDefault();stopNow()}}}},{{capture:true,passive:false}});
window.addEventListener('touchcancel',e=>{{if(inputKind==='touch'&&changedTouchContains(e,activeTouchId)){{e.preventDefault();stopNow()}}}},{{capture:true,passive:false}});
document.querySelector('#stop').addEventListener('click',e=>{{e.preventDefault();stopNow(true)}});
window.addEventListener('blur',()=>stopNow());document.addEventListener('visibilitychange',()=>{{if(document.hidden)stopNow()}});window.addEventListener('pagehide',()=>stopNow());
setInterval(sendCommand,100);
</script></body></html>'''


class WebChassisController:
    def __init__(
        self,
        *,
        device: str = DEVICE,
        lock_path: str = CONTROL_LOCK,
        startup_zero_seconds: float = 0.5,
        tx_rate: float = TX_RATE,
        heartbeat_timeout: float = HEARTBEAT_TIMEOUT,
    ) -> None:
        self.device = device
        self.lock_path = lock_path
        self.startup_zero_seconds = max(0.0, float(startup_zero_seconds))
        self.tx_rate = max(20.0, float(tx_rate))
        self.state = ControlState(heartbeat_timeout)
        self._serial = None
        self._lock: Optional[ControlFileLock] = None
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._fault: Optional[str] = None

    def _write(self, frame: bytes) -> None:
        if self._serial is None:
            return
        self._serial.write(frame)

    def _drain_feedback(self) -> None:
        """Discard optional STM32 telemetry so a write-only controller cannot
        leave the USB-UART receive buffer growing without bound."""

        serial = self._serial
        if serial is None:
            return
        available = int(getattr(serial, "in_waiting", 0) or 0)
        if available > 0 and hasattr(serial, "read"):
            serial.read(min(available, 4096))

    def _send_zero(self, serial, count: int = 1, *, suppress_errors: bool = False) -> None:
        for _ in range(count):
            try:
                serial.write(ZERO_FRAME)
                serial.flush()
            except Exception:
                if not suppress_errors:
                    raise
            time.sleep(0.02)

    @property
    def fault(self) -> Optional[str]:
        return self._fault

    def _tx_loop(self) -> None:
        period = 1.0 / self.tx_rate
        while not self._stop_event.is_set():
            started = time.monotonic()
            vx, wz = self.state.snapshot(now=started)
            frame = encode_command(
                vx,
                0.0,
                wz,
                max_linear_x=MAX_SPEED,
                max_angular_z=MAX_ANGULAR,
                minimum_turning_radius=MINIMUM_TURNING_RADIUS,
                allow_reverse=True,
            )
            try:
                self._write(frame)
                self._drain_feedback()
            except Exception:
                self.state.clear()
                self._fault = "串口写入失败，网页控制已停止"
                break
            self._stop_event.wait(max(0.0, period - (time.monotonic() - started)))

    def start(self) -> None:
        if self._thread is not None:
            return
        reject_nav2_activity()
        self._lock = ControlFileLock(self.lock_path)
        self._lock.acquire()
        try:
            self._fault = None
            reject_nav2_activity()
            self._serial = open_serial(self.device, 115200)
            # Always send at least one neutral frame before exposing the URL.
            self._send_zero(self._serial, 1)
            deadline = time.monotonic() + self.startup_zero_seconds
            while time.monotonic() < deadline:
                self._send_zero(self._serial, 1)
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._tx_loop, name="r680-web-tx", daemon=True)
            self._thread.start()
        except Exception:
            self.stop()
            raise

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        self._thread = None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self.state.clear()
        serial = self._serial
        try:
            if serial is not None:
                # Keep the local serial reference until the final zero frames
                # are flushed; otherwise shutdown can close without sending
                # the commanded stop/recenter frame.
                self._send_zero(serial, 3, suppress_errors=True)
                try:
                    serial.close()
                except Exception:
                    pass
        finally:
            self._serial = None
            lock = self._lock
            self._lock = None
            if lock is not None:
                lock.close()


class _Handler(BaseHTTPRequestHandler):
    server: "WebServer"

    def log_message(self, fmt, *args):
        self.server.log(fmt % args)

    def _json(self, status: int, payload: dict) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _read_payload(self) -> dict:
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length <= 0 or length > 8192:
                raise ValueError("invalid content length")
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("JSON body must be an object")
            return payload
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            raise CommandRejected("invalid JSON request") from exc

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path != "/":
            self._json(404, {"error": "not found"})
            return
        if parse_qs(parsed.query).get("token", [""])[0] != self.server.token:
            self._json(403, {"error": "invalid token"})
            return
        body = render_page(self.server.token).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self) -> None:  # noqa: N802
        try:
            if self.server.controller.fault:
                self._json(503, {"error": self.server.controller.fault})
                return
            payload = self._read_payload()
            token = payload.get("token")
            if not isinstance(token, str) or not hmac.compare_digest(token, self.server.token):
                raise CommandRejected("invalid token")
            client_id = payload.get("client_id", "")
            if not isinstance(client_id, str):
                raise CommandRejected("invalid client id")
            seq = payload.get("seq")
            if isinstance(seq, bool) or not isinstance(seq, int):
                raise CommandRejected("sequence must be an integer")
            if self.path == "/api/stop":
                self.server.controller.state.stop(client_id, seq)
            elif self.path == "/api/command":
                self.server.controller.state.accept(
                    client_id,
                    seq,
                    payload.get("action", ""),
                    payload.get("speed", DEFAULT_SPEED),
                    payload.get("steering", 0.0),
                )
            else:
                self._json(404, {"error": "not found"})
                return
            self._json(200, {"ok": True})
        except CommandRejected as exc:
            self._json(409, {"error": str(exc)})
        except Exception as exc:  # pragma: no cover - server diagnostics
            self._json(400, {"error": str(exc)})


class WebServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, token: str, controller: WebChassisController, log):
        super().__init__(address, _Handler)
        self.token = token
        self.controller = controller
        self.log = log


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", action="store_true", help="required live-control arming flag")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--advertise-host", default="192.0.2.30")
    parser.add_argument("--device", default=DEVICE)
    parser.add_argument("--control-lock", default=CONTROL_LOCK)
    return parser


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    if not args.arm:
        _parser().error("--arm is required; this program can write the chassis UART")
    if not 1024 <= args.port <= 65535:
        _parser().error("port must be between 1024 and 65535")
    token = secrets.token_urlsafe(24)
    controller = WebChassisController(device=args.device, lock_path=args.control_lock)
    server: Optional[WebServer] = None
    previous_sigterm = signal.getsignal(signal.SIGTERM)
    previous_sigint = signal.getsignal(signal.SIGINT)
    sighup = getattr(signal, "SIGHUP", None)
    previous_sighup = signal.getsignal(sighup) if sighup is not None else None
    terminating = False

    def handle_termination(_signum, _frame):
        nonlocal terminating
        if terminating:
            return
        terminating = True
        raise KeyboardInterrupt

    try:
        signal.signal(signal.SIGTERM, handle_termination)
        signal.signal(signal.SIGINT, handle_termination)
        if sighup is not None:
            signal.signal(sighup, handle_termination)
        controller.start()
        server = WebServer((args.bind, args.port), token, controller, print)
        print(f"Web chassis control ready: http://{args.advertise_host}:{args.port}/?token={token}", flush=True)
        print("按住动作按钮运动；松开自动停车并回中；Ctrl+C 退出并释放串口。", flush=True)
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        return 0
    finally:
        try:
            if server is not None:
                server.server_close()
            controller.stop()
        finally:
            signal.signal(signal.SIGINT, previous_sigint)
            signal.signal(signal.SIGTERM, previous_sigterm)
            if sighup is not None:
                signal.signal(sighup, previous_sighup)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
