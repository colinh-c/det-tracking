#!/usr/bin/env python3
"""Local dual viewfinder, wall references, and optional bounded manual jogs."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import secrets
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import Request, urlopen
import uuid

from calibration import CalibrationError, build_reference, save_reference

ROOT = Path(__file__).resolve().parent


class State:
    def __init__(self, source, motion_source=None):
        self.source = source.rstrip("/")
        self.motion_source = motion_source.rstrip("/") if motion_source else None
        self.motion_token = secrets.token_urlsafe(24)
        self.move_lock = threading.Lock()
        self.frozen = None
        self.last_saved = None
        self.lock = threading.Lock()

    def fetch(self, path):
        with urlopen(self.source + path, timeout=3) as response:
            return response.read(), dict(response.headers)

    def freeze(self):
        if not self.move_lock.acquire(blocking=False):
            raise CalibrationError("Wait for movement to finish before freezing")
        try:
            if self.motion_source:
                motion = self.motion('/motion')
                if motion.get('busy') or motion.get('hold_active'):
                    raise CalibrationError("Wait for movement to finish before freezing")
            return self._freeze()
        finally:
            self.move_lock.release()

    def _freeze(self):
        with ThreadPoolExecutor(max_workers=2) as pool:
            results = list(pool.map(self.fetch, ["/frame/0.jpg", "/frame/1.jpg"]))
        frames, metadata = [], []
        for image, headers in results:
            info = json.loads(next(value for key, value in headers.items() if key.lower() == "x-frame-info"))
            if info["age_s"] > 2 or info.get("error"):
                raise CalibrationError("Camera feed is stale; wait for both live views")
            frames.append(image)
            metadata.append(info)
        frozen = {"id": uuid.uuid4().hex, "created_at": time.time(), "frames": frames, "cameras": metadata}
        try:
            raw, _ = self.fetch("/encoders")
            frozen["encoders"] = json.loads(raw)
        except (OSError, ValueError):
            frozen["encoders"] = {"error": "Encoder readout unavailable at freeze"}
        with self.lock:
            self.frozen = frozen
        return {key: value for key, value in frozen.items() if key != "frames"}

    def motion(self, path, data=None):
        if not self.motion_source:
            raise CalibrationError("Manual movement is not configured")
        is_jog = path == "/motion/jog"
        needs_live_views = is_jog or path in ("/motion/hold/start", "/motion/hold/renew")
        acquired = False
        try:
            if needs_live_views:
                acquired = self.move_lock.acquire(blocking=False)
                if not acquired:
                    raise CalibrationError("Wait for the current movement to finish")
                with self.lock:
                    if self.frozen:
                        raise CalibrationError("Resume live views before moving")
                raw, _ = self.fetch("/status")
                status = json.loads(raw)
                if len(status) != 2 or any(item.get("error") or not 0 <= item.get("age_s", 99) <= 2 for item in status):
                    raise CalibrationError("Wait for both live camera feeds before moving")
            request = Request(self.motion_source + path)
            if data is not None:
                request = Request(self.motion_source + path,
                                  data=json.dumps(data, allow_nan=False).encode(),
                                  headers={"Content-Type": "application/json", "X-DET-Motion": "1"})
            with urlopen(request, timeout=45 if is_jog else 8) as response:
                return json.loads(response.read())
        finally:
            if acquired:
                self.move_lock.release()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, code, content_type, data, extra=None):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        for name, value in (extra or {}).items():
            self.send_header(name, value)
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def json(self, code, data):
        self.reply(code, "application/json", json.dumps(data, allow_nan=False).encode())

    def do_GET(self):
        path = urlparse(self.path).path
        try:
            if path in ("/session", "/motion") and self.headers.get("Host") not in (
                    f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"):
                self.json(403, {"error": "Use the local viewfinder address"})
                return
            if path == "/":
                self.reply(200, "text/html; charset=utf-8", (ROOT / "viewfinder.html").read_bytes())
            elif path == "/geometry.js":
                self.reply(200, "application/javascript", (ROOT / "geometry.js").read_bytes())
            elif path in ("/frame/0.jpg", "/frame/1.jpg", "/status", "/encoders"):
                data, headers = self.server.state.fetch(path)
                extra = {name: value for name, value in headers.items() if name.lower() == "x-frame-info"}
                self.reply(200, headers.get("Content-Type", "application/octet-stream"), data, extra)
            elif path in ("/frozen/0.jpg", "/frozen/1.jpg"):
                with self.server.state.lock:
                    frozen = self.server.state.frozen
                if not frozen:
                    raise CalibrationError("Freeze the views first")
                self.reply(200, "image/jpeg", frozen["frames"][int(path.split("/")[2][0])])
            elif path == "/session":
                with self.server.state.lock:
                    frozen = self.server.state.frozen
                    data = {"motion_enabled": bool(self.server.state.motion_source),
                            "motion_mode": "manual_step_and_hold", "motion_token": self.server.state.motion_token,
                            "last_saved": self.server.state.last_saved,
                            "freeze_id": frozen["id"] if frozen else None}
                self.json(200, data)
            elif path == "/motion":
                self.json(200, self.server.state.motion(path))
            else:
                self.reply(404, "text/plain", b"Not found")
        except (URLError, TimeoutError, OSError, ValueError, StopIteration) as error:
            self.json(503, {"error": str(error)})

    def do_POST(self):
        origin = self.headers.get("Origin")
        if origin and urlparse(origin).netloc != self.headers.get("Host"):
            self.json(403, {"error": "Use the local viewfinder page"})
            return
        if self.path.startswith("/motion/"):
            expected_hosts = (f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}")
            if (self.headers.get("Host") not in expected_hosts
                    or self.headers.get("Content-Type", "").split(";")[0].strip() != "application/json"
                    or not secrets.compare_digest(self.headers.get("X-DET-Token", ""), self.server.state.motion_token)):
                self.json(403, {"error": "Refresh the local dashboard before using movement controls"})
                return
        try:
            size = int(self.headers.get("Content-Length", 0))
            if not 0 <= size <= 65536:
                raise CalibrationError("Invalid request size")
            data = json.loads(self.rfile.read(size) or b"{}")
            if self.path == "/freeze":
                self.json(200, self.server.state.freeze())
            elif self.path == "/resume":
                with self.server.state.lock:
                    self.server.state.frozen = None
                self.json(200, {"live": True})
            elif self.path in ("/motion/jog", "/motion/stop", "/motion/hold/start", "/motion/hold/renew", "/motion/limits"):
                if not isinstance(data, dict):
                    raise CalibrationError("Invalid movement request")
                self.json(200, self.server.state.motion(self.path, data))
            elif self.path == "/save":
                with self.server.state.lock:
                    frozen = self.server.state.frozen
                if not frozen or data.get("freeze_id") != frozen["id"]:
                    raise CalibrationError("These marks belong to an older frozen view; freeze again")
                for index, view in enumerate(data["views"]):
                    view["size"] = frozen["cameras"][index]["size"]
                    view["camera"] = frozen["cameras"][index]
                reference = build_reference(data)
                reference["freeze_at"] = frozen["created_at"]
                reference["encoders_at_freeze"] = frozen["encoders"]
                try:
                    raw, _ = self.server.state.fetch("/encoders")
                    reference["encoders_at_save"] = json.loads(raw)
                except (OSError, ValueError):
                    reference["encoders_at_save"] = {"error": "Encoder readout unavailable"}
                destination = save_reference(ROOT / "captures", reference, frozen["frames"])
                with self.server.state.lock:
                    self.server.state.last_saved = {"path": str(destination), "reference": reference}
                self.json(200, {"path": str(destination), "status": reference["status"]})
            else:
                self.json(404, {"error": "Not found"})
        except (CalibrationError, KeyError, IndexError, TypeError, ValueError) as error:
            self.json(400, {"error": str(error)})
        except HTTPError as error:
            try:
                result = json.loads(error.read())
            except (ValueError, OSError):
                result = {"error": "Motor controller unavailable"}
            self.json(error.code, result)
        except (URLError, TimeoutError, OSError) as error:
            self.json(503, {"error": str(error)})


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="http://127.0.0.1:18082")
    parser.add_argument("--port", type=int, default=18182)
    parser.add_argument("--motion-source", default=None)
    args = parser.parse_args()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    server.daemon_threads = True
    server.state = State(args.source, args.motion_source)
    print(f"Viewfinder: http://127.0.0.1:{args.port}/; manual movement configured: {bool(args.motion_source)}", flush=True)
    server.serve_forever()
