#!/usr/bin/env python3
"""Native Pi cameras for the existing dual viewer. Loopback only; no motion.

Each camera owns a separate spawned process. Only the worker imports Picamera2
and Pillow. Receipt age is measured on this host, not synchronized exposure time.
Optional encoder diagnostics use a serialized background reader, never HTTP I2C.
"""
import argparse
import io
import json
import multiprocessing
import queue
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse


def publish(messages, event):
    """Keep a small latest-frame mailbox without blocking camera shutdown."""
    try:
        messages.put_nowait(event)
        return
    except queue.Full:
        pass
    try:
        messages.get_nowait()
    except queue.Empty:
        pass
    try:
        messages.put_nowait(event)
    except queue.Full:
        pass


def capture_worker(index, width, height, fps, quality, messages, stop_event):
    camera = None
    started = False
    info = {"device": f"csi:{index}", "sequence": 0, "size": [width, height],
            "backend": "Picamera2", "error": "Opening camera"}

    def failed(error):
        info["error"] = str(error)
        publish(messages, {"frame": None, "updated": None, "info": dict(info)})

    try:
        from picamera2 import Picamera2
        from PIL import Image

        camera = Picamera2(index)
        config = camera.create_video_configuration(
            main={"size": (width, height), "format": "BGR888"},
            controls={"FrameRate": fps}, buffer_count=4, queue=False)
        camera.configure(config)
        camera.start()
        started = True
        while not stop_event.is_set():
            # Picamera2 BGR888 arrays contain R,G,B bytes for Pillow.
            image = camera.capture_array("main")
            received = time.monotonic()
            wall_time = time.time()
            encoded = io.BytesIO()
            Image.fromarray(image).save(encoded, format="JPEG", quality=quality)
            info.update(sequence=info["sequence"] + 1, size=[image.shape[1], image.shape[0]],
                        received_at=wall_time, error=None)
            publish(messages, {"frame": encoded.getvalue(), "updated": received, "info": dict(info)})
    except Exception as error:
        failed(f"Camera {index}: {error}")
    finally:
        if camera is not None:
            try:
                if started:
                    camera.stop()
            except Exception as error:
                failed(f"Camera {index} stop failed: {error}")
            finally:
                try:
                    camera.close()
                except Exception as error:
                    failed(f"Camera {index} close failed: {error}")
        # A full mailbox must not keep a stopped child waiting for its feeder.
        if hasattr(messages, "cancel_join_thread"):
            messages.cancel_join_thread()


class CsiCamera:
    def __init__(self, index, width, height, fps, quality=80):
        self.index = index
        self.lock = threading.Lock()
        self.frame = None
        self.updated = None
        self.info = {"device": f"csi:{index}", "sequence": 0, "size": [width, height],
                     "backend": "Picamera2", "error": "Opening camera"}
        context = multiprocessing.get_context("spawn")
        self.messages = context.Queue(maxsize=2)
        self.stop_event = context.Event()
        self.process = context.Process(target=capture_worker,
                                       args=(index, width, height, fps, quality, self.messages, self.stop_event),
                                       daemon=True, name=f"csi-camera-{index}")
        try:
            self.process.start()
        except BaseException:
            self.messages.close()
            raise
        self.collector = threading.Thread(target=self.collect, daemon=True, name=f"csi-collector-{index}")
        self.collector.start()

    def collect(self):
        # Even Queue.get_nowait can block on an incomplete large pipe message.
        # Keep all IPC reads outside the HTTP path and outside the cache lock.
        while not self.stop_event.is_set():
            try:
                event = self.messages.get(timeout=.2)
            except queue.Empty:
                if not self.process.is_alive():
                    return
                continue
            except (EOFError, OSError, ValueError) as error:
                with self.lock:
                    self.frame = None
                    self.info["error"] = f"Camera stream ended: {error}"
                return
            with self.lock:
                self.frame, self.updated = event["frame"], event["updated"]
                self.info.update(event["info"])

    def snapshot(self):
        with self.lock:
            info = dict(self.info)
            info["age_s"] = 0 if self.updated is None else max(0, time.monotonic() - self.updated)
            if not self.process.is_alive() and not self.stop_event.is_set():
                if not info.get("error") or info["error"] == "Opening camera":
                    info["error"] = f"Camera process stopped ({self.process.exitcode})"
            if self.updated is not None and info["age_s"] > 2 and not info.get("error"):
                info["error"] = "Camera feed is stale"
            return self.frame, info


def shutdown_cameras(cameras, grace=2.0, force=1.0):
    """Signal both first, then shared deadlines for graceful stop/terminate/kill."""
    for camera in cameras:
        camera.stop_event.set()
    for action, budget in ((None, grace), ("terminate", force), ("kill", force)):
        if action:
            for camera in cameras:
                if camera.process.is_alive():
                    getattr(camera.process, action)()
        deadline = time.monotonic() + budget
        for camera in cameras:
            camera.process.join(max(0, deadline - time.monotonic()))
    stopped = all(not camera.process.is_alive() for camera in cameras)
    for camera in cameras:
        camera.messages.cancel_join_thread()
        camera.messages.close()
    return stopped


def disabled_encoders():
    return [{"bus": number, "address": "0x36", "enabled": False, "connected": False, "valid": False,
             "error": "Encoder reads disabled"} for number in (1, 3)]


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/status":
            self.reply(200, "application/json",
                       json.dumps([camera.snapshot()[1] for camera in self.server.cameras]).encode())
        elif path == "/encoders":
            readings = self.server.encoders.snapshot() if self.server.encoders is not None else disabled_encoders()
            self.reply(200, "application/json", json.dumps(readings).encode())
        elif path in ("/frame/0.jpg", "/frame/1.jpg"):
            frame, info = self.server.cameras[int(path.split("/")[2][0])].snapshot()
            if frame is None or info["age_s"] > 2 or info.get("error"):
                self.reply(503, "application/json", json.dumps(info).encode())
            else:
                self.reply(200, "image/jpeg", frame, info)
        else:
            self.reply(404, "text/plain", b"Not found")

    def reply(self, status, content_type, data, info=None):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        if info:
            self.send_header("X-Frame-Info", json.dumps(info, allow_nan=False))
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cameras", type=int, nargs=2, default=[0, 1])
    parser.add_argument("--width", type=int, default=1024)
    parser.add_argument("--height", type=int, default=768)
    parser.add_argument("--fps", type=float, default=15)
    parser.add_argument("--quality", type=int, default=80)
    parser.add_argument("--port", type=int, default=8082)
    parser.add_argument("--encoders", action="store_true", help="Enable cached AS5600 diagnostics on Bus 1 and Bus 3")
    args = parser.parse_args()
    if (args.cameras[0] == args.cameras[1] or min(args.cameras) < 0 or
            not 1 <= args.width <= 1456 or not 1 <= args.height <= 1088 or
            not 0 < args.fps <= 60 or not 1 <= args.quality <= 95 or not 1 <= args.port <= 65535):
        parser.error("Use distinct camera indices, a size within 1456x1088, fps 0..60, quality 1..95 and a valid port")
    stopping = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopping.set())
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    server.daemon_threads = True
    server.timeout = .2
    server.cameras = []
    server.encoders = None
    try:
        for index in args.cameras:
            server.cameras.append(CsiCamera(index, args.width, args.height, args.fps, args.quality))
        if args.encoders:
            from encoder_reader import EncoderPoller
            server.encoders = EncoderPoller()
            server.encoders.start()
        encoder_state = "Bus 1/Bus 3 encoder diagnostics enabled" if args.encoders else "encoders disabled"
        print(f"Native dual camera gateway on 127.0.0.1:{args.port}; {encoder_state}; motion disabled", flush=True)
        while not stopping.is_set():
            server.handle_request()
    finally:
        server.server_close()
        if server.encoders is not None:
            server.encoders.stop()
        if not shutdown_cameras(server.cameras):
            raise RuntimeError("A capture process did not exit after bounded shutdown")


if __name__ == "__main__":
    main()
