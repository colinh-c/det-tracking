"""Host-only tests: camera libraries are fakes and no network sockets are opened."""
import importlib.util
import io
import json
import multiprocessing
from pathlib import Path
import queue
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch

SOURCE = Path(__file__).with_name("csi_camera_gateway.py")
if SOURCE.exists():
    spec = importlib.util.spec_from_file_location("csi_camera_gateway", SOURCE)
    gateway = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gateway)
else:
    gateway = None


def blocked_worker():
    while True:
        time.sleep(60)


class FakeProcess:
    def __init__(self):
        self.alive = True
        self.exitcode = None

    def is_alive(self):
        return self.alive


class GatewayTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(gateway, "Native gateway implementation is missing")

    def camera(self):
        camera = gateway.CsiCamera.__new__(gateway.CsiCamera)
        camera.index = 0
        camera.messages = queue.Queue()
        camera.stop_event = threading.Event()
        camera.process = FakeProcess()
        camera.lock = threading.Lock()
        camera.frame = None
        camera.updated = None
        camera.info = {"device": "csi:0", "sequence": 0, "size": [640, 480], "error": "Opening camera"}
        self.addCleanup(camera.stop_event.set)
        return camera

    def collect(self, camera, wanted):
        collector = threading.Thread(target=camera.collect, daemon=True)
        collector.start()
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            with camera.lock:
                if wanted(camera.info):
                    return
            time.sleep(.005)
        self.fail("Collector did not publish the camera event")

    def reply(self, cameras, path, encoders=None):
        handler = gateway.Handler.__new__(gateway.Handler)
        handler.path = path
        handler.server = types.SimpleNamespace(cameras=cameras, encoders=encoders)
        replies = []
        handler.reply = lambda *args: replies.append(args)
        handler.do_GET()
        return replies[0]

    def test_real_frame_metadata_and_stale_frame_rejection(self):
        camera = self.camera()
        camera.messages.put({"frame": b"jpeg", "updated": 10.0,
                             "info": {"device": "csi:0", "sequence": 7, "size": [640, 480],
                                      "error": None, "received_at": 123.0}})
        self.collect(camera, lambda info: info["sequence"] == 7)
        with patch.object(gateway.time, "monotonic", return_value=10.25):
            response = self.reply([camera, camera], "/frame/0.jpg")
        self.assertEqual(response[:3], (200, "image/jpeg", b"jpeg"))
        self.assertEqual(response[3]["sequence"], 7)
        self.assertAlmostEqual(response[3]["age_s"], .25)
        with patch.object(gateway.time, "monotonic", return_value=13.0):
            response = self.reply([camera, camera], "/frame/0.jpg")
        self.assertEqual(response[0], 503)
        self.assertIn("stale", json.loads(response[2])["error"].lower())

    def test_capture_failure_and_worker_death_do_not_serve_previous_image(self):
        camera = self.camera()
        camera.frame = b"oldjpeg"
        camera.updated = time.monotonic()
        camera.info["error"] = None
        camera.messages.put({"frame": None, "updated": None, "info": {"error": "Capture failed"}})
        self.collect(camera, lambda info: info["error"] == "Capture failed")
        self.assertEqual(self.reply([camera, camera], "/frame/1.jpg")[0], 503)
        camera.info["error"] = None
        camera.frame = b"oldjpeg"
        camera.process.alive = False
        camera.process.exitcode = 9
        response = self.reply([camera, camera], "/frame/0.jpg")
        self.assertEqual(response[0], 503)
        self.assertIn("stopped", json.loads(response[2])["error"].lower())

    def test_partial_ipc_message_cannot_block_http_snapshot(self):
        camera = self.camera()
        entered, release = threading.Event(), threading.Event()

        class PartialMessage:
            def get(self, timeout=None):
                entered.set()
                release.wait(2)
                raise EOFError("partial JPEG")

            get_nowait = get

        camera.messages = PartialMessage()
        collector = threading.Thread(target=camera.collect, daemon=True)
        collector.start()
        self.assertTrue(entered.wait(.5))
        returned = []
        request = threading.Thread(target=lambda: returned.append(camera.snapshot()), daemon=True)
        try:
            request.start()
            request.join(.2)
            self.assertFalse(request.is_alive(), "HTTP snapshot blocked on partial camera IPC")
            self.assertIsNone(returned[0][0])
        finally:
            release.set()
            collector.join(.5)

    def test_worker_dying_during_opening_is_reported_as_stopped(self):
        camera = self.camera()
        camera.process.alive = False
        camera.process.exitcode = 2
        self.assertIn("stopped", camera.snapshot()[1]["error"].lower())

    def test_frame_reply_includes_metadata_required_by_viewfinder_freeze(self):
        handler = gateway.Handler.__new__(gateway.Handler)
        handler.request_version = "HTTP/1.1"
        handler.command = "GET"
        handler.requestline = "GET /frame/0.jpg HTTP/1.1"
        handler.wfile = io.BytesIO()
        handler.reply(200, "image/jpeg", b"jpeg", {"age_s": .1, "sequence": 3})
        headers, body = handler.wfile.getvalue().split(b"\r\n\r\n", 1)
        self.assertEqual(body, b"jpeg")
        info = next(line.split(b": ", 1)[1] for line in headers.split(b"\r\n") if line.startswith(b"X-Frame-Info:"))
        self.assertEqual(json.loads(info), {"age_s": .1, "sequence": 3})

    def test_disabled_encoders_never_import_hardware_library(self):
        with patch.dict(sys.modules, {"smbus": None, "smbus2": None}):
            response = self.reply([], "/encoders")
        self.assertEqual(response[0], 200)
        records = json.loads(response[2])
        self.assertEqual(len(records), 2)
        self.assertTrue(all(not item["enabled"] and not item["valid"] and not item["connected"] for item in records))

    def test_enabled_encoder_route_only_reads_cached_snapshot(self):
        records = [{"bus": 1, "enabled": True, "valid": False, "stale": False,
                    "raw": 1103, "field_weak": True, "error": "Check magnet gap"},
                   {"bus": 3, "enabled": True, "valid": False, "stale": True}]
        calls = []
        poller = types.SimpleNamespace(snapshot=lambda: calls.append(True) or records)
        with patch.dict(sys.modules, {"smbus": None, "smbus2": None}):
            response = self.reply([], "/encoders", poller)
        self.assertEqual(response[0], 200)
        self.assertEqual(json.loads(response[2]), records)
        self.assertEqual(calls, [True])

    def run_main_without_hardware(self, enabled):
        lifecycle = []
        class Server:
            def __init__(self, address, handler):
                lifecycle.append(("bind", address))

            def handle_request(self):
                lifecycle.append("request")
                raise KeyboardInterrupt

            def server_close(self):
                lifecycle.append("server_close")

        class Poller:
            def start(self):
                lifecycle.append("encoder_start")

            def stop(self):
                lifecycle.append("encoder_stop")

        encoder_module = types.SimpleNamespace(EncoderPoller=Poller) if enabled else None
        with patch.object(sys, "argv", ["csi_camera_gateway.py"] + (["--encoders"] if enabled else [])), \
                patch.object(gateway, "ThreadingHTTPServer", Server), \
                patch.object(gateway, "CsiCamera", side_effect=lambda *args: lifecycle.append(("camera", args)) or object()), \
                patch.object(gateway, "shutdown_cameras", side_effect=lambda cameras: lifecycle.append(("camera_stop", len(cameras))) or True), \
                patch.object(gateway.signal, "signal"), patch("builtins.print"), \
                patch.dict(sys.modules, {"encoder_reader": encoder_module, "smbus": None, "picamera2": None}):
            with self.assertRaises(KeyboardInterrupt):
                gateway.main()
        return lifecycle

    def test_main_encoder_opt_in_starts_and_stops_cache_reader(self):
        lifecycle = self.run_main_without_hardware(True)
        self.assertEqual(lifecycle[0], ("bind", ("127.0.0.1", 8082)))
        self.assertEqual(lifecycle[-4:], ["request", "server_close", "encoder_stop", ("camera_stop", 2)])
        self.assertEqual(lifecycle.count("encoder_start"), 1)
        self.assertEqual(len([item for item in lifecycle if isinstance(item, tuple) and item[0] == "camera"]), 2)

    def test_main_default_does_not_import_or_start_encoder_reader(self):
        lifecycle = self.run_main_without_hardware(False)
        self.assertNotIn("encoder_start", lifecycle)
        self.assertNotIn("encoder_stop", lifecycle)
        self.assertEqual(lifecycle[-1], ("camera_stop", 2))

    def run_worker(self, failure=None, encode_ok=True):
        stop = threading.Event()
        messages = queue.Queue(maxsize=4)
        lifecycle = []
        image = types.SimpleNamespace(shape=(480, 640, 3))

        class Camera:
            def __init__(self, index):
                lifecycle.append(("open", index))

            def create_video_configuration(self, **kwargs):
                self.config = kwargs
                return kwargs

            def configure(self, config):
                lifecycle.append(("configure", config))
                if failure == "configure":
                    raise RuntimeError("configuration failed")

            def start(self):
                lifecycle.append("start")

            def capture_array(self, name):
                self_name = name
                if self_name != "main":
                    raise AssertionError("wrong stream")
                if failure == "capture":
                    raise RuntimeError("capture failed")
                stop.set()
                return image

            def stop(self):
                lifecycle.append("stop")
                if failure == "stop":
                    raise RuntimeError("stop failed")

            def close(self):
                lifecycle.append("close")

        def fromarray(actual):
            self.assertIs(actual, image)
            def save(output, format, quality):
                self.assertEqual(format, "JPEG")
                self.assertEqual(quality, 80)
                if not encode_ok:
                    raise OSError("JPEG encoding failed")
                output.write(b"encodedjpeg")
            return types.SimpleNamespace(save=save)

        with patch.dict(sys.modules, {
            "picamera2": types.SimpleNamespace(Picamera2=Camera),
            "cv2": None,
            "PIL": types.SimpleNamespace(Image=types.SimpleNamespace(fromarray=fromarray)),
        }):
            gateway.capture_worker(1, 640, 480, 15, 80, messages, stop)
        events = []
        while not messages.empty():
            events.append(messages.get_nowait())
        return lifecycle, events

    def test_worker_publishes_jpeg_with_real_size_then_releases_camera(self):
        lifecycle, events = self.run_worker()
        self.assertEqual(lifecycle[0], ("open", 1))
        self.assertEqual(lifecycle[1][1]["main"], {"size": (640, 480), "format": "BGR888"})
        self.assertEqual(lifecycle[1][1]["controls"], {"FrameRate": 15})
        self.assertEqual(lifecycle[-2:], ["stop", "close"])
        self.assertEqual(events[-1]["frame"], b"encodedjpeg")
        self.assertEqual(events[-1]["info"]["size"], [640, 480])
        self.assertEqual(events[-1]["info"]["sequence"], 1)

    def test_worker_error_paths_release_acquired_camera_and_report_failure(self):
        for failure in ("configure", "capture", "stop"):
            with self.subTest(failure=failure):
                lifecycle, events = self.run_worker(failure=failure)
                self.assertEqual(lifecycle[-1], "close")
                self.assertTrue(events[-1]["info"]["error"])
                self.assertIsNone(events[-1]["frame"])

    def test_jpeg_encoding_failure_is_visible_instead_of_silent_frozen_image(self):
        _, events = self.run_worker(encode_ok=False)
        self.assertIn("JPEG", events[-1]["info"]["error"])
        self.assertIsNone(events[-1]["frame"])

    def test_blocked_capture_process_is_terminated_with_bounded_shutdown(self):
        context = multiprocessing.get_context("spawn")
        process = context.Process(target=blocked_worker, daemon=True)
        process.start()
        camera = types.SimpleNamespace(process=process, stop_event=context.Event(), messages=context.Queue())
        started = time.monotonic()
        self.assertTrue(gateway.shutdown_cameras([camera], grace=.1, force=.5))
        self.assertLess(time.monotonic() - started, 2.0)
        self.assertFalse(process.is_alive())


if __name__ == "__main__":
    unittest.main()
