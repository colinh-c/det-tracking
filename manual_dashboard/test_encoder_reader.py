"""Host-only AS5600 diagnostics and bounded cached polling tests."""
import importlib.util
from pathlib import Path
import sys
import subprocess
import threading
import time
import types
import unittest
from unittest.mock import patch


spec = importlib.util.spec_from_file_location(
    "encoder_reader_under_test", Path(__file__).with_name("encoder_reader.py"))
gateway = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gateway)

class RegisterBus:
    def __init__(self, registers):
        self.registers = registers

    def read_byte_data(self, address, register):
        assert address == 0x36
        value = self.registers[register]
        if isinstance(value, Exception):
            raise value
        return value

    def read_i2c_block_data(self, address, register, length):
        assert length == 2
        return self.read_byte_data(address, register)

    def close(self):
        pass


class EncoderDiagnosticsTests(unittest.TestCase):
    def read(self, overrides=None):
        registers = {0x0B: 0x20, 0x0C: [0x08, 0x00],
                     0x1A: 64, 0x1B: [0x04, 0xD2]}
        registers.update(overrides or {})
        module = types.ModuleType("smbus")
        module.SMBus = lambda number: RegisterBus(registers)
        with patch.dict(sys.modules, {"smbus": module}):
            return gateway.read_encoders()

    def test_connected_device_with_good_field_has_valid_angle_and_diagnostics(self):
        readings = self.read()
        self.assertEqual([item["bus"] for item in readings], [1, 3])
        for item in readings:
            self.assertIs(item.get("connected"), True)
            self.assertIs(item.get("magnet_detected"), True)
            self.assertIs(item.get("field_weak"), False)
            self.assertIs(item.get("field_strong"), False)
            self.assertIs(item["valid"], True)
            self.assertEqual(item["raw"], 2048)
            self.assertEqual(item["degrees"], 180)
            self.assertEqual(item.get("agc"), 64)
            self.assertEqual(item.get("magnitude"), 1234)

    def test_current_weak_field_still_reports_connection_and_untrusted_raw_angle(self):
        item = self.read({0x0B: 0x13, 0x1A: 128, 0x1B: [0, 123]})[0]
        self.assertIs(item.get("connected"), True)
        self.assertIs(item.get("magnet_detected"), False)
        self.assertIs(item.get("field_weak"), True)
        self.assertIs(item.get("field_strong"), False)
        self.assertIs(item["valid"], False)
        self.assertEqual(item["raw"], 2048)
        self.assertEqual(item.get("agc"), 128)
        self.assertEqual(item.get("magnitude"), 123)
        self.assertTrue(item.get("error"))

    def test_detected_magnet_is_not_enough_if_field_is_outside_range(self):
        for status, weak, strong in ((0x30, True, False),
                                     (0x28, False, True),
                                     (0x38, True, True)):
            with self.subTest(status=status):
                item = self.read({0x0B: status})[0]
                self.assertIs(item.get("magnet_detected"), True)
                self.assertIs(item.get("field_weak"), weak)
                self.assertIs(item.get("field_strong"), strong)
                self.assertIs(item["valid"], False)

    def test_optional_register_failure_does_not_discard_successful_primary_read(self):
        for register, missing, retained, value in (
                (0x1A, "agc", "magnitude", 1234),
                (0x1B, "magnitude", "agc", 64)):
            with self.subTest(register=register):
                item = self.read({register: OSError(121, "Remote I/O error")})[0]
                self.assertIs(item.get("connected"), True)
                self.assertIs(item["valid"], True)
                self.assertEqual(item["status"], 0x20)
                self.assertEqual(item["degrees"], 180)
                self.assertIsNone(item.get(missing))
                self.assertEqual(item.get(retained), value)
                self.assertIn(missing, item.get("diagnostic_errors", {}))
                self.assertFalse(item.get("error"))

    def test_missing_device_is_disconnected_with_unknown_magnet_state(self):
        item = self.read({0x0B: OSError(121, "Remote I/O error")})[0]
        self.assertIs(item.get("connected"), False)
        self.assertIs(item["valid"], False)
        self.assertIsNone(item.get("magnet_detected"))
        self.assertNotIn("degrees", item)
        self.assertTrue(item.get("error"))

    def test_failed_angle_read_preserves_successful_status_but_is_not_valid(self):
        item = self.read({0x0C: OSError(121, "Remote I/O error")})[0]
        self.assertIs(item.get("connected"), True)
        self.assertEqual(item.get("status"), 0x20)
        self.assertIs(item.get("magnet_detected"), True)
        self.assertIs(item["valid"], False)
        self.assertNotIn("degrees", item)
        self.assertTrue(item.get("error"))

    def test_missing_hardware_library_is_reported_without_camera_dependencies(self):
        with patch.dict(sys.modules, {"smbus": None, "cv2": None, "picamera2": None}):
            items = gateway.read_encoders()
        self.assertTrue(all(not item["valid"] and not item["connected"] for item in items))
        self.assertTrue(all(item["error"] == "Encoder reader unavailable" for item in items))


class PollerTests(unittest.TestCase):
    def wait_for(self, condition):
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            if condition():
                return
            time.sleep(.002)
        self.fail("Poller did not publish expected sample")

    def make_poller(self, read, **kwargs):
        poller = gateway.EncoderPoller(reader=read, **kwargs)
        self.addCleanup(lambda: poller.stop(timeout=.1))
        return poller

    def test_initial_records_and_single_reader_publish_each_bus(self):
        entered, release = threading.Event(), threading.Event()
        calls = []
        def read(bus):
            calls.append(bus)
            if bus == 3:
                entered.set()
                release.wait(2)
            return {"bus": bus, "connected": True, "valid": True, "raw": 123, "degrees": 10.8}
        poller = self.make_poller(read)
        initial = poller.snapshot()
        self.assertEqual([item["bus"] for item in initial], [1, 3])
        self.assertTrue(all(item["enabled"] and item["stale"] and not item["valid"] for item in initial))
        poller.start()
        poller.start()  # Starting again must not create concurrent I2C readers.
        self.assertTrue(entered.wait(.5))
        try:
            items = poller.snapshot()
            self.assertTrue(items[0]["valid"])
            self.assertFalse(items[1]["valid"])
            self.assertEqual(calls, [1, 3])
        finally:
            release.set()

    def test_stale_validity_uses_monotonic_clock_and_keeps_diagnostic_angle(self):
        clock = [10.0]
        poller = self.make_poller(lambda bus: {"bus": bus, "valid": True, "connected": True,
                                              "raw": 2048, "degrees": 180},
                                  clock=lambda: clock[0], interval=10, stale_after=2)
        poller.start()
        self.wait_for(lambda: poller.snapshot()[1]["valid"])
        clock[0] = 10.5
        fresh = poller.snapshot()[0]
        self.assertTrue(fresh["valid"])
        self.assertFalse(fresh["stale"])
        self.assertEqual(fresh["age_s"], .5)
        clock[0] = 13
        stale = poller.snapshot()[0]
        self.assertFalse(stale["valid"])
        self.assertTrue(stale["stale"])
        self.assertEqual(stale["raw"], 2048)
        self.assertEqual(stale["degrees"], 180)
        self.assertIn("stale", stale["error"].lower())

    def test_blocked_bus_does_not_block_snapshot_or_shutdown(self):
        entered, release = threading.Event(), threading.Event()
        def read(bus):
            entered.set()
            release.wait(2)
            return {"bus": bus, "valid": True}
        poller = self.make_poller(read)
        poller.start()
        self.assertTrue(entered.wait(.5))
        try:
            result = []
            request = threading.Thread(target=lambda: result.append(poller.snapshot()), daemon=True)
            request.start()
            request.join(.2)
            self.assertFalse(request.is_alive(), "Snapshot waited on I2C")
            self.assertFalse(result[0][0]["valid"])
            started = time.monotonic()
            self.assertFalse(poller.stop(timeout=.05))
            self.assertLess(time.monotonic() - started, .2)
            self.assertTrue(poller.thread.daemon)
        finally:
            release.set()
            poller.stop(timeout=.5)

    def test_slow_completed_transaction_cannot_refresh_old_sample(self):
        clock = [10.0]
        def read(bus):
            if bus == 1:
                clock[0] = 20.0
            return {"bus": bus, "connected": True, "valid": True, "degrees": 12}
        poller = self.make_poller(read, clock=lambda: clock[0], interval=10, stale_after=2)
        poller.start()
        self.wait_for(lambda: poller.snapshot()[1]["valid"])
        old, current = poller.snapshot()
        self.assertTrue(old["stale"])
        self.assertFalse(old["valid"])
        self.assertEqual(old["age_s"], 10)
        self.assertFalse(current["stale"])

    def test_fresh_weak_field_retains_angle_without_becoming_valid(self):
        poller = self.make_poller(lambda bus: {"bus": bus, "connected": True, "valid": False,
                                  "field_weak": True, "degrees": 96.94, "raw": 1103,
                                  "error": "Check magnet gap"}, interval=10)
        poller.start()
        self.wait_for(lambda: poller.snapshot()[1].get("raw") == 1103)
        item = poller.snapshot()[0]
        self.assertFalse(item["stale"])
        self.assertFalse(item["valid"])
        self.assertEqual(item["error"], "Check magnet gap")
        self.assertEqual(item["raw"], 1103)

    def test_reader_exception_replaces_previous_success_and_does_not_kill_poller(self):
        failed = threading.Event()
        count = {1: 0, 3: 0}
        def read(bus):
            count[bus] += 1
            if bus == 1 and count[bus] > 1:
                failed.set()
                raise RuntimeError("Unexpected adapter failure")
            return {"bus": bus, "connected": True, "valid": True, "degrees": 12}
        poller = self.make_poller(read, interval=.02)
        poller.start()
        self.wait_for(lambda: poller.snapshot()[0]["valid"])
        self.assertTrue(failed.wait(.5))
        self.wait_for(lambda: not poller.snapshot()[0]["valid"])
        item = poller.snapshot()[0]
        self.assertFalse(item["stale"])
        self.assertIn("failure", item["diagnostic"].lower())
        self.assertNotIn("degrees", item)
        self.assertTrue(poller.snapshot()[1]["valid"])
        self.assertTrue(poller.thread.is_alive())

    def test_snapshot_copies_nested_diagnostics(self):
        poller = self.make_poller(lambda bus: {"bus": bus, "valid": True,
                                  "diagnostic_errors": {"agc": "missing"}}, interval=10)
        poller.start()
        self.wait_for(lambda: poller.snapshot()[1]["valid"])
        item = poller.snapshot()[0]
        item["diagnostic_errors"]["agc"] = "changed by caller"
        self.assertEqual(poller.snapshot()[0]["diagnostic_errors"]["agc"], "missing")


class IsolatedReaderTests(unittest.TestCase):
    def test_default_poller_uses_isolated_reader(self):
        poller = gateway.EncoderPoller()
        self.assertIsInstance(poller.reader, gateway.IsolatedReader)
        self.assertTrue(poller.stop())

    def test_child_command_is_fixed_and_success_preserves_diagnostics(self):
        calls = []
        class Process:
            returncode = 0
            def poll(self):
                return 0
            def communicate(self, timeout):
                return '{"bus": 3, "connected": true, "valid": false, "raw": 3899, "field_weak": true}', ''
        def popen(command, **kwargs):
            calls.append((command, kwargs))
            return Process()
        reader = gateway.IsolatedReader(popen=popen)
        item = reader(3)
        self.assertEqual(item["raw"], 3899)
        self.assertFalse(item["valid"])
        self.assertEqual(calls[0][0], [sys.executable, str(Path(gateway.__file__).resolve()), "--bus", "3"])
        self.assertNotIn("shell", calls[0][1])

    def test_unreaped_timed_out_child_prevents_any_second_process(self):
        calls = []
        class Process:
            returncode = None
            def poll(self):
                return None
            def communicate(self, timeout):
                raise subprocess.TimeoutExpired("mock", timeout)
            def terminate(self):
                calls.append("terminate")
            def kill(self):
                calls.append("kill")
        def popen(*args, **kwargs):
            calls.append("spawn")
            return Process()
        reader = gateway.IsolatedReader(popen=popen)
        first = reader(1)
        second = reader(3)
        self.assertFalse(first["valid"])
        self.assertIn("timed out", first["error"].lower())
        self.assertIn("still stopping", second["error"].lower())
        self.assertEqual(calls.count("spawn"), 1)
        self.assertEqual(calls.count("terminate"), 1)
        self.assertEqual(calls.count("kill"), 1)

    def test_real_blocked_child_is_killed_and_parent_remains_responsive(self):
        children = []
        def popen(command, **kwargs):
            process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], **kwargs)
            children.append(process)
            return process
        reader = gateway.IsolatedReader(popen=popen, timeout=.1)
        started = time.monotonic()
        item = reader(1)
        self.assertFalse(item["valid"])
        self.assertLess(time.monotonic() - started, 1.0)
        self.assertIsNotNone(children[0].poll())

    def test_stop_cancels_actual_child_without_waiting_for_i2c_timeout(self):
        entered = threading.Event()
        children = []
        def popen(command, **kwargs):
            process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], **kwargs)
            children.append(process)
            entered.set()
            return process
        reader = gateway.IsolatedReader(popen=popen, timeout=1)
        poller = gateway.EncoderPoller(reader=reader)
        try:
            poller.start()
            self.assertTrue(entered.wait(.5))
            started = time.monotonic()
            self.assertTrue(poller.stop(timeout=.5))
            self.assertLess(time.monotonic() - started, .6)
            self.assertIsNotNone(children[0].poll())
        finally:
            reader.stop()
            poller.stop()


if __name__ == "__main__":
    unittest.main()
