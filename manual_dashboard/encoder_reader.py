"""Read-only AS5600 diagnostics on Bus 1 and Bus 3, with no camera dependency.

Bus numbers do not establish a mechanical axis, direction, or zero. The poller
serializes bounded reader subprocesses; consumers only read its cache. Native
smbus calls stay outside the camera/HTTP interpreter, even if they hold its GIL.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time

BUSES = (1, 3)


def empty_reading(number):
    return {"bus": number, "address": "0x36", "enabled": True,
            "connected": False, "valid": False, "magnet_detected": None,
            "field_weak": None, "field_strong": None, "agc": None, "magnitude": None}


def read_encoder(number):
    """Perform one bus read. Only the background poller should call this live."""
    if number not in BUSES:
        raise ValueError("Only the verified Bus 1 and Bus 3 are supported")
    info = empty_reading(number)
    try:
        from smbus import SMBus
        bus = SMBus(number)
        try:
            status = bus.read_byte_data(0x36, 0x0B)
            info.update(connected=True, status=status, magnet_detected=bool(status & 0x20),
                        field_weak=bool(status & 0x10), field_strong=bool(status & 0x08),
                        read_at=time.time())
            hi, lo = bus.read_i2c_block_data(0x36, 0x0C, 2)
            raw = ((hi << 8) | lo) & 0xFFF
            info.update(raw=raw, degrees=raw * 360 / 4096,
                        valid=info["magnet_detected"] and not info["field_weak"] and not info["field_strong"])
            if not info["valid"]:
                info["error"] = "Magnet not detected" if not info["magnet_detected"] else "Check magnet gap"
            # Optional diagnostics cannot erase a successful status and angle.
            try:
                info["agc"] = bus.read_byte_data(0x36, 0x1A)
            except OSError as error:
                info.setdefault("diagnostic_errors", {})["agc"] = str(error)
            try:
                hi, lo = bus.read_i2c_block_data(0x36, 0x1B, 2)
                info["magnitude"] = ((hi << 8) | lo) & 0xFFF
            except OSError as error:
                info.setdefault("diagnostic_errors", {})["magnitude"] = str(error)
        finally:
            bus.close()
    except OSError as error:
        info["valid"] = False
        info["error"] = ("Encoder angle read failed" if info["connected"] else
                         "Encoder bus unavailable" if error.errno == 2 else "No encoder response")
        info["diagnostic"] = str(error)
    except ImportError as error:
        info["error"] = "Encoder reader unavailable"
        info["diagnostic"] = str(error)
    return info


def read_encoders():
    """Synchronous diagnostics helper; never used in an HTTP request."""
    return [read_encoder(number) for number in BUSES]


class IsolatedReader:
    """One child at a time, with bounded waits and no retry over a stuck child."""
    def __init__(self, popen=None, timeout=1.0):
        if not math.isfinite(timeout) or not 0 < timeout <= 1:
            raise ValueError("Reader timeout must be positive and at most one second")
        self.popen = subprocess.Popen if popen is None else popen
        self.timeout = timeout
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.process = None

    def failure(self, number, message, diagnostic=None):
        info = empty_reading(number)
        info["error"] = message
        if diagnostic:
            info["diagnostic"] = str(diagnostic)[:500]
        return info

    def __call__(self, number):
        if number not in BUSES:
            raise ValueError("Only Bus 1 and Bus 3 are supported")
        if self.stopping.is_set():
            return self.failure(number, "Encoder reader stopped")
        with self.lock:
            previous = self.process
        if previous is not None:
            if previous.poll() is None:
                return self.failure(number, "Previous encoder reader is still stopping")
            # A child that resisted kill can exit later. Reap it before another
            # bus read, closing its local pipes without an unbounded wait.
            try:
                previous.communicate(timeout=.05)
            except (subprocess.TimeoutExpired, OSError):
                return self.failure(number, "Previous encoder reader is still stopping")
        command = [sys.executable, str(Path(__file__).resolve()), "--bus", str(number)]
        process = self.popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        with self.lock:
            self.process = process
        # stop() may have raced the child creation. Never leave that child live.
        if self.stopping.is_set():
            self.stop()
        try:
            output, diagnostic = process.communicate(timeout=self.timeout)
        except subprocess.TimeoutExpired:
            self.finish_timed_out(process)
            return self.failure(number, "Encoder read timed out")
        with self.lock:
            if self.process is process:
                self.process = None
        if process.returncode != 0:
            return self.failure(number, "Encoder reader failed", diagnostic)
        try:
            info = json.loads(output)
            if (not isinstance(info, dict) or info.get("bus") != number or
                    not isinstance(info.get("valid"), bool)):
                raise ValueError("Unexpected encoder reader result")
            return info
        except (ValueError, TypeError) as error:
            return self.failure(number, "Encoder reader returned invalid data", error)

    def finish_timed_out(self, process):
        for action in (process.terminate, process.kill):
            try:
                action()
            except OSError:
                pass
            try:
                process.communicate(timeout=.15)
            except (subprocess.TimeoutExpired, OSError):
                continue
            with self.lock:
                if self.process is process:
                    self.process = None
            return
        # Keep this child recorded if it cannot be reaped. Starting a second
        # reader could overlap I2C operations, so later calls report the fault.

    def stop(self):
        """Request cancellation without waiting on the child or its pipes."""
        self.stopping.set()
        with self.lock:
            process = self.process
        if process is not None:
            try:
                process.kill()
            except OSError:
                pass


class EncoderPoller:
    def __init__(self, reader=None, interval=.5, stale_after=2.0, clock=None):
        if not all(math.isfinite(value) and value > 0 for value in (interval, stale_after)):
            raise ValueError("Poll interval and stale threshold must be positive and finite")
        self.reader = IsolatedReader() if reader is None else reader
        self.clock = time.monotonic if clock is None else clock
        self.interval = interval
        self.stale_after = stale_after
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.records = {number: (empty_reading(number), None) for number in BUSES}

    def start(self):
        with self.lock:
            if self.stop_event.is_set():
                raise RuntimeError("A stopped encoder poller cannot be restarted")
            if self.thread is None:
                self.thread = threading.Thread(target=self.poll, name="as5600-poller", daemon=True)
                self.thread.start()

    def poll(self):
        while not self.stop_event.is_set():
            for number in BUSES:
                if self.stop_event.is_set():
                    return
                # Age starts before I2C, so a delayed transaction cannot make an
                # old angle look fresh when an optional register finally returns.
                started = self.clock()
                try:
                    info = self.reader(number)
                except Exception as error:
                    info = empty_reading(number)
                    info.update(error="Encoder read failed", diagnostic=str(error))
                if self.stop_event.is_set():
                    return
                with self.lock:
                    self.records[number] = (copy.deepcopy(info), started)
            if self.stop_event.wait(self.interval):
                return

    def snapshot(self):
        """Copy cached data without I2C, thread joins, or queue operations."""
        with self.lock:
            records = copy.deepcopy(self.records)
        now = self.clock()
        result = []
        for number in BUSES:
            info, started = records[number]
            age = None if started is None else max(0, now - started)
            stale = started is None or age > self.stale_after
            info.update(bus=number, enabled=True, age_s=age, stale=stale)
            if self.stop_event.is_set():
                info.update(valid=False, error="Encoder polling stopped")
            elif stale:
                if info.get("error"):
                    info["last_error"] = info["error"]
                info.update(valid=False, error="Waiting for encoder reading" if started is None else
                            "Encoder reading is stale")
            result.append(info)
        return result

    def stop(self, timeout=.5):
        """Cancel the isolated reader and bound the collector thread wait."""
        if not math.isfinite(timeout) or not 0 <= timeout <= 1:
            raise ValueError("Shutdown wait must be between zero and one second")
        self.stop_event.set()
        if isinstance(self.reader, IsolatedReader):
            self.reader.stop()
        if self.thread is not None:
            self.thread.join(timeout)
            return not self.thread.is_alive()
        return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Read one AS5600 without configuring it")
    parser.add_argument("--bus", type=int, choices=BUSES, required=True)
    arguments = parser.parse_args()
    print(json.dumps(read_encoder(arguments.bus), allow_nan=False), flush=True)
