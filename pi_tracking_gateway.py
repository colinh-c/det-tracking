#!/usr/bin/env python3
"""Separate Pi automatic velocity gateway; bind only 127.0.0.1:8084.

Run after stopping the manual motion service (both use exclusive serial):
  python3 pi_tracking_gateway.py --port 8084
Forward Mac localhost:18084 to Pi localhost:8084 with authenticated SSH.
No cameras or control algorithms run here. GET /status; POST /velocity with
Command JSON + generation + ttl_s; POST /stop. POST requires X-DET-Motion: 1.

Startup requires CAP VEL_LEASE_V1, then verifies PING/STAT/DIAG, STOP,
ENC 0, MODE SLEW, ACC 75, DSTAT, STOP, and
stable step counts. It never clears faults, zeros, homes, changes current,
flashes firmware, retries a command, or reconnects. A fault requires restart.
Stop/expiry revokes the generation before waiting for any serial operation.

The required launch firmware enforces LVEL expiry in its 1 kHz timer interrupt.
Legacy firmware is refused; there is NO fallback to unleased VEL. The launch
firmware copy is supplied under firmware/ and must be separately reviewed and
installed before live use. STOP acknowledgement plus stable motor counters is
not independent evidence of physical rest.

Serial protocol/diagnostics adapted from ../DET_Calibration_2026-10-27/
motion_gateway.py SHA256 2ca9fcd2d5df7b6ed5d2e225032679557490cdab806d0966c625db4a1d063429.
Reviewed firmware: gimbal_skr_pico.ino, VEL/STOP/STAT/DIAG handlers.
"""
import argparse
import os
import json
import math
from pathlib import Path
import hashlib
import re
import secrets
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit


DEVICE = os.environ.get('DET_MOTOR_DEVICE', '/dev/ttyACM0')
MAX_VELOCITY = 15.0
MAX_LEASE = .5
STOP_TIMEOUT = .85
SETTLE_TIMEOUT = 1.5
POSITION_TOLERANCE = 16 * 1.8 / (256 * 160) + .00001
SOFT_LIMITS = {'az': (-180.0, 180.0), 'el': (-10.0, 95.0)}
LIMITS_PROFILE = str(Path.home() / 'gimbal' / 'motion_limits.json')


class LinkError(RuntimeError):
    pass


class GatewayError(RuntimeError):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


class SerialLink:
    """The reviewed manual gateway transaction contract, narrowed to velocity."""
    def __init__(self, port):
        self.port = port

    @property
    def is_open(self):
        return self.port.is_open

    def close(self):
        self.port.close()

    def request(self, command, timeout, stop=False):
        terminators = {'CAP': 'OK CAP VEL_LEASE_V1', 'LSTAT': 'OK LSTAT ',
                       'PING': 'OK PONG', 'DIAG': 'OK DIAG', 'DSTAT': 'OK DSTAT', 'STOP': 'OK STOP',
                       'ENC 0': 'OK ENC 0', 'MODE SLEW': 'OK MODE ', 'ACC 75': 'OK ACC 75.00'}
        if command == 'STAT':
            terminal = 'OK '
        elif re.fullmatch(r'LVEL \d+ \d+ \d+ -?\d+\.\d+ -?\d+\.\d+', command):
            terminal = 'OK LVEL '
        elif command in terminators:
            terminal = terminators[command]
        else:
            raise LinkError('Command is outside the tracking gateway protocol')
        deadline = time.monotonic() + timeout
        payload = (command + '\n').encode('ascii')
        lines, pending = [], b''
        try:
            if self.port.write(payload) != len(payload):
                raise LinkError('Incomplete serial write; outcome is unknown')
            while time.monotonic() < deadline:
                self.port.timeout = min(.05, max(.001, deadline-time.monotonic()))
                pending += self.port.read_until(b'\n', size=512-len(pending))
                if len(pending) >= 512:
                    raise LinkError('Oversized controller response')
                if not pending.endswith(b'\n'):
                    continue
                line = pending.decode('ascii').strip()
                pending = b''
                if not line:
                    continue
                lines.append(line)
                if len(lines) > 32:
                    raise LinkError('Too many controller response lines')
                if not stop and line.startswith(('#', '!', 'ERR')):
                    raise LinkError('Controller reset or fault: ' + line)
                matches = line.startswith(terminal) if terminal.endswith(' ') else line == terminal
                if matches:
                    return lines
                if command in ('DIAG', 'DSTAT') and line.startswith(('AZ ', 'EL ')):
                    continue
                if not stop:
                    raise LinkError('Unexpected controller response: ' + line)
        except LinkError:
            raise
        except Exception as error:
            raise LinkError('Serial transaction failed: ' + str(error)) from error
        raise LinkError('Controller reply timed out; no command was retried')


def parse_status(lines):
    if len(lines) != 1:
        raise LinkError('Invalid STAT response length')
    fields = lines[0].split()
    if len(fields) != 9 or fields[0] != 'OK':
        raise LinkError('Malformed STAT response')
    try:
        numbers = [float(value) for value in fields[1:5]]
        if not all(math.isfinite(value) for value in numbers):
            raise ValueError('nonfinite position')
        if not all(re.fullmatch(r'-?\d+', value) for value in fields[5:]):
            raise ValueError('noninteger status')
        lost_az, lost_el, mode, flags = map(int, fields[5:])
        if mode not in range(5) or not 0 <= flags <= 15:
            raise ValueError('unknown status bits')
    except ValueError as error:
        raise LinkError('Invalid STAT values: ' + str(error)) from error
    return {'position': {'az': numbers[0], 'el': numbers[1]},
            'lost_steps': (lost_az, lost_el), 'mode': mode, 'flags': flags}


def validate_diagnostics(lines, microsteps=None):
    if len(lines) != 3 or lines[-1] != 'OK DIAG':
        raise LinkError('Incomplete driver diagnostics')
    for axis, line in zip(('AZ', 'EL'), lines[:2]):
        try:
            fields = line.split()
            if fields[0] != axis:
                raise ValueError('axis order')
            values = dict(field.split('=', 1) for field in fields[1:])
            required = ('conn', 'toff', 'mres', 's2ga', 's2gb', 'ot', 'otpw', 'en_pin', 'drvst')
            data = {key: int(values[key], 0) for key in required}
        except (ValueError, KeyError, IndexError) as error:
            raise LinkError('Malformed driver diagnostics') from error
        if data['conn'] != 0 or not 1 <= data['toff'] <= 15 or data['en_pin'] != 0:
            raise LinkError(axis + ' driver is not ready')
        if any(data[key] != 0 for key in ('s2ga', 's2gb', 'ot', 'otpw')) or data['drvst'] & 0x3F:
            raise LinkError(axis + ' driver reports a short or temperature fault')
        if data['mres'] not in (16, 256) or (microsteps is not None and data['mres'] != microsteps):
            raise LinkError(axis + ' microstep setting is unexpected')


def validate_driver_status(lines):
    """Runtime health from exactly one DRV_STATUS read per driver.

    TMCStepper 0.7.3 test_connection classifies zero/all-ones as failed reads.
    The original full diagnostic's 0x3f mask covers both temperature bits and
    all four short-circuit bits. Open-load flags are allowed at standstill.
    """
    if len(lines) != 3 or lines[-1] != 'OK DSTAT':
        raise LinkError('Incomplete fast driver status')
    for axis, line in zip(('AZ', 'EL'), lines[:2]):
        match = re.fullmatch(axis + r' drvst=0x([0-9A-Fa-f]{1,8}) en_pin=([01])', line)
        if match is None:
            raise LinkError('Malformed fast driver status')
        raw = int(match.group(1), 16)
        if raw in (0, 0xFFFFFFFF) or match.group(2) != '0':
            raise LinkError(axis + ' driver is not ready')
        if raw & 0x3F:
            raise LinkError(axis + ' driver reports a short or temperature fault')


def parse_lease(lines):
    if len(lines) != 1:
        raise LinkError('Invalid LSTAT response length')
    fields = lines[0].split()
    if (len(fields) != 5 or fields[:2] != ['OK', 'LSTAT'] or
            fields[2] not in ('0', '1') or fields[3] not in ('0', '1') or
            not re.fullmatch(r'\d+', fields[4]) or not 0 <= int(fields[4]) <= 0xFFFFFFFF):
        raise LinkError('Malformed firmware lease status/reference')
    return dict(active=fields[2] == '1', tripped=fields[3] == '1', reference_us=int(fields[4]))


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise GatewayError(400, name + ' must be finite')
    return float(value)


class TrackingGateway:
    def __init__(self, link, clock=time.monotonic, sleep=time.sleep, start_watchdog=True, limits_path=None):
        self.link, self.clock, self.sleep = link, clock, sleep
        self.state_lock = threading.RLock()
        self.io_lock = threading.Lock()
        self.stop_lock = threading.Lock()
        self.generation = secrets.token_hex(24)
        self.sequence = -1
        self.initialized = False
        self.firmware_watchdog = False
        self.active = False
        self.deadline = 0.0
        self.pending_deadline = None
        self.pending_stops = 0
        self.fault = None
        self.closed = False
        self.position = None
        self.last_reason = 'not_initialized'
        self.stop_acknowledged = False
        self.stopped_step_count_confirmed = False
        self.last_status = 0.0
        self.last_diag = 0.0
        self.soft_limits = {axis: list(bounds) for axis, bounds in SOFT_LIMITS.items()}
        self.limits_path = str(limits_path) if limits_path is not None else None
        self.limits_sha256 = None
        self.limits_verified = False
        if limits_path is not None:
            try:
                raw = Path(limits_path).read_bytes()
                profile = _decode(raw)
                values = profile['limits']
                if (profile.get('schema_version') != 1 or profile.get('acknowledged') is not True or
                        set(values) != {'az_min', 'az_max', 'el_min', 'el_max'}):
                    raise ValueError('Expected an acknowledged manual limits profile')
                for axis in ('az', 'el'):
                    bounds = [_finite(values[axis+suffix], axis+suffix) for suffix in ('_min', '_max')]
                    if not bounds[0] < bounds[1] or any(abs(x)/(1.8/(256*160)) >= 2147483648 for x in bounds):
                        raise ValueError('Invalid limits range')
                    self.soft_limits[axis] = bounds
                self.limits_sha256 = hashlib.sha256(raw).hexdigest()
            except (OSError, ValueError, KeyError, TypeError, GatewayError) as error:
                raise LinkError('User limits profile is unavailable or invalid; Apply limits in manual controls: '+str(error)) from error
        self.shutdown = threading.Event()
        self.worker = None
        if start_watchdog:
            self.worker = threading.Thread(target=self._watchdog, name='det-motion-lease', daemon=True)
            self.worker.start()

    def status(self):
        with self.state_lock:
            return dict(initialized=self.initialized, connected=self.link.is_open,
                        active=self.active, stopping=self.pending_stops > 0,
                        generation=self.generation, sequence=self.sequence, fault=self.fault,
                        lease_remaining_s=max(0.0, self.deadline-self.clock()) if self.active else 0.0,
                        last_reason=self.last_reason, position=dict(self.position) if self.position else None,
                        stop_acknowledged=self.stop_acknowledged,
                        stopped_step_count_confirmed=self.stopped_step_count_confirmed,
                        max_velocity_deg_s=MAX_VELOCITY, max_lease_s=MAX_LEASE,
                        firmware_watchdog=self.firmware_watchdog, closed=self.closed,
                        limits=self.limits_status())

    def limits_status(self):
        bounds = {axis: list(values) for axis, values in self.soft_limits.items()}
        return {'az': bounds['az'], 'el': bounds['el'], 'soft_limits_deg': bounds,
                'max_velocity_deg_s': MAX_VELOCITY, 'max_acceleration_deg_s2': 75.,
                'acceleration_deg_s2': 75., 'readback_available': False,
                'acknowledged': self.limits_verified,
                'source': 'saved_acknowledged_profile' if self.limits_path else 'firmware_defaults_assumed',
                'profile_path': self.limits_path, 'profile_sha256': self.limits_sha256}

    def _check(self, generation, deadline=None, require_ready=True):
        with self.state_lock:
            if self.closed or self.fault:
                raise GatewayError(503, self.fault or 'Gateway is closed')
            if self.pending_stops or generation != self.generation:
                raise GatewayError(409, 'Session was stopped; intentional re-arm is required')
            if require_ready and not self.initialized:
                raise GatewayError(503, 'Gateway is not initialized')
            if deadline is not None and self.clock() >= deadline:
                raise GatewayError(409, 'Command expired before it could be applied')
            if self.active and self.clock() >= self.deadline:
                raise GatewayError(409, 'The previous command lease has expired')

    def _request(self, command, timeout=.15, stop=False):
        if not self.link.is_open:
            raise LinkError('Serial device is disconnected; restart only after recovery')
        return self.link.request(command, timeout, stop=stop)

    def _read_status(self, tracking=None, require_disabled=True):
        result = parse_status(self._request('STAT'))
        if result['flags'] & 4 or result['mode'] == 4 or any(result['lost_steps']):
            raise LinkError('Controller fault is latched; it will not be cleared')
        if self.limits_path:
            if result['flags'] & 8:
                self.limits_verified = False
                raise LinkError('Controller reset detected; restore manual controls and Apply user limits before automatic operation')
            # All services preserve ENC 0 and share the acknowledged profile.
            # No LIM replay or claim of firmware register readback is made.
            self.limits_verified = True
        if require_disabled and result['flags'] != 0:
            raise LinkError('Encoder-watchdog configuration changed or controller reset')
        if tracking is not None and result['mode'] != (3 if tracking else 1):
            raise LinkError('Controller mode changed outside this gateway')
        for name, value in result['position'].items():
            low, high = self.soft_limits[name]
            if not low-POSITION_TOLERANCE <= value <= high+POSITION_TOLERANCE:
                raise LinkError('Step count is outside existing firmware travel limits')
        with self.state_lock:
            self.position = result['position']
            self.last_status = self.clock()
        return result['position']

    def _stable(self, require_disabled):
        deadline = self.clock()+SETTLE_TIMEOUT
        previous, count = None, 0
        while self.clock() < deadline:
            position = self._read_status(tracking=False, require_disabled=require_disabled)
            stable = previous is not None and all(abs(position[a]-previous[a]) < .00001 for a in ('az', 'el'))
            count = count+1 if stable else 1
            if count >= 3:
                return position
            previous = position
            self.sleep(.04)
        raise LinkError('Stable step counts were not confirmed after STOP')

    def fresh_pose(self):
        """Fresh counted position, not a claim of physical output angle.

        Serial ownership wait is capped at 100 ms; STAT has the existing
        150 ms transaction bound. Age starts before STAT, conservatively.
        """
        with self.state_lock:
            generation = self.generation
        self._check(generation)
        if not self.io_lock.acquire(timeout=.1):
            raise GatewayError(503, 'Fresh position serial link is busy')
        error = None
        try:
            self._check(generation)
            with self.state_lock: tracking = self.active
            sampled_after = self.clock()
            position = self._read_status(tracking=tracking)
            self._check(generation)
            return {'position': dict(position), 'age_s': max(0., self.clock()-sampled_after),
                    'generation': generation, 'source': 'controller_step_counts_unqualified',
                    'limits': self.limits_status()}
        except GatewayError:
            raise
        except Exception as exc:
            error = exc
        finally:
            self.io_lock.release()
        self._latch(error)
        self.stop(reason='fault')
        raise GatewayError(503, str(error))

    def _stop_locked(self, require_disabled=True):
        # Caller owns io_lock; no concurrent STAT or VEL may consume this reply.
        lines = self._request('STOP', STOP_TIMEOUT, stop=True)
        if not lines or lines[-1] != 'OK STOP':
            raise LinkError('STOP was not acknowledged')
        with self.state_lock:
            self.stop_acknowledged = True
        notices = [x for x in lines[:-1] if x.startswith(('#', '!', 'ERR'))]
        if notices:
            raise LinkError('Controller notice during STOP: ' + notices[-1])
        self._stable(require_disabled)
        if self.firmware_watchdog:
            lease = parse_lease(self._request('LSTAT'))
            if lease['active'] or lease['tripped']:
                raise LinkError('Firmware lease did not clear at rest after STOP')
        with self.state_lock:
            self.stopped_step_count_confirmed = True

    def _latch(self, error):
        with self.state_lock:
            self.fault = self.fault or str(error)
            self.generation = secrets.token_hex(24)
            self.active = False
            self.pending_deadline = None
            self.last_reason = 'fault'

    def initialize(self):
        with self.state_lock:
            if self.initialized:
                return self.status()
            generation = self.generation
        try:
            with self.io_lock:
                self._check(generation, require_ready=False)
                if self._request('CAP') != ['OK CAP VEL_LEASE_V1']:
                    raise LinkError('Launch firmware with VEL_LEASE_V1 is required; legacy VEL is forbidden')
                with self.state_lock:
                    self.firmware_watchdog = True
                if self._request('PING') != ['OK PONG']:
                    raise LinkError('PING was not acknowledged')
                self._read_status(require_disabled=False)
                validate_diagnostics(self._request('DIAG'))
                self._stop_locked(require_disabled=False)
                before = dict(self.position)
                for command, expected in [('ENC 0', ['OK ENC 0']),
                                          ('MODE SLEW', ['OK MODE 16 VMAX 15.000']),
                                          ('ACC 75', ['OK ACC 75.00'])]:
                    self._check(generation, require_ready=False)
                    if self._request(command) != expected:
                        raise LinkError(command + ' was not acknowledged exactly')
                validate_diagnostics(self._request('DIAG'), microsteps=16)
                # Probe the runtime protocol before ready, so earlier leased
                # firmware without DSTAT cannot start automatic motion.
                validate_driver_status(self._request('DSTAT'))
                self._check(generation, require_ready=False)
                self._stop_locked()
                if any(abs(before[a]-self.position[a]) > POSITION_TOLERANCE for a in ('az', 'el')):
                    raise LinkError('Mode conversion shifted the reported position')
                self._check(generation, require_ready=False)
                with self.state_lock:
                    self.initialized = True
                    self.last_reason = 'ready'
                    self.last_diag = self.clock()
            return self.status()
        except GatewayError:
            raise
        except Exception as error:
            self._latch(error)
            try: self.stop(reason='fault')
            except GatewayError: pass
            raise GatewayError(503, str(error)) from error

    def velocity(self, payload, received_at=None):
        received = self.clock() if received_at is None else received_at
        keys = {'timestamp', 'valid_until', 'enabled', 'reason', 'sequence',
                'az_velocity_deg_s', 'el_velocity_deg_s', 'ttl_s', 'generation'}
        if not isinstance(payload, dict) or set(payload) != keys:
            raise GatewayError(400, 'Velocity payload has missing or unknown fields')
        if type(payload['enabled']) is not bool or not isinstance(payload['reason'], str) or len(payload['reason']) > 1000:
            raise GatewayError(400, 'Invalid enabled/reason fields')
        timestamp = _finite(payload['timestamp'], 'timestamp')
        valid_until = _finite(payload['valid_until'], 'valid_until')
        ttl = _finite(payload['ttl_s'], 'ttl_s')
        az = _finite(payload['az_velocity_deg_s'], 'az_velocity_deg_s')
        el = _finite(payload['el_velocity_deg_s'], 'el_velocity_deg_s')
        sequence, generation = payload['sequence'], payload['generation']
        if type(sequence) is not int or not 0 <= sequence <= 0xFFFFFFFF or not isinstance(generation, str):
            raise GatewayError(400, 'Invalid sequence or generation')
        if not 0 < ttl <= MAX_LEASE or valid_until <= timestamp or ttl > valid_until-timestamp:
            raise GatewayError(400, 'Invalid or expired relative command lifetime')
        if max(abs(az), abs(el)) > MAX_VELOCITY:
            raise GatewayError(400, 'Velocity exceeds the existing firmware ceiling')
        attempted = False
        try:
            self._check(generation, received+ttl)
            if not payload['enabled']:
                return self.stop(reason='disabled')
            with self.io_lock:
                self._check(generation, received+ttl)
                with self.state_lock:
                    if sequence <= self.sequence:
                        raise GatewayError(409, 'Duplicate or out-of-order command')
                    tracking = self.active
                self._read_status(tracking=tracking)
                self._check(generation, received+ttl)
                lease = parse_lease(self._request('LSTAT'))
                if lease['tripped'] or lease['active'] != tracking:
                    raise LinkError('Firmware lease changed outside the active gateway session')
                self._check(generation, received+ttl)
                with self.state_lock:
                    self.sequence = sequence
                    self.pending_deadline = received+ttl
                    self.stop_acknowledged = False
                    self.stopped_step_count_confirmed = False
                attempted = True
                remaining_ms = min(500, int((received+ttl-self.clock())*1000))
                if remaining_ms < 1:
                    raise GatewayError(409, 'Command expired before firmware lease transmission')
                # Reference was sampled on the MCU BEFORE its LSTAT response.
                # Add only the Pi lifetime remaining AFTER receiving it. The
                # MCU therefore rejects delayed serial packets in its own clock.
                reference = lease['reference_us']
                wire = 'LVEL %d %d %d %.8f %.8f' % (sequence, remaining_ms, reference, az, el)
                if self._request(wire) != ['OK LVEL %d %d %d' % (sequence, remaining_ms, reference)]:
                    raise LinkError('Velocity acknowledgement is ambiguous')
                with self.state_lock:
                    # Atomic with Stop's generation revocation. No stale ACK
                    # may set active=True after a concurrent Stop invalidates it.
                    self._check(generation, received+ttl)
                    self.active = True
                    self.deadline = received+ttl
                    self.pending_deadline = None
                    self.last_reason = payload['reason'] or 'tracking'
            return self.status()
        except GatewayError:
            with self.state_lock:
                expired = self.active and self.clock() >= self.deadline
            if attempted or expired:
                try: self.stop(reason='lease_expired' if expired else 'cancelled_or_expired')
                except GatewayError: pass
            raise
        except Exception as error:
            self._latch(error)
            try: self.stop(reason='fault')
            except GatewayError: pass
            raise GatewayError(503, str(error)) from error

    def stop(self, reason='stopped'):
        # Revoke the token BEFORE waiting for the serial lock. A stale queued
        # request cannot slip through after STOP finishes, even at a higher seq.
        with self.state_lock:
            self.generation = secrets.token_hex(24)
            self.sequence = -1
            self.active = False
            self.pending_deadline = None
            self.deadline = 0.0
            self.pending_stops += 1
            self.last_reason = 'fault' if self.fault else reason
            self.stop_acknowledged = False
            self.stopped_step_count_confirmed = False
        try:
            with self.stop_lock:
                with self.io_lock:
                    self._stop_locked(require_disabled=self.initialized)
        except Exception as error:
            self._latch(error)
            raise GatewayError(503, 'Stop attempted but not fully verified: ' + str(error)) from error
        finally:
            with self.state_lock:
                self.pending_stops -= 1
        return self.status()

    def poll(self):
        with self.state_lock:
            if self.closed or self.pending_stops or self.fault:
                return
            now = self.clock()
            expired = (self.active and now >= self.deadline) or (self.pending_deadline is not None and now >= self.pending_deadline)
            generation, active = self.generation, self.active
            # Fresh /pose clients also update last_status. They must not defer
            # the independent 0.5 s driver-health cadence indefinitely.
            due = active and (now-self.last_status >= .05 or now-self.last_diag >= .5)
        if expired:
            self.stop(reason='lease_expired')
            return
        if not due or not self.io_lock.acquire(blocking=False):
            return
        error = None
        try:
            self._check(generation)
            self._read_status(tracking=True)
            if self.clock()-self.last_diag >= .5:
                self._check(generation)
                validate_driver_status(self._request('DSTAT'))
                self.last_diag = self.clock()
        except GatewayError:
            pass
        except Exception as exc:
            error = exc
        finally:
            self.io_lock.release()
        if error is not None:
            self._latch(error)
            self.stop(reason='fault')

    def _watchdog(self):
        while not self.shutdown.wait(.01):
            try:
                self.poll()
            except GatewayError:
                pass  # The fault is visible in status; no automatic retry.
            except Exception as error:
                self._latch(error)
                try: self.stop(reason='fault')
                except GatewayError: pass

    def close(self):
        with self.state_lock:
            if self.closed:
                return
            self.closed = True
        self.shutdown.set()
        try:
            self.stop(reason='shutdown')
        finally:
            if self.worker is not None and self.worker is not threading.current_thread():
                self.worker.join(STOP_TIMEOUT+SETTLE_TIMEOUT+.5)
            with self.io_lock:
                self.link.close()


def _decode(data):
    def reject(value):
        raise ValueError('Nonfinite JSON value: ' + value)
    def unique(pairs):
        obj = {}
        for key, value in pairs:
            if key in obj:
                raise ValueError('Duplicate JSON key')
            obj[key] = value
        return obj
    return json.loads(data, parse_constant=reject, object_pairs_hook=unique)


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(.5)

    def log_message(self, *args):
        pass

    def _reply(self, code, data):
        raw = json.dumps(data, allow_nan=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(raw)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try: self.wfile.write(raw)
        except (BrokenPipeError, ConnectionResetError): pass

    def _host_ok(self):
        try:
            value = self.headers.get('Host', '')
            parsed = urlsplit('http://'+value)
            return (parsed.hostname in ('127.0.0.1', 'localhost', '::1') and
                    parsed.port is not None and 0 < parsed.port < 65536 and
                    not parsed.username and not parsed.password and not parsed.path and
                    not parsed.query and not parsed.fragment)
        except ValueError:
            return False

    def do_GET(self):
        if not self._host_ok():
            self._reply(403, {'error': 'Local Host header required'})
        elif self.path == '/status':
            self._reply(200, self.server.gateway.status())
        elif self.path == '/pose':
            try:
                self._reply(200, self.server.gateway.fresh_pose())
            except GatewayError as error:
                self._reply(error.status, {'error': str(error)})
        else:
            self._reply(404, {'error': 'Not found'})

    def do_POST(self):
        received = self.server.gateway.clock()
        if (not self._host_ok() or self.headers.get('X-DET-Motion') != '1' or
                self.headers.get('Origin') is not None or
                self.headers.get('Content-Type', '').split(';')[0].strip() != 'application/json'):
            self._reply(403, {'error': 'Local JSON request with X-DET-Motion: 1 required'})
            return
        try:
            if self.headers.get('Transfer-Encoding'):
                raise GatewayError(400, 'Chunked requests are not accepted')
            length = int(self.headers.get('Content-Length', '-1'))
            if not 0 <= length <= 8192:
                raise GatewayError(400, 'Invalid request length')
            raw = self.rfile.read(length)
            if len(raw) != length:
                raise GatewayError(400, 'Incomplete request body')
            value = _decode(raw)
            if self.path == '/velocity':
                result = self.server.gateway.velocity(value, received_at=received)
            elif self.path == '/stop':
                if value != {}:
                    raise GatewayError(400, 'Stop body must be an empty object')
                result = self.server.gateway.stop()
            else:
                raise GatewayError(404, 'Not found')
            self._reply(200, result)
        except GatewayError as error:
            self._reply(error.status, {'error': str(error), 'status': self.server.gateway.status()})
        except (ValueError, OSError) as error:
            self._reply(400, {'error': 'Malformed request: ' + str(error)})


def make_server(gateway, port=8084):
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True
    server.block_on_close = False
    server.gateway = gateway
    return server


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8084)
    parser.add_argument('--serial', default=DEVICE)
    parser.add_argument('--limits-profile', default=LIMITS_PROFILE)
    args = parser.parse_args(argv)
    import serial
    link = SerialLink(serial.Serial(args.serial, 115200, timeout=.05, write_timeout=.15, exclusive=True))
    gateway = TrackingGateway(link, limits_path=args.limits_profile)
    server = None
    stopping = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopping.set())
    try:
        gateway.initialize()
        if not stopping.is_set():
            server = make_server(gateway, args.port)
            server.timeout = .1
            print(json.dumps(gateway.status()), flush=True)
            while not stopping.is_set():
                server.handle_request()
    finally:
        if server is not None:
            server.server_close()
        gateway.close()


if __name__ == '__main__':
    main()
