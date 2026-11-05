#!/usr/bin/env python3
"""Manual DET jogs. Reported positions are open-loop motor step counts.

The existing Pico firmware uses 1.8/(256*160) degrees per fine unit. Its
STOP delays 300 ms, so its acknowledgement alone is not evidence of rest.
This service never clears faults, reconnects automatically, or changes current.
"""
import argparse
import json
import math
import os
from pathlib import Path
import struct
import tempfile
import re
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


DEVICE = os.environ.get('DET_MOTOR_DEVICE', '/dev/ttyACM0')
FINE_DEG = 1.8 / (256 * 160)
SLEW_MICROSTEPS = 16
POSITION_TOLERANCE = 16 * FINE_DEG + .00001
MAX_VELOCITY = 15.0
ACCELERATION = 75.0
STOP_SECONDS = .85
STOP_SETTLE_SECONDS = 1.5
HOLD_LEASE_SECONDS = .75
# Existing firmware defaults, also used as host fences while a saved profile
# is unconfirmed. Manual initialization never sends LIM to apply or widen them.
SOFT_LIMITS = {'az': (-180.0, 180.0), 'el': (-10.0, 95.0)}
LIMITS_PROFILE = str(Path.home() / 'gimbal' / 'motion_limits.json')


class LinkError(Exception):
    pass


class MotionError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


class SerialLink:
    """One write per transaction, bounded replies, no flushing or retrying."""
    def __init__(self, port):
        self.port = port

    @property
    def is_open(self):
        return self.port.is_open

    def close(self):
        self.port.close()

    def request(self, command, timeout, stop=False):
        terminators = {'PING': 'OK PONG', 'DIAG': 'OK DIAG', 'STOP': 'OK STOP',
                       'ENC 0': 'OK ENC 0', 'MODE SLEW': 'OK MODE ', 'ACC 75': 'OK ACC 75.00'}
        if command == 'STAT':
            terminal = 'OK '
        elif re.fullmatch(r'MOVR -?\d+\.\d+ -?\d+\.\d+', command):
            terminal = 'OK MOV'
        elif re.fullmatch(r'LIM -?\d+\.\d+ -?\d+\.\d+ -?\d+\.\d+ -?\d+\.\d+', command):
            terminal = 'OK LIM'
        elif re.fullmatch(r'VEL -?\d+\.\d+ -?\d+\.\d+', command):
            terminal = 'OK VEL'
        elif command in terminators:
            terminal = terminators[command]
        else:
            raise LinkError('Command is outside the manual gateway protocol')
        deadline = time.monotonic() + timeout
        payload = (command + '\n').encode('ascii')
        lines, pending = [], b''
        try:
            if self.port.write(payload) != len(payload):
                raise LinkError('Incomplete serial write; outcome is unknown')
            while time.monotonic() < deadline:
                self.port.timeout = min(.05, max(.001, deadline - time.monotonic()))
                chunk = self.port.read_until(b'\n', size=512 - len(pending))
                pending += chunk
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
                if command == 'DIAG' and line.startswith(('AZ ', 'EL ')):
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



def validate_limits(payload):
    keys = ('az_min', 'az_max', 'el_min', 'el_max')
    if not isinstance(payload, dict) or set(payload) != set(keys):
        raise MotionError(400, 'Limits need exactly az_min, az_max, el_min, el_max')
    def f32(value): return struct.unpack('<f', struct.pack('<f', value))[0]
    factor = f32(1.0 / f32(FINE_DEG))
    values = {}
    for key in keys:
        value = payload[key]
        try:
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError()
            # Match the eight decimal places sent to LIM and float storage.
            value = f32(float('%.8f' % value))
            fine = f32(value * factor)
            if not math.isfinite(value) or not -2147483648 <= fine < 2147483648:
                raise ValueError()
        except (ValueError, OverflowError, struct.error):
            raise MotionError(400, key+' must be finite and fit the firmware position counter')
        values[key] = value
    result = {axis: [values[axis+'_min'], values[axis+'_max']] for axis in ('az', 'el')}
    if any(bounds[0] >= bounds[1] for bounds in result.values()):
        raise MotionError(400, 'Each minimum must be smaller than its maximum at firmware precision')
    return result


def quantize_jog(position, axis, delta, limits=None):
    """Preserve the current counter residue when choosing whole motor pulses."""
    counts = {name: int(round(value / FINE_DEG)) for name, value in position.items()}
    low, high = (SOFT_LIMITS if limits is None else limits)[axis]
    requested_target = counts[axis] * FINE_DEG + delta
    if not low <= requested_target <= high:
        raise MotionError(400, 'Jog would exceed the known firmware limits')
    units_per_pulse = 256 // SLEW_MICROSTEPS
    pulses = int(round(delta / (units_per_pulse * FINE_DEG)))
    # An endpoint exactly on a firmware fence can stall one pulse inside it
    # because the planner subtracts one pulse of lookahead from the distance.
    minimum = math.ceil((low / FINE_DEG - counts[axis] + 1e-6) / units_per_pulse)
    maximum = math.floor((high / FINE_DEG - counts[axis] - 1e-6) / units_per_pulse)
    if minimum > maximum:
        raise MotionError(400, 'Limits contain no reachable motor pulse position')
    pulses = max(minimum, min(maximum, pulses))
    counts[axis] += pulses * units_per_pulse
    return pulses * units_per_pulse * FINE_DEG, {name: value * FINE_DEG for name, value in counts.items()}


class MotionController:
    def __init__(self, link, clock=time.monotonic, sleep=time.sleep, limits_path=None):
        self.link, self.clock, self.sleep = link, clock, sleep
        self.io_lock = threading.Lock()
        self.operation_lock = threading.Lock()
        self.state_lock = threading.RLock()
        self.stop_lock = threading.Lock()
        self.abort = threading.Event()
        self.connected = False
        self.initialized = False
        self.blocked = False
        self.busy = False
        self.stopping = False
        self.position = None
        self.rest_position = None
        self.updated = None
        self.last_error = None
        self.hold = None
        self.hold_thread = None
        self.used_hold_ids = set()
        self.last_hold_reason = None
        self.soft_limits = {axis: list(bounds) for axis, bounds in SOFT_LIMITS.items()}
        self.limits_path = Path(limits_path) if limits_path else None
        self.limits_confirmed = False
        self.limits_confirmation_required = True
        self.limits_source = 'firmware_defaults_assumed'
        self.limits_detail = 'Limits have not been applied; manual jogs use the existing firmware default fences'
        self.limits_saved = False
        self._saved_profile_pending = False
        if self.limits_path and self.limits_path.exists():
            try:
                profile = json.loads(self.limits_path.read_text())
                if profile.get('schema_version') != 1 or not isinstance(profile.get('acknowledged'), bool):
                    raise ValueError('Unknown limits profile format')
                self.soft_limits = validate_limits(profile['limits'])
                self.limits_saved = True
                if profile['acknowledged']:
                    self.limits_source = 'saved_profile'
                    self._saved_profile_pending = True
                    self._limits_pending('Checking saved limits against controller reset status')
                else:
                    self.limits_source = 'saved_draft'
                    self._limits_pending('Saved limits await explicit Apply; manual jogs use the existing firmware default fences')
            except (OSError, ValueError, KeyError, TypeError, MotionError) as error:
                self._limits_pending('Saved limits could not be loaded; enter and Apply limits: '+str(error))


    def _limits_pending(self, detail):
        self.limits_confirmed = False
        self.limits_confirmation_required = True
        self.limits_detail = detail

    def _manual_limits(self):
        # Draft values remain visible for Apply but cannot restrict manual jogs.
        # The MCU retains its existing fences; these defaults are host checks,
        # not a claim of firmware readback or permission to change its limits.
        return self.soft_limits if self.limits_confirmed else SOFT_LIMITS

    def _save_limits(self, payload, position, acknowledged=True):
        if self.limits_path is None:
            return
        profile = {'schema_version': 1, 'limits': payload, 'acknowledged': acknowledged,
                   'readback_available': False, 'position_reference': dict(position),
                   ('acknowledged_at_unix_s' if acknowledged else 'unconfirmed_at_unix_s'): time.time(),
                   'confirmation_policy': 'retain_acknowledged_profile_if_encoder_disabled_flag_is_preserved; explicit_apply_after_reset'}
        self.limits_path.parent.mkdir(parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(prefix='.motion-limits-', dir=str(self.limits_path.parent))
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(profile, stream, allow_nan=False, indent=2)
                stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
            os.replace(name, self.limits_path)
            directory_fd = os.open(self.limits_path.parent, os.O_RDONLY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if os.path.exists(name): os.unlink(name)
        self.limits_saved = True

    def set_limits(self, payload):
        proposed = validate_limits(payload)
        if not self.operation_lock.acquire(blocking=False):
            raise MotionError(409, 'Wait for the current movement to finish before applying limits')
        claimed_busy = False
        try:
            with self.state_lock:
                if self.busy or self.hold or self.stopping:
                    raise MotionError(409, 'Release movement controls before applying limits')
                if self.blocked:
                    raise MotionError(503, self.last_error or 'Controller fault requires recovery')
                self.abort.clear()
                self.busy = True
                claimed_busy = True
            # Overrides only stale host bounds. STAT/driver/EST checks remain.
            # Validate position BEFORE initialization can issue its idle STOPs.
            self._read_status(require_disabled=False, limits_override=proposed)
            if not self.initialized:
                self._initialize(limits_override=proposed)
            else:
                validate_diagnostics(self._request('DIAG'), microsteps=SLEW_MICROSTEPS)
            position = self._stable(self.clock()+.6, limits_override=proposed)
            wire_values = [proposed['az'][0], proposed['az'][1], proposed['el'][0], proposed['el'][1]]
            command = 'LIM '+' '.join('%.8f' % value for value in wire_values)
            if self._request(command) != ['OK LIM']:
                raise LinkError('Limits acknowledgement is ambiguous; Apply again after recovery')
            canonical = dict(zip(('az_min', 'az_max', 'el_min', 'el_max'), wire_values))
            with self.state_lock:
                self.soft_limits = proposed
                self.limits_confirmed = True
                self.limits_confirmation_required = False
                self.limits_source = 'user_applied'
                self.limits_detail = None
                self.limits_saved = False
                self._saved_profile_pending = False
                self.rest_position = dict(position)
            try:
                self._save_limits(canonical, position)
            except OSError as error:
                self.limits_detail = 'Controller acknowledged limits, but saving the profile failed: '+str(error)
                raise MotionError(503, self.limits_detail) from error
            result = self.status()
            result['busy'] = False
            return result
        except LinkError as error:
            self._latch(error)
            raise MotionError(503, self.last_error) from error
        finally:
            if claimed_busy:
                with self.state_lock: self.busy = False
            self.operation_lock.release()

    def _latch(self, message):
        with self.state_lock:
            self.blocked = True
            self.last_error = str(message)

    def _cancel_check(self):
        with self.state_lock:
            if self.hold and self.clock() >= self.hold['deadline']:
                self.hold['reason'] = self.hold['reason'] or 'lease_expired'
                self.abort.set()
        if self.abort.is_set():
            raise MotionError(409, 'Jog stopped by user')

    def _request(self, command, timeout=.25, cancellable=True, stop=False):
        with self.io_lock:
            if cancellable:
                self._cancel_check()
                with self.state_lock:
                    if self.blocked:
                        raise LinkError(self.last_error or 'Manual recovery is required')
            if self.link is None or not self.link.is_open:
                raise LinkError('Controller is disconnected; restart service after recovery')
            try:
                return self.link.request(command, timeout, stop=stop)
            except LinkError:
                with self.state_lock:
                    self.connected = False
                raise

    def _read_status(self, timeout=.25, cancellable=True, require_disabled=None, allow_track=False, limits_override=None):
        result = parse_status(self._request('STAT', timeout, cancellable))
        # Preserve reset detection for an already initialized operation. A fresh
        # gateway may initialize, but an unexpected reset must still latch.
        disabled = self.initialized if require_disabled is None else require_disabled
        with self.state_lock:
            self.connected = True
            self.position = dict(result['position'])
            self.updated = self.clock()
        if result['flags'] & 4 or result['mode'] == 4 or any(result['lost_steps']):
            raise LinkError('Controller fault is asserted; it will not be cleared automatically')
        if self._saved_profile_pending:
            self._saved_profile_pending = False
            if not result['flags'] & 8:
                # This service family exclusively owns serial and preserves
                # ENC 0 across handoff. Firmware reset restores ENC 1 (bit8).
                # This is acknowledged-profile continuity, not register readback.
                self.limits_confirmed = True
                self.limits_confirmation_required = False
                self.limits_detail = None
        owned_track = allow_track and self.hold is not None and self.hold['running']
        if result['mode'] == 3 and not owned_track:
            raise LinkError('Controller is in tracking mode; stop it before manual recovery')
        if owned_track and result['mode'] != 3:
            raise LinkError('Continuous movement ended outside the gateway')
        if result['flags'] & 8 and (self.limits_confirmed or self.limits_confirmation_required):
            self._limits_pending('Controller reset or encoder configuration changed; Apply limits again in the current coordinate system')
            self.initialized = False
            self.rest_position = None
        if disabled and result['flags'] != 0:
            raise LinkError('Controller configuration changed or reset; manual recovery required')
        for axis, value in result['position'].items():
            low, high = (self.soft_limits if limits_override is None else limits_override)[axis]
            if limits_override is not None and not low <= value <= high:
                raise MotionError(400, 'Current '+axis+' position must be inside the proposed limits')
            if limits_override is None and self.limits_confirmed and not low - POSITION_TOLERANCE <= value <= high + POSITION_TOLERANCE:
                self._limits_pending('Current position is outside the saved bounds; enter and Apply limits containing it')
        return result['position']

    def _stable(self, deadline, target=None, cancellable=True, require_disabled=None, limits_override=None):
        previous, count = None, 0
        while self.clock() < deadline:
            if cancellable:
                self._cancel_check()
            position = self._read_status(min(.2, max(.001, deadline - self.clock())),
                                         cancellable, require_disabled, limits_override=limits_override)
            stable = previous is not None and all(abs(position[a] - previous[a]) < .00001 for a in ('az', 'el'))
            at_target = target is None or all(abs(position[a] - target[a]) <= POSITION_TOLERANCE for a in ('az', 'el'))
            count = count + 1 if stable and at_target else (1 if at_target else 0)
            if count >= 3:
                return position
            previous = position
            self.sleep(min(.04, max(0, deadline - self.clock())))
        raise LinkError('Stable target step counts were not confirmed before the deadline')

    def _stop_command(self):
        lines = self._request('STOP', STOP_SECONDS, cancellable=False, stop=True)
        if not lines or lines[-1] != 'OK STOP':
            raise LinkError('Stop was not acknowledged')
        notices = [line for line in lines[:-1] if line.startswith(('#', '!', 'ERR'))]
        if notices:
            self._latch('Controller notice during stop: ' + notices[-1])

    def _initialize(self, limits_override=None):
        if self._request('PING') != ['OK PONG']:
            raise LinkError('Unexpected PING response')
        self._read_status(limits_override=limits_override)
        validate_diagnostics(self._request('DIAG'))
        self._stop_command()
        position = self._stable(self.clock() + STOP_SETTLE_SECONDS, limits_override=limits_override)
        if not self.limits_confirmed:
            # ENC 0 clears the firmware reset marker. Invalidate durable profile
            # continuity BEFORE that write so another gateway cannot mistake an
            # unapplied draft for acknowledged limits after a service restart.
            payload = {axis+'_'+bound: self.soft_limits[axis][index]
                       for axis in ('az', 'el') for index, bound in enumerate(('min', 'max'))}
            try:
                self._save_limits(payload, position, acknowledged=False)
            except OSError as error:
                raise LinkError('Could not save unconfirmed limits before manual initialization: '+str(error)) from error
            self._saved_profile_pending = False
        if self._request('ENC 0') != ['OK ENC 0']:
            raise LinkError('Old encoder feedback could not be disabled')
        if self._request('MODE SLEW') != ['OK MODE 16 VMAX 15.000']:
            raise LinkError('Slew mode or the controller speed ceiling was not confirmed')
        if self._request('ACC 75') != ['OK ACC 75.00']:
            raise LinkError('Controller default acceleration was not confirmed')
        validate_diagnostics(self._request('DIAG'), microsteps=SLEW_MICROSTEPS)
        # setUstep quantizes position but retains the old fine-unit target.
        # Retarget to the actual coarse position before waiting for rest.
        self._stop_command()
        settled = self._stable(self.clock() + STOP_SETTLE_SECONDS, require_disabled=True, limits_override=limits_override)
        if any(abs(settled[axis] - position[axis]) > POSITION_TOLERANCE for axis in ('az', 'el')):
            raise LinkError('Position shifted by more than one coarse step during mode change')
        position = settled
        with self.state_lock:
            self.initialized = True
            self.rest_position = dict(position)

    def status(self, refresh=False):
        if refresh and self.operation_lock.acquire(blocking=False):
            try:
                if not self.stopping:
                    self._read_status(cancellable=False)
            except LinkError as error:
                self._latch(error)
            finally:
                self.operation_lock.release()
        with self.state_lock:
            return {'connected': self.connected, 'initialized': self.initialized,
                    'available': self.connected and not self.blocked,
                    'busy': self.busy or self.stopping, 'recovery_required': self.blocked,
                    'position': dict(self.position) if self.position else None,
                    'limits': {'soft_limits_deg': {a: list(b) for a, b in self.soft_limits.items()},
                               'manual_soft_limits_deg': {a: list(b) for a, b in self._manual_limits().items()},
                               'confirmation_required': self.limits_confirmation_required,
                               'acknowledged': self.limits_confirmed, 'readback_available': False,
                               'profile_saved': self.limits_saved, 'source': self.limits_source,
                               'detail': self.limits_detail,
                               'profile_path': str(self.limits_path) if self.limits_path else None,
                               'acceleration_deg_s2': ACCELERATION,
                               'max_velocity_deg_s': MAX_VELOCITY,
                               'microsteps': SLEW_MICROSTEPS},
                    'encoder_feedback': False, 'open_loop': True,
                    'physical_motion_verified': False, 'position_source': 'controller_step_counts',
                    'position_age_seconds': None if self.updated is None else max(0, self.clock() - self.updated),
                    'hold_active': self.hold is not None,
                    'hold_started': bool(self.hold and self.hold['running']),
                    'hold_id': self.hold['id'] if self.hold else None,
                    'hold_axis': self.hold['axis'] if self.hold else None,
                    'hold_velocity_deg_s': self.hold['velocity'] if self.hold else 0,
                    'hold_state': ('stopping' if self.abort.is_set() else 'running' if self.hold['running'] else 'initializing') if self.hold else 'idle',
                    'lease_seconds': HOLD_LEASE_SECONDS,
                    'hold_lease_remaining_seconds': max(0, self.hold['deadline'] - self.clock()) if self.hold else 0,
                    'last_hold_reason': self.last_hold_reason,
                    'limits_confirmation_required': self.limits_confirmation_required,
                    'last_error': self.last_error,
                    'caveat': 'Manual open-loop jogs. Encoder feedback is unavailable; reported positions are motor step counts.'}

    def jog(self, axis, delta):
        if axis not in ('az', 'el') or isinstance(delta, bool) or not isinstance(delta, (int, float)):
            raise MotionError(400, 'Use axis az or el and a numeric delta')
        try:
            delta = float(delta)
        except (OverflowError, ValueError):
            raise MotionError(400, 'Delta must be a finite nonzero number')
        if delta == 0 or not math.isfinite(delta):
            raise MotionError(400, 'Delta must be a finite nonzero number')
        if not self.operation_lock.acquire(blocking=False):
            raise MotionError(409, 'Another operation is in progress; jogs are not queued')
        operation = {'event': 'jog', 'axis': axis, 'requested_delta_deg': delta}
        try:
            with self.state_lock:
                if self.blocked:
                    raise MotionError(503, self.last_error or 'Manual recovery is required')
                if self.stopping:
                    raise MotionError(409, 'Stop is in progress')
                self.abort.clear()
                self.busy = True
            if not self.initialized:
                self._initialize()
            else:
                validate_diagnostics(self._request('DIAG'), microsteps=SLEW_MICROSTEPS)
            position = self._stable(self.clock() + .6)
            if self.rest_position and any(abs(position[a] - self.rest_position[a]) > POSITION_TOLERANCE for a in ('az', 'el')):
                raise LinkError('Position changed outside a completed gateway operation')
            effective_delta, target = quantize_jog(position, axis, delta, self._manual_limits())
            operation.update(commanded_delta_deg=effective_delta, quantized_target=target)
            settled = position
            if effective_delta != 0:
                daz, dele = (effective_delta, 0.0) if axis == 'az' else (0.0, effective_delta)
                command = f'MOVR {daz:.8f} {dele:.8f}'
                operation['command'] = command
                # Use the normal firmware rates, allowing both ramps and rest.
                deadline = self.clock() + abs(effective_delta) / MAX_VELOCITY + 2 * MAX_VELOCITY / ACCELERATION + 2.0
                reply = self._request(command)
                if reply != ['OK MOV']:
                    raise LinkError('Move acknowledgement is ambiguous')
                settled = self._stable(deadline, target=target)
            self._cancel_check()
            with self.state_lock:
                self.rest_position = dict(settled)
                self.last_error = None
            result = self.status()
            result['busy'] = False
            result['step_count_target_confirmed'] = True
            result.update(requested_delta_deg=delta, commanded_delta_deg=effective_delta,
                          quantized_target=target, motion_command_sent=effective_delta != 0)
            operation['outcome'] = 'step_count_target_confirmed' if effective_delta != 0 else 'below_one_motor_step'
            return result
        except LinkError as error:
            self._latch(error)
            try:
                self._stop_command()
                position = self._stable(self.clock() + STOP_SETTLE_SECONDS, cancellable=False)
                with self.state_lock:
                    self.rest_position = dict(position)
            except LinkError as stop_error:
                self._latch(str(error) + '; stop failed: ' + str(stop_error))
            raise MotionError(503, self.last_error) from error
        except MotionError as error:
            operation['outcome'] = str(error)
            raise
        finally:
            with self.state_lock:
                self.busy = False
                operation['final_position'] = dict(self.position) if self.position else None
                operation.setdefault('outcome', self.last_error or 'jog not completed')
            self.operation_lock.release()
            print(json.dumps(operation, allow_nan=False), flush=True)

    @staticmethod
    def _validate_hold_id(hold_id):
        if not isinstance(hold_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', hold_id):
            raise MotionError(400, 'A unique hold_id is required')

    def hold_start(self, hold_id, axis, velocity):
        self._validate_hold_id(hold_id)
        if axis not in ('az', 'el') or isinstance(velocity, bool) or not isinstance(velocity, (int, float)):
            raise MotionError(400, 'Use axis az or el and a numeric velocity')
        try:
            velocity = float(velocity)
        except (ValueError, OverflowError):
            raise MotionError(400, 'Velocity must be finite, nonzero, and within the controller limit')
        if not math.isfinite(velocity) or velocity == 0 or abs(velocity) > MAX_VELOCITY:
            raise MotionError(400, 'Velocity must be finite, nonzero, and within the controller limit')
        with self.state_lock:
            if hold_id in self.used_hold_ids:
                raise MotionError(410, 'This hold has ended or its ID was already used')
        if not self.operation_lock.acquire(blocking=False):
            raise MotionError(409, 'Another operation is in progress')
        try:
            with self.state_lock:
                if hold_id in self.used_hold_ids:
                    raise MotionError(410, 'This hold was cancelled before it started')
                if self.blocked:
                    raise MotionError(503, self.last_error or 'Manual recovery is required')
                if self.stopping:
                    raise MotionError(409, 'Stop is in progress')
                self.used_hold_ids.add(hold_id)
                self.hold = {'id': hold_id, 'axis': axis, 'velocity': velocity,
                             'deadline': self.clock() + HOLD_LEASE_SECONDS,
                             'running': False, 'reason': None}
                self.abort.clear()
                self.busy = True
                self.hold_thread = threading.Thread(target=self._hold_worker, args=(self.hold,),
                                                    name='det-manual-hold', daemon=True)
                self.hold_thread.start()
        except Exception as error:
            with self.state_lock:
                if self.hold and self.hold['id'] == hold_id:
                    self.hold = None
                    self.busy = False
            self.operation_lock.release()
            if isinstance(error, MotionError):
                raise
            self._latch('Could not start continuous movement: ' + str(error))
            raise MotionError(503, self.last_error) from error
        return self.status()

    def hold_renew(self, hold_id):
        self._validate_hold_id(hold_id)
        with self.state_lock:
            if not self.hold or self.hold['id'] != hold_id:
                raise MotionError(410, 'This hold is no longer active')
            if self.abort.is_set() or self.blocked or self.clock() >= self.hold['deadline']:
                self.hold['reason'] = self.hold['reason'] or 'lease_expired'
                self.abort.set()
                raise MotionError(410, 'This hold has ended; renewing cannot restart it')
            self.hold['deadline'] = self.clock() + HOLD_LEASE_SECONDS
            return self.status()

    def _hold_worker(self, hold):
        operation = {'event': 'hold', 'hold_id': hold['id'], 'axis': hold['axis'],
                     'requested_velocity_deg_s': hold['velocity']}
        try:
            if not self.initialized:
                self._initialize()
            else:
                validate_diagnostics(self._request('DIAG'), microsteps=SLEW_MICROSTEPS)
            position = self._stable(self.clock() + .6)
            if self.rest_position and any(abs(position[a] - self.rest_position[a]) > POSITION_TOLERANCE for a in ('az', 'el')):
                raise LinkError('Position changed outside a completed gateway operation')
            axis, velocity = hold['axis'], hold['velocity']
            low, high = self._manual_limits()[axis]
            at_limit = lambda p: p[axis] >= high - POSITION_TOLERANCE if velocity > 0 else p[axis] <= low + POSITION_TOLERANCE
            if not at_limit(position):
                az, el = (velocity, 0.0) if axis == 'az' else (0.0, velocity)
                command = f'VEL {az:.8f} {el:.8f}'
                operation['command'] = command
                if self._request(command) != ['OK VEL']:
                    raise LinkError('Velocity acknowledgement is ambiguous')
                with self.state_lock:
                    hold['running'] = True
                while True:
                    self._cancel_check()
                    position = self._read_status(timeout=.2, allow_track=True)
                    if at_limit(position):
                        break
                    self.sleep(.04)
            with self.state_lock:
                hold['reason'] = hold['reason'] or 'travel_limit'
        except MotionError:
            with self.state_lock:
                hold['reason'] = hold['reason'] or 'stopped'
        except LinkError as error:
            self._latch(error)
            with self.state_lock:
                hold['reason'] = 'fault'
        except Exception as error:
            self._latch('Continuous movement worker failed: ' + str(error))
            with self.state_lock:
                hold['reason'] = 'fault'
        finally:
            try:
                self.stop(hold_id=hold['id'])
            except MotionError:
                pass
            with self.state_lock:
                self.last_hold_reason = hold['reason'] or 'stopped'
                operation.update(outcome=self.last_hold_reason,
                                 final_position=dict(self.position) if self.position else None,
                                 error=self.last_error)
                self.hold = None
                self.busy = False
            self.operation_lock.release()
            print(json.dumps(operation, allow_nan=False), flush=True)

    def stop(self, hold_id=None, global_stop=False):
        if hold_id is not None:
            self._validate_hold_id(hold_id)
        with self.state_lock:
            if hold_id is not None:
                self.used_hold_ids.add(hold_id)
                if not global_stop and (not self.hold or self.hold['id'] != hold_id):
                    result = self.status()
                    result.update(stop_requested=False, cancelled_hold_id=hold_id)
                    return result
            if self.hold:
                self.hold['reason'] = self.hold['reason'] or 'stopped'
            self.abort.set()
            if not self.stop_lock.acquire(blocking=False):
                result = self.status()
                result['stop_requested'] = True
                return result
            self.stopping = True
        try:
            self._stop_command()
            settled = False
            try:
                position = self._stable(self.clock() + STOP_SETTLE_SECONDS, cancellable=False)
                with self.state_lock:
                    self.rest_position = dict(position)
                settled = True
            except LinkError as error:
                self._latch(error)
            result = self.status()
            result.update(stop_acknowledged=True, stopped_step_count_confirmed=settled)
            return result
        except LinkError as error:
            self._latch(error)
            raise MotionError(503, 'Stop was attempted but could not be verified: ' + str(error)) from error
        finally:
            with self.state_lock:
                self.stopping = False
            self.stop_lock.release()

    def close(self):
        # Existing HTTP handlers can outlive the listening loop. Refuse every
        # subsequent jog before stopping, including the gap before port close.
        self._latch('Motion gateway is shutting down')
        if self.initialized or self.busy:
            try:
                self.stop()
            except MotionError:
                pass
        worker = self.hold_thread
        if worker and worker.ident is not None and worker is not threading.current_thread():
            worker.join(STOP_SECONDS + STOP_SETTLE_SECONDS + .5)
        with self.io_lock:
            if self.link:
                self.link.close()


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup()
        self.connection.settimeout(1.0)

    def _json(self, code, value):
        body = json.dumps(value, allow_nan=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path != '/motion':
            return self._json(404, {'error': 'Not found'})
        self._json(200, self.server.controller.status(refresh=True))

    def do_POST(self):
        try:
            if self.headers.get('X-DET-Motion') != '1' or self.headers.get('Origin') not in (None, 'http://127.0.0.1:18182'):
                raise MotionError(403, 'Motion request origin or header is invalid')
            if self.headers.get('Content-Type', '').split(';')[0].strip().lower() != 'application/json':
                raise MotionError(415, 'Content-Type must be application/json')
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 512:
                    raise ValueError()
                raw = self.rfile.read(length)
                if len(raw) != length:
                    raise ValueError()
                payload = json.loads(raw)
                if not isinstance(payload, dict):
                    raise ValueError()
            except (ValueError, UnicodeError, OSError):
                raise MotionError(400, 'Expected a small JSON object')
            if self.path == '/motion/jog':
                if set(payload) != {'axis', 'delta'}:
                    raise MotionError(400, 'Jog needs exactly axis and delta')
                result = self.server.controller.jog(payload['axis'], payload['delta'])
            elif self.path == '/motion/limits':
                result = self.server.controller.set_limits(payload)
            elif self.path == '/motion/hold/start':
                if set(payload) != {'hold_id', 'axis', 'velocity'}:
                    raise MotionError(400, 'Hold start needs exactly hold_id, axis, and velocity')
                result = self.server.controller.hold_start(payload['hold_id'], payload['axis'], payload['velocity'])
                return self._json(202, result)
            elif self.path == '/motion/hold/renew':
                if set(payload) != {'hold_id'}:
                    raise MotionError(400, 'Hold renew needs exactly hold_id')
                result = self.server.controller.hold_renew(payload['hold_id'])
            elif self.path == '/motion/stop':
                if set(payload) - {'hold_id', 'global'} or ('global' in payload and not isinstance(payload['global'], bool)):
                    raise MotionError(400, 'Stop takes an optional hold_id and global boolean')
                if 'hold_id' in payload:
                    self.server.controller._validate_hold_id(payload['hold_id'])
                result = self.server.controller.stop(hold_id=payload.get('hold_id'), global_stop=payload.get('global', False))
            else:
                raise MotionError(404, 'Not found')
            self._json(200, result)
        except MotionError as error:
            self._json(error.status, {'error': str(error), **self.server.controller.status()})

    def do_OPTIONS(self):
        self._json(403, {'error': 'Cross-origin requests are not enabled'})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8083)
    parser.add_argument('--limits-profile', default=LIMITS_PROFILE)
    args = parser.parse_args()
    link = None
    open_error = None
    try:
        import serial
        link = SerialLink(serial.Serial(DEVICE, 115200, timeout=.05, write_timeout=.2, exclusive=True))
    except Exception as error:
        open_error = 'Controller connection failed: ' + str(error)
    controller = MotionController(link, limits_path=args.limits_profile)
    if open_error:
        controller._latch(open_error)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    server.daemon_threads = True
    server.block_on_close = False
    server.controller = controller
    server.timeout = .2
    stopping = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopping.set())
    try:
        while not stopping.is_set():
            server.handle_request()
    finally:
        controller.close()
        server.server_close()


if __name__ == '__main__':
    main()
