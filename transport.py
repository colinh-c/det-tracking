"""Mac transport for existing Pi camera/encoder HTTP and leased motor HTTP.

Camera timestamps are conservative publication-time estimates, NOT exposure
timestamps: Mac receive time - reported Pi age - request round-trip time.
No Pi wall/monotonic clock is subtracted from a Mac clock. Encoder read_at is
used only as an opaque sample identity to prevent repeated samples aging anew.
"""
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, replace
import json
import math
import threading
import time
import uuid
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from urllib.parse import urlsplit

import cv2
import numpy as np

from det_types import CameraModel, Command, Frame, Pose, angle_error


class TransportError(RuntimeError):
    pass


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(name + ' must be a finite number')
    return float(value)


def _base_url(value):
    parsed = urlsplit(value)
    if parsed.scheme not in ('http', 'https') or not parsed.hostname or parsed.username or parsed.password:
        raise ValueError('Use an HTTP base URL without embedded credentials')
    if parsed.query or parsed.fragment or parsed.path not in ('', '/'):
        raise ValueError('Base URL must not contain a path, query, or fragment')
    return value.rstrip('/')


def _json(raw):
    def constant(value):
        raise ValueError('Nonfinite JSON value: ' + value)
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError('Duplicate JSON field: ' + key)
            result[key] = value
        return result
    return json.loads(raw, parse_constant=constant, object_pairs_hook=unique)


def _http(url, timeout, data=None, limit=65536):
    """One bounded request, never retried. Returns bytes, headers, local times."""
    headers = {'Accept': 'application/json, image/jpeg', 'Cache-Control': 'no-cache'}
    if data is not None:
        headers.update({'Content-Type': 'application/json', 'X-DET-Motion': '1'})
        data = json.dumps(data, allow_nan=False, separators=(',', ':')).encode()
    started = time.monotonic()
    try:
        with urlopen(Request(url, data=data, headers=headers), timeout=timeout) as response:
            raw = response.read(limit + 1)
            if len(raw) > limit:
                raise TransportError('HTTP response is oversized')
            result_headers = dict(response.headers.items())
        return raw, {k.lower(): v for k, v in result_headers.items()}, started, time.monotonic()
    except HTTPError as error:
        # urlopen raises before entering its context manager for 4xx/5xx.
        # Close that response explicitly during repeated unavailable-camera polls.
        error.close()
        raise TransportError('HTTP request failed: ' + str(error)) from error
    except (OSError, ValueError) as error:
        raise TransportError('HTTP request failed: ' + str(error)) from error


def align_angle(samples, timestamp, circular=False, max_gap_s=.25, max_extrapolation_s=.05):
    """Interpolate or briefly extrapolate actual encoder samples, never commands.

    Bounds cover sample spacing and distance outside the observed time interval.
    They do not claim that cache publication time equals optical exposure time.
    """
    if not samples:
        raise ValueError('No encoder history')
    for stamp, angle in samples:
        if abs(timestamp-stamp) <= 1e-9:
            return angle % 360 if circular else angle
    if len(samples) < 2:
        raise ValueError('Two encoder samples are required for time alignment')
    if timestamp < samples[0][0]:
        pair = samples[:2]
        outside = samples[0][0]-timestamp
    elif timestamp > samples[-1][0]:
        pair = samples[-2:]
        outside = timestamp-samples[-1][0]
    else:
        pair = next((samples[i:i+2] for i in range(len(samples)-1)
                     if samples[i][0] <= timestamp <= samples[i+1][0]), None)
        outside = 0.0
    if pair is None or outside > max_extrapolation_s+1e-9:
        raise ValueError('Frame is outside the bounded encoder extrapolation horizon')
    (t0, a0), (t1, a1) = pair
    if not 0 < t1-t0 <= max_gap_s+1e-9:
        raise ValueError('Encoder samples are too far apart for alignment')
    delta = angle_error(a1, a0) if circular else a1-a0
    result = a0 + delta*(timestamp-t0)/(t1-t0)
    return result % 360 if circular else result


class PiSource:
    """Read both JPEGs and cached AS5600 readings concurrently.

    encoder_config: calibrated (bool), az/el each containing bus,
    zero_deg, sign (+/-1), offset_deg. Optional max_age_s (.5),
    frame_max_age_s (.5), max_alignment_gap_s (.25),
    max_pose_extrapolation_s (.05), and camera_indices. An axes.az/el wrapper
    is also accepted for the original standalone transport configuration.
    Versioned gateways expose a process UUID on every HTTP response. A new UUID
    corroborated by encoders and at least one camera resets frame counters and
    encoder histories; every successful batch reply must agree, and an
    old generation cannot return. At most 64 retired generations are retained.
    Explicit allow_legacy_gateway=True permits unversioned preview only: every
    pose is invalid, and sequence regression still requires a new PiSource.
    """
    def __init__(self, camera_base_url: str, cameras: dict, encoder_config: dict, timeout_s=.3):
        self.base_url = _base_url(camera_base_url)
        self.timeout = _number(timeout_s, 'timeout_s')
        if not 0 < self.timeout <= 5:
            raise ValueError('timeout_s must be in (0, 5]')
        self.cameras = dict(cameras)
        if not self.cameras or not all(isinstance(v, CameraModel) and v.name == k for k, v in self.cameras.items()):
            raise ValueError('Camera names must match the CameraModel names')
        self.config = dict(encoder_config)
        self.allow_legacy_gateway = self.config.get('allow_legacy_gateway', False)
        if type(self.allow_legacy_gateway) is not bool:
            raise ValueError('allow_legacy_gateway must be a boolean')
        self.indices = dict(self.config.get('camera_indices', {'narrow': 0, 'wide': 1}))
        for name in self.cameras:
            index = self.indices.get(name)
            if isinstance(index, bool) or not isinstance(index, int) or index not in (0, 1):
                raise ValueError('Each camera needs a verified camera_index of 0 or 1')
        if len({self.indices[n] for n in self.cameras}) != len(self.cameras):
            raise ValueError('Camera indices must be distinct')
        self.max_age = _number(self.config.get('max_age_s', .5), 'max_age_s')
        self.frame_max_age = _number(self.config.get('frame_max_age_s', .5), 'frame_max_age_s')
        self.max_gap = _number(self.config.get('max_alignment_gap_s', .25), 'max_alignment_gap_s')
        self.max_extrapolation = _number(self.config.get('max_pose_extrapolation_s', .05), 'max_pose_extrapolation_s')
        if min(self.max_age, self.frame_max_age, self.max_gap) <= 0 or self.max_extrapolation < 0:
            raise ValueError('Freshness bounds must be positive')
        self.axes = self.config.get('axes', self.config)
        # Unregistered configurations can still display images with invalid pose.
        if self.config.get('calibrated') is True:
            buses = []
            for name in ('az', 'el'):
                axis = self.axes.get(name, {})
                bus = axis.get('bus')
                if type(bus) is not int or bus not in (1, 3):
                    raise ValueError('Registered axes require bus 1 or 3')
                buses.append(bus)
                if _number(axis.get('sign'), name + '.sign') not in (-1, 1):
                    raise ValueError('Encoder sign must be +1 or -1')
                _number(axis.get('zero_deg'), name + '.zero_deg')
                _number(axis.get('offset_deg'), name + '.offset_deg')
            if len(set(buses)) != 2:
                raise ValueError('Axes must use distinct encoder buses')
        self._pool = ThreadPoolExecutor(max_workers=len(self.cameras)+1, thread_name_prefix='det-http')
        self._lock = threading.Lock()
        self._closed = threading.Event()
        self._sequences = {}
        self._encoder_samples = {}
        self._axis_history = {'az': [], 'el': []}
        self.generation = None
        self.generation_revision = 0
        self.generation_changed = False
        self.generation_verified = False
        self._legacy_seen = False
        self._retired_generations = set()
        self.encoder_readings = []
        self.camera_info = {}
        self.errors = {}

    @staticmethod
    def _generation_id(value):
        if not isinstance(value, str):
            raise ValueError('Gateway generation must be a UUID string')
        try:
            parsed = uuid.UUID(value)
        except (ValueError, AttributeError) as error:
            raise ValueError('Gateway generation must be a UUID string') from error
        if not parsed.int or str(parsed) != value:
            raise ValueError('Gateway generation must be a canonical nonzero UUID')
        return value

    def _batch_generation(self, encoder_result, frame_results, corroborated):
        responses = list(frame_results.values())
        if encoder_result is not None:
            responses.append(encoder_result)
        if not responses:
            raise ValueError('No gateway replies to establish a generation')
        generations = [response[1].get('x-gateway-generation') for response in responses]
        infos = {}
        for name, result in frame_results.items():
            info = _json(result[1].get('x-frame-info', '{}'))
            if not isinstance(info, dict):
                raise ValueError('Camera metadata must be an object')
            if info.get('gateway_generation') != result[1].get('x-gateway-generation'):
                raise ValueError('Camera generation metadata disagrees with its HTTP header')
            infos[name] = info
        if all(generation is None for generation in generations):
            if self.generation is not None:
                raise ValueError('Gateway generation downgrade to legacy is forbidden')
            if not self.allow_legacy_gateway:
                raise ValueError('Gateway generation is missing; legacy preview requires explicit configuration')
            self._legacy_seen = True
            self.camera_info = infos
            return
        if any(generation is None for generation in generations):
            raise ValueError('Mixed versioned and legacy gateway replies')
        generations = {self._generation_id(generation) for generation in generations}
        if len(generations) != 1:
            raise ValueError('Mixed gateway generations in one observation batch')
        generation = generations.pop()
        if generation in self._retired_generations:
            raise ValueError('Reply belongs to a retired gateway generation')
        if generation != self.generation:
            if not corroborated:
                raise ValueError('A new gateway generation requires encoder and camera replies')
            if len(self._retired_generations) >= 64:
                raise ValueError('Gateway generation retirement limit reached; create a new source')
            if self.generation is not None:
                self._retired_generations.add(self.generation)
            self.generation_changed = self.generation is not None or self._legacy_seen
            self.generation = generation
            self.generation_revision += 1
            self._sequences.clear()
            self._encoder_samples.clear()
            self._axis_history = {'az': [], 'el': []}
        self.generation_verified = True
        self.camera_info = infos

    @staticmethod
    def _public_encoder(item):
        """Validate and copy only public diagnostic fields, without requiring calibration."""
        public = {'bus': item['bus']}
        for key in ('enabled', 'connected', 'valid', 'magnet_detected', 'field_weak', 'field_strong', 'stale'):
            if key in item:
                if item[key] is not None and type(item[key]) is not bool:
                    raise ValueError('Malformed encoder diagnostic: ' + key)
                public[key] = item[key]
        for key in ('degrees', 'age_s', 'read_at', 'raw', 'status', 'agc', 'magnitude'):
            if key in item:
                if item[key] is not None:
                    _number(item[key], key)
                public[key] = item[key]
        for key in ('address', 'error'):
            if key in item:
                if item[key] is not None and not isinstance(item[key], str):
                    raise ValueError('Malformed encoder diagnostic: ' + key)
                public[key] = item[key]
        if 'diagnostic_errors' in item:
            errors = item['diagnostic_errors']
            if not isinstance(errors, dict) or not all(isinstance(v, str) for v in errors.values()):
                raise ValueError('Malformed encoder diagnostic errors')
            public['diagnostic_errors'] = {k:v for k,v in errors.items() if k in ('agc', 'magnitude')}
        return public

    def _pose(self, result):
        raw, _, started, received = result
        data = _json(raw)
        if not isinstance(data, list):
            raise ValueError('Encoder response must be a list')
        by_bus = {}
        for item in data:
            if (not isinstance(item, dict) or type(item.get('bus')) is not int
                    or item['bus'] not in (1, 3) or item['bus'] in by_bus):
                raise ValueError('Malformed or duplicate encoder bus')
            by_bus[item['bus']] = item
        self.encoder_readings = [self._public_encoder(item) for item in data]
        if self.config.get('calibrated') is not True:
            raise ValueError('Encoder axis registration is not calibrated')
        if not self.generation_verified:
            raise ValueError('Legacy gateway preview has no verified encoder generation')
        values, stamps = {}, []
        for axis_name in ('az', 'el'):
            axis = self.axes[axis_name]
            item = by_bus.get(axis['bus'], {})
            if (any(item.get(key) is not True for key in ('enabled', 'connected', 'valid', 'magnet_detected'))
                    or any(item.get(key) is not False for key in ('field_weak', 'field_strong', 'stale'))
                    or item.get('error')):
                raise ValueError(axis_name + ' encoder is invalid: ' + str(item.get('error') or 'status/magnet'))
            degrees = _number(item.get('degrees'), 'encoder degrees')
            age = _number(item.get('age_s'), 'encoder age')
            if not 0 <= degrees < 360 or age < 0:
                raise ValueError('Encoder angle or age is out of range')
            stamp = received - age - (received-started)
            # read_at is only an identifier. It may be any finite remote epoch.
            identity = item.get('read_at')
            if identity is not None:
                _number(identity, 'encoder read_at')
            key = (identity, degrees)
            previous = self._encoder_samples.get(axis['bus'])
            if previous is not None and previous[0] == key:
                stamp = min(stamp, previous[1])
            self._encoder_samples[axis['bus']] = (key, stamp)
            if time.monotonic() - stamp > self.max_age:
                raise ValueError(axis_name + ' encoder sample is stale')
            angle = axis['sign'] * angle_error(degrees, axis['zero_deg']) + axis['offset_deg']
            values[axis_name] = angle % 360 if axis_name == 'az' else angle
            stamps.append(stamp)
            history = self._axis_history[axis_name]
            if previous is not None and previous[0] == key and history:
                history[-1] = (min(history[-1][0], stamp), values[axis_name])
            else:
                if history and stamp <= history[-1][0]:
                    # An uncertain/regressed transport timestamp cannot become
                    # an artificial infinite angular rate in the next sample.
                    history.clear()
                history.append((stamp, values[axis_name]))
                del history[:-64]
        common_time = min(stamps)
        try:
            return self._pose_at(common_time)
        except ValueError as error:
            self.errors['encoder_alignment'] = str(error)
            return Pose(common_time, values['az'], values['el'], False)

    def _pose_at(self, timestamp):
        values = {axis: align_angle(self._axis_history[axis], timestamp, axis == 'az',
                                    self.max_gap, self.max_extrapolation)
                  for axis in ('az', 'el')}
        return Pose(timestamp, values['az'], values['el'], True)

    def _frame(self, name, result, pose):
        raw, headers, started, received = result
        if headers.get('content-type', '').split(';')[0].strip() != 'image/jpeg':
            raise ValueError('Camera did not return a JPEG')
        info = _json(headers.get('x-frame-info', '{}'))
        if not isinstance(info, dict) or info.get('error'):
            raise ValueError('Camera metadata reports an error')
        sequence = info.get('sequence')
        if type(sequence) is not int or sequence <= 0:
            raise ValueError('Camera sequence must be a positive integer')
        if sequence <= self._sequences.get(name, -1):
            extra = '; legacy restart requires a new source instance' if not self.generation_verified else ''
            raise ValueError('Duplicate or regressed camera sequence' + extra)
        age = _number(info.get('age_s'), 'frame age')
        stamp = received - age - (received-started)
        if age < 0 or time.monotonic()-stamp > self.frame_max_age:
            raise ValueError('Camera frame is stale')
        camera = self.cameras[name]
        if info.get('size') != [camera.width, camera.height]:
            raise ValueError('Camera metadata dimensions differ from calibration')
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
        if image is None or image.shape != (camera.height, camera.width):
            raise ValueError('Invalid JPEG or decoded dimensions differ from calibration')
        self._sequences[name] = sequence
        frame_pose = replace(pose, timestamp=stamp, valid=False)
        if pose.valid:
            try:
                frame_pose = self._pose_at(stamp)
            except ValueError as error:
                self.errors[name+'_pose'] = str(error)
        return Frame(name, stamp, image, frame_pose, sequence)

    def read(self):
        with self._lock:
            if self._closed.is_set():
                raise TransportError('Camera source is closed')
            futures = {name: self._pool.submit(_http, self.base_url+'/frame/%d.jpg' % self.indices[name],
                                              self.timeout, limit=8*1024*1024)
                       for name in self.cameras}
            encoder_future = self._pool.submit(_http, self.base_url+'/encoders', self.timeout)
            self.errors = {}
            self.encoder_readings = []
            self.camera_info = {}
            self.generation_verified = False
            self.generation_changed = False
            frame_results = {}
            encoder_result = None
            try:
                encoder_result = encoder_future.result()
            except Exception as error:
                self.errors['encoders'] = str(error)
            for name, future in futures.items():
                try:
                    frame_results[name] = future.result()
                except Exception as error:
                    self.errors[name] = str(error)
            try:
                self._batch_generation(encoder_result, frame_results,
                                       encoder_result is not None and bool(frame_results))
            except Exception as error:
                self.errors['generation'] = str(error)
                return [], Pose(time.monotonic(), 0, 0, False)
            try:
                if encoder_result is None:
                    raise ValueError(self.errors.get('encoders', 'No encoder response'))
                pose = self._pose(encoder_result)
            except Exception as error:
                self.errors['encoders'] = str(error)
                pose = Pose(time.monotonic(), 0, 0, False)
            if self.generation_changed or not self.generation_verified:
                # The first batch after a restart cannot drive control while
                # the runtime disarms and discards its prior tracking state.
                pose = replace(pose, valid=False)
            frames = []
            for name, result in frame_results.items():
                try:
                    frames.append(self._frame(name, result, pose))
                except Exception as error:
                    self.errors[name] = str(error)
            # A sibling request can consume time after the pose was validated.
            if time.monotonic()-pose.timestamp > self.max_age:
                pose = replace(pose, valid=False)
                frames = [replace(f, pose=replace(f.pose, valid=False)) for f in frames]
            if self._closed.is_set():
                raise TransportError('Camera source closed during read')
            return frames, pose

    def close(self):
        self._closed.set()
        self._pool.shutdown(wait=False, cancel_futures=True)


class HttpMotor:
    """A single arm session. Stop/failure is latched until a new instance.

    timeout_s is also the conservative outbound transport allowance subtracted
    from Mac command validity. Use e.g. .05 s with a .25 s executor lifetime,
    after measuring the real link. STOP has its own 3 s acknowledgement budget.
    """
    def __init__(self, base_url: str, timeout_s=.3):
        self.base_url = _base_url(base_url)
        self.timeout = _number(timeout_s, 'timeout_s')
        if not 0 < self.timeout <= 1:
            raise ValueError('Motor HTTP timeout must be in (0, 1]')
        self._generation = None
        self._last_sequence = -1
        self._stopped = threading.Event()
        self._send_lock = threading.Lock()
        self._used = False
        # Bind at intentional client construction, not lazily at first motion.
        # Otherwise an already-created client could fetch a NEW token after a
        # remote Stop and unintentionally turn that Stop into a fresh arm.
        status = self._call('/status')
        if (status.get('initialized') is not True or status.get('firmware_watchdog') is not True or
                status.get('fault') or status.get('stopping') or status.get('closed')):
            raise TransportError('Motor gateway is not ready with a verified firmware watchdog')
        self._generation = status.get('generation')
        if not isinstance(self._generation, str) or not self._generation:
            raise TransportError('Motor gateway did not provide a generation')

    def _call(self, path, data=None, timeout=None):
        raw, _, _, _ = _http(self.base_url+path, self.timeout if timeout is None else timeout, data)
        result = _json(raw)
        if not isinstance(result, dict):
            raise TransportError('Motor gateway returned a non-object response')
        return result

    def send(self, command: Command):
        if not isinstance(command, Command):
            raise TransportError('Expected a Command')
        if type(command.enabled) is not bool:
            raise TransportError('Command enabled must be boolean')
        if not command.enabled:
            return self.stop()
        with self._send_lock:
            try:
                for name in ('timestamp', 'valid_until', 'az_velocity_deg_s', 'el_velocity_deg_s'):
                    _number(getattr(command, name), name)
                if type(command.sequence) is not int or command.sequence <= self._last_sequence or command.sequence < 0:
                    raise ValueError('Command sequence must increase')
                if command.timestamp > time.monotonic() or command.valid_until-time.monotonic() <= self.timeout:
                    raise ValueError('Command is stale or lacks the transport allowance')
            except ValueError as error:
                raise TransportError(str(error)) from error
            if self._stopped.is_set():
                raise TransportError('Motor session is stopped; intentional re-arm requires a new client')
            try:
                ttl = command.valid_until-time.monotonic()-self.timeout
                if ttl <= 0 or self._stopped.is_set():
                    raise TransportError('Command expired or session stopped before transmission')
                payload = asdict(command)
                payload.update(ttl_s=ttl, generation=self._generation)
                self._used = True
                result = self._call('/velocity', payload)
                if (self._stopped.is_set() or result.get('generation') != self._generation or
                        result.get('sequence') != command.sequence or result.get('active') is not True or result.get('fault')):
                    raise TransportError('Velocity was not acknowledged for this session')
                self._last_sequence = command.sequence
                return result
            except Exception as error:
                try:
                    self.stop()
                except Exception:
                    pass
                if isinstance(error, TransportError):
                    raise
                raise TransportError(str(error)) from error

    def stop(self):
        # Do not wait behind an in-flight send before issuing the cancellation.
        self._stopped.set()
        result = self._call('/stop', {}, timeout=3.0)
        if result.get('stop_acknowledged') is not True or result.get('stopped_step_count_confirmed') is not True:
            raise TransportError('Motor stop was not fully confirmed')
        return result

    def close(self):
        if self._used and not self._stopped.is_set():
            self.stop()
        else:
            self._stopped.set()
