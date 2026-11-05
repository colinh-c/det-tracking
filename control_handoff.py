#!/usr/bin/env python3
"""Local manual/flight ownership relay. Starting the server never claims motors.

Explicit bootstrap replaces only the verified single-port manual SSH forward.
It leaves camera tunnels alone, and keeps the relay running after a flight.
Service transitions occur only on claim, owner release, or manual takeover.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import secrets
import shlex
import signal
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.error import HTTPError, URLError
from urllib.parse import urlsplit
from urllib.request import Request, urlopen

from transport import HttpMotor

def _ssh_host(value=None):
    return value or os.environ.get('DET_SSH_HOST') or 'pi@raspberrypi.local'


def _ssh_options(host_key_alias=None):
    options = ['-o', 'BatchMode=yes', '-o', 'ConnectTimeout=3',
               '-o', 'StrictHostKeyChecking=yes', '-o', 'ExitOnForwardFailure=yes',
               '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=3']
    alias = host_key_alias if host_key_alias is not None else os.environ.get('DET_SSH_HOST_KEY_ALIAS')
    if alias:
        options += ['-o', 'HostKeyAlias='+alias]
    return options


BASE_URL = 'http://127.0.0.1:18083'


class HandoffError(RuntimeError):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status


def _json_request(url, payload=None, timeout=1):
    request = Request(url, data=None if payload is None else json.dumps(payload, allow_nan=False).encode(),
                      headers={'Content-Type': 'application/json', 'X-DET-Motion': '1'})
    try:
        with urlopen(request, timeout=timeout) as response:
            value = json.loads(response.read(65537))
    except HTTPError as error:
        try: detail = json.loads(error.read(65537)).get('error', str(error))
        except Exception: detail = str(error)
        raise HandoffError(error.code, detail) from error
    except (OSError, ValueError) as error:
        raise HandoffError(503, str(error)) from error
    if not isinstance(value, dict):
        raise HandoffError(502, 'Expected an object response')
    return value


def _session(value):
    if not isinstance(value, str) or not re.fullmatch(r'[a-f0-9]{48}', value):
        raise HandoffError(400, 'Invalid flight session')
    return value


class PiBackend:
    """Only explicit owner transitions invoke SSH; no deployment or flashing."""
    urls = {'manual': 'http://127.0.0.1:18086', 'automatic': 'http://127.0.0.1:18084',
            'encoders': 'http://127.0.0.1:18085'}
    def __init__(self, ssh_host=None, host_key_alias=None, pi_root=None,
                 manual_gateway=None, auto_gateway=None):
        self.ssh_host = _ssh_host(ssh_host)
        self.ssh_options = _ssh_options(host_key_alias)
        root = (pi_root or os.environ.get('DET_PI_ROOT') or '/home/pi/gimbal').rstrip('/')
        manual = manual_gateway or os.environ.get('DET_MANUAL_GATEWAY') or root+'/manual_dashboard/motion_gateway.py'
        automatic = auto_gateway or os.environ.get('DET_AUTO_GATEWAY') or root+'/pi_tracking_gateway.py'
        self.units = {'manual': ('det-manual-motion', manual, 8083),
                      'automatic': ('det-hil-motion', automatic, 8084)}

    def request(self, target, path, payload=None, timeout=1):
        return _json_request(self.urls[target]+path, payload, timeout)

    def service(self, name, start):
        unit, script, port = self.units[name]
        if start:
            command = shlex.join(['systemd-run', '--user', '--unit='+unit, '--collect',
                                  '--property=TimeoutStopSec=5s', '/usr/bin/python3', '-u',
                                  script, '--port', str(port)])
        else:
            # A stopped/collected unit may already be absent. Other stop errors
            # are not ignored; exclusive serial ownership must be released.
            quoted = shlex.quote(unit+'.service')
            command = ('state=$(systemctl --user show -p LoadState --value '+quoted+'); '
                       'if [ "$state" != "not-found" ]; then systemctl --user stop '+quoted+'; fi')
        result = subprocess.run(['ssh', *self.ssh_options, self.ssh_host, command], capture_output=True,
                                text=True, timeout=12)
        if result.returncode:
            raise HandoffError(503, 'Pi service transition failed: '+result.stderr.strip())
        if start:
            deadline = time.monotonic()+5
            endpoint = '/status' if name == 'automatic' else '/motion'
            while time.monotonic() < deadline:
                try:
                    status = self.request(name, endpoint, timeout=.3)
                    if status.get('connected') and (name == 'manual' or status.get('initialized')):
                        return
                except HandoffError:
                    pass
                time.sleep(.05)
            raise HandoffError(503, name+' service did not become ready')


class ControlHandoff:
    def __init__(self, backend, base_url=BASE_URL):
        self.backend, self.base_url = backend, base_url.rstrip('/')
        self.state = threading.Condition(threading.RLock())
        self.transition = threading.Lock()
        self.mode, self.session, self.generation = 'manual', None, None
        self.epoch = 0
        self.inflight = {}
        self.used_holds = set()
        self.last_error = None

    def status(self):
        with self.state:
            return {'protocol': 'DET_HANDOFF_V1', 'mode': self.mode,
                    'automatic_owned': self.session is not None, 'last_error': self.last_error}

    def _check(self, token):
        if self.session != token or self.mode not in ('claiming', 'automatic'):
            raise HandoffError(409, 'Flight ownership was revoked by manual control or Stop')

    def _owned_request(self, token, path, payload=None, timeout=.3):
        with self.state:
            self._check(token)
            epoch = self.epoch
            self.inflight[epoch] = self.inflight.get(epoch, 0)+1
        try:
            value = self.backend.request('automatic', path, payload, timeout=timeout)
            with self.state: self._check(token)
            return value
        finally:
            with self.state:
                self.inflight[epoch] -= 1
                self.state.notify_all()

    def _drain(self, before_epoch):
        deadline = time.monotonic()+4
        with self.state:
            while any(count for epoch, count in self.inflight.items() if epoch < before_epoch):
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise HandoffError(503, 'Prior automatic HTTP requests did not finish')
                self.state.wait(remaining)

    def claim(self):
        with self.state:
            entry_epoch = self.epoch
            if self.mode != 'manual' or self.session or any(self.inflight.values()):
                raise HandoffError(409, 'Wait for manual requests to finish before claiming')
        manual = self.backend.request('manual', '/motion', timeout=.5)
        if manual.get('busy') or manual.get('hold_active') or manual.get('last_error') or not manual.get('connected'):
            raise HandoffError(409, 'Manual controls must be idle and healthy before claiming')
        limits = manual.get('limits')
        if (not isinstance(limits, dict) or limits.get('acknowledged') is not True or
                limits.get('confirmation_required') or manual.get('limits_confirmation_required')):
            raise HandoffError(409, 'Confirm movement limits before automatic tracking')
        try:
            bounds, position = limits['soft_limits_deg'], manual['position']
            inside = all(len(bounds[axis]) == 2 and
                         all(isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)
                             for x in (*bounds[axis], position[axis])) and
                         bounds[axis][0] <= position[axis] <= bounds[axis][1]
                         for axis in ('az', 'el'))
        except (KeyError, TypeError, ValueError):
            inside = False
        if not inside:
            raise HandoffError(409, 'Move inside the programmed movement limits before automatic tracking')
        with self.state:
            if (self.mode != 'manual' or self.session or self.epoch != entry_epoch or
                    any(self.inflight.values())):
                raise HandoffError(409, 'Manual control is not available to claim')
            self.epoch += 1
            token = self.session = secrets.token_hex(24)
            self.mode = 'claiming'
        try:
            with self.transition:
                with self.state: self._check(token)
                # Stop is outside any motion-request lock.
                self.backend.request('manual', '/motion/stop', {'global': True}, timeout=3)
                self.backend.service('manual', False)
                with self.state: self._check(token)
                self.backend.service('automatic', True)
                with self.state: self._check(token)
                value = self._owned_request(token, '/pose')
                with self.state:
                    self._check(token)
                    self.mode = 'automatic'
                    self.generation = value['generation']
                return {'session': token, 'position': value['position'], 'generation': value['generation'],
                        'motor_base_url': self.base_url+'/flight/'+token+'/motor',
                        'limits': value.get('limits')}
        except Exception:
            # A takeover owns its own restoration. Stale claim cleanup must
            # never stop the user's newly requested motion.
            with self.state: still_owned = self.session == token
            if still_owned:
                self.release(token)
            raise

    def _restore_manual(self, expected_epoch, manual_only=False):
        with self.transition:
            with self.state:
                if self.epoch != expected_epoch: return
            try:
                self._drain(expected_epoch)
            except HandoffError:
                # A late old manual dispatch may have followed urgent Stop.
                # Cancel again, but never announce a new owner on failed drain.
                try: self.backend.request('manual', '/motion/stop', {'global': True}, timeout=3)
                except HandoffError: pass
                raise
            if not manual_only: self.backend.service('automatic', False)
            with self.state:
                if self.epoch != expected_epoch: return
            # Stop/restart only during an actual automatic ownership transition.
            # Existing manual mode is never restarted for an ordinary jog/Stop.
            try:
                ready = self.backend.request('manual', '/motion', timeout=.3)
                available = ready.get('connected') is True
            except HandoffError:
                available = False
            if not available: self.backend.service('manual', True)
            # All old dispatches are now finished. Stop once more before a
            # newer manual request may enter; an old request could have reached
            # the Pi just after the urgent Stop sent before drainage.
            self.backend.request('manual', '/motion/stop', {'global': True}, timeout=3)
            with self.state:
                if self.epoch == expected_epoch: self.mode = 'manual'

    def _revoke(self, token=None, force_manual=False, expected_epoch=None):
        with self.state:
            if expected_epoch is not None and self.epoch != expected_epoch:
                raise HandoffError(409, 'A newer control request superseded this request')
            if token is not None and self.session != token:
                return None
            if self.mode == 'manual' and self.session is None and not force_manual:
                return None
            manual_only = self.mode == 'manual'
            retired_epoch = self.epoch
            self.inflight[retired_epoch] = self.inflight.get(retired_epoch, 0)+1
            self.epoch += 1
            epoch = self.epoch
            self.session = self.generation = None
            self.mode = 'restoring'
        # Urgent cancellation is sent before waiting for service transitions or
        # outstanding requests. The firmware lease independently bounds loss.
        try:
            requests = [('manual', '/motion/stop', {'global': True})]
            if not manual_only: requests.insert(0, ('automatic', '/stop', {}))
            for target, path, body in requests:
                try: self.backend.request(target, path, body, timeout=3)
                except HandoffError: pass  # One of the services normally is absent.
        finally:
            with self.state:
                self.inflight[retired_epoch] -= 1
                self.state.notify_all()
        try:
            self._restore_manual(epoch, manual_only=manual_only)
        except Exception as error:
            with self.state: self.last_error = str(error)
            raise
        return epoch

    def release(self, token):
        _session(token)
        changed = self._revoke(token)
        return {'released': changed is not None, **self.status()}

    def pose(self, token):
        _session(token)
        value = self._owned_request(token, '/pose')
        with self.state:
            self._check(token)
            if value.get('generation') != self.generation:
                raise HandoffError(409, 'Automatic controller generation changed')
        return value

    def motor(self, token, path, payload=None):
        _session(token)
        if path not in ('/status', '/velocity', '/stop'):
            raise HandoffError(404, 'Unknown motor path')
        return self._owned_request(token, path, payload, timeout=3 if path == '/stop' else .3)

    @staticmethod
    def _motion_values(payload, key):
        if payload.get('axis') not in ('az', 'el'):
            raise HandoffError(400, 'Invalid axis')
        value = payload.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise HandoffError(400, 'Invalid motion value')

    def _manual_dispatch(self, path, payload, expected_epoch, timeout):
        with self.state:
            if self.mode != 'manual' or self.epoch != expected_epoch:
                raise HandoffError(409, 'Manual request was superseded or takeover is still pending')
            # This also invalidates a claim's earlier idle-status observation.
            self.epoch += 1
            dispatch_epoch = self.epoch
            self.inflight[dispatch_epoch] = self.inflight.get(dispatch_epoch, 0)+1
        try:
            return self.backend.request('manual', path, payload, timeout=timeout)
        finally:
            with self.state:
                self.inflight[dispatch_epoch] -= 1
                self.state.notify_all()

    def manual(self, path, payload=None):
        if path == '/motion' and payload is None:
            with self.state: mode = self.mode
            if mode != 'manual':
                return {'connected': True, 'initialized': True, 'busy': mode != 'automatic',
                        'hold_active': False, 'last_error': self.last_error,
                        'automatic_active': True, 'handoff': self.status()}
            return self.backend.request('manual', path, timeout=.5)
        if not isinstance(payload, dict): raise HandoffError(400, 'Expected a JSON object')
        with self.state: entry_epoch = self.epoch
        if path == '/motion/limits':
            with self.state:
                if self.mode != 'manual':
                    raise HandoffError(409, 'Stop the automatic test before changing limits')
            return self._manual_dispatch(path, payload, entry_epoch, timeout=8)
        if path == '/motion/hold/renew':
            if set(payload) != {'hold_id'}: raise HandoffError(400, 'Invalid hold renewal')
            with self.state:
                if self.mode != 'manual': raise HandoffError(410, 'Prior manual hold has ended')
            return self.backend.request('manual', path, payload, timeout=.5)
        if path == '/motion/stop':
            if set(payload)-{'hold_id', 'global'} or ('global' in payload and type(payload['global']) is not bool):
                raise HandoffError(400, 'Invalid Stop request')
            if 'hold_id' in payload:
                hold = payload['hold_id']
                if not isinstance(hold, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', hold):
                    raise HandoffError(400, 'Invalid hold identifier')
                # Release may arrive before start and must cancel that delayed
                # start even across a complete manual/automatic service swap.
                with self.state: self.used_holds.add(hold)
            if 'hold_id' in payload and not payload.get('global'):
                with self.state:
                    if self.mode != 'manual':
                        return {'stop_requested': False, 'cancelled_hold_id': payload['hold_id']}
                return self.backend.request('manual', path, payload, timeout=3)
            changed = self._revoke(force_manual=True)
            if changed is not None:
                # Revocation already sent urgent Stop. A second late Stop here
                # could cancel a newer user's jog after restoration completed.
                return {'stop_requested': True, **self.manual('/motion')}
            return self.backend.request('manual', path, payload, timeout=3)
        if path == '/motion/jog':
            if set(payload) != {'axis', 'delta'}: raise HandoffError(400, 'Invalid jog')
            self._motion_values(payload, 'delta')
        elif path == '/motion/hold/start':
            if set(payload) != {'hold_id', 'axis', 'velocity'}: raise HandoffError(400, 'Invalid hold start')
            self._motion_values(payload, 'velocity')
            hold = payload['hold_id']
            if not isinstance(hold, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', hold):
                raise HandoffError(400, 'Invalid hold identifier')
            with self.state:
                if hold in self.used_holds: raise HandoffError(410, 'Prior hold cannot restart')
                self.used_holds.add(hold)
        else: raise HandoffError(404, 'Unknown manual path')
        takeover_epoch = self._revoke(expected_epoch=entry_epoch)
        expected = entry_epoch if takeover_epoch is None else takeover_epoch
        return self._manual_dispatch(path, payload, expected, timeout=30 if path.endswith('/jog') else 1)


class SessionMotor(HttpMotor):
    def __init__(self, base_url, session, timeout_s=.05):
        self.prefix = '/flight/'+_session(session)+'/motor'
        super().__init__(base_url, timeout_s)
    def _call(self, path, data=None, timeout=None):
        return super()._call(self.prefix+path, data, timeout)


class HandoffClient:
    def __init__(self, base_url=BASE_URL, timeout_s=.5):
        self.base_url, self.timeout = base_url.rstrip('/'), timeout_s
    def claim(self): return _json_request(self.base_url+'/flight/claim', {}, 30)
    def pose(self, session):
        started = time.monotonic()
        value = _json_request(self.base_url+'/flight/'+_session(session)+'/pose', timeout=self.timeout)
        value['age_s'] += time.monotonic()-started
        return value
    def release(self, session):
        return _json_request(self.base_url+'/flight/release', {'session': _session(session)}, 30)
    def motor(self, session, timeout_s=.05): return SessionMotor(self.base_url, session, timeout_s)


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        super().setup(); self.connection.settimeout(1)
    def log_message(self, *args): pass
    def reply(self, code, value):
        body = json.dumps(value, allow_nan=False).encode()
        self.send_response(code); self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body))); self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        try: self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError): pass
    def dispatch(self, post):
        try:
            expected = {'127.0.0.1:'+str(self.server.server_port), 'localhost:'+str(self.server.server_port)}
            if self.headers.get('Host') not in expected or self.headers.get('Origin') is not None:
                raise HandoffError(403, 'Local server requests only')
            payload = None
            if post:
                if self.headers.get('X-DET-Motion') != '1' or self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                    raise HandoffError(403, 'Local JSON motion header required')
                if self.headers.get('Transfer-Encoding'): raise HandoffError(400, 'Chunking not supported')
                length = int(self.headers.get('Content-Length', '-1'))
                if not 0 <= length <= 8192: raise HandoffError(400, 'Invalid request size')
                from pi_tracking_gateway import _decode
                raw = self.rfile.read(length)
                if len(raw) != length: raise HandoffError(400, 'Incomplete request')
                payload = _decode(raw)
            handoff = self.server.handoff
            if self.path == '/handoff' and not post: result = handoff.status()
            elif self.path == '/flight/claim' and post and payload == {}: result = handoff.claim()
            elif self.path == '/flight/release' and post and isinstance(payload, dict) and set(payload) == {'session'}:
                result = handoff.release(payload['session'])
            elif self.path.startswith('/motion'): result = handoff.manual(self.path, payload)
            else:
                match = re.fullmatch(r'/flight/([a-f0-9]{48})/(pose|motor/(?:status|velocity|stop))', self.path)
                if not match: raise HandoffError(404, 'Unknown path')
                session, action = match.groups()
                if action == 'pose' and not post: result = handoff.pose(session)
                elif action == 'motor/status' and not post: result = handoff.motor(session, '/status')
                elif action in ('motor/velocity', 'motor/stop') and post:
                    result = handoff.motor(session, '/'+action.split('/')[1], payload)
                else: raise HandoffError(405, 'Incorrect method')
            self.reply(200, result)
        except HandoffError as error: self.reply(error.status, {'error': str(error)})
        except (ValueError, KeyError, OSError) as error: self.reply(400, {'error': str(error)})
    def do_GET(self): self.dispatch(False)
    def do_POST(self): self.dispatch(True)


def make_server(handoff, port=18083):
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    server.daemon_threads = True; server.block_on_close = False; server.handoff = handoff
    return server



def _raw_manual_tunnel_args():
    return ['ssh', '-N', '-f', *_ssh_options(), '-L',
            '127.0.0.1:18083:127.0.0.1:8083', _ssh_host()]


def _verify_raw_manual_tunnel(pid):
    result = subprocess.run(['ps', '-p', str(pid), '-o', 'command='], capture_output=True,
                            text=True, check=True, timeout=2)
    actual = shlex.split(result.stdout.strip())
    expected = _raw_manual_tunnel_args()
    if not actual or Path(actual[0]).name != 'ssh' or actual[1:] != expected[1:]:
        raise HandoffError(409, 'Port18083 is not the exact known single-port manual SSH tunnel')


def _assert_manual_idle():
    value = _json_request(BASE_URL+'/motion', timeout=.5)
    if (value.get('connected') is not True or value.get('busy') or value.get('hold_active') or
            value.get('stopping') or value.get('blocked') or value.get('last_error')):
        raise HandoffError(409, 'Release manual controls and resolve any fault before explicit Run')


def bootstrap_handoff():
    """EXPLICIT RUN ONLY. Keep relay/tunnels alive to retain manual controls.

    Does not stop any Pi service. Verifies the existing single-port Mac SSH
    process before replacing its forward. Failure restores that same forward.
    """
    try:
        ready = _json_request(BASE_URL+'/handoff', timeout=.3)
        if ready.get('protocol') == 'DET_HANDOFF_V1': return ready
    except HandoffError:
        pass
    _assert_manual_idle()
    listeners = subprocess.run(['lsof', '-nP', '-iTCP:18083', '-sTCP:LISTEN', '-t'],
                               capture_output=True, text=True, timeout=2, check=True)
    pids = set(listeners.stdout.split())
    if len(pids) != 1 or not next(iter(pids)).isdigit():
        raise HandoffError(409, 'Cannot identify exactly one existing manual forward')
    pid = int(next(iter(pids)))
    _verify_raw_manual_tunnel(pid)
    directory = Path(tempfile.mkdtemp(prefix='det-handoff-'))
    control = directory/'ssh.sock'
    command = ['ssh', '-M', '-S', str(control), '-N', '-f', *_ssh_options()]
    for local, remote in ((18086, 8083), (18084, 8084), (18085, 8085)):
        command += ['-L', '127.0.0.1:%d:127.0.0.1:%d' % (local, remote)]
    command.append(_ssh_host())
    replaced = False
    process = None
    try:
        subprocess.run(command, check=True, capture_output=True, text=True, timeout=8)
        # Check again immediately before replacing the Mac route. Never interrupt
        # a user's ongoing jog/hold merely because the launcher was clicked.
        _assert_manual_idle()
        _verify_raw_manual_tunnel(pid)
        os.kill(pid, signal.SIGTERM)
        replaced = True
        deadline = time.monotonic()+2
        while time.monotonic() < deadline:
            check = subprocess.run(['lsof', '-nP', '-iTCP:18083', '-sTCP:LISTEN', '-t'],
                                   capture_output=True, text=True, timeout=1)
            if not check.stdout.strip(): break
            time.sleep(.05)
        else: raise HandoffError(503, 'Previous manual route did not release its port')
        with (directory/'relay.log').open('ab') as log:
            process = subprocess.Popen([sys.executable, '-u', str(Path(__file__).resolve()), '--port', '18083'],
                                       stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                       cwd=str(Path(__file__).resolve().parent), start_new_session=True)
        deadline = time.monotonic()+4
        while time.monotonic() < deadline:
            if process.poll() is not None: raise HandoffError(503, 'Manual relay exited during startup')
            try:
                ready = _json_request(BASE_URL+'/handoff', timeout=.3)
                if ready.get('protocol') == 'DET_HANDOFF_V1':
                    _assert_manual_idle()
                    metadata = {'relay_pid': process.pid, 'ssh_control_socket': str(control),
                                'directory': str(directory), 'protocol': 'DET_HANDOFF_V1',
                                'camera_tunnel_untouched': True}
                    (directory/'runtime.json').write_text(json.dumps(metadata, indent=2)+'\n')
                    return metadata
            except HandoffError:
                pass
            time.sleep(.05)
        raise HandoffError(503, 'Manual relay did not become ready')
    except Exception:
        if process is not None:
            process.terminate()
            try: process.wait(timeout=3)
            except subprocess.TimeoutExpired: pass
        if replaced:
            # No Pi service was changed. Re-establish the original direct route.
            subprocess.run(_raw_manual_tunnel_args(), check=True, capture_output=True,
                           text=True, timeout=8)
        subprocess.run(['ssh', '-S', str(control), '-O', 'exit', _ssh_host()],
                       capture_output=True, text=True, timeout=5)
        raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=18083)
    parser.add_argument('--ssh-host', help='Pi SSH target, or DET_SSH_HOST (default pi@raspberrypi.local)')
    parser.add_argument('--ssh-host-key-alias', help='Optional trusted known_hosts alias, or DET_SSH_HOST_KEY_ALIAS')
    parser.add_argument('--pi-root', help='Absolute repository path on Pi, or DET_PI_ROOT (default /home/pi/gimbal)')
    parser.add_argument('--manual-gateway', help='Absolute manual gateway script override, or DET_MANUAL_GATEWAY')
    parser.add_argument('--auto-gateway', help='Absolute automatic gateway script override, or DET_AUTO_GATEWAY')
    args = parser.parse_args(argv)
    backend = PiBackend(args.ssh_host, args.ssh_host_key_alias, args.pi_root,
                        args.manual_gateway, args.auto_gateway)
    handoff = ControlHandoff(backend, 'http://127.0.0.1:'+str(args.port))
    server = make_server(handoff, args.port)  # Occupied port fails; never kills its owner.
    stopped = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM): signal.signal(signum, lambda *_: stopped.set())
    server.timeout = .2
    try:
        while not stopped.is_set(): server.handle_request()
    finally:
        with handoff.state: token = handoff.session
        if token: handoff.release(token)
        server.server_close()


if __name__ == '__main__': main()
