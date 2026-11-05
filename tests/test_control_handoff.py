"""Only ownership and urgent cancellation boundaries, with no hardware."""
import threading
import unittest


class Backend:
    def __init__(self):
        self.calls = []
        self.jog_started = threading.Event()
        self.jog_release = None
        self.manual_stop = threading.Event()
    def request(self, target, path, payload=None, timeout=1):
        self.calls.append((target, path, payload))
        if path == '/motion/jog' and self.jog_release is not None:
            self.jog_started.set(); self.jog_release.wait(2)
        if path == '/motion/stop': self.manual_stop.set()
        if path == '/pose':
            return {'position': {'az': 10., 'el': 2.}, 'age_s': .002, 'generation': 'pi-generation'}
        return {'initialized': True, 'firmware_watchdog': True, 'connected': True,
                'generation': 'pi-generation', 'position': {'az': 10., 'el': 2.},
                'stop_acknowledged': True, 'stopped_step_count_confirmed': True}
    def service(self, name, start): self.calls.append(('service', name, start))


class HandoffTests(unittest.TestCase):
    def make(self):
        from control_handoff import ControlHandoff
        backend = Backend()
        return ControlHandoff(backend), backend
    def test_preparation_read_does_not_claim_or_change_services(self):
        handoff, backend = self.make()
        handoff.manual('/motion')
        self.assertEqual(backend.calls, [('manual', '/motion', None)])
    def test_manual_takeover_revokes_old_motor_and_old_release_cannot_stop_new_user_jog(self):
        from control_handoff import HandoffError
        handoff, backend = self.make()
        claim = handoff.claim()
        handoff.manual('/motion/jog', {'axis': 'az', 'delta': .5})
        before = list(backend.calls)
        self.assertFalse(handoff.release(claim['session'])['released'])
        with self.assertRaises(HandoffError):
            handoff.motor(claim['session'], '/velocity', {})
        self.assertEqual(before, backend.calls)
        self.assertEqual(backend.calls[-1], ('manual', '/motion/jog', {'axis': 'az', 'delta': .5}))
    def test_old_scoped_hold_messages_do_not_cancel_automatic_owner(self):
        from control_handoff import HandoffError
        handoff, backend = self.make()
        handoff.manual('/motion/hold/start', {'hold_id': 'old', 'axis': 'az', 'velocity': .5})
        claim = handoff.claim(); before = list(backend.calls)
        with self.assertRaises(HandoffError):
            handoff.manual('/motion/hold/renew', {'hold_id': 'old'})
        self.assertFalse(handoff.manual('/motion/stop', {'hold_id': 'old'})['stop_requested'])
        with self.assertRaises(HandoffError):
            handoff.manual('/motion/hold/start', {'hold_id': 'old', 'axis': 'az', 'velocity': .5})
        self.assertEqual(before, backend.calls)
        self.assertEqual(handoff.pose(claim['session'])['generation'], 'pi-generation')
    def test_stop_reaches_backend_while_manual_jog_request_waits(self):
        handoff, backend = self.make(); backend.jog_release = threading.Event()
        thread = threading.Thread(target=lambda: handoff.manual('/motion/jog', {'axis': 'az', 'delta': .5}))
        thread.start(); self.assertTrue(backend.jog_started.wait(.5))
        stop = threading.Thread(target=lambda: handoff.manual('/motion/stop', {'global': True}))
        stop.start(); self.assertTrue(backend.manual_stop.wait(.5)); self.assertTrue(thread.is_alive())
        backend.jog_release.set(); thread.join(1); stop.join(1)
        self.assertFalse(stop.is_alive())

    def test_delayed_old_automatic_stop_drains_before_next_owner_can_start(self):
        from control_handoff import HandoffError
        handoff, backend = self.make()
        claim = handoff.claim()
        entered, release, urgent = threading.Event(), threading.Event(), threading.Event()
        original = backend.request
        first = [True]
        def request(target, path, payload=None, timeout=1):
            if target == 'automatic' and path == '/stop':
                if first[0]:
                    first[0] = False; entered.set(); release.wait(2)
                else: urgent.set()
            return original(target, path, payload, timeout)
        backend.request = request
        old_errors = []
        def old_stop():
            try: handoff.motor(claim['session'], '/stop', {})
            except HandoffError as error: old_errors.append(error)
        old = threading.Thread(target=old_stop); old.start(); self.assertTrue(entered.wait(.5))
        takeover = threading.Thread(target=lambda: handoff.manual('/motion/jog', {'axis': 'az', 'delta': .5}))
        takeover.start(); self.assertTrue(urgent.wait(.5))
        with self.assertRaises(HandoffError): handoff.claim()
        self.assertFalse(any(call[1] == '/motion/jog' for call in backend.calls))
        release.set(); old.join(1); takeover.join(1)
        self.assertFalse(old.is_alive() or takeover.is_alive()); self.assertTrue(old_errors)
        new = handoff.claim(); before = list(backend.calls)
        self.assertFalse(handoff.release(claim['session'])['released'])
        self.assertEqual(backend.calls, before)
        self.assertNotEqual(claim['session'], new['session'])

    def test_release_before_hold_start_tombstone_survives_service_transition(self):
        from control_handoff import HandoffError
        handoff, backend = self.make()
        claim = handoff.claim()
        handoff.manual('/motion/stop', {'hold_id': 'not-started-yet'})
        handoff.release(claim['session'])
        before = list(backend.calls)
        with self.assertRaises(HandoffError):
            handoff.manual('/motion/hold/start', {'hold_id': 'not-started-yet', 'axis': 'az', 'velocity': .5})
        self.assertEqual(backend.calls, before)

    def test_newer_stop_waits_for_old_cancellation_and_prevents_old_takeover_jog(self):
        from control_handoff import HandoffError
        handoff, backend = self.make(); handoff.claim()
        entered, release, newer_stop_sent = threading.Event(), threading.Event(), threading.Event()
        original, first = backend.request, [True]
        def request(target, path, payload=None, timeout=1):
            if target == 'manual' and path == '/motion/stop':
                if first[0]:
                    first[0] = False; entered.set(); release.wait(2)
                else: newer_stop_sent.set()
            return original(target, path, payload, timeout)
        backend.request = request
        errors = []
        def jog():
            try: handoff.manual('/motion/jog', {'axis': 'az', 'delta': 1.})
            except HandoffError as error: errors.append(error)
        old = threading.Thread(target=jog); old.start(); self.assertTrue(entered.wait(.5))
        stop = threading.Thread(target=lambda: handoff.manual('/motion/stop', {}))
        stop.start(); self.assertTrue(newer_stop_sent.wait(.5)); self.assertTrue(stop.is_alive())
        release.set(); old.join(1); stop.join(1)
        self.assertFalse(old.is_alive() or stop.is_alive()); self.assertTrue(errors)
        self.assertFalse(any(call[1] == '/motion/jog' for call in backend.calls))

    def test_limits_edit_passes_through_in_manual_and_cannot_mutate_automatic_run(self):
        from control_handoff import HandoffError
        handoff, backend = self.make()
        payload = {'az_min': 0, 'az_max': 1, 'el_min': 0, 'el_max': 1}
        handoff.manual('/motion/limits', payload)
        self.assertEqual(backend.calls[-1], ('manual', '/motion/limits', payload))
        handoff.claim(); before = list(backend.calls)
        with self.assertRaises(HandoffError): handoff.manual('/motion/limits', payload)
        self.assertEqual(backend.calls, before)

    def test_ordinary_manual_request_paused_before_dispatch_cannot_follow_newer_stop(self):
        from control_handoff import HandoffError
        handoff, backend = self.make()
        paused, release = threading.Event(), threading.Event()
        original = handoff._revoke
        def revoke(*args, **kwargs):
            result = original(*args, **kwargs)
            if threading.current_thread().name == 'old-jog':
                paused.set(); release.wait(2)
            return result
        handoff._revoke = revoke
        errors = []
        def jog():
            try: handoff.manual('/motion/jog', {'axis': 'az', 'delta': 1.})
            except HandoffError as error: errors.append(error)
        old = threading.Thread(target=jog, name='old-jog'); old.start()
        self.assertTrue(paused.wait(.5)); handoff.manual('/motion/stop', {})
        release.set(); old.join(1)
        self.assertTrue(errors)
        self.assertFalse(any(call[1] == '/motion/jog' for call in backend.calls))

    def test_claim_refuses_inflight_manual_limits_save_without_stopping_it(self):
        from control_handoff import HandoffError
        handoff, backend = self.make()
        entered, release = threading.Event(), threading.Event()
        original = backend.request
        def request(target, path, payload=None, timeout=1):
            if path == '/motion/limits': entered.set(); release.wait(2)
            return original(target, path, payload, timeout)
        backend.request = request
        worker = threading.Thread(target=lambda: handoff.manual('/motion/limits', {}))
        worker.start(); self.assertTrue(entered.wait(.5))
        before = list(backend.calls)
        with self.assertRaises(HandoffError): handoff.claim()
        self.assertEqual(backend.calls, before)
        release.set(); worker.join(1)
    def test_session_motor_factory_fences_constructor_and_stop_paths(self):
        from control_handoff import SessionMotor
        from unittest.mock import patch
        result = {'initialized': True, 'firmware_watchdog': True, 'generation': 'g',
                  'stop_acknowledged': True, 'stopped_step_count_confirmed': True}
        with patch('transport.HttpMotor._call', return_value=result) as call:
            motor = SessionMotor('http://127.0.0.1:18083', 'a'*48, .05)
            motor.stop()
        self.assertEqual(call.call_args_list[0].args[0], '/flight/'+'a'*48+'/motor/status')
        self.assertEqual(call.call_args_list[1].args[0], '/flight/'+'a'*48+'/motor/stop')


class DeploymentTests(unittest.TestCase):
    def test_clean_clone_service_starts_use_repository_paths_and_strict_ssh(self):
        from control_handoff import PiBackend
        from unittest.mock import patch
        import os
        import shlex
        import subprocess
        with patch.dict(os.environ, {}, clear=True):
            backend = PiBackend()
            with patch('control_handoff.subprocess.run', return_value=subprocess.CompletedProcess([], 0, '', '')) as run, patch.object(backend, 'request', return_value={'connected': True, 'initialized': True}):
                backend.service('manual', True)
                backend.service('automatic', True)
        commands = [call.args[0] for call in run.call_args_list]
        self.assertEqual(commands[0][-2], 'pi@raspberrypi.local')
        self.assertIn('StrictHostKeyChecking=yes', commands[0])
        self.assertFalse(any(value.startswith('HostKeyAlias=') for value in commands[0]))
        self.assertIn('/home/pi/gimbal/manual_dashboard/motion_gateway.py', shlex.split(commands[0][-1]))
        self.assertIn('/home/pi/gimbal/pi_tracking_gateway.py', shlex.split(commands[1][-1]))

    def test_environment_changes_ssh_identity_and_remote_service_paths(self):
        from control_handoff import PiBackend, _raw_manual_tunnel_args
        from unittest.mock import patch
        import os
        import shlex
        import subprocess
        settings = {'DET_SSH_HOST': 'operator@bench.local', 'DET_SSH_HOST_KEY_ALIAS': 'bench-key',
                    'DET_PI_ROOT': '/srv/tracker', 'DET_MANUAL_GATEWAY': '/opt/manual gateway.py',
                    'DET_AUTO_GATEWAY': '/opt/automatic.py'}
        with patch.dict(os.environ, settings, clear=True):
            backend = PiBackend()
            tunnel = _raw_manual_tunnel_args()
            with patch('control_handoff.subprocess.run', return_value=subprocess.CompletedProcess([], 0, '', '')) as run, patch.object(backend, 'request', return_value={'connected': True, 'initialized': True}):
                backend.service('manual', True)
                backend.service('automatic', True)
        commands = [call.args[0] for call in run.call_args_list]
        self.assertEqual(tunnel[-1], 'operator@bench.local')
        self.assertIn('StrictHostKeyChecking=yes', tunnel)
        self.assertIn('HostKeyAlias=bench-key', tunnel)
        self.assertEqual(commands[0][-2], 'operator@bench.local')
        self.assertIn('/opt/manual gateway.py', shlex.split(commands[0][-1]))
        self.assertIn('/opt/automatic.py', shlex.split(commands[1][-1]))


if __name__ == '__main__': unittest.main()
