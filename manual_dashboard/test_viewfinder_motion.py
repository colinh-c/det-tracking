import io
import json
import types
import unittest
from unittest.mock import patch

import viewfinder


class MotionProxyTests(unittest.TestCase):
    def state(self):
        return viewfinder.State('http://127.0.0.1:18082', 'http://127.0.0.1:18083')

    def post(self, state, path, data, token=True, origin=None, content_type='application/json'):
        handler = viewfinder.Handler.__new__(viewfinder.Handler)
        handler.server = types.SimpleNamespace(state=state, server_port=18182)
        handler.path = path
        body = json.dumps(data).encode()
        handler.headers = {'Host': '127.0.0.1:18182', 'Content-Length': str(len(body)),
                           'Content-Type': content_type}
        if token:
            handler.headers['X-DET-Token'] = state.motion_token
        if origin:
            handler.headers['Origin'] = origin
        handler.rfile = io.BytesIO(body)
        replies = []
        handler.json = lambda *args: replies.append(args)
        handler.do_POST()
        return replies[0]

    def test_untrusted_motion_requests_never_reach_controller(self):
        state = self.state()
        for options in ({'token': False}, {'origin': 'https://example.com'},
                        {'content_type': 'text/plain'}):
            with patch.object(state, 'motion') as send:
                self.assertEqual(self.post(state, '/motion/jog', {'axis': 'az', 'delta': .1}, **options)[0], 403)
                send.assert_not_called()

    def test_frozen_or_stale_views_block_jog_before_upstream_write(self):
        state = self.state()
        state.frozen = {'id': 'existing'}
        with patch.object(viewfinder, 'urlopen') as remote:
            with self.assertRaisesRegex(viewfinder.CalibrationError, 'Resume'):
                state.motion('/motion/jog', {'axis': 'az', 'delta': .1})
            remote.assert_not_called()
        state.frozen = None
        with patch.object(state, 'fetch', return_value=(json.dumps([{'age_s': 3}, {'age_s': .1}]).encode(), {})), patch.object(viewfinder, 'urlopen') as remote:
            with self.assertRaisesRegex(viewfinder.CalibrationError, 'live'):
                state.motion('/motion/jog', {'axis': 'az', 'delta': .1})
            remote.assert_not_called()

    def test_stop_bypasses_freeze_and_move_lock(self):
        state = self.state()
        state.frozen = {'id': 'existing'}
        state.move_lock.acquire()
        response = types.SimpleNamespace(read=lambda: b'{"stopped":true}')
        with patch.object(viewfinder, 'urlopen') as remote:
            remote.return_value.__enter__.return_value = response
            self.assertEqual(state.motion('/motion/stop', {}), {'stopped': True})
            self.assertEqual(remote.call_args.args[0].full_url, 'http://127.0.0.1:18083/motion/stop')

    def test_resume_clears_frozen_pose_before_jogs(self):
        state = self.state()
        state.frozen = {'id': 'old'}
        self.assertEqual(self.post(state, '/resume', {})[0], 200)
        self.assertIsNone(state.frozen)

    def test_inflight_motion_blocks_new_freeze(self):
        state = self.state()
        state.move_lock.acquire()
        with self.assertRaisesRegex(viewfinder.CalibrationError, 'movement'):
            state.freeze()

    def test_hold_routes_require_authentication_and_forward_identity(self):
        state = self.state()
        for path in ('/motion/hold/start', '/motion/hold/renew'):
            with patch.object(state, 'motion', return_value={'hold_active': True}) as send:
                self.assertEqual(self.post(state, path, {'hold_id': 'hold-1'}, token=False)[0], 403)
                send.assert_not_called()
                self.assertEqual(self.post(state, path, {'hold_id': 'hold-1'})[0], 200)
                send.assert_called_once_with(path, {'hold_id': 'hold-1'})

    def test_hold_start_and_renew_require_current_live_views(self):
        state = self.state()
        for path in ('/motion/hold/start', '/motion/hold/renew'):
            state.frozen = {'id': 'existing'}
            with patch.object(viewfinder, 'urlopen') as remote:
                remote.return_value.__enter__.return_value = types.SimpleNamespace(read=lambda: b'{}')
                with self.assertRaisesRegex(viewfinder.CalibrationError, 'Resume'):
                    state.motion(path, {'hold_id': 'hold-1'})
                remote.assert_not_called()
            state.frozen = None
            with patch.object(state, 'fetch', return_value=(b'[{"age_s":3},{"age_s":0.1}]', {})), patch.object(viewfinder, 'urlopen') as remote:
                with self.assertRaisesRegex(viewfinder.CalibrationError, 'live'):
                    state.motion(path, {'hold_id': 'hold-1'})
                remote.assert_not_called()

    def test_active_continuous_movement_blocks_freeze_between_renewals(self):
        state = self.state()
        with patch.object(state, 'motion', return_value={'busy': True, 'hold_active': True}), patch.object(state, '_freeze') as capture:
            with self.assertRaisesRegex(viewfinder.CalibrationError, 'movement'):
                state.freeze()
            capture.assert_not_called()

    def test_scoped_release_bypasses_camera_failure_and_preserves_identity(self):
        state = self.state()
        state.frozen = {'id': 'existing'}
        state.move_lock.acquire()
        with patch.object(state, 'fetch', side_effect=OSError('camera offline')) as fetch, patch.object(viewfinder, 'urlopen') as remote:
            remote.return_value.__enter__.return_value = types.SimpleNamespace(read=lambda: b'{"stop_acknowledged":true}')
            self.assertTrue(state.motion('/motion/stop', {'hold_id': 'hold-1'})['stop_acknowledged'])
            self.assertEqual(json.loads(remote.call_args.args[0].data), {'hold_id': 'hold-1'})
            fetch.assert_not_called()

    def test_limits_route_requires_auth_and_forwards_exact_payload(self):
        state = self.state()
        values = {'az_min': -30, 'az_max': 40, 'el_min': 0, 'el_max': 80}
        for options in ({'token': False}, {'origin': 'https://example.com'},
                        {'content_type': 'text/plain'}):
            with patch.object(state, 'motion') as send:
                self.assertEqual(self.post(state, '/motion/limits', values, **options)[0], 403)
                send.assert_not_called()
        with patch.object(state, 'motion', return_value={'limits': values}) as send:
            self.assertEqual(self.post(state, '/motion/limits', values)[0], 200)
            send.assert_called_once_with('/motion/limits', values)

    def test_limits_do_not_require_live_cameras_or_resume(self):
        state = self.state()
        state.frozen = {'id': 'existing'}
        state.move_lock.acquire()
        values = {'az_min': -30, 'az_max': 40, 'el_min': 0, 'el_max': 80}
        with patch.object(state, 'fetch', side_effect=OSError('camera offline')) as fetch, patch.object(viewfinder, 'urlopen') as remote:
            remote.return_value.__enter__.return_value = types.SimpleNamespace(read=lambda: b'{"applied":true}')
            self.assertTrue(state.motion('/motion/limits', values)['applied'])
            self.assertEqual(json.loads(remote.call_args.args[0].data), values)
            self.assertEqual(remote.call_args.args[0].headers['X-det-motion'], '1')
            fetch.assert_not_called()


if __name__ == '__main__':
    unittest.main()
