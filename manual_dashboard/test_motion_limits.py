import json
import os
import stat
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from test_motion_gateway import motion, Board, Clock


class LimitsBoard(Board):
    def __init__(self, clock):
        super().__init__(); self.clock = clock; self.boot_offset = 0
    def request(self, command, timeout, stop=False):
        if command == 'LSTAT':
            self.commands.append(command)
            return ['OK LSTAT 0 0 %d' % ((int(self.clock.now*1e6)+10000000+self.boot_offset)&0xffffffff)]
        if command.startswith('VEL '):
            self.commands.append(command); self.mode = 3; return ['OK VEL']
        if command.startswith('LIM '):
            self.commands.append(command); return ['OK LIM']
        return super().request(command, timeout, stop)


class LimitsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/'limits.json'; self.clock = Clock(); self.board = LimitsBoard(self.clock)
        self.controller = self.make()
        log = patch('builtins.print'); log.start(); self.addCleanup(log.stop)
    def make(self):
        return motion.MotionController(self.board, self.clock.monotonic, self.clock.sleep,
                                       limits_path=self.path)

    def test_directory_flush_failure_cannot_clear_reset_marker(self):
        real_fsync = os.fsync
        def fail_directory(fd):
            if stat.S_ISDIR(os.fstat(fd).st_mode):
                raise OSError('directory flush failed')
            return real_fsync(fd)
        with patch.object(motion.os, 'fsync', side_effect=fail_directory):
            with self.assertRaises(motion.MotionError):
                self.controller.jog('az', .1)
        self.assertNotIn('ENC 0', self.board.commands)
        self.assertFalse(any(c.startswith(('MOV', 'VEL')) for c in self.board.commands))
    def apply(self, **changes):
        value = {'az_min': -270., 'az_max': 270., 'el_min': -30., 'el_max': 110.}
        value.update(changes); return self.controller.set_limits(value)
    def test_expanded_user_bounds_ack_save_and_control_jog_without_motion_on_apply(self):
        result = self.apply()
        self.assertEqual(result['limits']['soft_limits_deg']['az'], [-270., 270.])
        self.assertTrue(result['limits']['acknowledged']); self.assertFalse(result['limits']['readback_available'])
        self.assertTrue(self.path.is_file())
        self.assertFalse(any(c.startswith(('MOV', 'VEL', 'TRK', 'ZERO', 'SYNC', 'CLR')) for c in self.board.commands))
        self.assertIn('LIM -270.00000000 270.00000000 -30.00000000 110.00000000', self.board.commands)
        self.controller.jog('el', 100.)
        with self.assertRaises(motion.MotionError): self.controller.jog('el', 20.)
    def test_invalid_busy_and_outside_position_do_not_send_lim_or_stop_user(self):
        for changes in ({'az_min': float('nan')}, {'az_min': True}, {'az_min': 1e20}, {'az_min': 300.}):
            with self.subTest(changes=changes), self.assertRaises(motion.MotionError): self.apply(**changes)
        self.controller.busy = True
        with self.assertRaises(motion.MotionError): self.apply()
        self.controller.busy = False; self.board.position[0] = 50.
        with self.assertRaises(motion.MotionError): self.apply(az_max=20.)
        self.assertFalse(any(c.startswith(('LIM', 'STOP', 'MOV', 'VEL')) for c in self.board.commands))
    def test_saved_limits_survive_handoff_without_reapplying_but_reset_requires_click(self):
        self.apply(); self.clock.now += 1
        restored = self.make(); self.board.commands.clear()
        status = restored.status(refresh=True)
        self.assertFalse(status['limits']['confirmation_required'])
        self.assertTrue(status['limits']['acknowledged'])
        self.assertFalse(any(c.startswith('LIM') for c in self.board.commands))
        self.board.flags = 11; self.board.boot_offset = -10000000
        status = restored.status(refresh=True)
        self.assertTrue(status['limits']['confirmation_required'])
        self.assertFalse(status['recovery_required'])
        restored.jog('az', 1.)
        self.assertFalse(restored.status()['limits']['acknowledged'])
        self.assertFalse(json.loads(self.path.read_text())['acknowledged'])
        self.controller = restored; self.apply()
        self.assertFalse(restored.status()['limits']['confirmation_required'])
    def test_apply_can_reconfirm_bad_saved_profile_but_cannot_clear_motor_fault(self):
        self.path.write_text('{invalid')
        self.controller = self.make()
        self.assertTrue(self.controller.status()['limits']['confirmation_required'])
        self.apply()
        self.board.bad_driver = True
        with self.assertRaises(motion.MotionError): self.apply()
        self.assertTrue(self.controller.blocked)
        self.board.bad_driver = False
        with self.assertRaises(motion.MotionError): self.apply()
        self.assertNotIn('CLR', self.board.commands)

    def write_profile(self, acknowledged=True):
        limits = {'az_min': -.05, 'az_max': .05, 'el_min': -.05, 'el_max': .05}
        self.path.write_text(json.dumps({'schema_version': 1, 'limits': limits, 'acknowledged': acknowledged}))
        return limits

    def assert_no_implicit_limits_or_coordinate_changes(self):
        self.assertFalse(any(c.startswith(('LIM ', 'ZERO', 'SYNC', 'CLR')) for c in self.board.commands))

    def test_fresh_reset_step_ignores_draft_bounds_and_journals_before_enc_zero(self):
        draft = self.write_profile(); self.board.position = [-3., 16.]
        self.controller = self.make(); request = self.board.request; observed = []
        def verify_journal(command, timeout, stop=False):
            if command == 'ENC 0':
                saved = json.loads(self.path.read_text()); observed.append(saved)
                self.assertIs(saved['acknowledged'], False)
            return request(command, timeout, stop)
        self.board.request = verify_journal
        result = self.controller.jog('az', .5)
        self.assertAlmostEqual(-2.5, result['position']['az'], delta=16*motion.FINE_DEG)
        self.assertEqual(1, len(observed))
        self.assertEqual(motion.validate_limits(draft), motion.validate_limits(observed[0]['limits']))
        self.assertTrue(result['available']); self.assertTrue(result['limits']['confirmation_required'])
        self.assertFalse(result['limits']['acknowledged']); self.assertIsNone(result['last_error'])
        self.assert_no_implicit_limits_or_coordinate_changes()

    def test_valid_unacknowledged_draft_survives_restart_with_encoder_bit_clear(self):
        draft = self.write_profile(acknowledged=False); self.board.flags = 0; self.board.position = [4., 8.]
        self.controller = self.make(); status = self.controller.status(refresh=True)
        self.assertEqual(motion.validate_limits(draft), status['limits']['soft_limits_deg'])
        self.assertTrue(status['limits']['confirmation_required']); self.assertFalse(status['limits']['acknowledged'])
        self.assertTrue(status['available']); self.assertIsNone(status['last_error'])
        self.controller.jog('el', .5); restarted = self.make().status(refresh=True)
        self.assertFalse(restarted['limits']['acknowledged'])
        self.assertEqual(motion.validate_limits(draft), restarted['limits']['soft_limits_deg'])
        self.assert_no_implicit_limits_or_coordinate_changes()

    def test_missing_or_corrupt_profile_allows_step_and_creates_unconfirmed_record(self):
        for content in (None, '{invalid'):
            with self.subTest(content=content):
                if self.path.exists(): self.path.unlink()
                if content is not None: self.path.write_text(content)
                self.board = LimitsBoard(self.clock); self.controller = self.make()
                result = self.controller.jog('az', .5)
                self.assertTrue(result['step_count_target_confirmed']); self.assertTrue(result['available'])
                self.assertFalse(result['limits']['acknowledged']); self.assertTrue(result['limits']['confirmation_required'])
                self.assertIsNone(result['last_error'])
                self.assertIs(json.loads(self.path.read_text())['acknowledged'], False)
                self.assert_no_implicit_limits_or_coordinate_changes()

    def test_fresh_reset_hold_runs_without_applied_limits_then_lease_stops(self):
        draft = self.write_profile(acknowledged=False); self.board.position = [-3., 16.]; self.controller = self.make()
        self.controller.hold_start('fresh-unconfirmed', 'az', .5); self.controller.hold_thread.join(2)
        self.assertFalse(self.controller.hold_thread.is_alive())
        self.assertIn('VEL 0.50000000 0.00000000', self.board.commands); self.assertIn('STOP', self.board.commands)
        status = self.controller.status(); self.assertFalse(status['recovery_required'])
        self.assertEqual('lease_expired', status['last_hold_reason']); self.assertFalse(status['limits']['acknowledged'])
        self.assertEqual(motion.validate_limits(draft), status['limits']['soft_limits_deg'])
        self.assert_no_implicit_limits_or_coordinate_changes()

    def test_persistence_failure_stops_before_enc_zero_or_motion(self):
        self.write_profile(); self.controller = self.make()
        with patch.object(motion.os, 'replace', side_effect=OSError('disk unavailable')):
            with self.assertRaises(motion.MotionError): self.controller.jog('az', .5)
        self.assertNotIn('ENC 0', self.board.commands)
        self.assertFalse(any(c.startswith(('MOVR ', 'VEL ', 'LIM ')) for c in self.board.commands))
        self.assertTrue(json.loads(self.path.read_text())['acknowledged'])

    def test_confirmed_bounds_still_limit_steps_and_hold(self):
        self.apply(az_min=-1., az_max=1., el_min=-1., el_max=1.); self.board.commands.clear()
        with self.assertRaises(motion.MotionError): self.controller.jog('az', 2.)
        self.board.position[0] = 1.; self.controller.rest_position = {'az': 1., 'el': 0.}
        self.controller.hold_start('confirmed-fence', 'az', .5); self.controller.hold_thread.join(2)
        self.assertFalse(any(c.startswith(('MOVR ', 'VEL ')) for c in self.board.commands))
        self.assertTrue(self.controller.status()['limits']['acknowledged'])

    def test_unconfirmed_profile_does_not_bypass_driver_or_controller_fault(self):
        for kind in ('controller', 'driver'):
            with self.subTest(kind=kind):
                self.write_profile(acknowledged=False); self.board = LimitsBoard(self.clock)
                if kind == 'controller': self.board.flags = 15
                else: self.board.bad_driver = True
                self.controller = self.make()
                with self.assertRaises(motion.MotionError): self.controller.jog('az', .5)
                self.assertTrue(self.controller.blocked)
                self.assertFalse(any(c.startswith(('MOVR ', 'VEL ')) for c in self.board.commands))
                self.assert_no_implicit_limits_or_coordinate_changes()


if __name__ == '__main__': unittest.main()
