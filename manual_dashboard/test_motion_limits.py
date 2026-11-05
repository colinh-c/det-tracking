import json
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
        with self.assertRaises(motion.MotionError): restored.jog('az', 1.)
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


if __name__ == '__main__': unittest.main()
