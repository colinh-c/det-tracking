"""Shared acknowledged limits are preserved, never replayed to the MCU."""
import json
from pathlib import Path
import tempfile
import unittest
from tests.test_pi_tracking_gateway import FakeLink
from pi_tracking_gateway import TrackingGateway, GatewayError, LinkError


class SharedLimitsTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.path = Path(self.tmp.name)/'limits.json'
        self.path.write_text(json.dumps({'schema_version': 1, 'acknowledged': True,
                             'limits': {'az_min': -.1357, 'az_max': 32.46187,
                                        'el_min': -5.16234, 'el_max': 42.30070}}))
        self.link = FakeLink(); self.link.flags = 0
        self.clock = [10.]
    def gateway(self):
        def sleep(dt): self.clock[0] += dt
        g = TrackingGateway(self.link, clock=lambda: self.clock[0], sleep=sleep,
                            start_watchdog=False, limits_path=self.path)
        self.addCleanup(g.close); return g
    def test_user_bounds_visible_in_status_pose_without_lim_zero_or_home(self):
        g = self.gateway(); g.initialize()
        limits = g.fresh_pose()['limits']
        self.assertEqual(limits['az'], [-.1357, 32.46187])
        self.assertEqual(limits['el'], [-5.16234, 42.30070])
        self.assertTrue(limits['acknowledged']); self.assertFalse(limits['readback_available'])
        self.assertEqual(g.status()['limits'], limits)
        self.assertFalse(any(c.startswith(('LIM', 'ZERO', 'SYNC', 'HOME')) for c in self.link.commands))
    def test_reset_marker_refuses_before_encoder_disable_and_limit_replay(self):
        self.link.flags = 8; g = self.gateway()
        with self.assertRaises(GatewayError): g.initialize()
        self.assertNotIn('ENC 0', self.link.commands)
        self.assertFalse(g.status()['initialized'])
        self.assertFalse(g.status()['limits']['acknowledged'])
        self.assertFalse(any(c.startswith('LIM') for c in self.link.commands))
        self.link.flags = 0  # let the fake's final cleanup acknowledge rest

    def test_manual_unconfirmed_profile_blocks_auto_even_after_encoder_disable(self):
        profile = json.loads(self.path.read_text())
        profile['acknowledged'] = False
        self.path.write_text(json.dumps(profile))
        self.link.flags = 0
        with self.assertRaisesRegex(LinkError, 'acknowledged'):
            self.gateway()
        self.assertEqual(self.link.commands, [])
    def test_count_outside_user_range_faults_even_inside_factory_range(self):
        g = self.gateway(); g.initialize()
        request = self.link.request
        def outside(command, timeout, stop=False):
            if command == 'STAT': return ['OK 40.00000 0.00000 0 0 0 0 1 0']
            return request(command, timeout, stop)
        self.link.request = outside
        with self.assertRaises(GatewayError): g.fresh_pose()
        self.assertIsNotNone(g.status()['fault'])
        self.link.request = request

if __name__ == '__main__': unittest.main()
