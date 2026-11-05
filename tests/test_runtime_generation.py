"""Runtime restart handling with in-memory source/motor doubles only."""
import json
import unittest
from unittest.mock import patch

import numpy as np

from det_types import Frame, Pose, Prediction
from launch_config import load_launch_config
from runtime import TrackingApp


class Source:
    def __init__(self):
        self.generation = 'gateway-a'
        self.generation_verified = True
        self.generation_revision = 1
        self.generation_changed = False
        self.encoder_readings = [{'bus': 1, 'status': 55, 'valid': False, 'field_weak': True}]
        self.camera_info = {'narrow': {'size': [640, 480], 'sequence': 10}}
        self.errors = {'encoders': 'Axis registration is not calibrated'}
        self.sequence = 10
        self.now = 100.
        self.pose_valid = False
        self.closed = False

    def read(self):
        pose = Pose(self.now, 0, 0, self.pose_valid)
        image = np.full((480, 640), 150, np.uint8)
        return [Frame('narrow', self.now, image, pose, self.sequence)], pose

    def close(self):
        self.closed = True


class Motor:
    def __init__(self, fail=False):
        self.stop_count = 0
        self.fail = fail
        self.commands = []

    def stop(self):
        self.stop_count += 1
        if self.fail:
            raise RuntimeError('Stop acknowledgement unavailable')

    def send(self, command):
        self.commands.append(command)

    def close(self):
        pass


class RuntimeGenerationTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_launch_config()
        self.source = Source()
        self.clock = patch('runtime.time.monotonic', side_effect=lambda: self.source.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.readiness = patch('runtime.live_readiness', return_value=[])
        self.readiness.start()
        self.addCleanup(self.readiness.stop)
        self.app = TrackingApp(self.cfg, 'live', self.source, allow_motion=True)
        self.addCleanup(self.app.close)

    def test_verified_restart_stops_discards_history_and_keeps_command_sequence(self):
        self.app.tick()
        old = self.app.engine
        old.perception.track = {'timestamp': 100, 'az': 20, 'el': 30, 'az_rate': 0, 'el_rate': 0}
        old.perception.appearance = {'narrow': {'old': True}}
        old.click_hint = Prediction(100, 20, 30, valid=True)
        old.previous = Prediction(100, 20, 30, valid=True)
        old.executor._integral[:] = 8
        self.app.command_sequence = 41
        self.app.launch_origin = 90
        self.app.phase_override = 'main'
        self.app.enabled = True
        motor = Motor()
        self.app.motor = motor
        self.source.generation = 'gateway-b'
        self.source.generation_revision = 2
        self.source.generation_changed = True
        self.source.pose_valid = True
        self.source.sequence = 1
        self.source.now = 100.1
        _, _, command, _ = self.app.tick()
        self.assertEqual(motor.stop_count, 1)
        self.assertEqual(motor.commands, [])
        self.assertIsNone(self.app.motor)
        self.assertFalse(self.app.enabled)
        self.assertIsNot(self.app.engine, old)
        self.assertIsNone(self.app.engine.click_hint)
        self.assertIsNone(self.app.engine.perception.track)
        self.assertEqual(self.app.engine.perception.appearance, {})
        self.assertTrue(np.all(self.app.engine.executor._integral == 0))
        self.assertEqual(command.sequence, 42)
        self.assertFalse(command.enabled)
        self.assertFalse(self.app.status['pose']['valid'])
        self.assertTrue(all(not frame.pose.valid for frame in self.app.frames.values()))
        self.assertEqual(self.app.launch_origin, 90)
        self.assertEqual(self.app.phase_override, 'main')
        self.assertEqual(self.app.status['gateway_generation'], 'gateway-b')
        self.assertTrue(self.app.status['generation_verified'])
        self.assertTrue(self.app.status['generation_changed'])
        reset = self.app.engine
        # Even an accidentally repeated source flag must not reset one epoch twice.
        self.source.now += .1
        self.source.sequence += 1
        self.app.tick()
        self.assertIs(self.app.engine, reset)
        self.assertEqual(motor.stop_count, 1)
        self.assertEqual(self.app.status['command']['sequence'], 43)

    def test_uuid_change_is_detected_even_if_transient_source_flag_was_missed(self):
        self.app.tick()
        old = self.app.engine
        self.source.now += .1
        self.source.generation = 'gateway-b'
        self.source.generation_revision = 2
        self.source.generation_changed = False
        self.app.tick()
        self.assertIsNot(self.app.engine, old)
        self.assertTrue(self.app.status['generation_changed'])

    def test_stop_failure_still_disarms_and_discards_old_engine(self):
        self.app.tick()
        old = self.app.engine
        motor = Motor(fail=True)
        self.app.motor = motor
        self.app.enabled = True
        self.source.now += .1
        self.source.generation = 'gateway-b'
        self.source.generation_changed = True
        self.source.generation_revision = 2
        self.app.tick()
        self.assertFalse(self.app.enabled)
        self.assertIsNone(self.app.motor)
        self.assertIsNot(self.app.engine, old)
        self.assertEqual(motor.commands, [])
        self.assertIn('Stop acknowledgement unavailable', self.app.error)
        with self.assertRaisesRegex(ValueError, 'fault'):
            self.app.action('start')

    def test_legacy_preview_cannot_arm_even_if_calibration_flags_are_ready(self):
        self.source.generation = None
        self.source.generation_verified = False
        self.source.generation_revision = 0
        self.cfg['encoders']['allow_legacy_gateway'] = True
        self.app.tick()
        self.assertEqual(self.app.status['connection_status'], 'legacy_preview')
        self.assertFalse(self.app.status['generation_verified'])
        with patch('transport.HttpMotor') as constructor:
            with self.assertRaisesRegex(ValueError, 'generation'):
                self.app.action('start')
            constructor.assert_not_called()

    def test_live_diagnostics_are_serializable_snapshots_and_do_not_promote_pose(self):
        self.app.tick()
        status = self.app.status
        self.assertEqual(status['connection_status'], 'verified')
        self.assertEqual(status['encoder_readings'], self.source.encoder_readings)
        self.assertEqual(status['source_errors'], self.source.errors)
        self.assertEqual(status['camera_info'], self.source.camera_info)
        self.assertFalse(status['pose']['valid'])
        self.source.encoder_readings[0]['status'] = 0
        self.source.camera_info['narrow']['size'][0] = 1
        self.source.errors.clear()
        self.assertEqual(status['encoder_readings'][0]['status'], 55)
        self.assertEqual(status['camera_info']['narrow']['size'], [640, 480])
        self.assertTrue(status['source_errors'])
        json.dumps(status, allow_nan=False)

    def test_unverified_changed_flag_does_not_accept_new_epoch(self):
        self.app.tick()
        old = self.app.engine
        self.source.now += .1
        self.source.generation_verified = False
        self.source.generation_changed = True
        self.source.errors = {'generation': 'Mixed gateway generations'}
        self.app.tick()
        self.assertIs(self.app.engine, old)
        self.assertFalse(self.app.status['generation_verified'])
        self.assertFalse(self.app.status['generation_changed'])
        self.assertEqual(self.app.status['connection_status'], 'unverified')


if __name__ == '__main__':
    unittest.main()
